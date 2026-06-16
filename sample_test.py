from re import sub
import copy
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import os
from itertools import product, accumulate

from krl_translator import KUKATranslator
from dataset_prep import generate_layer_toolpath

def discretize_lines_for_kinematics(deposition_lines, lines_parameters=None):
    """
    Converts a list of line dictionaries into flat parameter lists.
    """
  
    trajectory = []
    if lines_parameters is None:
        lines_parameters = {
            'points_per_line': 2,
            'safety_distance': 20.0
        }
    waypoint = {
        'point': np.array([-2*lines_parameters['safety_distance'], 0, 0, 1.0]),
        'sod': 100,
        'angle': 0,
        'rot_axis': np.array([0, 0, 1]),
        'velocity': 1.0
    }
    trajectory.append(waypoint)

    for line in deposition_lines:
        # Interpolate points along the line
        rot_axis = np.array(line['end']) - np.array(line['start'])  
        rot_axis = rot_axis / np.linalg.norm(rot_axis) if np.linalg.norm(rot_axis) > 0 else np.array([0, 0, 1])
        x_vals = np.array([line['start'][0] -lines_parameters.get('safety_distance', 20.0)*rot_axis[0]])
        y_vals = np.array([line['start'][1] -lines_parameters.get('safety_distance', 20.0)*rot_axis[1]])
        z_vals = np.array([line['start'][2] -lines_parameters.get('safety_distance', 20.0)*rot_axis[2]])
        x_vals = np.append(x_vals, np.linspace(line['start'][0], line['end'][0], lines_parameters.get('points_per_line', 2)))
        y_vals = np.append(y_vals, np.linspace(line['start'][1], line['end'][1], lines_parameters.get('points_per_line', 2)))
        z_vals = np.append(z_vals, np.linspace(line['start'][2], line['end'][2], lines_parameters.get('points_per_line', 2)))
        x_vals = np.append(x_vals, np.array([line['end'][0] +lines_parameters.get('safety_distance', 20.0)*rot_axis[0]]))
        y_vals = np.append(y_vals, np.array([line['end'][1] +lines_parameters.get('safety_distance', 20.0)*rot_axis[1]]))
        z_vals = np.append(z_vals, np.array([line['end'][2] +lines_parameters.get('safety_distance', 20.0)*rot_axis[2]]))

        for x, y, z in zip(x_vals, y_vals, z_vals):
            waypoint = {
                'point': np.array([x, y, z, 1.0]),
                'sod': line['sod'],
                'angle': 90-line['deposition_angle'],
                'rot_axis': rot_axis ,
                'velocity': line['velocity']/1000.0
            }
            trajectory.append(waypoint)
    
    last_line = deposition_lines[-1]
    rot_axis = np.array(last_line['end']) - np.array(last_line['start'])
    rot_axis = rot_axis / np.linalg.norm(rot_axis) if np.linalg.norm(rot_axis) > 0 else np.array([0, 0, 1])  
    waypoint = {
        'point': np.array([trajectory[-1]['point'][0]+lines_parameters['safety_distance']*rot_axis[0],trajectory[-1]['point'][1]+lines_parameters['safety_distance']*rot_axis[1], trajectory[-1]['point'][2]+lines_parameters['safety_distance']*rot_axis[2], 1.0]),
        'sod': 50,
        'angle': 0,
        'rot_axis': np.array([0, 0, 1]),
        'velocity': 0.5
    }
    trajectory.append(waypoint)
    return trajectory

def get_hardcoded_test_batch(tracks_parameters=None):
    """
    Defines a hardcoded batch of deposition lines for dataset generation.
    Returns the flat lists ready for the kinematic transformation function.
    """

    if tracks_parameters is None:
        tracks_parameters = {
            'substrate_width': 50.0
        }
    # Hardcoded Geometry (e.g., a simple 3-line back-and-forth raster)
    width = tracks_parameters.get('substrate_width', 50.0)
    hardcoded_lines = [
        {'start': [0, 15, 0],   'end': [width, 15, 0],   'sod': 100, 'deposition_angle': 90, 'velocity': 100},
        {'start': [width, 30, 0],'end': [0, 30, 0],    'sod': 30, 'deposition_angle': 90, 'velocity': 50},
        {'start': [0, 50, 0],  'end': [width, 50, 0],  'sod': 100, 'deposition_angle': 50, 'velocity': 100}  
    ]
    
    # Process into lists
    return [hardcoded_lines]


def generate_parameters_table(param_ranges=None, repeat=1):
    """
    Generates a table of test sample parameters from parameter ranges.

    Each parameter entry may be one of:
      - a list/tuple of explicit values
      - a dict with 'values' as an explicit list
      - a dict with 'min', 'max', and optional 'steps' for linear spacing

    Example:
      param_ranges = {
          'default_vel': {'min': 25, 'max': 100, 'steps': 4},
          'sod': {'values': [80, 100, 120]},
          'deposition_angle': {'values': [45, 60, 90]}
      }

    Returns a list of dictionaries, one per parameter combination.
    """
    default_ranges = {
        'velocity': {'values': [25]},
        'sod': {'values': [100]},
        'deposition_angle': {'values': [90]},
    }
    
    if param_ranges is None:
        param_ranges = default_ranges

    expanded_ranges = {}
    for name, spec in param_ranges.items():
        if isinstance(spec, dict):
            if 'values' in spec:
                expanded_ranges[name] = list(spec['values'])
            elif 'min' in spec and 'max' in spec:
                steps = int(spec.get('steps', 1))
                if steps <= 1:
                    expanded_ranges[name] = [spec['min']]
                else:
                    expanded_ranges[name] = np.linspace(spec['min'], spec['max'], num=steps).tolist()
            else:
                raise ValueError(
                    f"Parameter range for '{name}' must include 'values' or both 'min' and 'max'."
                )
        elif isinstance(spec, (list, tuple, np.ndarray)):
            expanded_ranges[name] = list(spec)
        else:
            raise ValueError(
                f"Parameter range for '{name}' must be a list, tuple, ndarray, or dict."
            )

    combinations = [
        dict(zip(expanded_ranges.keys(), values))
        for values in product(*expanded_ranges.values())
    ]


    if repeat > 1:
        repeated_combinations = []
        for combo in combinations:
            for _ in range(repeat):
                repeated_combinations.append(dict(combo))
        combinations = repeated_combinations

    return combinations

def visualize_parameter_combinations(combinations):
    """
    Utility function to visualize the generated parameter combinations.
    """
    print("Generated Parameter Combinations:")
    for i, combo in enumerate(combinations):
        print(f"Combination {i+1}: {combo}")

def print_deposition_tracks(deposition_lines):
    """
    Utility function to print deposition tracks parameters.
    """
    text = ""
    for i, line in enumerate(deposition_lines):
        text += f"Track {i+1}: {i%2 + 1} A{90-abs(90-line['deposition_angle'])} D{line['sod']} V{line['velocity']}\n"
    return text
def print_waypoints(waypoints, num = 10):
    """
    Utility function to print waypoints.
    """
    for i, waypoint in enumerate(waypoints):
        print(f"Waypoint {i+1}: {waypoint}")
        if i == num:
            break

def draw_3d(waypoints):
    """
    Utility function to visualize waypoints in 3D.
    """

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(projection='3d')
    points = np.array([waypoint['point'] for waypoint in waypoints])
    normals = np.array([6.4*waypoint['normal'] for waypoint in waypoints])
    tangents = np.array([2*waypoint['tangent'] for waypoint in waypoints])
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], color='red', s=1, label='Points')
    #ax.scatter(points[672:, 0], points[672:, 1], points[672:, 2], color='green', s=1, label='Points')
    
    #ax.quiver(points[-36:, 0], points[-36:, 1], points[-36:, 2], normals[-36:, 0], normals[-36:, 1], normals[-36:, 2], color='blue', label='Normals')
    ax.quiver(points[:36, 0], points[:36, 1], points[:36, 2], tangents[:36, 0], tangents[:36, 1], tangents[:36, 2], color='blue', label='Normals')
    
    #ax.scatter(0,0,0,color='green',s=5, label='Points')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    ax.axis('equal')
    
    plt.show()





def prepare_dataset_samples(tracks_parameters=None):
    """
    Main function to prepare dataset samples.
    """

    if tracks_parameters is None:
        tracks_parameters = {
            'parameter_ranges': {
                'deposition_angle': {'min': 50, 'max': 90, 'steps': 5},
                'sod': {'values': [30, 50, 70, 90, 100]},
                'velocity': {'values': [25, 50, 100]}
                
            },
            'substrate_width': 50.0,
            'intertrack_spacing': 15.0,
            'safety_offset': 10.0,
            'lengths_of_substrate': [50.0, 100.0, 150.0]
        }


    combinations = generate_parameters_table(tracks_parameters['parameter_ranges'], tracks_parameters['repeat'])

    deposition_lines = []
    i = 0
    j = 0
    y = tracks_parameters['safety_offset']
    substrate_lines = []
    for parameters in combinations:
        

        line = {
            'start': [(j % 2)*tracks_parameters['substrate_width'], y, 0],
            'end': [((j+1)% 2)*tracks_parameters['substrate_width'], y, 0],
            'sod': parameters['sod'],
            'deposition_angle': 90 + (-1)*((j % 2)*2-1)*(90 - parameters['deposition_angle']),
            'velocity': parameters['velocity']
        }
        substrate_lines.append(line)
        y += tracks_parameters['intertrack_spacing']
        j += 1
        if  y > tracks_parameters['lengths_of_substrate'][i] - tracks_parameters['safety_offset']:
            i+=1
            y = tracks_parameters['safety_offset']
            deposition_lines.append(substrate_lines)
            substrate_lines = []
            j=0

        if i >= len(tracks_parameters['lengths_of_substrate']):
            print("Warning: More parameter combinations than substrate lengths.")
            break
    if substrate_lines:
        deposition_lines.append(substrate_lines)
    return deposition_lines

def full_pipeline(method="hardcoded", tracks_parameters=None, lines_parameters=None, layer_parameters=None, program_parameters=None):
    """
    Executes the full dataset preparation pipeline.
    """

    if program_parameters is None:
        program_parameters = {
            'program_name': "plan_dessai",
            'routine_name': "Routine",
            'tool_id': [2,2,2,2,2,2],
            'base_id': [10, 9, 8, 7, 6, 5],
            'number_of_programs': 3,
            'number_of_substrates': [1,3,4],
            'output_dir': directory
        }
    if layer_parameters is None:
        layer_parameters = {
            'Layer_pos': [[0, 0, 0]] * 8,
            'layer_orientation': [R.from_matrix([[-1,0,0],[0,1,0],[0,0,-1]]).as_euler('zyx', degrees=True)] * 8
        }
    print("\n--- Generating Deposition Lines ---\n")
    if method == "hardcoded":
        lines = get_hardcoded_test_batch(tracks_parameters)
    elif method == "parameterized":
        lines = prepare_dataset_samples(tracks_parameters)
    elif method == "custom":
        waypoints = defaut_reparation_oval()
        return waypoints
    print(f"\nTotal line sets generated: {sum(len(line_set) for line_set in lines)}")
    print(f"Total trajectories generated: {len(lines)}\n")

    print("\n--- Discretizing Lines for Kinematics ---")
    trajectory = []
    for i, line_set in enumerate(lines):
        traj = discretize_lines_for_kinematics(line_set, lines_parameters)
        trajectory.append(traj)



    print(f"Total points generated: {sum(len(sublist) for sublist in trajectory)}\n")
    for i in range(min(2, len(trajectory))):
        for j in range(min(2, len(trajectory[i]))):
            print(f"Trajectory {i}, Point {j}: Pos={trajectory[i][j]['point'][:3]}, SoD={trajectory[i][j]['sod']}, Angle={trajectory[i][j]['angle']},vel={trajectory[i][j]['velocity']}")
    
    
    # 3. Pass into your kinematic function

    path = []
    for i,traj in enumerate(trajectory):
        subpath = generate_layer_toolpath(
            Layer_pos=layer_parameters['Layer_pos'][i],
            layer_orientation=layer_parameters['layer_orientation'][i],
            trajectory=traj
        )
        path.append(subpath)
        


    print("\n--- Kinematic Toolpath Generated ---")
    #for i, p in enumerate(path[0]):
    #    print(f"P{i+1}: LIN {{X {p['X']:>6}, Y {p['Y']:>6}, Z {p['Z']:>6}, A {p['A']:>6}, B {p['B']:>6}, C {p['C']:>6}}}")

    translator = KUKATranslator(
    program_name=program_parameters['program_name'],
    routine_name=program_parameters['routine_name'],
    tool_id=program_parameters['tool_id'],
    base_id=program_parameters['base_id']
)
    translator.change_substrates = my_change_substrates
    print("\n--- Generating KRL Programs ---")
    print(f"Total programs to generate: {len(path)}")
  
    k = 0
    accumulated_subs = list(accumulate(program_parameters['number_of_substrates']))


    if program_parameters['number_of_programs'] != 1:
        translator.program_name = f"{program_parameters['program_name']}_{1}"
    translator.generate_programs(trajectory=path[:accumulated_subs[0]], output_dir=program_parameters['output_dir'])

    for i in range(program_parameters['number_of_programs']-1):
        translator.program_name = f"{program_parameters['program_name']}_{i+2}"
        translator.generate_programs(trajectory=path[accumulated_subs[i]:accumulated_subs[i+1]], output_dir=program_parameters['output_dir'])
    text = ""
    for i , line in enumerate(lines):
        text += f"\n\n\n--- Substrate {i+1} Parameters ---\n\n"
        text += print_deposition_tracks(line)
    
    txt_file = os.path.join(program_parameters['output_dir'], f"{program_parameters['program_name']}_parameters.txt")
    with open(txt_file, 'w') as f:
        f.write(text)
    print(f"\nParameter details saved to: {txt_file}")

    
def defaut_reparation_oval(T= 5, t = 0.2 ,R0= 145,r0= 11,s = 2,theta0 = 45,alpha=57, gamma=90, sod=50,v=100):
    """this function is to generate the points, normals, and tangents for an oval-like paths
    T: Total Thickness of defaut
    t: thickness of one layer of deposition
    R0: middle radius of defaut arc
    r0: initial radius of defaut cross-section
    s: deplacement between each track
    theta0: angle of defaut cross-section
    alpha: angle of defaut arc
    gamma: deposition angle
    sod: Standoff distance
    v: deposition velocity
    
    """

    waypoints = []
    for i in range(T//t):
        r = r0-t*i
        dth = s/r
        for theta in np.arange(90, theta0, -dth):
            T12 = np.array([[0,-1,r],[1,0 ,0]])
            X2 = np.array([r*np.cos(np.deg2rad(theta)), r*np.sin(np.deg2rad(theta)),1])
            X1 = T12@X2
            RR = [R0+X1[1],R0-X1[1]]
            poses = []
            Tangents = [] #in global base
            for R1 in RR:
                poses.append(np.array([RR*np.cos(np.deg2rad(180+alpha)),RR*np.sin(np.deg2rad(180+alpha)), T-X1[0]]))
                poses.append(np.array([RR*np.cos(np.deg2rad(180-alpha)),RR*np.sin(np.deg2rad(180-alpha)), T-X1[0]]))
                Tangents.append(np.array([np.cos(np.deg2rad(90+alpha)),np.sin(np.deg2rad(90+alpha)),0]))
                Tangents.append(np.array([np.cos(np.deg2rad(90-alpha)),np.sin(np.deg2rad(90-alpha)),0]))
            Tangents[2] = -Tangents[2]
            Tangents[3] = -Tangents[3]


            na = np.array([np.sin(np.deg2rad(90+theta)),0,np.cos(np.deg2rad(90+theta))])
            for pose, tangent in zip(poses, Tangents):
                tr = np.identity(4)
                tr[:3,:] = np.column_stack((np.cross(tangent,np.array([0,0,1])),tangent,np.array([0,0,1])),pose)

                waypoint = {
                    'point': pose,
                    'sod': sod,
                    'angle': gamma,
                    'velocity': v,
                    'tangent' : tangent,
                    'normal' : na,
                    'transform' : tr,
                    'mode': 'CIRC'
                }

                waypoints.append(waypoint)
    return waypoints


    
def defaut_reparation_balayage(T= 5, t = 0.2 ,R0= 250,r0= 11,s = 2,theta0 = 45,alpha0=10, gamma=90, sod=50,v=100):
    """this function is to generate the points, normals, and tangents for an oval-like paths
    T: Total Thickness of defaut
    t: thickness of one layer of deposition
    R0: middle radius of defaut arc
    r0: initial radius of defaut cross-section
    s: deplacement between each track
    theta0: angle of defaut cross-section
    alpha: angle of defaut arc
    gamma: deposition angle
    sod: Standoff distance
    v: deposition velocity
    
    """
    safety = 3
    k = 0
    waypoints = []
    alphas = [alpha0+safety, alpha0,-alpha0,-alpha0-safety]
    for i in range(int(T//t)):
        r = r0-t*i
        print(r)
        dth = np.rad2deg(s/r)
        ranging = np.arange(theta0, 180-theta0, dth) if i%2 == 0 else np.arange(180-theta0, theta0, -dth)
        for theta in ranging:
            T12 = np.array([[0,-1,r0],[1,0 ,0]])
            X2 = np.array([r*np.cos(np.deg2rad(theta)), r*np.sin(np.deg2rad(theta)),1])
            X1 = T12@X2
            RR = R0+X1[1]
            '''
            print(f'RR: {RR}')
            print(f'T12: {T12}')
            print(f'X1: {X1}')
            print(f'X2: {X2}')
            print(f'r: {r}')'''

            poses = []
            Tangents = [] #in global base
            for alpha in alphas:
                poses.append(np.array([RR*np.cos(np.deg2rad(180+alpha)),RR*np.sin(np.deg2rad(180+alpha)), T-X1[0]]))
                Tangents.append(((-1)**k)*np.array([np.cos(np.deg2rad(90+alpha)),np.sin(np.deg2rad(90+alpha)),0]))

            
            modes = ['LIN','CIRC','CIRC','CIRC']
            na = [np.array([np.sin(np.deg2rad(90+theta)),0,np.cos(np.deg2rad(90+theta))]),np.array([np.sin(np.deg2rad(90-theta)),0,np.cos(np.deg2rad(90-theta))])]
            for pose, tangent, mode in zip(poses, Tangents,modes):
                tr = np.identity(4)
                tr[:3,:] = np.column_stack((np.cross(tangent,np.array([0,0,(-1)**k])),tangent,np.array([0,0,(-1)**k]),pose))

                waypoint = {
                    'point': pose,
                    'sod': sod,
                    'angle': (-1)**k*(90-gamma),
                    'velocity': v,
                    'tangent' : tangent,
                    'normal' : tr[:3,:3]@na[k%2],
                    'mode': mode
                }

                waypoints.append(waypoint)
            k+=1
        



    return waypoints
            






                



def my_change_substrates(i = 0, trajectory = None):
    """Example implementation of a substrate change routine."""
    lengths_of_substrate = [297.0, 350.0, 310.0, 350.0,225.0,220.0,267.0,315.0]
    point1 = copy.deepcopy(trajectory[-1])
    point1['Y'] = lengths_of_substrate[i] + 15.0  # Move to a safe Y position beyond the current substrate
    point1['Z'] = -20.0
    point1['A'] = 0.0
    point1['B'] = 0.0
    point1['C'] = 0.0
    point1['VEL'] = 1.0
    point2 = copy.deepcopy(point1)
    point2['X'] = trajectory[0]['X']  # Move back to the starting X position of the next substrate
    print(f"Changing substrate after trajectory {i} with {len(trajectory)} points.")
    return [point1, point2]

# --- Execution & Verification ---
if __name__ == "__main__":

    tracks_parameters = {
        'parameter_ranges': {
            #'deposition_angle': {'min': 50, 'max': 90, 'steps': 5},
            'deposition_angle': {'values': [90,80,70,60,50]},
            'sod': {'values': [30, 50, 70, 90, 100]},
            'velocity': {'values': [50, 100, 150]}
                    
            
        },
        'repeat': 2 ,
        'substrate_width': 50.0,
        'intertrack_spacing': 15.0,
        'safety_offset': 10.0,
        'lengths_of_substrate': [297.0, 350.0, 310.0, 350.0,225.0,220.0,267.0,315.0]
    }

    lines_parameters = {
        'points_per_line': 2,
        'safety_distance': 50.0
    }

    layer_parameters = {
        'Layer_pos': [[0, 0, 0]]*8,
        'layer_orientation': [R.from_matrix([[-1,0,0],[0,1,0],[0,0,-1]]).as_euler('zyx', degrees=True)]*8
    }

    directory = "/mnt/bureau_folder/Codes/Robot source code/Autogenerated"
    if not os.path.exists(directory):
        directory = "./robot_programs"
        

    program_parameters = {
        'program_name': "ilf_test",
        'routine_name': "Routine",
        'tool_id': [2,2,2,2,2,2],
        'base_id': [10, 9, 8, 7, 6, 5],
        'number_of_programs': 3,
        'number_of_substrates': [1,3,4],
        'output_dir': directory
    }
    # parameterized, hardcoded
    #full_pipeline(method="parameterized", tracks_parameters=tracks_parameters, lines_parameters=lines_parameters, layer_parameters=layer_parameters, program_parameters=program_parameters)    
    waypoints = defaut_reparation_balayage()
    draw_3d(waypoints)
    print(len(waypoints))
