
import logging
from re import sub
import copy
import sys
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import os
from itertools import product, accumulate
from krl_translator import KUKATranslator
from dataset_prep import generate_layer_toolpath_experimental as generate_layer_toolpath
import pandas as pd

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
        'point': np.array([-2*lines_parameters['safety_distance'], 0, 0]),
        'sod': 100,
        'angle': 0,
        'tangent': np.array([0, 0, 1]),
        'normal': np.array([0, 0, 1]),
        'velocity': 1.0,
        'mode': 'LIN'
    }
    trajectory.append(waypoint)



    for line in deposition_lines:
        # Interpolate points along the line
        tangent = np.array(line['end']) - np.array(line['start'])  
        tangent = tangent / np.linalg.norm(tangent) if np.linalg.norm(tangent) > 0 else np.array([0, 0, 1])
        x_vals = np.array([line['start'][0] -lines_parameters.get('safety_distance', 20.0)*tangent[0]])
        y_vals = np.array([line['start'][1] -lines_parameters.get('safety_distance', 20.0)*tangent[1]])
        z_vals = np.array([line['start'][2] -lines_parameters.get('safety_distance', 20.0)*tangent[2]])
        x_vals = np.append(x_vals, np.linspace(line['start'][0], line['end'][0], lines_parameters.get('points_per_line', 2)))
        y_vals = np.append(y_vals, np.linspace(line['start'][1], line['end'][1], lines_parameters.get('points_per_line', 2)))
        z_vals = np.append(z_vals, np.linspace(line['start'][2], line['end'][2], lines_parameters.get('points_per_line', 2)))
        x_vals = np.append(x_vals, np.array([line['end'][0] +lines_parameters.get('safety_distance', 20.0)*tangent[0]]))
        y_vals = np.append(y_vals, np.array([line['end'][1] +lines_parameters.get('safety_distance', 20.0)*tangent[1]]))
        z_vals = np.append(z_vals, np.array([line['end'][2] +lines_parameters.get('safety_distance', 20.0)*tangent[2]]))
        normal = np.array([0, 0, 1])  
        for x, y, z in zip(x_vals, y_vals, z_vals):
            waypoint = {
                'point': np.array([x, y, z]),
                'sod': line['sod'],
                'angle': 90-line['deposition_angle'],
                'tangent': tangent,
                'normal': normal,
                'velocity': line['velocity']/1000.0,
                'mode': 'LIN'
            }
            trajectory.append(waypoint)
    
    last_line = deposition_lines[-1]
    tangent = np.array(last_line['end']) - np.array(last_line['start'])
    tangent = tangent / np.linalg.norm(tangent) if np.linalg.norm(tangent) > 0 else np.array([0, 0, 1])
    waypoint = {
        'point': np.array([trajectory[-1]['point'][0]+lines_parameters['safety_distance']*tangent[0],trajectory[-1]['point'][1]+lines_parameters['safety_distance']*tangent[1], trajectory[-1]['point'][2]+lines_parameters['safety_distance']*tangent[2]]),
        'sod': 50,
        'angle': 0,
        'tangent': tangent,
        'normal': np.array([0, 0, 1]),
        'velocity': 0.5,
        'mode': 'LIN'
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
def draw_kuka_points(waypoints):
    """
    Utility function to visualize waypoints in 3D.
    """
    pass
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
    
    ax.quiver(points[-36:, 0], points[-36:, 1], points[-36:, 2], normals[-36:, 0], normals[-36:, 1], normals[-36:, 2], color='blue', label='Normals')
    #ax.quiver(points[:36, 0], points[:36, 1], points[:36, 2], tangents[:36, 0], tangents[:36, 1], tangents[:36, 2], color='blue', label='Normals')
    
    #ax.scatter(0,0,0,color='green',s=5, label='Points')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    ax.axis('equal')
    
    plt.show()
def draw_path(waypoints):
    """
    Utility function to visualize waypoints in 3D and also single points on the path.
    """
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(projection='3d')
    points = np.array([waypoint['point'] for waypoint in waypoints])
    ax.plot(points[:, 0], points[:, 1], points[:, 2], color='red', label='Path')
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], color='blue', s=5, label='Sample Points')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    ax.axis('equal')


    
    plt.show()

def prepare_dataset_samples(**kwargs):
    """
    Main function to prepare dataset samples.
    """
    tracks_parameters = kwargs.get('tracks_parameters', None)
    lines_parameters = kwargs.get('lines_parameters', None)
    layer_parameters = kwargs.get('layer_parameters', None)
    program_parameters = kwargs.get('program_parameters', None)
    combinations_dir = tracks_parameters.get('combinations_dir', None)
    
    if combinations_dir is None:
        logging.info(f"No parameter file provided. Generating parameter combinations from ranges.")
        combinations = generate_parameters_table(tracks_parameters['parameter_ranges'], tracks_parameters['repeat'])
    else:
        logging.info(f"Loading parameters from {combinations_dir}")
        df_imported = pd.read_csv(combinations_dir)
        df_imported = df_imported.round(decimals=0)
        combinations = df_imported.to_dict(orient='records')
    
    logging.info(f"Parameters combinations number to be generated: {len(combinations)}")
    
    logging.info(f"\n--- Generating Parameterized Lines ---\n")
    i, j = 0, 0
    substrate_lines, deposition_lines = [], []
    y = tracks_parameters['safety_offset']
    prev_sod = 30
    for parameters in combinations:
        if tracks_parameters['adaptive_spacing']:
            y += (parameters['sod'] - prev_sod) * 4.0/70.0
        line = {
            'start': [(j % 2)*tracks_parameters['substrate_width'], y, 0],
            'end': [((j+1)% 2)*tracks_parameters['substrate_width'], y, 0],
            'sod': parameters['sod'],
            'deposition_angle': 90 + (1)*((j % 2)*2-1)*(90 - parameters['deposition_angle']),
            'velocity': parameters['velocity']
        }
        print(f"Track {j+1} on Substrate {i+1}: Start {round(line['start'][1], 2)}, End {round(line['end'][1], 2)}, SOD {round(line['sod'], 2)}, Angle {round(line['deposition_angle'], 2)}, Velocity {round(line['velocity'], 2)}")
        substrate_lines.append(line)

        prev_sod = parameters['sod']
        y += tracks_parameters['intertrack_spacing']
        

        j += 1
        if  y > tracks_parameters['lengths_of_substrate'][i] - tracks_parameters['safety_offset']:
            i+=1
            y = tracks_parameters['safety_offset']
            deposition_lines.append(substrate_lines)
            prev_sod = 30
            substrate_lines = []
            j=0

        if i >= len(tracks_parameters['lengths_of_substrate']):
            logging.warning("More parameter combinations than substrate lengths.")
            break

    if substrate_lines:
        deposition_lines.append(substrate_lines)


    text = ""
    for i , substrate_lines in enumerate(deposition_lines):
        text += f"\n\n\n--- Substrate {i+1} Parameters ---\n\n"
        text += print_deposition_tracks(substrate_lines)
    
    txt_file = os.path.join(program_parameters['output_dir'], f"{program_parameters['program_name']}_parameters.txt")
    with open(txt_file, 'w') as f:
        f.write(text)
        f.close()
    logging.info(f"\nParameter details saved to: {txt_file}")

    logging.info(f"Total deposition lines generated: {sum(len(sublist) for sublist in deposition_lines)}")
    logging.info(f"Total substrates generated: {len(deposition_lines)}")
    logging.info(f"Total deposition lines per substrate: {[len(sublist) for sublist in deposition_lines]}")
    routines = []
    for i, line_set in enumerate(deposition_lines):
        routine = discretize_lines_for_kinematics(line_set, lines_parameters)
        routines.append(routine)

    logging.info(f"Routines are ready for KUKA transformation. Total routines: {len(routines)}")

    return routines

def full_pipeline(funcs=prepare_dataset_samples, **kwargs):
    """
    Executes the full dataset preparation pipeline.
    """

    default_tracks_parameters = {
        'parameter_ranges': {
            #'deposition_angle': {'min': 50, 'max': 90, 'steps': 5},
            'deposition_angle': {'min': 50, 'max': 90, 'steps': 5},
            'sod': {'min': 30, 'max': 100,  'steps': 5},
            'velocity': {'min': 50, 'max': 150, 'steps': 3}
        },
        'repeat': 1 ,
        'substrate_width': 50.0,
        'intertrack_spacing': 15.0,
        'adaptive_spacing': True,
        'safety_offset': 10.0,
        'lengths_of_substrate': [240,320,350,360]
    }
    default_lines_parameters = {
        'points_per_line': 2,
        'safety_distance': 50.0
    }
    default_layer_parameters = {
        'Layer_pos': [[0,0,0]]*8,
        'layer_orientation': [R.from_matrix([[-1,0,0],[0,1,0],[0,0,-1]]).as_euler('zyx', degrees=True)]*8
    }
    default_program_parameters = {
        'program_name': "profile_defaut",
        'routine_name': "Routine",
        'tool_id': [2,2,2,2,2,2],
        'base_id': [10, 9, 8, 7, 6, 5],
        'number_of_programs': 1,
        'number_of_substrates': [4],
        'output_dir': "output"
    }
    if funcs is None:
        funcs = [defaut_reparation_balayage_2]

    tracks_parameters = kwargs.get('tracks_parameters', default_tracks_parameters)
    lines_parameters = kwargs.get('lines_parameters', default_lines_parameters)
    layer_parameters = kwargs.get('layer_parameters', default_layer_parameters)
    program_parameters = kwargs.get('program_parameters', default_program_parameters)

    routines = funcs(tracks_parameters=tracks_parameters, lines_parameters=lines_parameters, layer_parameters=layer_parameters, program_parameters=program_parameters)
    draw_path(routines[-1])
    draw_3d(routines[-1])
    logging.info(f"Generating layers Routines")
    kuka_routines = generate_layer_toolpath(
            Layer_poses=layer_parameters['Layer_pos'],
            layer_orientations=layer_parameters['layer_orientation'],
            routines=routines
        )


    logging.info(f"Generating KUKA Programs")
    translator = KUKATranslator(
        program_name=program_parameters['program_name'],
        routine_name=program_parameters['routine_name'],
        tool_id=program_parameters['tool_id'],
        base_id=program_parameters['base_id']
    )
    translator.change_substrates = my_change_substrates
    accumulated_subs = list(accumulate(program_parameters['number_of_substrates']))

    if program_parameters['number_of_programs'] != 1:
        translator.program_name = f"{program_parameters['program_name']}_{1}"
    translator.generate_programs(trajectory=kuka_routines[:accumulated_subs[0]], output_dir=program_parameters['output_dir'])

    for i in range(program_parameters['number_of_programs']-1):
        translator.program_name = f"{program_parameters['program_name']}_{i+2}"
        translator.generate_programs(trajectory=kuka_routines[accumulated_subs[i]:accumulated_subs[i+1]], output_dir=program_parameters['output_dir'])
        logging.info(f"\n--- KUKA Program {translator.program_name} Generated ---\n")
    



def defaut_reparation_oval(T= 5, t = 0.2 ,R0= 145,r0= 11,s = 2,theta0 = 45,alpha=57, gamma=90, sod=50,v=100, **kwargs):
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
def defaut_reparation_balayage(T= 5, t = 0.2 ,R0= 145,r0= 12.5,s = 2,theta0 = 37,alpha0=21, gamma=90, sod=50,v=0.1, **kwargs):
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
            alphas.reverse() 
            
            modes = ['LIN','CIRC','CIRC','LIN']
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
def defaut_reparation_balayage_2(d= 2, p = 6, l = 8, t = 0.4 ,R0= 145,s = 2,theta0 = 40,alpha0=22.5, gamma=90, sod=50,v=0.1,ext_circ = False, **kwargs):
    """this function is to generate the points, normals, and tangents for an oval-like paths
    d: Total Thickness of defaut
    p: lenght of the circular part of the cross-section 
    l: length of the linear part of the cross-section
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

    

    logging.info(f"Generating waypoints for defaut_reparation_balayage_2")
    safety = 6
    k = 0
    waypoints = []
    if ext_circ:
        alphas = [alpha0+safety+10,alpha0+safety, 0,-(alpha0+safety),-(alpha0+safety+10)]
        modes = ['CIRC','LIN','CIRC','CIRC','CIRC']
        vitesse = [1,v,v,v,1]
    else:
        alphas = [alpha0+safety, alpha0,-alpha0,-alpha0-safety]
        modes = ['LIN','CIRC','CIRC','LIN']
        vitesse = [1,v,v,1]
    
    th0 = 90 - theta0
    r0 = d + p*math.tan(np.deg2rad(th0))
    for i in range(int(d//t)):
        r = r0-t*i
        dth = np.rad2deg(s/r)

        X1 = []
        if i%2 == 0:
            thetas1 = np.arange(th0, 90, dth)
            dd = 1
            

        else:
            thetas1 = np.arange(180-th0,90, -dth)
            dd = -1


            
        for theta in thetas1:
            T12 = np.array([[0,-1,r0],[1,0 ,dd*l/2.0]])
            X2 = np.array([r*np.cos(np.deg2rad(theta)), r*np.sin(np.deg2rad(theta)),1])
            X1.append(T12@X2)

        ys = np.arange(X1[-1][1]-dd*s,-dd*l/2.0,-dd*s)
        xs = np.array([t*i]*ys.size)

        thetas2 = np.array([90]*ys.size)
        for x,y in zip(xs,ys):
            X1.append(np.array([x,y]))

        val = s - abs(X1[-1][1]+ dd*l/2.0)
        th_start = 90+np.rad2deg(math.asin(dd*val/r))
        if i%2 == 0:
            thetas3 = np.arange(th_start,180-th0, dth)
            if (180-th0) - thetas3[-1] > dth/4:
                thetas3= np.concatenate((thetas3,np.array([thetas3[-1]+dth])))
        else:
            thetas3 = np.arange(th_start,th0, -dth)
            if  thetas3[-1] - th0 > dth/2:
                thetas3= np.concatenate((thetas3,np.array([thetas3[-1]-dth])))

            
        for theta in thetas3:
            T12 = np.array([[0,-1,r0],[1,0 ,-dd*l/2.0]])
            X2 = np.array([r*np.cos(np.deg2rad(theta)), r*np.sin(np.deg2rad(theta)),1])
            X1.append(T12@X2)
            
        thetas = np.concatenate((thetas1,thetas2,thetas3))
        ths = thetas.tolist()

            
        

        for theta, x1 in zip(ths,X1):

            RR = R0+x1[1]
            

            poses = []
            Tangents = [] #in global base
            for alpha in alphas:
                poses.append(np.array([RR*np.cos(np.deg2rad(180+alpha)),RR*np.sin(np.deg2rad(180+alpha)), d-x1[0]]))
                Tangents.append(((-1)**k)*np.array([np.cos(np.deg2rad(90+alpha)),np.sin(np.deg2rad(90+alpha)),0]))
            alphas.reverse()
            
            
            na = [np.array([np.sin(np.deg2rad(90+theta)),0,np.cos(np.deg2rad(90+theta))]),np.array([np.sin(np.deg2rad(90-theta)),0,np.cos(np.deg2rad(90-theta))])]
            for pose, tangent, mode,vel in zip(poses, Tangents,modes,vitesse):
                
                tr = np.identity(4)
                tr[:3,:] = np.column_stack((np.cross(tangent,np.array([0,0,(-1)**k])),tangent,np.array([0,0,(-1)**k]),pose))

                waypoint = {
                    'point': pose,
                    'sod': sod,
                    'angle': (-1)**k*(90-gamma),
                    'velocity': vel,
                    'tangent' : tangent,
                    'normal' : tr[:3,:3]@na[k%2],
                    'mode': mode
                }

                waypoints.append(waypoint)
            k+=1

    last_wp = copy.deepcopy(waypoints[-1])
    print(waypoints[-2])
    pose = last_wp['point']
    tan = last_wp['tangent']*np.array([0,1,0])/np.linalg.norm(last_wp['tangent']*np.array([0,1,0]))
    normal = np.array([0,0,1])
    pose = pose+50*tan
    waypoint = {
        'point': pose,
        'sod': 100,
        'angle': 0,
        'velocity': 0.5,
        'tangent' : tan,
        'normal' : normal,
        'mode': 'LIN'
    }
    waypoints.append(waypoint)

    waypoint = {
        'point': pose+100*np.array([-1,0,0]),
        'sod': 100,
        'angle': 0,
        'velocity': 0.5,
        'tangent' : tan,
        'normal' : normal,
        'mode': 'LIN'
    }
    waypoints.append(waypoint)
    return [waypoints]

    

def my_change_substrates(i = 0, trajectory = None):
    """Example implementation of a substrate change routine."""
    lengths_of_substrate = [360, 240, 320, 350]
    point1 = copy.deepcopy(trajectory[-1])
    point1['Y'] = lengths_of_substrate[i] + 20.0  # Move to a safe Y position beyond the current substrate
    point1['Z'] = -20.0
    point1['A'] = 0.0
    point1['B'] = 0.0
    point1['C'] = 0.0
    point1['VEL'] = 1.0
    point2 = copy.deepcopy(point1)
    point2['X'] = trajectory[0]['X']  # Move back to the starting X position of the next substrate
    logging.info(f"Changing substrate after trajectory {i} with {len(trajectory)} points.")
    return [point1, point2]

# --- Execution & Verification ---
if __name__ == "__main__":


    logging.basicConfig(
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
        level=logging.INFO,
        stream=sys.stdout  # Explicitly routes output to terminal standard output
    )
    tracks_parameters = {
        'parameter_ranges': {
            #'deposition_angle': {'min': 50, 'max': 90, 'steps': 5},
            'deposition_angle': {'min': 50, 'max': 90, 'steps': 5},
            'sod': {'min': 30, 'max': 100,  'steps': 5},
            'velocity': {'min': 50, 'max': 150, 'steps': 3}    
        },
        'combinations_dir': 'lhs_maximin_design.csv',
        'repeat': 1 ,
        'substrate_width': 50.0,
        'intertrack_spacing': 15.0,
        'adaptive_spacing': True,
        'safety_offset': 10.0,
        'lengths_of_substrate': [360, 240, 320, 350]
    }

    lines_parameters = {
        'points_per_line': 2,
        'safety_distance': 50.0
    }

    layer_parameters = {
        'Layer_pos': [[0,0,0]]*8,
        'layer_orientation': [R.from_matrix([[0,1,0],[-1,0,0],[0,0,1]]).as_euler('zyx', degrees=True)]*8

              }

    directory = "/mnt/bureau_folder/Codes/Robot source code/Autogenerated"
    if not os.path.exists(directory):
        directory = "./robot_programs"
        

    program_parameters = {
        'program_name': "repair_5pass",
        'routine_name': "Routine",
        'tool_id': [2,2,2,2,2,2],
        'base_id': [10, 9, 8, 7, 6, 5],
        'number_of_programs': 1,
        'number_of_substrates': [1],
        'output_dir': directory
    }

    
    # parameterized, hardcoded
    full_pipeline(funcs= prepare_dataset_samples,tracks_parameters=tracks_parameters, lines_parameters=lines_parameters, layer_parameters=layer_parameters, program_parameters=program_parameters)    

