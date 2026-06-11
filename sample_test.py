from re import sub

import numpy as np
from scipy.spatial.transform import Rotation as R
import os
from itertools import product

from krl_translator2 import KUKATranslator
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
        'point': np.array([-40, 0, 0, 1.0]),
        'sod': 100,
        'angle': 0,
        'rot_axis': np.array([0, 0, 1]),
        'velocity': 1000
    }
    trajectory.append(waypoint)

    for line in deposition_lines:
        # Interpolate points along the line
        rot_axis = np.array(line['end']) - np.array(line['start'])  # Assuming rot_axis is along the line direction
        x_vals = np.array([line['start'][0] -lines_parameters.get('safety_distance', 20.0)*np.cos(np.arctan2(rot_axis[1], rot_axis[0]))])
        y_vals = np.array([line['start'][1] -lines_parameters.get('safety_distance', 20.0)*np.sin(np.arctan2(rot_axis[1], rot_axis[0]))])
        z_vals = np.array([line['start'][2]])
        x_vals = np.append(x_vals, np.linspace(line['start'][0], line['end'][0], lines_parameters.get('points_per_line', 2)))
        y_vals = np.append(y_vals, np.linspace(line['start'][1], line['end'][1], lines_parameters.get('points_per_line', 2)))
        z_vals = np.append(z_vals, np.linspace(line['start'][2], line['end'][2], lines_parameters.get('points_per_line', 2)))
        x_vals = np.append(x_vals, np.array([line['end'][0] +lines_parameters.get('safety_distance', 20.0)*np.cos(np.arctan2(rot_axis[1], rot_axis[0]))]))
        y_vals = np.append(y_vals, np.array([line['end'][1] +lines_parameters.get('safety_distance', 20.0)*np.sin(np.arctan2(rot_axis[1], rot_axis[0]))]))
        z_vals = np.append(z_vals, np.array([line['end'][2]]))

        for x, y, z in zip(x_vals, y_vals, z_vals):
            waypoint = {
                'point': np.array([x, y, z, 1.0]),
                'sod': line['sod'],
                'angle': 90-line['deposition_angle'],
                'rot_axis': rot_axis / np.linalg.norm(rot_axis) if np.linalg.norm(rot_axis) > 0 else np.array([0, 0, 1]),
                'velocity': line['velocity']
            }
            trajectory.append(waypoint)
    

    waypoint = {
        'point': np.array([-50,trajectory[-1]['point'][1], 100, 1.0]),
        'sod': 100,
        'angle': 0,
        'rot_axis': np.array([0, 0, 1]),
        'velocity': 500
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
    hardcoded_lines = [
        {'start': [0, 10, 0],   'end': [tracks_parameters.get('substrate_width', 50.0), 10, 0],   'sod': 100, 'deposition_angle': 90, 'velocity': 25},
        {'start': [tracks_parameters.get('substrate_width', 50.0), 25, 0],'end': [0, 25, 0],    'sod': 100, 'deposition_angle': 50, 'velocity': 25},
        {'start': [0, 40, 0],  'end': [tracks_parameters.get('substrate_width', 50.0), 40, 0],  'sod': 100, 'deposition_angle': 90, 'velocity': 100},
        {'start': [tracks_parameters.get('substrate_width', 50.0), 55, 0],'end': [0, 55, 0],    'sod': 30, 'deposition_angle': 90, 'velocity': 25},
        {'start': [0, 75, 0],  'end': [tracks_parameters.get('substrate_width', 50.0), 75, 0],  'sod': 30, 'deposition_angle': 90, 'velocity': 100},
        {'start': [tracks_parameters.get('substrate_width', 50.0), 95, 0],'end': [0, 95, 0],    'sod': 30, 'deposition_angle': 50, 'velocity': 100}   
    ]
    
    # Process into lists
    return [hardcoded_lines]


def generate_parameters_table(param_ranges=None):
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
        'deposition_angle': {'values': [90]}
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
    #visualize_parameter_combinations(combinations)
    return combinations

def visualize_parameter_combinations(combinations):
    """
    Utility function to visualize the generated parameter combinations.
    """
    print("Generated Parameter Combinations:")
    for i, combo in enumerate(combinations):
        print(f"Combination {i+1}: {combo}")

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


    combinations = generate_parameters_table(tracks_parameters['parameter_ranges'])

    deposition_lines = []
    i = 0
    for parameters in combinations:
        y = tracks_parameters['safety_offset']
        substrate_lines = []
        while y <= tracks_parameters['lengths_of_substrate'][i] - tracks_parameters['safety_offset']:
            line = {
                'start': [0, y, 0],
                'end': [tracks_parameters['substrate_width'], y, 0],
                'sod': parameters['sod'],
                'deposition_angle': parameters['deposition_angle'],
                'velocity': parameters['velocity']
            }
            substrate_lines.append(line)
            y += tracks_parameters['intertrack_spacing']
        deposition_lines.append(substrate_lines)
        i += 1
        if i >= len(tracks_parameters['lengths_of_substrate']):
            print("Warning: More parameter combinations than substrate lengths.")
            break

    return deposition_lines

def full_pipeline(method="hardcoded", multiple_substrates=False, tracks_parameters=None, lines_parameters=None, layer_parameters=None, program_parameters=None):
    """
    Executes the full dataset preparation pipeline.
    """

    if program_parameters is None:
        program_parameters = {
            'program_name': "test_program",
            'routine_name': "Routine",
            'tool_id': 2,
            'base_id': 10,
            'output_dir': "./robot_programs"
        }
    if layer_parameters is None:
        layer_parameters = {
            'Layer_pos': [[0, 0, 0]],
            'layer_orientation': [R.from_matrix([[-1,0,0],[0,1,0],[0,0,-1]]).as_euler('zyx', degrees=True)]
        }
    print("\n--- Generating Deposition Lines ---\n")
    if method == "hardcoded":
        lines = get_hardcoded_test_batch(tracks_parameters)
    elif method == "parameterized":
        lines = prepare_dataset_samples(tracks_parameters)
    print(f"\nTotal line sets generated: {sum(len(line_set) for line_set in lines)}")
    print(f"Total trajectories generated: {len(lines)}\n")

    print("\n--- Discretizing Lines for Kinematics ---")
    trajectory = []
    for i, line_set in enumerate(lines):
        trajectory.append(discretize_lines_for_kinematics(line_set, lines_parameters))


    print(f"Total points generated: {sum(len(sublist) for sublist in trajectory)}\n")
    for i in range(min(5, len(trajectory))):
        for j in range(min(10, len(trajectory[i]))):
            print(f"Trajectory {i}, Point {j}: Pos={trajectory[i][j]['point'][:3]}, SoD={trajectory[i][j]['sod']}, Angle={trajectory[i][j]['angle']},vel={trajectory[i][j]['velocity']}")
    
    
    # 3. Pass into your kinematic function

    
    path = []
    if multiple_substrates:
        for i,traj in enumerate(trajectory):
            subpath = generate_layer_toolpath(
                Layer_pos=layer_parameters.get('Layer_pos', [[0, 0, 0]])[i],
                layer_orientation=layer_parameters.get('layer_orientation', [R.from_matrix([[-1,0,0],[0,1,0],[0,0,-1]]).as_euler('zyx', degrees=True)])[i],
                trajectory=traj
            )
            path.append(subpath)
        
    else:
        for i,traj in enumerate(trajectory):
            subpath = generate_layer_toolpath(
                Layer_pos=layer_parameters.get('Layer_pos', [[0, 0, 0]])[0],
                layer_orientation=layer_parameters.get('layer_orientation', [R.from_matrix([[-1,0,0],[0,1,0],[0,0,-1]]).as_euler('zyx', degrees=True)])[0],
                trajectory=traj
            )
            path.append(subpath)

    print("\n--- Kinematic Toolpath Generated ---")
    for i, p in enumerate(path[0]):
        print(f"P{i+1}: LIN {{X {p['X']:>6}, Y {p['Y']:>6}, Z {p['Z']:>6}, A {p['A']:>6}, B {p['B']:>6}, C {p['C']:>6}}}")

    translator = KUKATranslator(
    program_name=program_parameters.get('program_name', "test_program"),
    routine_name=program_parameters.get('routine_name', "Routine"),
    tool_id=program_parameters.get('tool_id', 2),
    base_id=program_parameters.get('base_id', 10)
)
    print("\n--- Generating KRL Programs ---")
    print(f"Total programs to generate: {len(path)}")
    for i, p in enumerate(path):
        print(f"\n--- Generating program {i}  with {len(p)} points ---")
        translator.program_name = f"{program_parameters.get('program_name', 'test_program')}_{i}"
        translator.generate_programs(trajectory=p, output_dir=program_parameters.get('output_dir', "./robot_programs"))




# --- Execution & Verification ---
if __name__ == "__main__":

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

    lines_parameters = {
        'points_per_line': 2,
        'safety_distance': 20.0
    }

    layer_parameters = {
        'Layer_pos': [[0, 0, 0]],
        'layer_orientation': [R.from_matrix([[-1,0,0],[0,1,0],[0,0,-1]]).as_euler('zyx', degrees=True)]
    }

    directory = "/mnt/bureau_folder/Codes/Robot source code/Autogenerated"
    if not os.path.exists(directory):
        directory = "./robot_programs"

    program_parameters = {
        'program_name': "Hardcoded_test",
        'routine_name': "Routine",
        'tool_id': 2,
        'base_id': 10,
        'output_dir': directory
    }

    full_pipeline(method="hardcoded", multiple_substrates=False, tracks_parameters=tracks_parameters, lines_parameters=lines_parameters, layer_parameters=layer_parameters, program_parameters=program_parameters)    