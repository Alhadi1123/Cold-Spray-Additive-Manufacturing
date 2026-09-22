import numpy as np
from scipy.spatial.transform import Rotation as R

def create_translation_matrix(x, y, z):
    """Creates a 4x4 translation matrix."""
    return np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])

def create_rotation_matrix(axis, angle_deg):
    """Creates a 4x4 rotation matrix around x, y, or z."""
    r = R.from_euler(axis, angle_deg, degrees=True).as_matrix()
    T = np.eye(4)
    T[:3, :3] = r
    return T

def create_rotation_matrix_abc(A_deg, B_deg, C_deg):
    """
    Creates a 4x4 homogeneous rotation matrix from KUKA A, B, C angles.
    A = Z rotation, B = Y rotation, C = X rotation.
    Intrinsic rotations (Z-Y-X sequence).
    """
    # Use intrinsic Z-Y-X Euler angles so the rotation maps correctly to the KUKA frame.
    r = R.from_euler('zyx', [A_deg, B_deg, C_deg], degrees=True).as_matrix()
    T = np.eye(4)
    T[:3, :3] = r
    return T

def create_rotation_matrix_arbitrary_axis(axis_vector, angle_deg):
    """
    Creates a 4x4 homogeneous rotation matrix around an arbitrary axis in space.
    
    :param axis_vector: A list or tuple [x, y, z] representing the axis of rotation.
    :param angle_deg: The rotation angle in degrees.
    :return: A 4x4 numpy array.
    """
    axis = np.array(axis_vector, dtype=float)
    norm = np.linalg.norm(axis)
    if norm == 0:
        raise ValueError("The rotation axis cannot be a zero vector.")
    unit_axis = axis / norm
    angle_rad = np.radians(angle_deg)
    rot_vec = unit_axis * angle_rad
    r_matrix = R.from_rotvec(rot_vec).as_matrix()
    T = np.eye(4)
    T[:3, :3] = r_matrix
    return T


def generate_layer_toolpath(Layer_pos, layer_orientation, trajectory):
    """
    Generate KUKA toolpath poses for a single layer.

    The function builds the transform from the drawing base to the layer base,
    applies point positions in the drawing frame, rotates the tool axis from
    the layer base into the drawing frame, and then applies the tool offset.
    The final pose is converted back to KUKA A/B/C angles in the drawing base.
    """
    T_layer_pos = create_translation_matrix(*Layer_pos[:3])
    T_layer_rot = create_rotation_matrix_abc(*layer_orientation[:3])
    T_layer_base = T_layer_pos @ T_layer_rot

    kuka_poses = []
    for wp in trajectory:
        pos_in_drawing = T_layer_base @ wp['point']
        T_local_pos = create_translation_matrix( *pos_in_drawing[:3])

        rot_axis_drawing = T_layer_rot[:3, :3] @ wp['rot_axis']
        T_angle = create_rotation_matrix_arbitrary_axis(rot_axis_drawing, wp['angle'])

        T_standoff = create_translation_matrix(0, 0, -wp['sod'])
        T_offset = T_angle @ T_standoff

        T_final = T_local_pos @ T_offset
        pos = T_final[:3, 3]

        rot_matrix = T_final[:3, :3]
        abc_euler = R.from_matrix(rot_matrix).as_euler('zyx', degrees=True)
        
        kuka_pose = {
            'X': round(pos[0], 2),
            'Y': round(pos[1], 2),
            'Z': round(pos[2], 2),
            'A': round(abc_euler[0], 2), 
            'B': round(abc_euler[1], 2), 
            'C': round(abc_euler[2], 2),
            'VEL': wp['velocity']  
        }
        kuka_poses.append(kuka_pose)
        
    return kuka_poses



def generate_layer_toolpath_experimental(Layer_poses, layer_orientations, routines):
    """
    Generate KUKA toolpath poses for a single layer.

    The function builds the transform from the drawing base to the layer base,
    applies point positions in the drawing frame, rotates the tool axis from
    the layer base into the drawing frame, and then applies the tool offset.
    The final pose is converted back to KUKA A/B/C angles in the drawing base.
    """


    total_poses = []

    for i,routine in enumerate(routines):
        
        Layer_pos = Layer_poses[i]
        layer_orientation = layer_orientations[i]

        T_layer_pos = create_translation_matrix(*Layer_pos[:3])
        T_layer_rot = create_rotation_matrix_abc(*layer_orientation[:3])
        T_layer_base = T_layer_pos @ T_layer_rot

        poses = []
        for wp in routine:

            T_angle = create_rotation_matrix_arbitrary_axis(wp['tangent'], wp['angle'])
            deposition_vector_local = -1*(T_angle[:3, :3] @ wp['normal'])
            deposition_vector = T_layer_base[:3, :3] @ deposition_vector_local

            tool_y = np.cross(deposition_vector, np.array([1, 0, 0]))
            tool_x = np.cross(tool_y, deposition_vector)
            local_pos = np.array([*wp['point'],1])
            pos = T_layer_base@local_pos
            kuka_pose = pos[:3]-wp['sod']*deposition_vector

            rotation_matrix = np.column_stack((tool_x, tool_y, deposition_vector))


            abc_euler = R.from_matrix(rotation_matrix).as_euler('zyx', degrees=True)
            
            pose = {
                'X': round(kuka_pose[0], 2),
                'Y': round(kuka_pose[1], 2),
                'Z': round(kuka_pose[2], 2),
                'A': round(abc_euler[0], 2), 
                'B': round(abc_euler[1], 2), 
                'C': round(abc_euler[2], 2),
                'VEL': wp['velocity'],
                'mode': wp['mode']
            }
            poses.append(pose)

        total_poses.append(poses)

    return total_poses


if __name__ == "__main__":
    points = [np.array([0, 0, 0, 1]),
              np.array([50, 0, 0, 1]),
              np.array([100, 0, 0, 1]),
              np.array([150, 0, 0, 1]),
              np.array([200, 0, 0, 1])]
    standoffs = [30.0] * len(points)
    rot_axes = [np.array([1, 0, 0])] * len(points)
    angles = [30.0] * len(points)
    velocities = [25.0] * len(points)
    trajectory = [{'point': p, 'sod': s, 'rot_axis': r, 'angle': a, 'velocity': v} for p, s, r, a, v in zip(points, standoffs, rot_axes, angles, velocities)]
    path = generate_layer_toolpath(
        Layer_pos=[100.0, 200.0, 0],
        layer_orientation=[0, 0, 0],
        trajectory=trajectory
    )
    
    
    for i, p in enumerate(path):
        print(f"P{i+1}: LIN {{X {p['X']:>6}, Y {p['Y']:>6}, Z {p['Z']:>6}, A {p['A']:>6}, B {p['B']:>6}, C {p['C']:>6}}}")
