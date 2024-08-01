import numpy as np

def calculate_orientation(theta1, theta2, theta3):
    # Convert angles to radians
    theta1 = np.radians(theta1)
    theta2 = np.radians(theta2)
    theta3 = np.radians(theta3)
    
    # Define the rotation matrices for each camera position
    R1 = np.array([[np.cos(np.radians(30)), -np.sin(np.radians(30)), 0],
                   [np.sin(np.radians(30)),  np.cos(np.radians(30)), 0],
                   [0, 0, 1]])

    R2 = np.array([[np.cos(np.radians(150)), -np.sin(np.radians(150)), 0],
                   [np.sin(np.radians(150)),  np.cos(np.radians(150)), 0],
                   [0, 0, 1]])

    R3 = np.array([[np.cos(np.radians(270)), -np.sin(np.radians(270)), 0],
                   [np.sin(np.radians(270)),  np.cos(np.radians(270)), 0],
                   [0, 0, 1]])

    # Vectors in local coordinates
    v1_local = np.array([np.sin(theta1), 0, np.cos(theta1)])
    v2_local = np.array([np.sin(theta2), 0, np.cos(theta2)])
    v3_local = np.array([np.sin(theta3), 0, np.cos(theta3)])

    # Global vectors
    v1_global = R1 @ v1_local
    v2_global = R2 @ v2_local
    v3_global = R3 @ v3_local

    # Average vector
    v_avg = (v1_global + v2_global + v3_global) / 3
    v_avg = v_avg / np.linalg.norm(v_avg)  # Normalize

    # Define the rotation matrix
    # For simplicity, we align the wire with the z-axis in its local frame
    z_axis = np.array([0, 0, 1])
    v_avg_norm = v_avg / np.linalg.norm(v_avg)
    cos_theta = np.dot(z_axis, v_avg_norm)
    rotation_axis = np.cross(z_axis, v_avg_norm)
    sin_theta = np.linalg.norm(rotation_axis)
    
    if sin_theta != 0:
        rotation_axis = rotation_axis / sin_theta
    else:
        rotation_axis = np.array([1, 0, 0])
        
    K = np.array([[0, -rotation_axis[2], rotation_axis[1]],
                  [rotation_axis[2], 0, -rotation_axis[0]],
                  [-rotation_axis[1], rotation_axis[0], 0]])

    R = np.eye(3) + sin_theta * K + (1 - cos_theta) * (K @ K)

    # Euler angles (yaw, pitch, roll)
    yaw = np.arctan2(R[1, 0], R[0, 0])
    pitch = np.arcsin(-R[2, 0])
    roll = np.arctan2(R[2, 1], R[2, 2])

    yaw, pitch, roll = np.degrees([yaw, pitch, roll])

    return R, (roll, pitch, yaw)

# Example usage:
theta1, theta2, theta3 = 10, 20, 30  # Replace with actual measurements
rotation_matrix, euler_angles = calculate_orientation(theta1, theta2, theta3)
print("Rotation Matrix:")
print(rotation_matrix)
print("Euler Angles (Roll, Pitch, Yaw):")
print(euler_angles)
