# rotation_matrix from_spherical_system

import numpy as np

# Define the angles in degrees
theta1 = np.deg2rad(15)  # Deviation angle from z-axis for Camera 1
theta2 = np.deg2rad(10)  # Deviation angle from z-axis for Camera 2
theta3 = np.deg2rad(5)   # Deviation angle from z-axis for Camera 3

# Define the azimuthal angles of each camera
phi1 = np.deg2rad(30)
phi2 = np.deg2rad(150)
phi3 = np.deg2rad(-90)  # 270 degrees is equivalent to -90 degrees

# Convert spherical to Cartesian coordinates for each camera's view
d1 = np.array([
    np.sin(theta1) * np.cos(phi1),
    np.sin(theta1) * np.sin(phi1),
    np.cos(theta1)
])

d2 = np.array([
    np.sin(theta2) * np.cos(phi2),
    np.sin(theta2) * np.sin(phi2),
    np.cos(theta2)
])

d3 = np.array([
    np.sin(theta3) * np.cos(phi3),
    np.sin(theta3) * np.sin(phi3),
    np.cos(theta3)
])

# Compute the average direction vector
d_avg = (d1 + d2 + d3) / 3
d_avg = d_avg / np.linalg.norm(d_avg)  # Normalize the vector

# Assume initial direction (z-axis) as [0, 0, 1]
z_axis = np.array([0, 0, 1])

# Compute the rotation vector (cross product)
cross_prod = np.cross(z_axis, d_avg)
cross_prod_norm = np.linalg.norm(cross_prod)

# Compute the angle of rotation
angle = np.arccos(np.dot(z_axis, d_avg))

# Construct the rotation matrix using Rodrigues' rotation formula
if cross_prod_norm != 0:
    k = cross_prod / cross_prod_norm  # Normalized axis of rotation
    K = np.array([
        [0, -k[2], k[1]],
        [k[2], 0, -k[0]],
        [-k[1], k[0], 0]
    ])
    I = np.eye(3)
    rot_matrix = I + np.sin(angle) * K + (1 - np.cos(angle)) * np.dot(K, K)
else:
    rot_matrix = np.eye(3)  # No rotation needed

# Extract Euler angles (roll, pitch, yaw) from the rotation matrix
def rotation_matrix_to_euler_angles(R):
    """
    Convert a rotation matrix to Euler angles (ZYX convention).
    Returns angles in degrees.
    """
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0

    return np.rad2deg(np.array([z, y, x]))

# Compute Euler angles
yaw, pitch, roll = rotation_matrix_to_euler_angles(rot_matrix)

# Output the results
print(f"Direction Vector (Normalized): {d_avg}")
print(f"Rotation Matrix:\n{rot_matrix}")
print(f"Euler Angles (Yaw, Pitch, Roll): {yaw:.2f}, {pitch:.2f}, {roll:.2f}")
