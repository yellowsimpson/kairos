import numpy as np

def euler_to_quaternion(r):
    (roll, pitch, yaw) = (r[0], r[1], r[2])
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

def quaternion_to_r_matrix(quaternion):
    (qx,qy,qz,qw) = (quaternion[0],quaternion[1],quaternion[2],quaternion[3])
    r_matrix = np.array([[1-2*(qy**2 + qz**2),  2*(qx*qy - qw*qz),    2*(qx*qz + qw*qy),    0],
                         [2*(qx*qy + qw*qz),    1-2*(qx**2 + qz**2),  2*(qy*qz - qw*qx),    0],
                         [2*(qx*qz - qw*qy),    2*(qy*qz + qw*qx),    1-2*(qx**2 + qy**2),  0],
                         [0,                    0,                    0,                    1]])
    return r_matrix

def e_to_r_matrix(r):
    quaternion = euler_to_quaternion(r)
    r_matrix = quaternion_to_r_matrix(quaternion)
    return r_matrix

def point_to_rad(vertical, horizontal):
    angle = np.arctan2(vertical, horizontal)
    if angle < 0:
        angle += 2 * np.pi
    return angle

def create_tf_matrix(xyz):
    matrix = np.eye(4)
    matrix[0, 3] = xyz[0]
    matrix[1, 3] = xyz[1]
    matrix[2, 3] = xyz[2]
    
    return matrix