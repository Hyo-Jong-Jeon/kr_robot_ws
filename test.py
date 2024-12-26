from scipy.spatial.transform import Rotation as R
import numpy as np
from math import pi

def tool_moveL(base_pos, tool_vector):
    # Base pos ZYZ -> XYZ
    base_vector = np.array(base_pos[3:])
    zyz_matrix = R.from_euler('zyz', base_vector).as_matrix()
    xyz_vector = R.from_matrix(zyz_matrix).as_euler('xyz')
    xyz_matrix = R.from_euler('xyz', xyz_vector).as_matrix()
    
    # Using matrix transfomation tool_vector
    normalized_tool_vector = np.array(tool_vector) / np.linalg.norm(tool_vector)
    tool_direction = xyz_matrix @ normalized_tool_vector
    
    return tool_direction

def calculate_tool_direction(base_pos, tool_vector_a):
    base_vector = np.array(base_pos[3:])
    tool_vector =  np.array(tool_vector_a)
    
    xyz_matrix = R.from_euler('xyz', base_vector).as_matrix()
    tool_direction = xyz_matrix @ tool_vector
    
    return tool_direction

def abc():
    a = np.array([0, 0, 1, 0, 0, 0])
    b = np.array([1, 0, 1])
    return a + b


# print(tool_moveL([0.0, 0.0, 0.0, 3.442, 0.0, 0.0], [0, 0, 1]))
# print(calculate_tool_direction([0.0, 0.0, 0.0, -2.99, 0.0, 0.0], [0, 0, 10]))
a = [1,2,3,4,5,6]
print(a[:3])