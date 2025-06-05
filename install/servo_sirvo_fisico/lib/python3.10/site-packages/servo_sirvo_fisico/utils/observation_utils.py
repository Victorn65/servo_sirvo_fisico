import numpy as np

from . import math_utils

def observate_from_deltas(delta_x: float, delta_y: float, theta_offset: float = 0.) -> np.ndarray:
    """Get the distance and angle between two points using its delta coordinates. It 
    also applies an offset to the angle if required.
    
    Args:
        delta_x (float): The difference in x coordinates.
        delta_y (float): The difference in y coordinates.
        theta_offset (float, optional): An offset to apply to the angle. Defaults to 0.
        
    Returns:
        np.ndarray: An array containing the distance and the angle (distance, angle).
    """
    angle = math_utils.get_normalized_angle(math_utils.get_angle_between_deltas(delta_x, delta_y) - theta_offset)
    
    return np.array([np.linalg.norm((delta_x, delta_y)), angle])

def observate_from_deltas_linearized(delta_x: float, delta_y: float) -> np.ndarray:
    """Get the linearized observation matrix from the delta coordinates. It evaluates the Jacobian of the 
    observation function with respect to the robot state.
    
    Args:
        delta_x (float): The difference in x coordinates.
        delta_y (float): The difference in y coordinates.
        
    Returns:
        np.ndarray: The evaluated Jacobian matrix of the observation function.
    """
    p = delta_x**2 + delta_y**2
    return np.array([[-delta_x/np.sqrt(p), -delta_y/np.sqrt(p), 0],
                     [     delta_y/p,          -delta_x/p,     -1]])