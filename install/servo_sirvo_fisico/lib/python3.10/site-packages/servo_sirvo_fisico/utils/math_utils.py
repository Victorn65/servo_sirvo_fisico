import numpy as np

from geometry_msgs.msg import Pose2D

def get_normalized_angle(angle: float) -> float:
    """Normalize an angle to be between -pi and pi.
    
    Args:
        angle (float): The angle to normalize.
        
    Returns:
        float: The normalized angle.
    """
    return np.arctan2(np.sin(angle), np.cos(angle))

def symmetric_theta_shift(theta: float) -> float:
    """Shift an angle from (-pi, pi) to (0, 2*pi).
    
    Args:
        theta (float): The angle to shift.
        
    Returns:
        float: The shifted angle in the range [0, 2*pi).
    """
    if theta >= 0:
        return theta
    else:
        return 2 * np.pi + theta
    
def normalize_and_symmetric_theta_shift(theta: float) -> float:
    """Normalize an angle to (-pi, pi) and then maps it (0, 2*pi).
    
    Args:
        theta (float): The angle to normalize and shift.
        
    Returns:
        float: The normalized and shifted angle in the range [0, 2*pi).
    """
    return symmetric_theta_shift(get_normalized_angle(theta))

def get_angle_between_deltas(delta_x: float, delta_y: float) -> float:
    """Calculate the angle between two points given their delta coordinates.
    
    Args:
        delta_x (float): The difference in x coordinates.
        delta_y (float): The difference in y coordinates.
        
    Returns:
        float: The angle between the two points in radians.
    """
    return np.arctan2(delta_y, delta_x)

def get_angle_between_poses(pose1: Pose2D, pose2: Pose2D) -> float:
    """Calculate the angle between two poses. (pose2 with respect to pose1)
    
    Args:
        pose1 (Pose2D): The first pose.
        pose2 (Pose2D): The second pose.
        
    Returns:
        float: The angle between the two poses in radians.
    """ 
    angle_between_poses = get_angle_between_deltas(pose2.x-pose1.x, pose2.y-pose1.y)

    return get_normalized_angle(angle_between_poses - pose1.theta)