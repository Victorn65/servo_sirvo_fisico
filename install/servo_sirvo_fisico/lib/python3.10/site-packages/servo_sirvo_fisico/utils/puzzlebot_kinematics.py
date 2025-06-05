import numpy as np

def get_puzzlebot_kinematic_model(r: float, l: float) -> np.ndarray:
    """Get the kinematic model of the Puzzlebot robot to go from wheel speeds to robot velocities.
    (v, w) = KinematicModel @ (wr, wl)
    
    Args:
        r (float): The radius of the wheels.
        l (float): The distance between the wheels.
        
    Returns:
        np.ndarray: The kinematic model matrix of the Puzzlebot robot.
    """
    return np.array([[r/2, r/2],
                   [r/l, -r/l]])

def get_inverse_puzzlebot_kinematic_model(r: float, l: float) -> np.ndarray:
    """Get the inverse kinematic model of the Puzzlebot robot to go from robot velocities to wheel speeds.
    (wr, wl) = InverseKinematicModel @ (v, w)
    
    Args:
        r (float): The radius of the wheels.
        l (float): The distance between the wheels.
    
    Returns:
        np.ndarray: The inverse kinematic model matrix of the Puzzlebot robot.
    """
    return np.linalg.inv(get_puzzlebot_kinematic_model(r, l))

def get_linearized_puzzlebot_model_matrix(v: float, theta: float, delta_t: float) -> np.ndarray:
    """Get the linearized model matrix of the Puzzlebot robot to go from the states to the next states.
    
    Args:
        v (float): The linear velocity of the robot.
        theta (float): The orientation angle of the robot.
        delta_t (float): The time step for the model.
    
    Returns:    
        np.ndarray: The linearized model matrix of the Puzzlebot robot.
    """
    return np.array([[1, 0, -v*np.sin(theta)*delta_t],
                     [0, 1,  v*np.cos(theta)*delta_t],
                     [0, 0,            1           ]])

def get_linearized_puzzlebot_input_model_matrix(r: float, l: float, theta: float, delta_t: float) -> np.ndarray:
    """Get the linearized input model matrix of the Puzzlebot robot to go from the inputs to the change produced by the inputs.
    
    Args:
        r (float): The radius of the wheels.
        l (float): The distance between the wheels.
        theta (float): The orientation angle of the robot.
        delta_t (float): The time step for the model.
    
    Returns:
        np.ndarray: The linearized input model matrix of the Puzzlebot robot.
    """
    return (1/2*r*delta_t) * np.array([[np.cos(theta), np.cos(theta)],
                                        [np.sin(theta), np.sin(theta)],
                                        [     2/l,          -2/l     ]])

speeds_decomposer = lambda v, w, theta: np.array([v * np.cos(theta), v * np.sin(theta), w])