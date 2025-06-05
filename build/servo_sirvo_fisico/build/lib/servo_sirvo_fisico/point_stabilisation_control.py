import servo_sirvo_fisico.utils.math_utils as math_utils
import numpy as np
import rclpy
import transforms3d as t3d

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class PuzzlebotController(Node):
    def __init__(self):
        super().__init__('puzzlebot_controller_node')

        # Declare parameters
        self.declare_parameter('controller_update_rate', 50.0) # Controller update rate in Hz
        self.declare_parameter('distance_tolerance', 0.05) # Distance tolerance to reach the target in meters
        self.declare_parameter('angular_tolerance', 0.034) # Angular tolerance to reach the target in radians
        self.declare_parameter('angular_adjustment', 0.1745) # Angular adjustment first rotate then move in radians
        self.declare_parameter('following_walls_distance', 0.20) # Distance to the wall when following it in meters
        self.declare_parameter('front_stop_distance', 0.18) # Distance to stop in front of an obstacle in meters
        self.declare_parameter('lookahead_distance', 0.22) # Lookahead distance for the BUG2 outercorner controller in meters
        self.declare_parameter('p2p_v_Kp', 0.8) # Proportional gain for the point-to-point linear speed
        self.declare_parameter('p2p_w_Kp', 1.5) # Proportional gain for the point-to-point angular speed
        self.declare_parameter('fw_w_Kp', 3.0) # Proportional gain for the following walls angular speed
        self.declare_parameter('fw_e_Kp', 24.0) # Proportional gain for the following walls distance to the wall error
        self.declare_parameter('fw_linear_speed', 0.1) # Linear speed for the following walls controller in m/s
        self.declare_parameter('fw_outer_corner_angular_speed', 2.4) # Angular speed for the following walls outer corner controller in rad/s
        self.declare_parameter('fw_outer_corner_linear_speed', 0.3) # Linear speed for the following walls outer corner controller in m/s
        self.declare_parameter('v_max', 0.4) # Maximum linear speed in m/s
        self.declare_parameter('w_max', 3.14) # Maximum angular speed in rad/s
        self.declare_parameter('side_open_angle', 0.5236) # Angle opening for the side regions in radians
        self.declare_parameter('front_open_angle', 0.4) # Angle opening for the front region in radians
        self.declare_parameter('target_open_angle', 0.5) # Angle opening for the target region in radians
        self.declare_parameter('controller_type', "BUG2") # Controller type, can be 'BUG0' or 'BUG2'

        # Get the parameters
        self.update_rate = self.get_parameter('controller_update_rate').get_parameter_value().double_value
        self.distance_tolerance = self.get_parameter('distance_tolerance').get_parameter_value().double_value
        self.angular_tolerance = self.get_parameter('angular_tolerance').get_parameter_value().double_value
        self.angular_adjustment = self.get_parameter('angular_adjustment').get_parameter_value().double_value
        self.following_walls_distance = self.get_parameter('following_walls_distance').get_parameter_value().double_value
        self.front_stop_distance = self.get_parameter('front_stop_distance').get_parameter_value().double_value
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.p2p_v_Kp = self.get_parameter('p2p_v_Kp').get_parameter_value().double_value
        self.p2p_w_Kp = self.get_parameter('p2p_w_Kp').get_parameter_value().double_value
        self.fw_w_Kp = self.get_parameter('fw_w_Kp').get_parameter_value().double_value
        self.fw_e_Kp = self.get_parameter('fw_e_Kp').get_parameter_value().double_value
        self.fw_linear_speed = self.get_parameter('fw_linear_speed').get_parameter_value().double_value
        self.fw_outer_corner_angular_speed = self.get_parameter('fw_outer_corner_angular_speed').get_parameter_value().double_value
        self.fw_outer_corner_linear_speed = self.get_parameter('fw_outer_corner_linear_speed').get_parameter_value().double_value
        self.v_max = self.get_parameter('v_max').get_parameter_value().double_value
        self.w_max = self.get_parameter('w_max').get_parameter_value().double_value
        self.side_open_angle = self.get_parameter('side_open_angle').get_parameter_value().double_value
        self.front_open_angle = self.get_parameter('front_open_angle').get_parameter_value().double_value
        self.target_open_angle = self.get_parameter('target_open_angle').get_parameter_value().double_value
        self.controller_type = self.get_parameter('controller_type').get_parameter_value().string_value

        # Subscribers
        self.create_subscription(Pose2D, 'setpoint', self.setpoint_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'ground_truth', self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_reached_publisher = self.create_publisher(Bool, 'goal_reached', qos_profile_sensor_data)

        # Node variables
        self.controller_timer = None # Timer for the controller callback
        self.robot_pose = Pose2D() # Robot pose [x, y, theta] in meters and radians
        self.robot_setpoint = Pose2D() # Robot setpoint [x, y, theta] in meters and radians
        self.closest_object_angle = 0.0 # Angle to the closest object in radians
        self.controller_mode = 'p2p_controller' # Controller mode, can be 'p2p_controller' or 'following_walls'
        self.following_walls_directions = {'left': 1, 'right': -1} # Directions for following walls, left is positive, right is negative
        self.following_walls_direction = 'right' # Initial direction for following walls
        self.min_side_region_distance = 0.0 # Minimum distance in the side region
        self.min_front_region_distance = 0.0 # Minimum distance in the front region
        self.min_back_side_region_distance = 0.0 # Minimum distance in the back side region
        self.min_back_side_outside_region_distance = 0.0 # Minimum distance in the negated back side outside region
        self.target_line = None # Imaginary line function for the BUG2 controller
        self.last_error_y = None # Last error in the y direction for the BUG2 controller
        self.lidar_min_range = 0.15 # Minimum range of the lidar in meters
        self.collision_time = self.get_clock().now() # Time when the robot entered a collision state

        # Log the node start
        self.get_logger().info('Puzzlebot Controller Node has been started.')

    def setpoint_callback(self, msg):
        # If the controller timer is running, cancel it
        if self.controller_timer is not None:
            self.controller_timer.cancel()

        # Start the controller timer with the new update rate   
        self.controller_timer = self.create_timer(1.0/self.update_rate, self.controller_callback)
        # Update the robot pose with the new setpoint
        self.robot_setpoint = msg

    def odom_callback(self, msg):
        # Extract [x, y] position from the odometry message
        self.robot_pose.x = msg.pose.pose.position.x
        self.robot_pose.y = msg.pose.pose.position.y
        # Extract quaternion orientation from the odometry message
        q = msg.pose.pose.orientation
        # Convert quaternion to Euler angles and update the robot pose
        _, _, self.robot_pose.theta = t3d.euler.quat2euler([q.w, q.x, q.y, q.z])

    def lidar_callback(self, msg):
        # Get the lidar readings
        lidar_readings = np.array(msg.ranges)

        # Get the closest object angle
        self.closest_object_angle = math_utils.get_normalized_angle(msg.angle_min+np.argmin(lidar_readings)*msg.angle_increment)

        # Get the lidar minimum range
        self.lidar_min_range = msg.range_min

        # Calculate the angle error to face the target
        theta_to_face_target_error = math_utils.get_angle_between_poses(self.robot_pose, self.robot_setpoint)
        # Get the minimum distance from the target region
        min_target_region_distance = self._get_min_distance_from_region(lidar_readings, theta_to_face_target_error, msg.angle_min, self.target_open_angle, self.target_open_angle, msg.angle_increment, msg.range_min)

        # Get the side region center angle based on the following walls direction
        side_center_angle = self._get_side_center_angle(self.following_walls_direction, msg.angle_min)
        # Get the minimum distance from the side region
        self.min_side_region_distance = self._get_min_distance_from_region(lidar_readings, side_center_angle, msg.angle_min, self.side_open_angle, self.side_open_angle, msg.angle_increment, msg.range_min)

        # Get the front region data
        front_center_angle = -msg.angle_min
        # Get the minimum distance from the front region
        self.min_front_region_distance = self._get_min_distance_from_region(lidar_readings, front_center_angle, msg.angle_min, self.front_open_angle, self.front_open_angle, msg.angle_increment, msg.range_min)

        # Get the minimum distance from the back side region
        self.min_back_side_region_distance = self._get_min_distance_from_region(lidar_readings, side_center_angle, msg.angle_min, self.side_open_angle*0.5, self.side_open_angle, msg.angle_increment, msg.range_min)

        # Get the minimum distance from the negated back side outside region
        self.min_back_side_outside_region_distance = self._get_min_distance_outside_region(lidar_readings, side_center_angle, msg.angle_min, self.side_open_angle*0.5, self.side_open_angle, msg.angle_increment, msg.range_min)

        # Controller mode state machine
        # Check the current state
        if self.controller_mode == 'p2p_controller':
            # Check if the robot is close enough to a wall from the front
            if self.min_front_region_distance < self.following_walls_distance*1.5:
                # Choice a random direction to follow the walls
                self.following_walls_direction = np.random.choice(list(self.following_walls_directions.keys()))

                # Calculate the target line params for the BUG2 controller
                m = (self.robot_setpoint.y-self.robot_pose.y)/(self.robot_setpoint.x-self.robot_pose.x)
                b = self.robot_pose.y - m * self.robot_pose.x
                # Define the target line function
                self.target_line = lambda x : m*x + b

                # Reset the last error in the y direction
                self.last_error_y = self.target_line(self.robot_pose.x) - self.robot_pose.y

                # Set the controller mode to following walls
                self.controller_mode = 'following_walls'
        elif self.controller_mode == 'following_walls':
            # Check the controller type
            if self.controller_type == 'BUG0':
                # Check if the robot has far enough free space to go directly to the target
                if (min_target_region_distance > self._get_euclidian_distance_between_poses(self.robot_pose, self.robot_setpoint)):
                    # Set the controller mode to point-to-point controller
                    self.controller_mode = 'p2p_controller'
            elif self.controller_type == 'BUG2':
                # Get the error in the y direction to the target line
                error_y = self.target_line(self.robot_pose.x) - self.robot_pose.y
                # Check if there is a sign change in the error in the y direction (which means the robot has crossed the target line)
                if ((error_y > 0 and self.last_error_y < 0) or (error_y < 0 and self.last_error_y > 0)):
                    # Set the controller mode to point-to-point controller
                    self.controller_mode = 'p2p_controller'
            else:
                raise ValueError("Controller type must be 'BUG0' or 'BUG2'.")

    def controller_callback(self):
        # Calculate the distance to the setpoint
        distance_to_target = self._get_euclidian_distance_between_poses(self.robot_pose, self.robot_setpoint)
        
        # Create a Twist message to publish the robot velocity
        twist_msg = Twist()
        
        # Check the controller mode (p2p_controller or following_walls)
        if self.controller_mode == 'following_walls':
            # If entered collision state, perform a collision recovery algorithm
            if self.get_clock().now() - self.collision_time < rclpy.duration.Duration(seconds=0.75):
                # If entered collision state, turn the robot away from the wall
                linear_speed = -0.1
                angular_speed = -self.following_walls_directions[self.following_walls_direction] * self.w_max
            else:
                # Get the angle to separate the robot from the wall faster
                angle_to_separate = math_utils.get_normalized_angle(self.closest_object_angle + np.pi)

                # Calculate the tangent angle to the wall
                tangent_angle = math_utils.get_normalized_angle(angle_to_separate + \
                                                                self.following_walls_directions[self.following_walls_direction] * np.pi/2)

                # Calculate the linear speed (based on how close the robot is to the wall from the front)
                linear_speed = self.fw_linear_speed
                if self.min_front_region_distance < self.front_stop_distance:
                    linear_speed = 0.05
                elif self.min_front_region_distance < 2*self.front_stop_distance:
                    linear_speed /= 2

                # Calculate the error to the distance to the wall
                distance_to_wall_error = self.min_side_region_distance - self.following_walls_distance

                # Calculate the angular speed (using an obstacle avoidance algorithm with wall distance control)
                angular_speed = self.fw_w_Kp * tangent_angle + \
                                self.following_walls_directions[self.following_walls_direction] * self.fw_e_Kp * distance_to_wall_error

                # Perform a outer corner control algorithm
                if self.min_back_side_region_distance < self.lookahead_distance and self.min_back_side_outside_region_distance > self.lookahead_distance:
                    linear_speed = self.fw_outer_corner_linear_speed
                    angular_speed = self.following_walls_directions[self.following_walls_direction] * self.fw_outer_corner_angular_speed

                # Store the time when the robot entered a collision state to perform the collision recovery algorithm
                if self.min_front_region_distance < self.lidar_min_range+0.01:
                    self.collision_time = self.get_clock().now()

            # Constrain speeds
            linear_speed = np.clip(linear_speed, -self.v_max, self.v_max)
            angular_speed = np.clip(angular_speed, -self.w_max, self.w_max)

            # Update the Twist message with the calculated velocities
            twist_msg.linear.x = linear_speed
            twist_msg.angular.z = angular_speed
        elif self.controller_mode == 'p2p_controller':
            # Calculate the angle error to face the target
            theta_to_face_target_error = math_utils.get_angle_between_poses(self.robot_pose, self.robot_setpoint)

            # Perform orientation control on the move
            linear_speed = self.p2p_v_Kp * distance_to_target
            angular_speed = self.p2p_w_Kp * theta_to_face_target_error

            # Perform a first rotate then move control
            if abs(theta_to_face_target_error) > self.angular_adjustment:
                linear_speed = 0

            # If the robot is on the distance tolerance to the target, stop the robot and only rotate
            if distance_to_target < self.distance_tolerance:
                linear_speed = 0
                angular_speed = self.p2p_w_Kp * (self.robot_setpoint.theta - self.robot_pose.theta)

            # Constrain speeds
            linear_speed = np.clip(linear_speed, -self.v_max, self.v_max)
            angular_speed = np.clip(angular_speed, -self.w_max, self.w_max)
            
            # Update the Twist message with the calculated velocities
            twist_msg.linear.x = linear_speed
            twist_msg.angular.z = angular_speed

        # Check if the robot has reached the target
        if distance_to_target < self.distance_tolerance and abs(self.robot_setpoint.theta-self.robot_pose.theta) < self.angular_tolerance:
            twist_msg = Twist() # Set the Twist message to zero to stop the robot
            self.goal_reached_publisher.publish(Bool(data=True))
            self.controller_timer.cancel()
            self.get_logger().info('Target reached, stopping the robot.')

        # Publish the Twist message
        self.cmd_vel_publisher.publish(twist_msg)

    def _get_euclidian_distance_between_poses(self, pose1, pose2):
        # Calculate the Euclidean distance between two poses with sign
        return np.sqrt((pose1.x - pose2.x) ** 2 + (pose1.y - pose2.y) ** 2)
        
    def _get_indices_from_angle(self, start_angle: float, end_angle: float, angle_increment: float) -> tuple:
        """Get the start and end indices for the lidar readings based on the angles.
        
        Args:
            start_angle (float): The start angle in radians.
            end_angle (float): The end angle in radians.
            angle_increment (float): The lidar angle increment in radians.
            
        Returns:
            tuple: A tuple containing the start and end indices for the lidar readings.
        """
        start_angle = math_utils.normalize_and_symmetric_theta_shift(start_angle)
        end_angle = math_utils.normalize_and_symmetric_theta_shift(end_angle)
        # Get the start and end indices for the left region
        start_index = int(start_angle / angle_increment)
        end_index = int(end_angle / angle_increment)

        return start_index, end_index
    
    def _get_min_distance_from_indices(self, lidar_readings: np.array, start_index: int, end_index: int, range_min: float) -> float:
        """Get the minimum distance from the lidar readings from start to end indices.
        
        Args:
            lidar_readings (np.array): The lidar readings as a numpy array.
            start_index (int): The start index for the readings.
            end_index (int): The end index for the readings.
            range_min (float): The minimum range of the lidar.
            
        Returns:
            float: The minimum distance from the valid readings, clamped to range_min.
        """
        # Check if overflow occurs and create a valid readings list
        if start_index > end_index:
            right_readings = lidar_readings[start_index:]
            left_readings = lidar_readings[:end_index]
            valid_readings = np.concatenate((right_readings, left_readings))
        else:
            valid_readings = lidar_readings[start_index:end_index]

        # Return the minimum distance from the valid readings with range minimum as lower bound
        return max(np.min(valid_readings), range_min)
    
    def _get_min_distance_outside_indices(self, lidar_readings: np.array, start_index: int, end_index: int, range_min: float) -> float:
        """Get the minimum distance from the lidar readings outside the specified indices.
        
        Args:
            lidar_readings (np.array): The lidar readings as a numpy array.
            start_index (int): The start index for the readings.
            end_index (int): The end index for the readings.
            range_min (float): The minimum range of the lidar.
            
        Returns:
            float: The minimum distance from the valid readings outside the specified indices, clamped to range_min.
        """
        # Check if overflow occurs and create a valid readings list
        if start_index > end_index:
            valid_readings = lidar_readings[end_index:start_index]
        else:
            right_readings = lidar_readings[end_index:]
            left_readings = lidar_readings[:start_index]
            valid_readings = np.concatenate((right_readings, left_readings))

        # Return the minimum distance from the valid readings with range minimum as lower bound
        return max(np.min(valid_readings), range_min)
    
    def _get_min_distance_from_region(self, lidar_readings: np.array, center_angle: float, offset: float, front_open_angle: float, back_open_angle: float, angle_increment: float, range_min: float) -> float:
        """Get the minimum distance from the lidar readings in a specific region defined by angles.
        
        Args:
            lidar_readings (np.array): The lidar readings as a numpy array.
            center_angle (float): The center angle of the target region in radians.
            offset (float): The offset angle from the robot front (x axis) in radians.
            front_open_angle (float): The region to open the region by the front in radians.
            back_open_angle (float): The region to open the region by the back in radians.
            angle_increment (float): The lidar angle increment in radians.
            range_min (float): The minimum range of the lidar.
            
        Returns:
            float: The minimum distance from the target region, clamped to range_min.
        """
        if center_angle < np.pi:
            start_angle = math_utils.normalize_and_symmetric_theta_shift(center_angle - offset - front_open_angle)
            end_angle = math_utils.normalize_and_symmetric_theta_shift(center_angle - offset + back_open_angle)
        else:
            start_angle = math_utils.normalize_and_symmetric_theta_shift(center_angle - offset - back_open_angle)
            end_angle = math_utils.normalize_and_symmetric_theta_shift(center_angle - offset + front_open_angle)
        # Get the start and end indices for the region
        start_index, end_index = self._get_indices_from_angle(start_angle, end_angle, angle_increment)

        return self._get_min_distance_from_indices(lidar_readings, start_index, end_index, range_min)

    def _get_min_distance_outside_region(self, lidar_readings: np.array, center_angle: float, offset: float, front_open_angle: float, back_open_angle: float, angle_increment: float, range_min: float) -> float:
        """Get the minimum distance from the lidar readings outside a specific region defined by angles.
        
        Args:
            lidar_readings (np.array): The lidar readings as a numpy array.
            center_angle (float): The center angle of the target region in radians.
            offset (float): The offset angle from the robot front (x axis) in radians.
            front_open_angle (float): The region to open the region by the front in radians.
            back_open_angle (float): The region to open the region by the back in radians.
            angle_increment (float): The lidar angle increment in radians.
            range_min (float): The minimum range of the lidar.
            
        Returns:
            float: The minimum distance from the outside of the target region, clamped to range_min.
        """
        # Get the start and end angles for the target region
        start_angle = math_utils.normalize_and_symmetric_theta_shift(center_angle - offset - front_open_angle)
        end_angle = math_utils.normalize_and_symmetric_theta_shift(center_angle - offset + back_open_angle)
        # Get the start and end indices for the region
        start_index, end_index = self._get_indices_from_angle(start_angle, end_angle, angle_increment)

        return self._get_min_distance_outside_indices(lidar_readings, start_index, end_index, range_min)

    def _get_side_center_angle(self, direction: str, angle_min: float) -> float:
        """Get the center angle for the side region based on the direction.
        
        Args:
            direction (str): The direction of the side region, can be 'left' or 'right'.
            angle_min (float): The minimum angle of the lidar readings in radians.
            
        Returns:
            float: The center angle for the side region in radians.
        """
        if direction == 'left':
            return np.pi/2 - angle_min
        elif direction == 'right':
            return 3*np.pi/2 - angle_min
        else:
            raise ValueError("Direction must be 'left' or 'right'.")

def main():
    rclpy.init()
    node = PuzzlebotController()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().info('Node interrupted. Shutting down...')
        node.get_logger().error(f'Error: {e}')
        if rclpy.ok():
            rclpy.shutdown()
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()