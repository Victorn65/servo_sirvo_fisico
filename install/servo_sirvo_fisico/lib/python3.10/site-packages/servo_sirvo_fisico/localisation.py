import rclpy 
from rclpy.node import Node 
from nav_msgs.msg import Odometry 
from std_msgs.msg import Float32 
from rclpy import qos 
import numpy as np 
import transforms3d 
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from aruco_msgs.msg import MarkerArray
from puzzlebot_aruco_msgs.msg import ArucoObservation
import math
import time

class Localisation(Node): 
    def __init__(self): 
        super().__init__('localisation') 
        
        # Suscriptores
        self.wr_sub = self.create_subscription(Float32, 'VelocityEncR', self.wr_callback, qos.qos_profile_sensor_data) 
        self.wl_sub = self.create_subscription(Float32, 'VelocityEncL', self.wl_callback, qos.qos_profile_sensor_data) 
        self.meas_sub = self.create_subscription(ArucoObservation, 'aruco_observation', self.aruco_callback, qos.qos_profile_sensor_data)

        self.tf_broadcaster = TransformBroadcaster(self)

        # Publicador
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)      

        # Parámetros físicos
        self.r = 0.05  # radio ruedas (m)
        self.L = 0.18  # separación entre ruedas (m)

        # Estado inicial
        self.w = 0.0  # velocidad angular
        self.v = 0.0  # velocidad lineal
        self.x = 0.0  # posición x
        self.y = 0.0  # posición y
        self.theta = 0.0  # orientación (yaw)
        self.wr = 0.0  # velocidad rueda derecha
        self.wl = 0.0  # velocidad rueda izquierda

        self.prev_time_ns = self.get_clock().now().nanoseconds  

        # Inicializar matriz de covarianza
        self.Sigma = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.1],
            [0.0, 0.1, 1.0]
        ])
        self.Q = np.array([
            [0.00000567, -0.0000065, -0.0000163],
            [-0.0000065, 0.00002631, 0.00003897],
            [-0.0000163, 0.00003897, 0.00011256]
        ])
        
        # Matriz de covarianza de la medición (ArUco)
        self.RCAM = np.diag([0.01, 0.05])  # [variance in range, variance in bearing (rad)]
        #[191.26581, 0.0, 169.60164, 0.0, 255.02285, 109.55441, 0.0, 0.0, 1.0]
        # Mapa de ArUcos (ID: (x, y))
        self.aruco_map = {
            0: (0.9, 0.6),   # ejemplo: Aruco ID 0 está en (2.0 m, 1.0 m)
            1: (1.2, -1.48),
            2: (0.3, 0.28), 
            3: (1.5, 0.0),
            4: (-1.0, -1.48)   # agrega los que tengas
        }

        # Para throttling de ArUco
        self.last_aruco_time = time.time()
        self.min_aruco_interval = 1.0  # seconds

        # Timer
        timer_period = 0.02 
        self.timer = self.create_timer(timer_period, self.timer_callback) 

    def wr_callback(self, msg): 
        self.wr = msg.data 

    def wl_callback(self, msg): 
        self.wl = msg.data 

    def timer_callback(self): 
        v, w = self.get_robot_vel(self.wr, self.wl) 
        self.update_pose(v, w)  
        odom_msg = self.fill_odom_message(self.x, self.y, self.theta) 
        self.odom_pub.publish(odom_msg) 

        # Also publish the transform from odom to base_footprint
        self.publish_tf()

    def publish_tf(self):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        quat = transforms3d.euler.euler2quat(0, 0, self.theta)
        tf_msg.transform.rotation.x = quat[1]
        tf_msg.transform.rotation.y = quat[2]
        tf_msg.transform.rotation.z = quat[3]
        tf_msg.transform.rotation.w = quat[0]
        self.tf_broadcaster.sendTransform(tf_msg)

    def aruco_callback(self, msg):
        current_time = time.time()
        # Throttle ArUco processing to avoid flooding
        if current_time - self.last_aruco_time < self.min_aruco_interval:
            return
        self.last_aruco_time = current_time

        aruco_id = msg.id
        distance = msg.distance
        angle = msg.angle

        #sus
        landmark_x, landmark_y = self.aruco_map[aruco_id]

        self.ekf_correction_with_landmark(distance,angle,landmark_x,landmark_y)

    def get_robot_vel(self, wr, wl): 
        v = self.r * (wr + wl) / 2.0 
        w = self.r * (wr - wl) / self.L 
        return v, w 

    

    def ekf_correction_with_landmark(self, distance, angle, landmark_x, landmark_y):
        # 1. Convert relative position to polar coordinates (actual measurement)
        rho_meas = distance
        alpha_meas = angle

        # 2. Actual sensor measurement
        z = np.array([rho_meas, alpha_meas])

        # 3. Calculate expected measurement from current state
        dx = landmark_x- self.x
        dy = landmark_y - self.y
        p = dx**2 + dy**2
        sqrt_p = math.sqrt(p)

        
        z_hat = np.array([
            sqrt_p,
            math.atan2(dy, dx) - self.theta
        ])

        # 4. Normalize innovation angle
        alpha_diff = z[1] - z_hat[1]
        alpha_diff = math.atan2(math.sin(alpha_diff), math.cos(alpha_diff))
        y_k = np.array([z[0] - z_hat[0], alpha_diff])

        # 5. Calculate Jacobian G_k
        G = np.array([
            [-dx / sqrt_p, -dy / sqrt_p, 0],
            [dy / p,       -dx / p,      -1]
        ])

        # 6. Kalman gain
        S = G @ self.Sigma @ G.T + self.RCAM
        S_inv = np.linalg.inv(S)
      
 

        K = self.Sigma @ G.T @ S_inv

        # 7. Update state
        delta = K @ y_k
        
        # Check for invalid corrections
        if np.any(np.isnan(delta)) or np.any(np.isinf(delta)):
            self.get_logger().error("Invalid correction (NaN or Inf)! Skipping.")
            return

        self.x += delta[0]
        self.y += delta[1]
        self.theta += delta[2]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # Normalize angle

        # 8. Update covariance matrix
        I = np.eye(3)
        self.Sigma = (I - K @ G) @ self.Sigma

        # Debug output (throttled)
        

    def update_pose(self, v, w): 
        current_time_ns = self.get_clock().now().nanoseconds
        dt = (current_time_ns - self.prev_time_ns) * 1e-9  
        self.prev_time_ns = current_time_ns

        # Skip invalid time intervals
        if dt <= 0:
            return

        # Movement
        dx = v * math.cos(self.theta) * dt
        dy = v * math.sin(self.theta) * dt
        dtheta = w * dt

        self.x += dx
        self.y += dy
        self.theta += dtheta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # Normalize angle

        # Update covariance matrix (EKF Predict Step)
        H = np.array([
            [1.0, 0.0, -v * dt * math.sin(self.theta)],
            [0.0, 1.0,  v * dt * math.cos(self.theta)],
            [0.0, 0.0, 1.0]
        ])
        self.Sigma = H @ self.Sigma @ H.T + self.Q

    def fill_odom_message(self, x, y, yaw): 
        odom_msg = Odometry()  
        odom_msg.header.stamp = self.get_clock().now().to_msg()  
        odom_msg.header.frame_id = 'odom' 
        odom_msg.child_frame_id = 'base_footprint'  

        odom_msg.pose.pose.position.x = x  
        odom_msg.pose.pose.position.y = y  
        odom_msg.pose.pose.position.z = 0.0   

        quat = transforms3d.euler.euler2quat(0, 0, yaw)  
        odom_msg.pose.pose.orientation.w = quat[0] 
        odom_msg.pose.pose.orientation.x = quat[1] 
        odom_msg.pose.pose.orientation.y = quat[2] 
        odom_msg.pose.pose.orientation.z = quat[3] 
        
        # Assign covariance to odometry message (pose only)
        cov = np.zeros((6, 6))
        cov[0, 0] = self.Sigma[0, 0]
        cov[0, 1] = self.Sigma[0, 1]
        cov[1, 0] = self.Sigma[1, 0]
        cov[1, 1] = self.Sigma[1, 1]
        cov[0, 5] = self.Sigma[0, 2]
        cov[5, 0] = self.Sigma[2, 0]
        cov[1, 5] = self.Sigma[1, 2]
        cov[5, 1] = self.Sigma[2, 1]
        cov[5, 5] = self.Sigma[2, 2]
        odom_msg.pose.covariance = cov.flatten().tolist()

        return odom_msg 

def main(args=None): 
    rclpy.init(args=args) 
    node = Localisation() 
    try: 
        rclpy.spin(node) 
    except KeyboardInterrupt: 
        pass 
    finally: 
        if rclpy.ok(): 
            rclpy.shutdown() 
        node.destroy_node() 

if __name__ == '__main__': 
    main()