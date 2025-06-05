import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
from rclpy import qos 


# Publica transformadas de mi robot

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        # velocidades de las ruedas
        self.wr_sub = self.create_subscription(Float32, 'VelocityEncR', self.wr_callback, qos.qos_profile_sensor_data)
        self.wl_sub = self.create_subscription(Float32, 'VelocityEncL', self.wl_callback, qos.qos_profile_sensor_data)

        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        
        # Publisher JointState
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # Variables 
        self.left_wheel_angle = 0.0  # Ángulo acumulado de la rueda izquierda [rad]
        self.right_wheel_angle = 0.0  # Ángulo acumulado de la rueda derecha [rad]
        self.wr = 0.0  # Velocidad angular rueda derecha [rad-seg]
        self.wl = 0.0  # Velocidad angular rueda izquierda [rad-seg]
        self.prev_time = self.get_clock().now()

        self.tf_broadcaster = TransformBroadcaster(self)
        
       
        self.joint_state = JointState()

        
        self.joint_state.name = ['wheel_left_joint', 'wheel_right_joint']  
    
    def wr_callback(self, msg):
        self.wr = msg.data
    
    def wl_callback(self, msg):
        self.wl = msg.data
    
    def odom_callback(self,msg):

        # Create a TransformStamped message for the odom to base_footprint transform and assign the odometry message pose values
        odom_base_footprint_transform = TransformStamped()
        odom_base_footprint_transform.header.stamp = self.get_clock().now().to_msg()
        odom_base_footprint_transform.header.frame_id = 'odom'
        odom_base_footprint_transform.child_frame_id = 'base_footprint'
        odom_base_footprint_transform.transform.translation.x = msg.pose.pose.position.x
        odom_base_footprint_transform.transform.translation.y = msg.pose.pose.position.y
        odom_base_footprint_transform.transform.translation.z = 0.0
        odom_base_footprint_transform.transform.rotation.x = msg.pose.pose.orientation.x
        odom_base_footprint_transform.transform.rotation.y = msg.pose.pose.orientation.y
        odom_base_footprint_transform.transform.rotation.z = msg.pose.pose.orientation.z
        odom_base_footprint_transform.transform.rotation.w = msg.pose.pose.orientation.w

        # Publish the transform
        self.tf_broadcaster.sendTransform(odom_base_footprint_transform)  


        

        ###############LLANTAS##############################
        
        dt = (self.get_clock().now() - self.prev_time).nanoseconds / 1e9



        self.left_wheel_angle += self.wl * dt
        self.right_wheel_angle += self.wr * dt
        
        # Normalizar angles
        self.left_wheel_angle = math.atan2(math.sin(self.left_wheel_angle), math.cos(self.left_wheel_angle))
        self.right_wheel_angle = math.atan2(math.sin(self.right_wheel_angle), math.cos(self.right_wheel_angle))
        # Update the wheels angles based on the wheels speeds and dt
       

        # Create a JointState message for the wheels angles
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.header.frame_id = 'base_footprint'
        joint_state_msg.name = ['wheel_right_joint', 'wheel_left_joint']
        joint_state_msg.position = [self.left_wheel_angle, self.right_wheel_angle]
        joint_state_msg.velocity = [0.0, 0.0]
        joint_state_msg.effort = [0.0, 0.0]

        # Publish the joint state
        self.joint_state_publisher.publish(joint_state_msg)   

        # Last time update
        self.prev_time = self.get_clock().now()

      
 

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()