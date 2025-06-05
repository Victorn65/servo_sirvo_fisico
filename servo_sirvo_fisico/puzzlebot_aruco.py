import servo_sirvo_fisico.utils.observation_utils as observation_utils
import cv2
import numpy as np
import rclpy
import tf2_geometry_msgs

from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from puzzlebot_aruco_msgs.msg import ArucoObservation
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener

class PuzzlebotAruco(Node):
    def __init__(self):
        super().__init__('puzzlebot_aruco_node')

        # Namespace
        self.namespace = self.namespace = self.get_namespace().strip('/')

        # Declare parameters
        # self.declare_parameter('aruco_side_length', 0.15) # Side length of the ArUco markers in meters
        # self.declare_parameter('camera_matrix', [0.1234, 0.6543, 0.345678, 0.8765, 0.4567, 0.78984, 0.34566, 0.3673, 0.676]) # Camera intrinsic matrix (3x3)
        # self.declare_parameter('camera_distortion', [0.4375, 0.3344, 0.890, 0.6798, 0.567657]) # Camera distortion coefficients (5x1)
        self.declare_parameter('aruco_side_length', 0.15) # Side length of the ArUco markers in meters
        self.declare_parameter('camera_matrix', [191.26581, 0.0, 169.60164, 0.0, 255.02285, 109.55441, 0.0, 0.0, 1.0]) # Camera intrinsic matrix (3x3)
        self.declare_parameter('camera_distortion', [-0.348494, 0.113557, -0.000324, 0.000785, 0.0]) # Camera distortion coefficients (5x1)
        self.declare_parameter('camera_optical_frame', 'camera_link_optical') # Frame ID of the camera optical frame

        # Get the parameters
        self.aruco_side_length = self.get_parameter('aruco_side_length').get_parameter_value().double_value
        self.camera_matrix = np.array(self.get_parameter('camera_matrix').get_parameter_value().double_array_value).reshape((3, 3))
        self.dist_coeffs = np.array(self.get_parameter('camera_distortion').get_parameter_value().double_array_value).reshape((5, 1))
        self.camera_optical_frame = self.get_parameter('camera_optical_frame').get_parameter_value().string_value

        # Subscribers
        self.create_subscription(Image, 'video_source/raw', self.camera_callback, 10)

        # Publishers
        self.aruco_image_publisher = self.create_publisher(Image, 'aruco_image', qos_profile_sensor_data)
        self.aruco_observation_publisher = self.create_publisher(ArucoObservation, 'aruco_observation', qos_profile_sensor_data)

        # Node variables
        self.tf_buffer = Buffer() # TF2 buffer to store transforms
        self.tf_listener = TransformListener(self.tf_buffer, self) # TF2 listener to listen for transforms
        self.bridge = CvBridge() # Bridge to convert between ROS Image messages and OpenCV images
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50) # Predefined ArUco dictionary
        self.parameters = cv2.aruco.DetectorParameters() # Detector parameters for ArUco markers
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters) # ArUco detector instance

        self.get_logger().info("Puzzlebot Aruco Node has started")

    def camera_callback(self, msg):
        # Convert the ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Transform the image to grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # Detect ArUco markers in the image
        corners, ids, _ = self.detector.detectMarkers(gray_image)
        ids = ids.flatten() if ids is not None else []
        if len(corners) > 0:
            for corner, aruco_id in zip(corners, ids):
                # Estimate the pose of the detected markers
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, self.aruco_side_length, self.camera_matrix, self.dist_coeffs)
                # Draw the axis of the marker
                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec[0][0], tvec[0][0], 0.05)
                # Get the marker center
                top_left, top_right, bottom_right, bottom_left = corner.reshape((4, 2))
                top_left, top_right, bottom_right, bottom_left = tuple(map(int, top_left)), tuple(map(int, top_right)), tuple(map(int, bottom_right)), tuple(map(int, bottom_left))
                # Draw the marker corners
                cv2.line(cv_image, top_left, top_right, (0, 255, 0), 2)
                cv2.line(cv_image, top_right, bottom_right, (0, 255, 0), 2)
                cv2.line(cv_image, bottom_right, bottom_left, (0, 255, 0), 2)
                cv2.line(cv_image, bottom_left, top_left, (0, 255, 0), 2)
                # Annotate the marker ID on the image
                cv2.putText(cv_image, str(aruco_id), (bottom_left[0], bottom_left[1]+15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                # Create a PointStamped message to represent the marker position
                marker_point = PointStamped()
                marker_point.header.stamp = self.get_clock().now().to_msg()
                marker_point.header.frame_id = self.camera_optical_frame.lstrip('/')
                marker_point.point.x = tvec[0][0][0]
                marker_point.point.y = tvec[0][0][1]
                marker_point.point.z = tvec[0][0][2]
                # Ensure no leading slash


                target_frame = f"{self.namespace}/base_footprint" if self.namespace else "base_footprint"
                transformed_point = self.tf_buffer.transform(marker_point, target_frame, timeout=rclpy.duration.Duration(seconds=1.0))


                # Calculate the distance and angle from the camera to the marker in the (x, y) plane
                distance, angle = observation_utils.observate_from_deltas(transformed_point.point.x, transformed_point.point.y, 0.0)
                # Annotate the distance on the image
                cv2.putText(cv_image, f"{distance:.2f} m", (top_right[0]+7, top_right[1]+12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                # Annotate the angle on the image
                cv2.putText(cv_image, f"{angle:.2f} rad", (top_right[0]+7, top_right[1]+27), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Publish the detected ArUco markers
                aruco_observation_msg = ArucoObservation()
                aruco_observation_msg.id = int(aruco_id)
                aruco_observation_msg.distance = distance
                aruco_observation_msg.angle = angle
                self.aruco_observation_publisher.publish(aruco_observation_msg)
                
        # Publish the annotated image
        aruco_image_header = msg.header
        aruco_image_header.stamp = self.get_clock().now().to_msg()
        aruco_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8', header=aruco_image_header)
        self.aruco_image_publisher.publish(aruco_image_msg)

def main():
    rclpy.init()
    node = PuzzlebotAruco()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().info('Node interrupted. Shutting down...')
        node.get_logger().info(f'Error: {e}')
        if rclpy.ok():
            rclpy.shutdown()
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()