import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import yaml
import xml.etree.ElementTree as ET
import json
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():

    use_sim_time_param = {'use_sim_time': False}

    robot_xacro_filename = 'puzzlebot.xacro'

    package_share_dir = get_package_share_directory('servo_sirvo_fisico')

    # Get path to the package where robot models are stored
    robot_path = os.path.join(package_share_dir, 'urdf', robot_xacro_filename)

    # Generate robot_description dynamically by calling xacro with the selected parameters
    robot_description = Command(['xacro ', str(robot_path),
                                ' camera_frame:=', 'camera_link_optical',
                                ' lidar_frame:=', 'laser',
                                ' tof_frame:=', 'tof_link'])

    # Node that publishes TF and joint states from robot_description
    robot_state_publisher_node = Node(package="robot_state_publisher",
                                        executable="robot_state_publisher",
                                        output="screen",
                                        parameters=[{
                                            "robot_description": ParameterValue(robot_description, value_type=str),
                                            "use_sim_time": use_sim_time_param,
                                        }],)




    # Map to odom transform node
    map_odom_transform_node = Node(name='map_odom_transform',
                                    package='tf2_ros',
                                    executable='static_transform_publisher',
                                    output='screen',
                                    arguments=['--x', '0', '--y', '0', '--z', '0',
                                                '--yaw', '0', '--pitch', '0', '--roll', '0',
                                                '--frame-id', 'map', '--child-frame-id', 'odom'],)

    # Map to point transform node
    # These are the positions of the points in the map frame
    # Adjust the x, y, z, yaw, pitch, roll values as needed for your setup
    # The points were the ones used for the puzzlebot setpoint
    # map_point_1_tf = (-0.8, -1.2, 0, 0, 0, 0)
    # map_point_2_tf = (1.2, 1.16, 0, 0, 0, 0)  
    # map_point_3_tf = (-1.2, 1.0, 0, 0, 0, 0)
    # map_point_start_tf = (1.2, -1.1, 0, 0, 0, 0)
    
    map_point_1 = Node(name='map_point1_transform',
                                     package='tf2_ros',
                                     executable='static_transform_publisher',
                                     output='screen',
                                     arguments=['--x', '-0.8', '--y', '-1.2`', '--z', '0',
                                                 '--yaw', '0', '--pitch', '0', '--roll', '0',
                                                 '--frame-id', 'map', '--child-frame-id', 'point1'],)

    map_point_2 = Node(name='map_point2_transform',
                                     package='tf2_ros',
                                     executable='static_transform_publisher',
                                     output='screen',
                                     arguments=['--x', '1.2', '--y', '1.16', '--z', '0',
                                                 '--yaw', '0', '--pitch', '0', '--roll', '0',
                                                 '--frame-id', 'map', '--child-frame-id', 'point2'],)

    map_point_3 = Node(name='map_point3_transform',
                                     package='tf2_ros',
                                     executable='static_transform_publisher',
                                     output='screen',
                                     arguments=['--x', '-1.2', '--y', '1.0', '--z', '0',
                                                 '--yaw', '0', '--pitch', '0', '--roll', '0',
                                                 '--frame-id', 'map', '--child-frame-id', 'point3'],)
    map_point_start = Node(name='map_point_start_transform',
                                        package='tf2_ros',
                                        executable='static_transform_publisher',
                                        output='screen',
                                        arguments=['--x', '1.2', '--y', '-1.1', '--z', '0',
                                                    '--yaw', '0', '--pitch', '0', '--roll', '0',
                                                    '--frame-id', 'map', '--child-frame-id', 'point_start'],)

    # TFs estáticos de map -> aruco#
    # These are the positions of the ArUco markers in the map frame
    # Adjust the x, y, z, yaw, pitch, roll values as needed for your setup  
    # aruco_0_tf = (0.9, 0.6, 0, 0, 0, 0)
    # aruco_1_tf = (1.2, -1.48, 0, 0, 0, 0)
    # aruco_2_tf = (0.3, 0.28, 0, 0, 0, 0)
    # aruco_3_tf = (-1.5, 0.0, 0, 0, 0, 0)
    # aruco_4_tf = (-1.0, -1.48, 0, 0, 0, 0)
    # aruco_5_tf = (-1.2, 1.48, 0, 0, 0, 0)

    aruco_0_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='aruco0_tf',
        output='screen',
        arguments=['--x', '0.9', '--y', '0.6', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'map', '--child-frame-id', 'aruco0']
    )

    aruco_1_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='aruco1_tf',
        output='screen',
        arguments=['--x', '1.2', '--y', '-1.48', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'map', '--child-frame-id', 'aruco1']
    )

    aruco_2_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='aruco2_tf',
        output='screen',
        arguments=['--x', '0.3', '--y', '0.28', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'map', '--child-frame-id', 'aruco2']
    )

    aruco_3_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='aruco3_tf',
        output='screen',
        arguments=['--x', '-1.5', '--y', '0.0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'map', '--child-frame-id', 'aruco3']
    )

    aruco_4_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='aruco4_tf',
        output='screen',
        arguments=['--x', '-1.0', '--y', '-1.48', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'map', '--child-frame-id', 'aruco4']
    )

    aruco_5_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='aruco5_tf',
        output='screen',
        arguments=['--x', '-1.2', '--y', '1.48', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'map', '--child-frame-id', 'aruco5']
    )

    # -----------------------------------------------------------------------------
    #                          BUILD FINAL LAUNCH DESCRIPTION
    # -----------------------------------------------------------------------------

    rviz_config_path = os.path.join('servo_sirvo_fisico', 'rviz', 'puzzlebot_view.rviz')

    rviz_node = Node(name='rviz2',
                        package='rviz2',
                        executable='rviz2',
                        output='screen',)
                        # arguments=['-d', rviz_config_path],)

    arbolito  = Node(name='rqt_tf_tree',
                        package='rqt_tf_tree',
                        executable='rqt_tf_tree',
                        output='screen',)

    localisation_node = Node(
        package='servo_sirvo_fisico',
        executable='localisation',
        name='localisation',
        output='screen',
    )

    point_stabilisation_control_node = Node(
        package='servo_sirvo_fisico',
        executable='point_stabilisation_control',
        name='point_stabilisation_control',
        output='screen',
    )

    joint_pub = Node(
        package='servo_sirvo_fisico',
        executable='joint_state_pub',
        parameters=[use_sim_time_param]
    )

    
    image_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        output='screen',
    )

    aruco_node = Node(
        package='servo_sirvo_fisico',
        executable='puzzlebot_aruco',
        name='puzzlebot_aruco',
        output='screen',
    )



    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        SetEnvironmentVariable('USE_SIM_TIME', 'False'),

        map_odom_transform_node,
        rviz_node,
        localisation_node,
        point_stabilisation_control_node,
        aruco_node,
        image_node,
        robot_state_publisher_node,
        joint_pub,
        arbolito,
        map_point_1,
        map_point_2,
        map_point_3,
        map_point_start,
        aruco_0_tf,
        aruco_1_tf,
        aruco_2_tf,
        aruco_3_tf,
        aruco_4_tf,
        aruco_5_tf,
    ])