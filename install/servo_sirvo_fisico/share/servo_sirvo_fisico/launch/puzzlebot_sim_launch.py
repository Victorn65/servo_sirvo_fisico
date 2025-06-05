import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    use_sim_time_param = {'use_sim_time': True}

    # robot_xacro_filename = 'puzzlebot.xacro'

    # package_share_dir = get_package_share_directory('servo_sirvo_fisico')

    # # Get path to the package where robot models are stored
    # robot_path = os.path.join(package_share_dir, 'urdf', robot_xacro_filename)

    # # Generate robot_description dynamically by calling xacro with the selected parameters
    # robot_description = Command(['xacro ', str(robot_path),
    #                             ' camera_frame:=', 'camera_link_optical',
    #                             ' lidar_frame:=', 'laser',
    #                             ' tof_frame:=', 'tof_link'])

    # # Node that publishes TF and joint states from robot_description
    # robot_state_publisher_node = Node(package="robot_state_publisher",
    #                                     executable="robot_state_publisher",
    #                                     output="screen",
    #                                     parameters=[{
    #                                         "robot_description": ParameterValue(robot_description, value_type=str),
    #                                         "use_sim_time": use_sim_time_param,
    #                                     }],)
    
    


    localisation = Node(
        package='servo_sirvo_fisico',
        executable='localisation',
        parameters=[use_sim_time_param]
    )

    # joint_pub = Node(
    #     package='servo_sirvo_fisico',
    #     executable='joint_state_pub',
    #     parameters=[use_sim_time_param]
    # )

    kinematic = Node(
        package='servo_sirvo_fisico',
        executable='puzzlebot_kinematic_model',
        parameters=[use_sim_time_param]
    )

    my_rqt_node = Node(
        package='rqt_tf_tree',
        executable='rqt_tf_tree',
        output='screen',
        parameters=[use_sim_time_param]
    )

    static_tf_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
        parameters=[use_sim_time_param]
    )

    


    return LaunchDescription([
   
        static_tf_1,
        my_rqt_node,
        localisation,
        kinematic,
        #joint_pub,
        #robot_state_publisher_node,#comentar y llamar launch gazebo en caso de simulación
        
    ])