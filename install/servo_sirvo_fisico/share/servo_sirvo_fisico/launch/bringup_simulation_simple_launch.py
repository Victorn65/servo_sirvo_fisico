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



def generate_launch_description():
    # -----------------------------------------------------------------------------
    #                          SIMULATION CONFIGURATION
    # -----------------------------------------------------------------------------

    world = '/home/victorn65/ros2_ws/src/servo_sirvo_fisico/worlds/maze_aruco.world'
    pause = 'true'
    verbosity = '4'
    use_sim_time = 'True'

    robot_config_list = [
        {
            'name': '',
            'type': 'puzzlebot_jetson_lidar_ed',
            'x': 0.0, 'y': 0.0, 'yaw': 0.0,
            'lidar_frame': 'laser_frame',
            'camera_frame': 'camera_link_optical',
            'tof_frame': 'tof_link'
        }
    ]

    # -----------------------------------------------------------------------------
    #                         LOAD GAZEBO WORLD
    # -----------------------------------------------------------------------------

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('servo_sirvo_fisico'),
                'launch',
                'gazebo_world_launch.py'
            )
        ),
        launch_arguments={
            'world': world,
            'pause': pause,
            'verbosity': verbosity,
            'use_sim_time': use_sim_time
        }.items()
    )


    #POSICIÓN DE LOS ARUCOS
    # Get the arucos poses
    with open('/home/victorn65/ros2_ws/src/servo_sirvo_fisico/worlds/maze_aruco.world', 'r') as file:
        world_content = file.read()
    tree = ET.ElementTree(ET.fromstring(world_content))
    root = tree.getroot()
    aruco_poses = {'aruco_poses': {}}
    for aruco in root.find('world').findall('include'):
        aruco_id = int(aruco.find('uri').text[-1])
        pose = aruco.find('pose').text.split()[:2]
        aruco_poses['aruco_poses'][aruco_id] = pose.copy()
    aruco_poses['aruco_poses'] = json.dumps(aruco_poses['aruco_poses'])



    # Map to odom transform node
    map_odom_transform_node = Node(name='map_odom_transform',
                                    package='tf2_ros',
                                    executable='static_transform_publisher',
                                    output='screen',
                                    arguments=['--x', '0', '--y', '0', '--z', '0',
                                                '--yaw', '0', '--pitch', '0', '--roll', '0',
                                                '--frame-id', 'map', '--child-frame-id', 'odom'],)


    # -----------------------------------------------------------------------------
    #                       SPAWN EACH ROBOT DYNAMICALLY
    # -----------------------------------------------------------------------------

    robot_launches = []
    for robot in robot_config_list:
        robot_name = robot['name']
        robot_type = robot['type']
        x = str(robot.get('x', 0.0))
        y = str(robot.get('y', 0.0))
        yaw = str(robot.get('yaw', 0.0))
        lidar_frame = robot.get('lidar_frame', 'laser_frame')
        camera_frame = robot.get('camera_frame', 'camera_link_optical')
        tof_frame = robot.get('tof_frame', 'tof_link')
        prefix = f'{robot_name}/' if robot_name != '' else ''

        robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('servo_sirvo_fisico'),
                    'launch',
                    'gazebo_puzzlebot_launch.py'
                )
            ),
            launch_arguments={
                'robot': robot_type,
                'robot_name': robot_name,
                'x': x,
                'y': y,
                'yaw': yaw,
                'prefix': prefix,
                'lidar_frame': lidar_frame,
                'camera_frame': camera_frame,
                'tof_frame': tof_frame,
                'use_sim_time': use_sim_time
            }.items()
        )

       
        robot_launches.append(robot_launch)


      # Load bridge config
    bridge_config_path = os.path.join(
        get_package_share_directory('servo_sirvo_fisico'),
        'config',
        'bridge_sim.yaml'
    )

    with open(bridge_config_path, 'r') as f:
        bridge_conf = yaml.safe_load(f)

    # Construct bridges from robot_config_list
        # Construct bridges from robot_config_list
    bridges = []
    for robot in robot_config_list:
        robot_namespace = robot['name']  # empty string if no namespace
        for bridge in bridge_conf:
            ros_topic = bridge['ros_topic_name']
            ros_type = bridge['ros_type_name']
            gz_type = bridge['gz_type_name']

            if ros_topic == 'clock':
                # clock topic: no namespace, bracket format [gz_type
                bridges.append(f'{ros_topic}@{ros_type}[{gz_type}')
            elif ros_topic == 'cmd_vel':
                # cmd_vel topic: namespace + ]gz_type
                if robot_namespace:
                    bridges.append(f'{robot_namespace}/{ros_topic}@{ros_type}]{gz_type}')
                else:
                    bridges.append(f'{ros_topic}@{ros_type}]{gz_type}')
            else:
                # other topics: namespace + [gz_type
                if robot_namespace:
                    bridges.append(f'{robot_namespace}/{ros_topic}@{ros_type}[{gz_type}')
                else:
                    bridges.append(f'{ros_topic}@{ros_type}[{gz_type}')


    # -----------------------------------------------------------------------------
    #                          BUILD FINAL LAUNCH DESCRIPTION
    # -----------------------------------------------------------------------------

    rviz_config_path = os.path.join('servo_sirvo_fisico', 'rviz', 'puzzlebot_view.rviz')

    rviz_node = Node(name='rviz2',
                        package='rviz2',
                        executable='rviz2',
                        output='screen',
                        arguments=['-d', rviz_config_path],)


    #Brigde con ROS-GAZEBO

    gz_bridge_node = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=bridges,
    output='screen'
    )

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
        SetEnvironmentVariable('USE_SIM_TIME', 'True'),
        gazebo_launch,
        *robot_launches,
        gz_bridge_node,
        map_odom_transform_node,
        rviz_node,
        localisation_node,
        point_stabilisation_control_node,
        aruco_node,
        image_node,
    ])