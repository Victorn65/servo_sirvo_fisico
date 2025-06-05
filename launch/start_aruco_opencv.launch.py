import os 
from launch import LaunchDescription 

from launch_ros.actions import Node 

  

def generate_launch_description(): 

                

    

     

    aruco_opencv  = Node( 

        package='aruco_opencv',  

        executable='aruco_tracker_autostart',  

        name='aruco_tracker',  

         

        parameters=[  

            {'cam_base_topic': '/video_source'},  

            {'marker_size': 0.14},  

            # Select the ArUco dictionary (e.g., ARUCO_ORIGINAL, 4X4_50, 4x4_100)  

            {'marker_dict': '4X4_50'}, 

        ], 

        output='screen',  

    )  

         

    ld = [ aruco_opencv] 

  

    return LaunchDescription(ld) 