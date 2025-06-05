from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'servo_sirvo_fisico'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),


    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ]+ [
        (os.path.join('share', package_name, root), [os.path.join(root, file)]) 
        for root, _, files in os.walk('urdf') for file in files
    ]+
    [
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),]
    +
    [
        (os.path.join('share', package_name, root), [os.path.join(root, file)]) 
        for root, _, files in os.walk('meshes') for file in files
    ]+
    [
        (os.path.join('share', package_name, root), [os.path.join(root, file)]) 
        for root, _, files in os.walk('models') for file in files
    ]+
    [
        (os.path.join('share', package_name, root), [os.path.join(root, file)]) 
        for root, _, files in os.walk('worlds') for file in files
    ]
    +
    [
        (os.path.join('share', package_name, root), [os.path.join(root, file)]) 
        for root, _, files in os.walk('plugins') for file in files
    ],

    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fredy_hector',
    maintainer_email='fredy@outlook.com',
    description='Puzzlebot chicos',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'point_stabilisation_control = servo_sirvo_fisico.point_stabilisation_control:main',
            'localisation = servo_sirvo_fisico.localisation:main',
            'joint_state_pub = servo_sirvo_fisico.joint_state_pub:main',
            'dynamic_tf = servo_sirvo_fisico.dynamic_tf:main',
            'static_tf = servo_sirvo_fisico.static_tf:main',
            'puzzlebot_kinematic_model= servo_sirvo_fisico.puzzlebot_kinematic_model:main',
            'aruco_debug= servo_sirvo_fisico.aruco_debug:main',
            'puzzlebot_aruco= servo_sirvo_fisico.puzzlebot_aruco:main',
            
            
        ],
    },
)


