from setuptools import setup

package_name = 'gelsight_ros2_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gelsight_publisher.launch.py']),
    ],
    install_requires=['setuptools', 'rclpy', 'sensor_msgs', 'cv_bridge', 'PyYAML', 'opencv-python'],
    zip_safe=True,
    maintainer='tailai.cheng',
    maintainer_email='tailai.cheng@example.com',
    description='ROS2 publisher for GelSight Mini using gs_sdk FastCamera',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gelsight_publisher_node = gelsight_ros2_publisher.gelsight_publisher_node:main',
        ],
    },
)
