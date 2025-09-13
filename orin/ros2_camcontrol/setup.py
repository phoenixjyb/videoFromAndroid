from setuptools import setup

package_name = 'ros2_camcontrol'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='WebSocket H.264 ingest to ROS2 Image publisher',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ws_to_image = ros2_camcontrol.ws_to_image:main',
        ],
    },
)

