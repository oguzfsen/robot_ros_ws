from setuptools import find_packages, setup

package_name = 'robot_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'requests'],
    zip_safe=True,
    maintainer='oguz',
    maintainer_email='oguz@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "brake_node = robot_core.brake_node:main",
            "compass_node = robot_core.compass_node:main",
            "core_node = robot_core.core_node:main",
            "database_node = robot_core.database_node:main",
            "encoder_node = robot_core.encoder_node:main",
            "gnss_rover_node = robot_core.gnss_rover_node:main",
            "hubmotor_node = robot_core.hubmotor_node:main",
            "i2c_srv_node = robot_core.i2c_srv_node:main",
            "rc_node = robot_core.rc_node:main",
            "relay_node = robot_core.relay_node:main",
            "robot_cloud_node = robot_core.signalr_node:main",
            "signalr_node = robot_core.signalr_node:main",
            "steer_node = robot_core.steer_node:main",
            "steer_zero_node = robot_core.steer_zero_node:main",
            "telemetry_node = robot_core.telemetry_node:main",
            
            "compass_listener_node = robot_core.listeners.compass_listener_node:main",
            "encoder_listener_node = robot_core.listeners.encoder_listener_node:main",
            "rc_listener_node = robot_core.listeners.rc_listener_node:main",
        ],
    },
)
