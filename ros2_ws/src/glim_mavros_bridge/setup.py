from setuptools import setup

package_name = 'glim_mavros_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nonsaya-n',
    maintainer_email='nonsaya-n@local',
    description='Republish GLIM Odometry to MAVROS for EKF3 external nav.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = glim_mavros_bridge.bridge_node:main',
        ],
    },
)


