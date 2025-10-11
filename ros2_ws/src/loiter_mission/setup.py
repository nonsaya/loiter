from setuptools import setup

package_name = 'loiter_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/minimal_mission.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nonsaya-n',
    maintainer_email='nonsaya-n@local',
    description='Minimal mission: arm→takeoff(1m)→hover(10s)→land→disarm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'minimal_mission_node = loiter_mission.minimal_mission_node:main',
        ],
    },
)


