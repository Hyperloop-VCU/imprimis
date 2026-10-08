from setuptools import find_packages, setup

package_name = 'imprimis_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/basic_nav.launch.py']),
        ('share/' + package_name + '/launch', ['launch/localization.launch.py']),
        ('share/' + package_name + '/launch', ['launch/nav2_minimal_bringup.launch.py']),
        ('share/' + package_name + '/launch', ['launch/robot_mission.launch.py']),
        ('share/' + package_name + '/launch', ['launch/sim_mission.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='imprimis-pc',
    maintainer_email='rayneralla@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'map_goal_to_odom = imprimis_navigation.map_goal_to_odom:main',
            'speed_governor = imprimis_navigation.speed_governor:main',
            'sim_control_gui = imprimis_navigation.sim_control_gui:main'
        ],
    },
)
