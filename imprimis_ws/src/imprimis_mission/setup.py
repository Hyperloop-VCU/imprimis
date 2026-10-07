from glob import glob

from setuptools import find_packages, setup

package_name = 'imprimis_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/courses', glob('courses/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Julian B. Lewis',
    maintainer_email='lewisjb3@vcu.edu',
    description='Lap manager and control window for the IMPRIMIS simulator.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'lap_manager = imprimis_mission.lap_manager:main',
            'control_gui = imprimis_mission.control_gui:main',
            'course_watcher = imprimis_mission.course_sync:main',
            'speed_governor = imprimis_mission.speed_governor:main',
        ],
    },
)
