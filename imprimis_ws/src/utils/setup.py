from setuptools import find_packages, setup

package_name = 'utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyperlabs',
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
            'gps_add_sim_covariance = utils.gps_add_sim_covariance:main',
            'fix_lidar_delay = utils.fix_lidar_delay:main',
            'wait_for_tf = utils.wait_for_tf:main',
            'odom_remapper = utils.odom_remapper:main',
            'gps_monitor = utils.gps_monitor:main'
        ],
    },
)
