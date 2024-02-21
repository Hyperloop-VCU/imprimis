from setuptools import setup

package_name = 'PID_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='nerallarn@vcu.edu',
    description='PID controller for imprimis. Uses encoder input and cmd_vel input to control the motor speeds.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'PID_node = PID_pkg.PID_node:main'
        ],
    },
)
