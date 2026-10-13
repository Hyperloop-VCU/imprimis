from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ntrip_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
    ],

        # Force inclusion of auxiliary modules if find_packages misses them
    py_modules=[
        'ntrip_client.login_params',
    ],
    
    zip_safe=True,

    maintainer='asher-abraham',
    maintainer_email='abrahama5@vcu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    install_requires=['setuptools', 'flask'],
    
    entry_points={
        # Alias = executable path
        'console_scripts': [
            'ntrip_node = ntrip_client.ntrip_node:main',
            'simulated_gps_node = ntrip_client.simulated_gps_node:main'
        ],
    },
)
