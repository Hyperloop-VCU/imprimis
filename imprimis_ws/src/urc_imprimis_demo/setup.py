from setuptools import find_packages, setup

package_name = 'urc_imprimis_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/urc_imprimis.launch.py']),
        ('share/' + package_name, ['config/bno055.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ray',
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
            'urc_imprimis_translator_node = urc_imprimis_demo.urc_imprimis_translator_node:main'
        ],
    },
)
