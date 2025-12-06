from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'simu_rnftsmc' 

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']), 
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tomtom',
    maintainer_email='user@example.com',
    description='RNFTSMC Controller package for path tracking.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    
    entry_points={
        'console_scripts': [
            'rnftsmc_controller = simu_rnftsmc.controller:main', 
        ],
    },
)