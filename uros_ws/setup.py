from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'pid_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # This line ensures ROS 2 can find your package resource
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
        
        # Install YAML files from the config folder
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
        # Install any launch files from the launch folder
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mcr2',
    maintainer_email='a01285451@tec.mx',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # <executable_name> = <package_name>.<python_file_without_.py>:<function_name>
            'input = pid_control.input:main',
        ],
    },
)
