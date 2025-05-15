import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),

        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # Include config YAML files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

        # Include URDF or xacro files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),\
        
        ('share/ament_index/resource_index/packages', ['resource/rover_control']),
        ('share/rover_control', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='template-rbc',
    maintainer_email='template-rbc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tibalt_control = rover_control.tibalt_control:main',
        ],
    },
)
