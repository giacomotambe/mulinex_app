import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ik_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Davide De Benedittis',
    maintainer_email='davide.debenedittis@gmail.com',
    description='Inverse kinematics controller for the Mulinex robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ik_controller = ik_controller.ik_controller:main',
        ],
    },
)
