from setuptools import find_packages, setup
from glob import glob  # serve per includere tutti i file in una cartella

package_name = 'omnicar_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Aggiungi questa riga per includere i launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # Se hai file di config
        # ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lorenzo Martignetti',
    maintainer_email='lorenzo.martignetti@ing.unipi.it',
    description='Omnicar test',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'omnicar_test_node = omnicar_test.omnicar_test_node:main'
        ],
    },
)