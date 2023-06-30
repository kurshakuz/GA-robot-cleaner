from glob import glob
import os

from setuptools import setup

package_name = 'diff_robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shyngys',
    maintainer_email='abilkasov@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dust_particles_node = diff_robot_control.dust_particles_node:main',
            'velocity_teleop = diff_robot_control.velocity_teleop:main',
            'velocity_evolutionary_algorithm = diff_robot_control.velocity_evolutionary_algorithm:main',
        ],
    },
)
