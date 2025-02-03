import os
from glob import glob
from setuptools import setup

package_name = 'drozd_s5646665_assignment2'
lib = 'drozd_s5646665_assignment2/lib'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, lib],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "resource"), glob("resource/*.png")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='simone',
    maintainer_email='simone.maccio@edu.unige.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crane_sim_node = drozd_s5646665_assignment2.crane_sim_node:main',
            'motor_x_node = drozd_s5646665_assignment2.motor_x_node:main',
            'motor_y_node = drozd_s5646665_assignment2.motor_y_node:main',
            'robot_logic_node = drozd_s5646665_assignment2.robot_logic_node:main', 
        ],
    },
)
