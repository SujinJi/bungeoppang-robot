import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bungeoppang'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         (os.path.join('share', 'bungeoppang', 'urdf'), glob('urdf/*')),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sujin',
    maintainer_email='suj50737@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hud = bungeoppang.hud_node:main',
            'customer = bungeoppang.customer_node:main',
            'voice_order = bungeoppang.voice_order_node:main',
            'my_node = bungeoppang.my_node:main',
            'pour_demo = bungeoppang.pour_demo:main',
            'pour_e0509 = bungeoppang.pour_e0509:main',
            'reach_check = bungeoppang.reach_check:main',
            'gripper_marker = bungeoppang.gripper_marker:main',
        ],
    },
)