import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bungeoppang2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='su',
    maintainer_email='su@todo.todo',
    description='붕어빵 로봇: 공정 제어 + 음성인식 + 사람 인식',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bake_sequence = bungeoppang2_yammy.bake_sequence_node:main',   # 전체 공정
            'flip_mold     = bungeoppang2_yammy.flip_mold_node:main',       # 뒤집기만
            'gripper_viz   = bungeoppang2_yammy.gripper_viz:main',          # RViz 그리퍼/작업대 표시
            'voice_node    = bungeoppang2_yammy.voice_node:main',           # 음성인식
            'text_order    = bungeoppang2_yammy.voice_node:main_text',      # (임시) 키보드로 주문
            'vision_node   = bungeoppang2_yammy.vision_node:main',          # 사람 인식
            'orchestrator  = bungeoppang2_yammy.orchestrator_node:main',    # 총괄 상태 머신
        ],
    },
)
