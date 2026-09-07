from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'team5_pitstop_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (
            'share/' + package_name,
            ['package.xml', 'requirements.txt', 'DEPENDENCIES.md'],
        ),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'),
            glob('models/*.onnx')),
    ],
    install_requires=[
        'setuptools',
        'numpy==1.26.4',
        'onnxruntime==1.29.0',
        'flatbuffers==25.12.19',
        'protobuf==7.36.1',
    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='294007885+qaiplatform6-ux@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'battery_monitor_node = team5_pitstop_pkg.battery_monitor_node:main',
            'team5_pit_request_node = team5_pitstop_pkg.team5_pit_request_node:main',
            'team5_remote_stop_node = team5_pitstop_pkg.team5_remote_stop_node:main',
            'team5_manual_pit_node = team5_pitstop_pkg.team5_manual_pit_node:main',
            'team5_cmd_arbiter_node = team5_pitstop_pkg.team5_cmd_arbiter_node:main',
            'stop_manager_node = team5_pitstop_pkg.stop_manager_node:main',
            'stop_sign_node = team5_pitstop_pkg.stop_sign_node:main',
            'hailo_stop_bridge_node = '
            'team5_pitstop_pkg.hailo_stop_bridge_node:main',
            'mac_vision_bridge_node = '
            'team5_pitstop_pkg.mac_vision_bridge_node:main',
            'preflight_check = team5_pitstop_pkg.preflight_check:main',
            'team5_status = team5_pitstop_pkg.team5_status:main',
        ],
    },
)
