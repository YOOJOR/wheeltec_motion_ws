from glob import glob
from setuptools import find_packages, setup

setup(
    name='wheeltec_motion_control', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/wheeltec_motion_control']),
        ('share/wheeltec_motion_control', ['package.xml', 'README.md']),
        ('share/wheeltec_motion_control/config', glob('config/*.yaml')),
        ('share/wheeltec_motion_control/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
    maintainer='WHEELTEC project maintainers', maintainer_email='maintainer@example.com',
    description='FAST-LIO feedback motion controller', license='Apache-2.0',
    entry_points={'console_scripts': [
        'motion_controller = wheeltec_motion_control.node:main',
        'motion_client = wheeltec_motion_control.client:main',
    ]},
)
