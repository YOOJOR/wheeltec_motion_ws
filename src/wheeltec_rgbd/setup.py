from glob import glob
from setuptools import find_packages, setup
setup(name='wheeltec_rgbd', version='0.1.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/wheeltec_rgbd']),
                  ('share/wheeltec_rgbd', ['package.xml']),
                  ('share/wheeltec_rgbd/config', glob('config/*.yaml')),
                  ('share/wheeltec_rgbd/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
      maintainer='WHEELTEC project maintainers', maintainer_email='maintainer@example.com',
      description='Fresh RGB-D camera interface', license='Apache-2.0',
      entry_points={'console_scripts': ['rgbd_adapter = wheeltec_rgbd.node:main',
                                       'rgbd_capture = wheeltec_rgbd.capture:main']})
