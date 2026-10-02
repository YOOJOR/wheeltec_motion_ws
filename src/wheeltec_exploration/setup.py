from glob import glob
from setuptools import find_packages, setup
setup(name='wheeltec_exploration', version='0.1.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/wheeltec_exploration']),
                  ('share/wheeltec_exploration', ['package.xml']),
                  ('share/wheeltec_exploration/config', glob('config/*.yaml'))],
      install_requires=['setuptools'], zip_safe=True, tests_require=['pytest'],
      maintainer='WHEELTEC project maintainers', maintainer_email='maintainer@example.com',
      description='Read-only random-frontier/FMM integration', license='Apache-2.0',
      entry_points={'console_scripts': ['exploration_preview = wheeltec_exploration.runtime:main',
                                       'exploration_demo = wheeltec_exploration.offline:main']})
