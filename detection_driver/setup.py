import os
from glob import glob
from setuptools import setup

package_name = 'detection_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('configs/*.yaml')),
        (os.path.join('share', package_name), glob('models/mobile_net/*.py')),
        (os.path.join('share', package_name), glob('models/mobile_net/*.pb')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Patrick Rebling',
    maintainer_email='repa1030@hs-karlsruhe.de',
    description='Driver for different detectors',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_driver = detection_driver.detection_driver:main'
        ],
    },
)    
