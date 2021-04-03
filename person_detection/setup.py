from glob import glob
import os

from setuptools import setup

package_name = 'person_detection'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('configs/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Patrick Rebling',
    maintainer_email='repa1030@hs-karlsruhe.de',
    description='Global launch files and configurations',
    license='MIT',
    tests_require=['pytest'],
)
