import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'learned_frontier_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # packages=find_packages(include=[package_name, package_name + '/yolov5', package_name + '/yolov5/utils']),
    data_files=[
        ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'weights'), glob(os.path.join('weights', '*.pt')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adrian',
    maintainer_email='',
    description='Frontier detection with a custom trained object detector',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frontier_detector = learned_frontier_detector.learned_frontier_detector:main'
        ],
    },
)
