import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'uwb_data_reader'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saman',
    maintainer_email='sammj2231@gmail.com',
    description='UWB Data Reader Node Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_reader = uwb_data_reader.data_reader:main'
        ],
    },
)