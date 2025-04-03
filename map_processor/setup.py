import os
from glob import glob
from setuptools import find_packages
from setuptools import setup

package_name = 'map_processor'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),

        # Copy yaml parameters file file
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Raul Castilla Arquillo',
    author_email='raulcastar@uma.es',
    maintainer='Raul Castilla Arquillo',
    maintainer_email='raulcastar@uma.es',
    keywords=['ROS'],
    description=(
        'Python map processor'
    ),
    license='MIT License',
    entry_points={
        'console_scripts': [
            'lc_map_processor = map_processor.map_processor:main',
        ],
    },
)