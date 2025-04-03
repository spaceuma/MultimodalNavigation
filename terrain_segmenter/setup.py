import os
from glob import glob
from setuptools import find_packages
from setuptools import setup

package_name = 'terrain_segmenter'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        
        # Copy needed omniunet file to lib folder
        (os.path.join('lib', package_name, 'lib_omniunet/model'), glob('lib_omniunet/model/*')),
        (os.path.join('lib', package_name, 'lib_omniunet/utils'), glob('lib_omniunet/utils/*')),
        (os.path.join('lib', package_name, 'lib_omniunet'), glob('src/__init__.py')),

        # Copy network weights
        (os.path.join('share', package_name, 'network_weights'), glob('network_weights/*')),

        # Copy class definition file
        (os.path.join('share', package_name, 'config'), glob('config/*.txt')),

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
        'Python terrain segmenter node'
    ),
    license='MIT License',
    entry_points={
        'console_scripts': [
            'lc_terrain_segmenter = terrain_segmenter.terrain_segmenter:main',
        ],
    },
)