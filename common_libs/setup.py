import os
from glob import glob
from setuptools import find_packages
from setuptools import setup

package_name = 'common_libs'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    install_requires=['setuptools'],
    zip_safe=True,
    author='Raul Castilla Arquillo',
    author_email='raulcastar@uma.es',
    maintainer='Raul Castilla Arquillo',
    maintainer_email='raulcastar@uma.es',
    description=(
        'Python common libs'
    ),
    license='MIT License',
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
)