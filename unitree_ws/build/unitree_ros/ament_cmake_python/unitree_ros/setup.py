from setuptools import find_packages
from setuptools import setup

setup(
    name='unitree_ros',
    version='1.2.0',
    packages=find_packages(
        include=('unitree_ros', 'unitree_ros.*')),
)
