# setup.py
from setuptools import setup, find_packages

setup(
    name="altamus_py",
    version="0.1",
    packages=["altamus_py"],
    install_requires=[
        'pypcd4',
        'numpy',
        'simplejson',
        'open3d-cpu',
    ]
)
