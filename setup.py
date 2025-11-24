# Tesi_Gemini_Robotics/setup.py
from setuptools import setup, find_packages

setup(
    name='tesi_gemini_robotics',
    version='0.2.0',
    package_dir={'': 'src'},
    packages=find_packages(where='src'),
)