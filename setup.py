from setuptools import setup, find_packages

setup(
    name="vlm_agent",
    version="1.1.0",
    description="Neuro-symbolic architecture for robotic manipulation using VLMs and PDDL",
    author="William Notaro",
    license="MIT",
    package_dir={"": "src"},
    packages=find_packages(where="src"),
    python_requires=">=3.8",
    install_requires=[
        "colorlog>=6.10.0",
        "coppeliasim-zmqremoteapi-client>=2.0.0",
        "matplotlib>=3.7.0",
        "numpy>=1.24.0",
        "open3d>=0.19.0",
        "opencv-python>=4.8.0",
        "Pillow>=10.0.0",
        "protobuf>=4.25.0",
        "python-dotenv>=1.0.0",
        "unified-planning>=1.3.0",
    ],
)