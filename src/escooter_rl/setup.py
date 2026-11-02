# Copyright (c) 2026, Autonomous E-Scooter Simulation Project.
# All rights reserved.

from setuptools import find_packages, setup

setup(
    name="escooter_rl",
    version="0.1.0",
    packages=find_packages(),
    author="Hong-Son Nguyen",
    description="Isaac Lab RL environment for self-balancing and goal navigation of autonomous E-Scooter",
    install_requires=[
        "torch",
        "numpy",
    ],
)
