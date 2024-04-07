import setuptools
import os

with open("README.md", "r") as fh:
    long_description = fh.read()

with open('requirements.txt') as f:
    required = f.read().splitlines()

setuptools.setup(
    name="mavlinkHandler",
    version="0.0.3",
    author="Nurullah Eren Acar",
    author_email="n.erenacar13@gmail.com",
    description="A controller library for UAVs, compatible with both ArduPilot and DroneKit",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/0EA/mavlinkHandler",
    packages=setuptools.find_packages(),
    install_requires=required,
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
        "Operating System :: OS Independent",
    ],
)