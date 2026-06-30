import os
from glob import glob
from setuptools import setup

package_name = "tracknetv3"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "*.yaml"))),
    ],
    install_requires=[
        "setuptools",
        "numpy",
        "opencv-python",
        "Pillow",
        "torch",
    ],
    zip_safe=True,
    maintainer="watonomous",
    maintainer_email="watonomous@uwaterloo.ca",
    description="TrackNetV3 shuttle trajectory tracking pipeline for humanoid robot",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tracknetv3_node = tracknetv3.tracknetv3_node:main",
        ],
    },
)
