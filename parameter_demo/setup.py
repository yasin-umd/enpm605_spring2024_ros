from setuptools import find_packages, setup
import os
from glob import glob

package_name = "parameter_demo"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob('launch/*.launch.py')),
        (os.path.join("share", package_name, "config"), glob('config/*.yaml')),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zeid",
    maintainer_email="zeidk@umd.edu",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "param_demo = parameter_demo.parameter_demo_interface:main",
        ],
    },
)
