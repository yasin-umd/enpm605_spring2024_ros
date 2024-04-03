import os
from glob import glob
from setuptools import setup

package_name = 'first_pkg'

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
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
            "minimal_node = first_pkg.minimal_node_demo:main",
            "advanced_node = first_pkg.advanced_node_demo:main",
            "qos_publisher_demo = first_pkg.qos_publisher_demo:main",
            "qos_subscriber_demo = first_pkg.qos_subscriber_demo:main",
        ],
    },
)
