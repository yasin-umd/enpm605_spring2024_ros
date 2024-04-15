from setuptools import setup

package_name = 'interface_demo'

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
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
            "message_demo = interface_demo.message_demo_interface:main",
            "server_demo = interface_demo.server_demo_interface:main",
            "client_demo = interface_demo.client_demo_interface:main",
        ],
    },
)
