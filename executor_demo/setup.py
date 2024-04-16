from setuptools import setup

package_name = 'executor_demo'

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
            "dual_mutex_demo = executor_demo.executor_demo_interface:main_dual_mutex",
            "mutex_reentrant_demo = executor_demo.executor_demo_interface:main_mutex_reentrant",
            "reentrant_demo = executor_demo.executor_demo_interface:main_reentrant",
            "single_threaded_demo = executor_demo.executor_demo_interface:main_singlethreaded",
        ],
    },
)
