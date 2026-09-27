from setuptools import find_packages, setup

package_name = "rectangle_odometry_test"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/rectangle_test.launch.py",
                "launch/primitive_rectangle_test.launch.py",
            ],
        ),
        (
            "share/" + package_name + "/config",
            ["config/slow_arc_navigation.yaml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Closed-loop rectangular motion tests with waypoint and primitive controllers.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "rectangle_route_node = rectangle_odometry_test.rectangle_route_node:main",
            "primitive_controller_node = rectangle_odometry_test.primitive_node:main",
        ],
    },
)
