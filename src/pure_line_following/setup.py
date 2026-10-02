from setuptools import find_packages, setup

package_name = "pure_line_following"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/pure_line_following.launch.py",
                "launch/pure_line_following_serial.launch.py",
            ],
        ),
        (
            "share/" + package_name + "/config",
            ["config/pure_line_following.yaml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Odometry-independent white-track line following.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pure_line_following_node = pure_line_following.node:main",
        ],
    },
)
