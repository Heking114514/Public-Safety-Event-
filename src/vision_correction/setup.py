from setuptools import find_packages, setup

package_name = "vision_correction"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/vision_correction.launch.py",
            "launch/vision_correction_navigation.launch.py",
        ]),
        ("share/" + package_name + "/config", ["config/vision_correction.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Realtime RGB landmark correction for fused planar odometry.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "vision_correction_node = vision_correction.node:main",
        ],
    },
)
