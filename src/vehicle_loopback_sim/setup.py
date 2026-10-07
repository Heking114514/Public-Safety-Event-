from glob import glob
from os.path import join

from setuptools import find_packages, setup


package_name = "vehicle_loopback_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (join("share", package_name, "config"), glob("config/*.yaml")),
        (join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (join("share", package_name, "web"), glob("vehicle_loopback_sim/web/*.html")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="hjh",
    maintainer_email="hjh@example.com",
    description="Truth-odometry loopback simulator for planner/controller tests.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "loopback_mission_node = vehicle_loopback_sim.mission:main",
            "loopback_route_controller = vehicle_loopback_sim.controller:main",
            "vehicle_loopback_gui_node = vehicle_loopback_sim.gui:main",
            "vehicle_loopback_sim_node = vehicle_loopback_sim.node:main",
        ],
    },
)
