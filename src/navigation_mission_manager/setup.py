from glob import glob
from os.path import join

from setuptools import find_packages, setup


package_name = "navigation_mission_manager"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md", "CHANGELOG.md"]),
        (join("share", package_name, "config"), glob("config/*.yaml")),
        (join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="maintainer",
    maintainer_email="maintainer@example.com",
    description="Autonomous navigation mission manager.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mission_manager_node = navigation_mission_manager.node:main",
            "reporter_stub_node = navigation_mission_manager.reporter_stub:main",
        ],
    },
)
