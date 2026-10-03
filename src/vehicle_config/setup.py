from setuptools import find_packages, setup


package_name = "vehicle_config"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (
            "share/" + package_name + "/config",
            ["config/vehicle_physical.yaml"],
        ),
    ],
    install_requires=["setuptools", "PyYAML"],
    tests_require=["pytest"],
    zip_safe=True,
    description="Shared physical configuration for the vehicle.",
    license="Apache-2.0",
)
