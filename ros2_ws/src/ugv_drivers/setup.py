from setuptools import find_packages, setup

package_name = "ugv_drivers"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ozhehkovski",
    maintainer_email="ozhehkovski.e@gmail.com",
    description="UGV hardware drivers: VESC diff drive, RPLIDAR, BNO085, camera, cmd mux, footprint.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vesc_driver = ugv_drivers.vesc_driver_node:main",
            "rplidar = ugv_drivers.rplidar_node:main",
            "bno085 = ugv_drivers.bno085_node:main",
            "camera = ugv_drivers.camera_node:main",
            "cmd_mux = ugv_drivers.cmd_mux_node:main",
            "footprint_publisher = ugv_drivers.footprint_node:main",
            "safety_governor = ugv_drivers.safety_governor_node:main",
        ],
    },
)
