from setuptools import find_packages, setup

package_name = "ugv_follow"

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
    description="Follow-me: YOLO (TensorRT) person detection + lidar ranging, single-target tracker, 40 cm gap.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["follow = ugv_follow.follow_node:main"]},
)
