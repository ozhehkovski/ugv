from setuptools import find_packages, setup

package_name = "ugv_mapping"

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
    description="Persistent SLAM maps (map manager) and the accessible-terrain layer.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "map_manager = ugv_mapping.map_manager_node:main",
            "accessibility = ugv_mapping.accessibility_node:main",
            "explorer = ugv_mapping.explorer_node:main",
        ],
    },
)
