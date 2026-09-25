from setuptools import find_packages, setup

package_name = "ugv_webui"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/static", ["static/index.html"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ozhehkovski",
    maintainer_email="ozhehkovski.e@gmail.com",
    description="Operator web panel: map, camera, joystick, emergency stop.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["webui = ugv_webui.webui_node:main"]},
)
