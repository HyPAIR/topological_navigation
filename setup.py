from setuptools import find_packages, setup
from glob import glob
import os

package_name = "topological_navigation"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools", "shapely", "yaml"],
    zip_safe=True,
    maintainer="Charlie Street",
    maintainer_email="me@charliestreet.net",
    description="Simple topological localisation and edge navigation.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "topological_localisation = topological_navigation.topological_localisation:main",
            "topological_map_visualiser = topological_navigation.topological_map_visualiser:main",
            "edge_navigation = topological_navigation.edge_navigation:main",
        ],
    },
)
