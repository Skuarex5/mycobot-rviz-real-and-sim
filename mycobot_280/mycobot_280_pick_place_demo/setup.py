from glob import glob
import os

from setuptools import setup

package_name = "mycobot_280_pick_place_demo"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="UruBots",
    maintainer_email="urubots@todo.todo",
    description="Simulation-only sequential pick-and-place motion demonstration for the Elephant Robotics myCobot 280 M5.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pick_place_demo_node = mycobot_280_pick_place_demo.pick_place_node:main",
        ],
    },
)
