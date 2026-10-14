from setuptools import find_packages, setup

package_name = "handover_fsm"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "handover_fsm_node = handover_fsm.node:main",
            "approach_controller = handover_fsm.approach_controller:main",
        ],
    },
)

