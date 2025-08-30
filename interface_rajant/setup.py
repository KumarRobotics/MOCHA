from setuptools import setup

package_name = "interface_rajant"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/rajant_nodes.launch.py"]),
        ("share/" + package_name + "/thirdParty", ["thirdParty/PeerRSSI-bcapi-11.26.1.jar"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Fernando Cladera",
    maintainer_email="fclad@seas.upenn.edu",
    description="The interface_rajant package",
    license="BSD 3-Clause License",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rajant_peer_rssi = interface_rajant.rajant_peer_rssi:main",
        ],
    },
)
