from setuptools import setup

package_name = "mocha_core"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="developer",
    maintainer_email="fclad@seas.upenn.edu",
    description="DCIST Distributed Database package",
    license="BSD 3-Clause License",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mocha = mocha_core.mocha:main",
            "database_server = mocha_core.database_server:main",
            "synchronize_channel = mocha_core.synchronize_channel:main",
            "zmq_comm_node = mocha_core.zmq_comm_node:main",
            "topic_publisher = mocha_core.topic_publisher:main",
            "translator = mocha_core.translator:main",
        ],
    },
)
