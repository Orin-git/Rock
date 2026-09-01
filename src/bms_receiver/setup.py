from setuptools import find_packages, setup


package_name = "bms_receiver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sunrise",
    maintainer_email="sunrise@todo.todo",
    description="Decode battery telemetry forwarded by the chassis controller.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "bms_receiver_node = bms_receiver.node:main",
        ],
    },
)
