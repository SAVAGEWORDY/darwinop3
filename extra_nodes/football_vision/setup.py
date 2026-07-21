import os
from glob import glob
from setuptools import find_packages, setup

package_name = "football_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "models"), glob("models/*.pt")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robotis",
    maintainer_email="glazov.y0@gmail.com",
    description="YOLO-детектор объектов футбольного поля для OP3.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vision_node = football_vision.vision_node:main",
        ],
    },
)
