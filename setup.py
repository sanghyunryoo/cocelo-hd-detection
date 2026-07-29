from setuptools import find_packages, setup

package_name = "weldline_goal_publisher"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/yolo_weldline_3d.launch.xml"]),
        ("share/" + package_name + "/config", ["config/yolo_weldline_3d.yaml"]),
        ("share/" + package_name + "/weights", ["weights/best.onnx"]),
        ("lib/" + package_name, ["launch.sh"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Cocelo Engineering",
    maintainer_email="engineering@cocelo.ai",
    description="RealSense RGB-D weld-line localization to Nav2 PoseStamped goals.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "weldline_goal_node = weldline_goal_publisher.yolo_node:main",
        ],
    },
)
