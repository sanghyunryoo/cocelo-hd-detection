from setuptools import find_packages, setup

package_name = "weldline_reflectivity_detector"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/yolo_weldline_3d.launch.xml", "launch/scenario_commander.launch.xml",
        ]),
        ("share/" + package_name + "/config", [
            "config/yolo_weldline_3d.yaml", "config/scenario.yaml",
        ]),
        ("share/" + package_name + "/weights", ["weights/best.onnx", "weights/person_yolov5n.onnx"]),
        ("lib/" + package_name, [
            "launch.sh", "scripts/annotated_image_viewer.py", "scripts/realsense_visualize.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Cocelo Engineering",
    maintainer_email="engineering@cocelo.ai",
    description="Python ROS 2 weld-line localization and scenario commander.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "yolo_weldline_3d_node = weldline_reflectivity_detector.yolo_node:main",
            "scenario_commander_node = weldline_reflectivity_detector.commander_node:main",
        ],
    },
)
