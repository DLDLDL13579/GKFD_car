from setuptools import find_packages, setup
import os
from glob import glob

package_name = "mqtt_bridge_ros2"

# Use __file__ relative path so glob resolves against the source directory
_launch_dir = os.path.join(os.path.dirname(__file__), "launch")

setup(
    name=package_name,
    version="0.0.2",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join(_launch_dir, "**.py")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nvidia",
    maintainer_email="nvidia@wheeltec.com",
    description="MQTT-ROS2 bridge + System Manager + WebRTC pusher",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "system_manager = mqtt_bridge_ros2.system_manager:main",
            "webrtc_pusher = mqtt_bridge_ros2.webrtc_pusher:main",
            "ws_mqtt_proxy = mqtt_bridge_ros2.ws_mqtt_proxy:main",
        ],
    },
)
