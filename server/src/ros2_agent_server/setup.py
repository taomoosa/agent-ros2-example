from setuptools import find_packages, setup

# HARDWARE INTEGRATION: install added launch/config files through data_files.
setup(
    name="ros2_agent_server",
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/ros2_agent_server"]),
        ("share/ros2_agent_server", ["package.xml"]),
    ],
    maintainer="agent-ros2-example maintainers",
    maintainer_email="maintainers@example.com",
    description="ROS2 HTTP gateway and robot bridge",
    license="Apache-2.0",
    entry_points={"console_scripts": ["ros2-agent-server = ros2_agent_server.runtime:main"]},
)
