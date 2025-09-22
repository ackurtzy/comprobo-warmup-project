from setuptools import find_packages, setup

package_name = "ros_behaviors_fsm"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="swisnoski",
    maintainer_email="swisnoski@olin.edu",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "teleop = ros_behaviors_fsm.teleop:main",
            "letterbox = ros_behaviors_fsm.letterbox:main",
            "wall_follower = ros_behaviors_fsm.wall_follower:main",
            "drive_shape = ros_behaviors_fsm.drive_shape:main",
            "finite_state_machine = ros_behaviors_fsm/finite_state_machine:main"
        ],
    },
)
