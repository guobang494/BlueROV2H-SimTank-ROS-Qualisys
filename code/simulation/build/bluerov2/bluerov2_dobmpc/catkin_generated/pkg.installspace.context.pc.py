# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "gazebo_msgs;message_generation;message_runtime".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lbluerov2_dobmpc;-lbluerov2_ampc".split(';') if "-lbluerov2_dobmpc;-lbluerov2_ampc" != "" else []
PROJECT_NAME = "bluerov2_dobmpc"
PROJECT_SPACE_DIR = "/root/catkin_ws/Bluerov2-Simulation-with-docker-env/install"
PROJECT_VERSION = "0.0.0"
