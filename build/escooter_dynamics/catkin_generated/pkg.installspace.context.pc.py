# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;nodelet;geometry_msgs;escooter_msgs;std_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lescooter_dynamics".split(';') if "-lescooter_dynamics" != "" else []
PROJECT_NAME = "escooter_dynamics"
PROJECT_SPACE_DIR = "/home/saun/E-scooter_ws/install"
PROJECT_VERSION = "0.0.0"
