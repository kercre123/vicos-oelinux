DESCRIPTION = "ros-eagle package group"
LICENSE = "MIT"

inherit packagegroup

PACKAGES = "${PN}"

RDEPENDS_${PN} = "\
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-packagegroup-ros-comm', 'packagegroup-ros-comm', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-actionlib', 'actionlib', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-bond', 'bond', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-bondpy', 'bondpy', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-bondcpp', 'bondcpp', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-smclib', 'smclib', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-class-loader', 'class-loader', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-actionlib-msgs', 'actionlib-msgs', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-diagnostic-msgs', 'diagnostic-msgs', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-nav-msgs', 'nav-msgs', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-geometry-msgs', 'geometry-msgs', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-sensor-msgs', 'sensor-msgs', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-shape-msgs', 'shape-msgs', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-stereo-msgs', 'stereo-msgs', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-trajectory-msgs', 'trajectory-msgs', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-visualization-msgs', 'visualization-msgs', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-dynamic-reconfigure', 'dynamic-reconfigure', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-tf2', 'tf2', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-tf2-msgs', 'tf2-msgs', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-tf2-py', 'tf2-py', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-tf2-ros', 'tf2-ros', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-tf', 'tf', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-image-transport', 'image-transport', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-nodelet-topic-tools', 'nodelet-topic-tools', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-nodelet', 'nodelet', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-pluginlib', 'pluginlib', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-cmake-modules', 'cmake-modules', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-rosconsole-bridge', 'rosconsole-bridge', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-ros-scripts', 'ros-scripts', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-camera-info-manager', 'camera-info-manager', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-python-rosdep', 'python-rosdep', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-python-json', 'python-json', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-git', 'git', d)} \
    ${@bb.utils.contains('DISTRO_FEATURES', 'ROS-32', 'lib32-python-empy', 'python-empy', d)} \
"  
