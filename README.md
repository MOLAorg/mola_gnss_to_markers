[![CI clang-format](https://github.com/MOLAorg/mola_gnss_to_markers/actions/workflows/check-clang-format.yml/badge.svg)](https://github.com/MOLAorg/mola_gnss_to_markers/actions/workflows/check-clang-format.yml)
[![CI Build colcon](https://github.com/MOLAorg/mola_gnss_to_markers/actions/workflows/build-ros.yml/badge.svg)](https://github.com/MOLAorg/mola_gnss_to_markers/actions/workflows/build-ros.yml)


| Distro | Build dev | Release |
| --- | --- | --- |
| ROS 2 Humble (u22.04) | [![Build Status](https://build.ros2.org/job/Hdev__mola_gnss_to_markers__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mola_gnss_to_markers__ubuntu_jammy_amd64/) | [![Version](https://img.shields.io/ros/v/humble/mola_gnss_to_markers)](https://index.ros.org/search/?term=mola_gnss_to_markers) |
| ROS 2 Jazzy (u24.04) | [![Build Status](https://build.ros2.org/job/Jdev__mola_gnss_to_markers__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__mola_gnss_to_markers__ubuntu_noble_amd64/) | [![Version](https://img.shields.io/ros/v/jazzy/mola_gnss_to_markers)](https://index.ros.org/search/?term=mola_gnss_to_markers) |
| ROS 2 Rolling (u24.04) | [![Build Status](https://build.ros2.org/job/Rdev__mola_gnss_to_markers__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mola_gnss_to_markers__ubuntu_noble_amd64/) | [![Version](https://img.shields.io/ros/v/rolling/mola_gnss_to_markers)](https://index.ros.org/search/?term=mola_gnss_to_markers) |


# mola_gnss_to_markers

Takes GNSS (GPS) readings, a MOLA georeferenced map, and publishes markers to visualize the datums as ellipsoids on the map.

## Usage

    ros2 launch mola_gnss_to_markers mola_gnss_to_markers_launch.py

## Input topics:

* ``/lidar_odometry/geo_ref_metadata`` (``mrpt_nav_interfaces/msg/GeoreferencingMetadata``)
* ``/gps``  (``sensor_msgs/msg/NavSatFix``)

## Output topics:

* ``gnns_georef_marker``  (``visualization_msgs/msg/Marker``)


## License

BSD-3-Clause
