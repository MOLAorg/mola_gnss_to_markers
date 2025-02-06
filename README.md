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
