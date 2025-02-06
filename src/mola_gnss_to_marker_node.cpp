/* -------------------------------------------------------------------------
 * mola_gnss_to_markers
 *
 * Copyright (C) 2025 Jose Luis Blanco (University of Almeria)
 *                    and contributors.
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
#include <mrpt/topography/conversions.h>

#include <cmath>

#include "mrpt_nav_interfaces/msg/georeferencing_metadata.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "visualization_msgs/msg/marker.hpp"

class MolaGnssToMarkerNode : public rclcpp::Node
{
   public:
    constexpr static std::size_t      MARKER_NUM_POINTS = 30;
    constexpr static std::string_view ENU_FRAME_ID      = "enu";

    MolaGnssToMarkerNode() : Node("mola_gnss_to_marker_node")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing GNSS to Marker Node");

        // REP-2003: https://ros.org/reps/rep-2003.html#id5
        // - Maps:  reliable transient-local
        const rclcpp::QoS mapQos    = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        const rclcpp::QoS sensorQos = rclcpp::SensorDataQoS();
        const rclcpp::QoS defaultQos = rclcpp::SystemDefaultsQoS();

        gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps", sensorQos,
            std::bind(&MolaGnssToMarkerNode::gps_callback, this, std::placeholders::_1));

        geo_metadata_subscription_ =
            this->create_subscription<mrpt_nav_interfaces::msg::GeoreferencingMetadata>(
                "/lidar_odometry/geo_ref_metadata", mapQos,
                std::bind(
                    &MolaGnssToMarkerNode::geo_metadata_callback, this, std::placeholders::_1));

        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/gnns_georef_marker", defaultQos);
    }

   private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr gps)
    {
        if (!georef_metadata_)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000000000,
                "Ignoring GPS reading: no map georeferenced metadata yet...");
            return;
        }
        if (gps->position_covariance_type == 0 ||
            gps->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000000000,
                "Ignoring GPS reading: no valid position fix yet, or invalid covariance...");
            return;
        }

        if (gps->position_covariance.size() != 9)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000000000,
                "Ignoring GPS reading: covariance is not 3x3 (!)");
            return;
        }

        // Do NOT publish the marker in UTM frame. It does not have float point precision enough.
        // Let's use ENU instead:

        const mrpt::topography::TGeodeticCoords gpsCoords = {
            gps->latitude, gps->longitude, gps->altitude};

        const mrpt::topography::TGeodeticCoords enuRefCoords = {
            georef_metadata_->latitude, georef_metadata_->longitude, georef_metadata_->height};

        mrpt::math::TPoint3D enu;

        mrpt::topography::geodeticToENU_WGS84(gpsCoords, enu, enuRefCoords);

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = ENU_FRAME_ID;
        marker.header.stamp    = this->now();
        marker.ns              = "gnss_markers";
        marker.id              = 0;
        marker.type            = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action          = visualization_msgs::msg::Marker::ADD;

        // Generate ellipsoid with 2*sigmas Confidence Interval:
        const double sigma_east  = std::sqrt(gps->position_covariance.at(0));
        const double sigma_north = std::sqrt(gps->position_covariance.at(4));

        for (std::size_t i = 0; i < MARKER_NUM_POINTS; i++)
        {
            const double ang =
                static_cast<double>(i) / static_cast<double>(MARKER_NUM_POINTS - 1) * 2 * M_PI;

            const double lx = sigma_east * std::cos(ang);
            const double ly = sigma_north * std::sin(ang);

            geometry_msgs::msg::Point p;
            p.x = enu.x + lx;
            p.y = enu.y + ly;
            p.z = enu.z + 0.0;
            marker.points.push_back(p);
        }

        marker.scale.x = 0.3;  // Line width

        marker.color.a = 0.6;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker_publisher_->publish(marker);
    }

    void geo_metadata_callback(
        const mrpt_nav_interfaces::msg::GeoreferencingMetadata::SharedPtr msg)
    {
        if (!msg->valid)
        {
            RCLCPP_WARN(this->get_logger(), "Ignoring georeferencing metadata since 'valid'=false");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Received valid georeferencing metadata");

        // Store metadata for future processing:
        georef_metadata_ = *msg;
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr  gps_subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::Subscription<mrpt_nav_interfaces::msg::GeoreferencingMetadata>::SharedPtr
        geo_metadata_subscription_;

    std::optional<mrpt_nav_interfaces::msg::GeoreferencingMetadata> georef_metadata_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MolaGnssToMarkerNode>());
    rclcpp::shutdown();
    return 0;
}
