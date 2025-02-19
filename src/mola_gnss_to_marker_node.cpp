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

        declare_parameter<std::string>("input_topic_gps", input_topic_gps_);
        get_parameter("input_topic_gps", input_topic_gps_);
        RCLCPP_INFO(this->get_logger(), "input_topic_gps: %s", input_topic_gps_.c_str());

        declare_parameter<std::string>("input_topic_georef_metadata", input_topic_georef_metadata_);
        get_parameter("input_topic_georef_metadata", input_topic_georef_metadata_);
        RCLCPP_INFO(
            this->get_logger(), "input_topic_georef_metadata: %s",
            input_topic_georef_metadata_.c_str());

        declare_parameter<std::string>("output_topic_marker", output_topic_marker_);
        get_parameter("output_topic_marker", output_topic_marker_);
        RCLCPP_INFO(this->get_logger(), "output_topic_marker: %s", output_topic_marker_.c_str());

        declare_parameter<double>("output_marker_line_width", output_marker_line_width_);
        get_parameter("output_marker_line_width", output_marker_line_width_);
        RCLCPP_INFO(
            this->get_logger(), "output_marker_line_width: %0.3f", output_marker_line_width_);

        declare_parameter<std::vector<double>>("output_marker_color", output_marker_color_);
        get_parameter("output_marker_color", output_marker_color_);
        RCLCPP_INFO(
            this->get_logger(), "output_marker_color: {%0.3f, %0.3f, %0.3f, %0.3f}",
            output_marker_color_[0], output_marker_color_[1], output_marker_color_[2],
            output_marker_color_[3]);

        // REP-2003: https://ros.org/reps/rep-2003.html#id5
        // - Maps:  reliable transient-local
        const rclcpp::QoS mapQos    = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        const rclcpp::QoS sensorQos = rclcpp::SensorDataQoS();
        const rclcpp::QoS defaultQos = rclcpp::SystemDefaultsQoS();

        gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            input_topic_gps_, sensorQos,
            std::bind(&MolaGnssToMarkerNode::gps_callback, this, std::placeholders::_1));

        geo_metadata_subscription_ =
            this->create_subscription<mrpt_nav_interfaces::msg::GeoreferencingMetadata>(
                input_topic_georef_metadata_, mapQos,
                std::bind(
                    &MolaGnssToMarkerNode::geo_metadata_callback, this, std::placeholders::_1));

        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
            output_topic_marker_, defaultQos);
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

        marker.scale.x = output_marker_line_width_;

        marker.color.a = output_marker_color_[3];
        marker.color.r = output_marker_color_[0];
        marker.color.g = output_marker_color_[1];
        marker.color.b = output_marker_color_[2];

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

    std::string         input_topic_gps_             = "/gps";
    std::string         input_topic_georef_metadata_ = "/lidar_odometry/geo_ref_metadata";
    std::string         output_topic_marker_         = "/gnss_georef_marker";
    double              output_marker_line_width_    = 0.3;
    std::vector<double> output_marker_color_         = {0.0, 1.0, 0.0, 0.6};

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
