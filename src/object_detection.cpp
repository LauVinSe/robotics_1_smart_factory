#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <cmath>
#include <vector>

struct Segment
{
    geometry_msgs::msg::Point start;
    geometry_msgs::msg::Point end;
    std::vector<geometry_msgs::msg::Point> points;  // All points in the segment
};

class ObjectDetection : public rclcpp::Node
{
public:
    ObjectDetection() : Node("object_detection_node")
    {
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ObjectDetection::laserCallback, this, std::placeholders::_1));
        
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/cylinder_marker", 10);

        geometry_msgs::msg::Point cylinder_location;

        RCLCPP_INFO(this->get_logger(), "Object detection node has started.");

    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Process laser scan to detect cylindrical object
        bool detected_cylinder = detectCylinder(*msg, cylinder_location_);

        // If a cylinder is detected, publish a marker in RViz
        if (detected_cylinder)
        {
            RCLCPP_INFO(this->get_logger(), "Cylinder detected at x: %.2f, y: %.2f", cylinder_location_.x, cylinder_location_.y);

            // Publish a marker in RViz
            visualization_msgs::msg::Marker marker = createCylinderMarker(cylinder_location_);
            marker_pub_->publish(marker);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No cylinder detected in this laser scan.");
        }
    }

    bool detectCylinder(const sensor_msgs::msg::LaserScan &scan, geometry_msgs::msg::Point &cylinder_location)
    {
        // This function processes the laser scan to detect a cylindrical object
        double cylinder_diameter = 0.3;  // 30cm diameter
        double radius = cylinder_diameter / 2.0;
        double tolerance = 0.05;  // Tolerance for detecting the cylinder shape
        bool cylinder_detected = false;

        // Loop through the laser scan ranges
        for (size_t i = 0; i < scan.ranges.size(); ++i)
        {
            double range = scan.ranges[i];
            if (std::isnan(range) || range > scan.range_max || range < scan.range_min)
            {
                continue;  // Skip invalid or out of range values
            }

            double angle = scan.angle_min + i * scan.angle_increment;

            // Compute coordinates (x, y) in Cartesian space
            double x = range * std::cos(angle);
            double y = range * std::sin(angle);

            // Check for objects with the approximate cylindrical shape (30 cm in diameter)
            if (std::abs(range - radius) <= tolerance)
            {
                // Found a possible cylindrical object
                RCLCPP_INFO(this->get_logger(), "Candidate cylinder found at x: %.2f, y: %.2f, range: %.2f", x, y, range);
                cylinder_location.x = x;
                cylinder_location.y = y;
                cylinder_location.z = 0.0;  // Assuming object is on the ground
                cylinder_detected = true;
                return cylinder_detected;
            }
        }

        // If no object is found, return an empty point (default 0.0)
        return cylinder_detected;
    }

    visualization_msgs::msg::Marker createCylinderMarker(const geometry_msgs::msg::Point &cylinder_location)
    {
        visualization_msgs::msg::Marker marker;
        
        marker.header.frame_id = "map";  // Assuming map frame
        marker.header.stamp = this->now();

        // Set marker properties
        marker.ns = "cylinder";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = cylinder_location;
        marker.pose.orientation.w = 1.0;  // No rotation
        marker.scale.x = 0.3;  // Diameter 30 cm
        marker.scale.y = 0.3;  // Diameter 30 cm
        marker.scale.z = 1.0;  // Set height of cylinder (can be adjusted)

        // Set color (e.g., green)
        marker.color.a = 1.0;  // Alpha (fully opaque)
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        return marker;
    }

    double angleConnectingPoints(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2)
    {
    return atan2(p2.y - p1.y, p2.x - p1.x);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    geometry_msgs::msg::Point cylinder_location_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectDetection>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
