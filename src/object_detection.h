#ifndef OBJECT_DETECTION_H
#define OBJECT_DETECTION_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h> // To use getYaw function from the quaternion of orientation
#include <tf2/transform_datatypes.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <mutex>
#include <string>

struct RangePoint {
    double angle; //!< Angle of the range point.
    double range; //!< Range of the range point.
};

struct Segment {
    geometry_msgs::msg::Point start;        //!< Start point of the segment.
    geometry_msgs::msg::Point end;          //!< End point of the segment.
    std::vector<RangePoint> ranges;         //!< Vector of range points in the segment.
};

struct ObjectStats{
    std::vector<geometry_msgs::msg::Point> midpoints; //!< Midpoints of detected objects.
    geometry_msgs::msg::Point midpoint;               //!< Midpoint of the object.
    std::vector<double> width;                        //!< Widths of detected objects.
};

class ObjectDetection : public rclcpp::Node {
public:
    ObjectDetection();

    std::vector<Segment> detectSegments(sensor_msgs::msg::LaserScan laserScan, 
                                        geometry_msgs::msg::PoseWithCovarianceStamped amcl_pose);

    bool detectObjects(std::vector<Segment> segments, std::vector<ObjectStats>& objects);

    bool checkCircularity(const Segment& segment, 
                        const geometry_msgs::msg::Point& midpoint, double threshold);

    geometry_msgs::msg::Point polarToCart(float range, float angle);

    double calculateDistance(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2);

    geometry_msgs::msg::Point localToGlobal(geometry_msgs::msg::Point point, 
                                            geometry_msgs::msg::PoseWithCovarianceStamped amcl_pose);

    double angleConnectingPoints(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2);

    visualization_msgs::msg::Marker produceMarkerPerson(geometry_msgs::msg::Point pt);

    cv::Point worldToPixel(double wx, double wy, double centerX, double centerY, double scale);

private:

    // Laser data
    sensor_msgs::msg::LaserScan laserScan_;

    // AMCL data
    geometry_msgs::msg::PoseWithCovarianceStamped amcl_;
    double amclPose_x_ = 0.0;
    double amclPose_y_ = 0.0;
    double amclYaw_ = 0.0;

    // Flag to check if AMCL data is initialized
    bool amcl_initialized_;

    // Mutexes
    std::mutex amclMutex_;
    std::mutex laserMutex_;

    unsigned int objectCount_ = 0;
    
};

#endif // OBJECT_DETECTION_H
