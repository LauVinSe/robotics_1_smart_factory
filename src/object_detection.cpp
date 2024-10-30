#include "object_detection.h"

/**
 * \class     ObjectDetection
 * \brief     A ROS2 node for detecting objects using laser scan and AMCL data.
 * \details   This class subscribes to laser scan and AMCL pose topics to detect objects in the environment. 
 *            It processes the data to identify segments, converts them into objects, and publishes the 
 *            detected objects as visualization markers. The detected objects can also be drawn on an image 
 *            for visualization purposes.
 * \author    Lauretius Vincent Setiadharma
 * \version   1.0.0
 * \date      2024-10-10
 * \pre       The system should have a properly configured laser scanner and AMCL localization running.
 * \bug       None reported as of 2024-10-10
 * \warning   None
 */

ObjectDetection::ObjectDetection()
{

}

std::vector<Segment> ObjectDetection::detectSegments(sensor_msgs::msg::LaserScan laserScan, 
                                                    geometry_msgs::msg::PoseWithCovarianceStamped amcl_pose)
{
    /**
     * @brief Detect segments in the laser scan data.
     * 
     * This function processes the laser scan data to detect segments based on the distance between points.
     * It identifies segments based on the distance threshold and minimum number of points required.
     * 
     * @return std::vector<Segment> A vector of detected segments.
     */

    // Defining segment threshold
    double pointThreshold = 0.15;          // Maximum distance for a point to be part of a segment
    double maxScanThreshold = 0.6;     // Minimum length of a valid segment
    double minScanThreshold = 0.4;
    unsigned int minPointsPerSegment = 6; // Minimum points required in a segment
    bool isPartOfSegment = false;         // Flag to check if a point is part of a segment

    // Initialize variables
    std::vector<Segment> segments;
    Segment currentSegment;

    // Ensure we have valid laser data
    if (laserScan.ranges.empty()) {
        RCLCPP_WARN(this->get_logger(), "Laser scan data is empty.");
        return segments;
    }

    // Get the first point from the laser scan and convert it to Cartesian coordinates
    geometry_msgs::msg::Point prevPoint = polarToCart(laserScan.ranges.at(0), laserScan.angle_min);

    // Loop through the laser scan data
    for (unsigned int i = 1; i < laserScan.ranges.size(); ++i)
    {
        float currentRange = laserScan.ranges.at(i);
        float currentAngle = laserScan.angle_min + i * laserScan.angle_increment;

        // Check if the range is a valid number
        if (std::isnan(currentRange) || currentRange > maxScanThreshold || currentRange < minScanThreshold)
        {
            if (isPartOfSegment)
            {
                // If current point is part of a segment, finalize it if it has enough points
                if (currentSegment.ranges.size() >= minPointsPerSegment)
                {
                    currentSegment.end = prevPoint;
                    segments.push_back(currentSegment);

                    // Convert to global coordinates
                    currentSegment.start = localToGlobal(currentSegment.start, amcl_pose);
                    currentSegment.end = localToGlobal(currentSegment.end, amcl_pose);
                }

                // Reset the segment flag
                isPartOfSegment = false;
            }
            continue; // Skip invalid or out-of-range values
        }

        // Convert the current point to Cartesian coordinates
        geometry_msgs::msg::Point currentPoint = polarToCart(currentRange, currentAngle);

        if (!isPartOfSegment)
        {
            // Start a new segment
            currentSegment.start = currentPoint;
            isPartOfSegment = true;
        }
        else
        {
            // Check if the current point is part of the segment
            if (calculateDistance(prevPoint, currentPoint) <= pointThreshold)
            {
                // Add the current point to the segment
                currentSegment.ranges.push_back({currentAngle, currentRange});
            }
            else
            {
                // Finalize the segment if it has enough points
                if (currentSegment.ranges.size() >= minPointsPerSegment)
                {
                    currentSegment.end = prevPoint;
                    segments.push_back(currentSegment);

                    // Convert to global coordinates
                    currentSegment.start = localToGlobal(currentSegment.start, amcl_pose);
                    currentSegment.end = localToGlobal(currentSegment.end, amcl_pose);
                }

                // Start a new segment
                currentSegment = Segment();
                currentSegment.start = currentPoint;
                currentSegment.ranges.push_back({currentAngle, currentRange});
                isPartOfSegment = true;
            }
        }
        prevPoint = currentPoint;
    }

    // Finalize the last segment if it has enough points
    if (isPartOfSegment && currentSegment.ranges.size() >= minPointsPerSegment)
    {
        currentSegment.end = prevPoint;
        currentSegment.start = localToGlobal(currentSegment.start, amcl_pose);
        currentSegment.end = localToGlobal(currentSegment.end, amcl_pose);
        segments.push_back(currentSegment);
    }

    return segments;
}

bool ObjectDetection::detectObjects(std::vector<Segment> segments, std::vector<ObjectStats>& objects) {

    /**
     * @brief Detect objects from the segments.
     * 
     * This function processes the detected segments to identify objects based on their circularity.
     * It calculates the midpoint of each segment and checks if the segments belong to the same object.
     * The function also calculates the average midpoint for each object.
     * 
     * @param segments The vector of detected segments.
     * @param objects The vector of detected objects.
     * @return bool True if objects are detected, false otherwise.
     */

    double width = 0;
    double widthThreshold = 1.0;          // Maximum width for an object segment
    double sameObjectThreshold = 1.0;    // Threshold to determine if segments belong to the same object
    double circularityThreshold = 0.02;  // Threshold for variance in distances to check circularity
    double distance = 0;
    bool foundNewObject = true;
    geometry_msgs::msg::Point midpoint;

    for (auto segment : segments) {
        // Display current segment
        // RCLCPP_INFO_STREAM(this->get_logger(), "Segment start: (" << segment.start.x << ", " << segment.start.y << ") end: (" << segment.end.x << ", " << segment.end.y << ")");

        // Calculate the distance between start and end points of the segment
        width = calculateDistance(segment.start, segment.end);

        // Show the width of the segment
        // RCLCPP_INFO_STREAM(this->get_logger(), "Width: " << width);

        // If the width exceeds the threshold, skip this segment
        if (width >= widthThreshold) {
            continue;
        }

        // Calculate the midpoint of the segment
        midpoint.x = (segment.start.x + segment.end.x) / 2.0;
        midpoint.y = (segment.start.y + segment.end.y) / 2.0;
        midpoint.z = segment.start.z;

        // Show midpoint of the segment
        // RCLCPP_INFO_STREAM(this->get_logger(), "Midpoint: (" << midpoint.x << ", " << midpoint.y << ")");

        // Calculate circularity by checking distances of all points in the segment to the midpoint
        bool isCircular = checkCircularity(segment, midpoint, circularityThreshold);
        // Show if the segment is circular or not
        // RCLCPP_INFO_STREAM(this->get_logger(), "Circular: " << isCircular);
        if (!isCircular) {
            continue; // Skip this segment if it is not circular (i.e., not a cylinder)
        }

        foundNewObject = true;

        // Check if the segment is part of an existing object
        for (auto& object : objects) {
            distance = calculateDistance(object.midpoints.back(), midpoint);

            // If the segment is close to an existing object, add it to that object
            if (distance < sameObjectThreshold) {
                foundNewObject = false;
                // object.width.push_back(width);
                // object.midpoints.push_back(midpoint);
                object.midpoint = midpoint;
                break;
            }
        }

        // If it's a new object, add it to the list of objects
        if (foundNewObject) {
            ObjectStats newObject;
            newObject.midpoints.push_back(midpoint);
            newObject.width.push_back(width);
            newObject.midpoint = midpoint;
            objects.push_back(newObject);
        }
    }

    // Calculate the average midpoint for each object
    for (auto& object : objects) {
        geometry_msgs::msg::Point avgMidPoint;
        avgMidPoint.x = 0.0;
        avgMidPoint.y = 0.0;
        avgMidPoint.z = 0.0;

        // Sum up all the midpoints of the object
        for (auto pt : object.midpoints) {
            avgMidPoint.x += pt.x;
            avgMidPoint.y += pt.y;
            avgMidPoint.z += pt.z;
        }

        // Calculate the average midpoint
        avgMidPoint.x /= object.midpoints.size();
        avgMidPoint.y /= object.midpoints.size();
        avgMidPoint.z /= object.midpoints.size();

        object.midpoint = avgMidPoint;
    }

    // Return true if any objects were detected
    return !objects.empty();
}

// Function to check the circularity of a segment
bool ObjectDetection::checkCircularity(const Segment& segment, const geometry_msgs::msg::Point& midpoint, double threshold) {

    /**
     * @brief Check the circularity of a segment.
     * 
     * This function calculates the distances of all points in the segment from the midpoint.
     * It then calculates the mean distance and variance of the distances to determine circularity.
     * 
     * @param segment The segment to check for circularity.
     * @param midpoint The midpoint of the segment.
     * @param threshold The threshold for variance in distances.
     * @return bool True if the segment is circular, false otherwise.
     */

    std::vector<double> distances;
    double sum = 0.0;

    // Calculate distances of all points in the segment from the midpoint
    for (auto range_point : segment.ranges) {
        geometry_msgs::msg::Point point = polarToCart(range_point.range, range_point.angle);
        double dist = calculateDistance(midpoint, point);
        distances.push_back(dist);
        sum += dist;
    }

    // Calculate the mean distance
    double mean = sum / distances.size();

    // Calculate the variance of the distances
    double variance = 0.0;
    for (auto dist : distances) {
        variance += (dist - mean) * (dist - mean);
    }
    variance /= distances.size();

    // If the variance is below the threshold, the points are roughly equidistant (circular)
    return variance <= threshold;
}


geometry_msgs::msg::Point ObjectDetection::polarToCart(float range, float angle)
{
    /**
     * @brief Convert polar coordinates to Cartesian coordinates.
     * 
     * This function converts polar coordinates (range and angle) to Cartesian coordinates (x, y).
     * 
     * @param range The range value from the laser scan.
     * @param angle The angle value from the laser scan.
     * @return geometry_msgs::msg::Point The Cartesian coordinates (x, y).
     */
    geometry_msgs::msg::Point cart;
    cart.x = range * std::cos(angle);
    cart.y = range * std::sin(angle);
    cart.z = 0.0; // Assuming 2D laser scan
    return cart;
}

double calculateDistance(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2)
{
    /**
     * @brief Calculate the Euclidean distance between two points.
     * 
     * This function calculates the Euclidean distance between two points in 2D space.
     * 
     * @param p1 The first point.
     * @param p2 The second point.
     * @return double The Euclidean distance between the two points.
     */
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

geometry_msgs::msg::Point ObjectDetection::localToGlobal(geometry_msgs::msg::Point point, geometry_msgs::msg::PoseWithCovarianceStamped amcl_pose)
{
    /**
     * @brief Convert local coordinates to global coordinates.
     * 
     * This function converts local coordinates to global coordinates based on the AMCL pose.
     * 
     * @param point The local point to convert.
     * @param amcl_pose The AMCL pose for the robot.
     * @return geometry_msgs::msg::Point The global point.
     */
    geometry_msgs::msg::Point p;
    double yaw = tf2::getYaw(amcl_pose.pose.pose.orientation);

    p.x = point.x * std::cos(yaw) - point.y * std::sin(yaw) + amcl_pose.pose.pose.position.x;
    p.y = point.x * std::sin(yaw) + point.y * std::cos(yaw) + amcl_pose.pose.pose.position.y;
    p.z = amcl_pose.pose.pose.position.z;

    return p;
}

double ObjectDetection::angleConnectingPoints(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2)
{
    /**
     * @brief Calculate the angle between two points.
     * 
     * This function calculates the angle between two points in 2D space.
     * 
     * @param p1 The first point.
     * @param p2 The second point.
     * @return double The angle between the two points.
     */
    return atan2(p2.y - p1.y, p2.x - p1.x);
}

visualization_msgs::msg::Marker ObjectDetection::produceMarkerPerson(geometry_msgs::msg::Point pt) {
    /**
     * @brief Produce a visualization marker for a person.
     * 
     * This function creates a visualization marker for a person at the specified point.
     * The marker is a cylinder with a blue color and 50% transparency.
     * 
     * @param pt The point where the person is located.
     * @return visualization_msgs::msg::Marker The visualization marker for the person.
     */
    visualization_msgs::msg::Marker marker;

    // Set the frame ID and time stamp.
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();

    // Set the lifetime of the marker (0 means forever)
    marker.lifetime = rclcpp::Duration(1000,0); 

    // Set the namespace and id for this marker to ensure a unique ID
    marker.ns = "object"; 
    marker.id = objectCount_++;

    // The marker type
    marker.type = visualization_msgs::msg::Marker::CYLINDER;

    // Set the marker action to ADD (we add it to the screen)
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set the position of the marker (where the person is standing on the floor)
    marker.pose.position.x = pt.x;
    marker.pose.position.y = pt.y;
    marker.pose.position.z = pt.z; // Setting the base of the cylinder at the person's feet

    // Set the orientation (no rotation needed, so quaternion is 0, 0, 0, 1)
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker (cylinder radius 0.15m, height 2m)
    marker.scale.x = 0.3; // Diameter of 0.3 meters
    marker.scale.y = 0.3; // Diameter of 0.3 meters
    marker.scale.z = 2.0; // Height of 2 meters

    // Set the color of the marker (blue with 50% transparency)
    marker.color.a = 0.5; // Alpha (transparency) set to 50%
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 250.0 / 255.0; // Blue color

    return marker;
}

cv::Point ObjectDetection::worldToPixel(double wx, double wy, double centerX, double centerY, double scale) {
    /**
     * @brief Convert world coordinates to pixel coordinates.
     * 
     * This function converts world coordinates to pixel coordinates based on the image scale.
     * 
     * @param wx The world x-coordinate.
     * @param wy The world y-coordinate.
     * @param centerX The center x-coordinate of the image.
     * @param centerY The center y-coordinate of the image.
     * @param scale The scale factor for conversion.
     * @return cv::Point The pixel coordinates.
     */
    int pixelX = static_cast<int>(centerX + (wx * 110));
    int pixelY = static_cast<int>(centerY - (wy * 103)); // Inverting y-coordinate
    return cv::Point(pixelX, pixelY);
}

void ObjectDetection::drawObjectsOnImage(geometry_msgs::msg::Point object) {
        /**
         * @brief Draw detected objects on an image.
         * 
         * This function loads an image, resizes it, and draws the detected objects on the image.
         * The objects are represented as red circles on the image.
         * 
         * @param object The detected object to draw on the image.
         */
        // Load the image
        cv::Mat image = cv::imread(map_png_path, cv::IMREAD_COLOR);
        if (image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open or find the image");
            return;
        }

        // Dimensions of the original image
        double originalWidth = image.cols;
        double originalHeight = image.rows;

        // Center of the image
        double centerX = originalWidth / 2.0;
        double centerY = originalHeight / 2.0;

        double scale = originalWidth / 10.0; // Scale factor for conversion
        // std::cout << originalWidth << " " << originalHeight << std::endl;
        // Convert world coordinates to pixel coordinates
        cv::Point objectPosition = worldToPixel(object.x, object.y, centerX, centerY, scale);

        // Resizing the image
        cv::Size targetSize(500, 550);
        cv::Mat resizedImage;
        cv::resize(image, resizedImage, targetSize);

        // Adjust the object's position according to the resized image
        double resizeScaleX = targetSize.width / originalWidth;
        double resizeScaleY = targetSize.height / originalHeight;
        cv::Point resizedPosition(static_cast<int>(objectPosition.x * resizeScaleX), static_cast<int>(objectPosition.y * resizeScaleY));

        // Draw the circle at the calculated position
        // Draw the object on the resized image
        cv::circle(resizedImage, resizedPosition, 10, cv::Scalar(0, 0, 255), -1); // Red circle for visibility

        // Show the image
        cv::imshow("Detected Objects", resizedImage);
        cv::waitKey(0); // Pause to display the image
    }

