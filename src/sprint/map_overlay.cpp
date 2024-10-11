#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <iostream>
#include <chrono>
#include <string>

/**
 * \class     ImageOverlayNode
 * \brief     A ROS2 node that overlays an occupancy grid map onto a preloaded image and displays the result using OpenCV.
 * \details   This node subscribes to the /rtabmap/map topic to receive occupancy grid maps. It then overlays the received 
 *            map onto a preloaded image from the file system and displays three windows using OpenCV: the original image, 
 *            the map, and the overlay.
 * \author    Lauretius Vincent Setiadharma
 * \version   1.0.0
 * \date      2024-10-10
 * \pre       Ensure that the /rtabmap/map topic is publishing OccupancyGrid messages, and the image file is present.
 * \bug       None reported as of 2024-10-10
 * \warning   None
 */

class ImageOverlayNode : public rclcpp::Node {
public:
    ImageOverlayNode() : Node("image_overlay_node"), map_received_(false) {
        // Subscribe to the /map (OccupancyGrid) topic
        map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/rtabmap/map", 10, std::bind(&ImageOverlayNode::mapCallback, this, std::placeholders::_1)
        );

        // Load the second image from file
        file_image_ = cv::imread("/home/vincent/git/warehouse_world/map/warehouse_map.png");
        if (file_image_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Could not load the image from file.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Image successfully loaded.");
        }

        // Create a timer to check if /map messages are being received
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Check every 100 ms
            std::bind(&ImageOverlayNode::checkMapMessage, this)
        );

        // Initialize the background image (black image) for overlay
        background_image_ = cv::Mat::zeros(cv::Size(500, 500), CV_8UC3);  // Black background of size 500x500
    }

private:
    /**
     * @brief Callback function for the /map subscriber.
     *
     * This function is called whenever a new message is received on the /map topic.
     * It processes the OccupancyGrid data, converts it to an OpenCV format, resizes
     * the images, overlays them, and displays the results in separate windows.
     *
     * @param msg The received OccupancyGrid message.
     */
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received map data");

        // Set the flag indicating that a map has been received
        map_received_ = true;

        // Convert the OccupancyGrid data to OpenCV format
        cv::Mat map_image = convertOccupancyGridToMat(msg);

        // Resize both images to 500x500 pixels
        cv::Mat resized_file_image, resized_map_image;
        cv::resize(file_image_, resized_file_image, cv::Size(500, 500), 0, 0, cv::INTER_LINEAR);
        cv::resize(map_image, resized_map_image, cv::Size(500, 500), 0, 0, cv::INTER_LINEAR);

        // Ensure that both images have the same number of channels (convert grayscale to BGR)
        if (resized_file_image.channels() == 1)
        {
            cv::cvtColor(resized_file_image, resized_file_image, cv::COLOR_GRAY2BGR);
        }
        if (resized_map_image.channels() == 1)
        {
            cv::cvtColor(resized_map_image, resized_map_image, cv::COLOR_GRAY2BGR);
        }

        // Overlay the resized map image onto the background image
        cv::Mat background_with_map = background_image_.clone();
        overlayImageCentered(background_with_map, resized_map_image);
        // cv::resize(background_with_map, background_with_map, cv::Size(500, 550), 0, 0, cv::INTER_LINEAR);

        // Overlay the two images
        cv::Mat blended_image;
        double alpha = 0.5;        // Blending factor for file_image_
        double beta = 1.0 - alpha; // Blending factor for map_image
        cv::addWeighted(resized_file_image, alpha, background_with_map, beta, 0.0, blended_image);

        // Display each image in separate windows
        cv::imshow(WINDOW_GT, resized_file_image);  // Ground truth image (resized to 500x500)
        cv::imshow(WINDOW_SLAM, resized_map_image); // SLAM map image centered on background
        cv::imshow(WINDOW_OVER, blended_image);     // Overlayed image

        cv::waitKey(10); // Allow OpenCV window to refresh
    }
    

    void overlayImageCentered(cv::Mat &background, const cv::Mat &overlay)
    {
        // Convert grayscale overlay to BGR if necessary
        cv::Mat overlayBGR;
        if (overlay.channels() == 1)
        {
            cv::cvtColor(overlay, overlayBGR, cv::COLOR_GRAY2BGR);
        }
        else
        {
            overlayBGR = overlay;
        }

        // Calculate the position to center the overlay on the background
        int x = (background.cols - overlayBGR.cols) / 2;
        int y = (background.rows - overlayBGR.rows) / 2;

        // Ensure the values of x and y are valid (greater than or equal to 0)
        x = std::max(x, 0);
        y = std::max(y, 0);

        // Make sure the overlay fits within the background
        for (int i = 0; i < overlayBGR.rows && y + i < background.rows; ++i)
        {
            for (int j = 0; j < overlayBGR.cols && x + j < background.cols; ++j)
            {
                cv::Vec3b &backgroundPixel = background.at<cv::Vec3b>(y + i, x + j);
                cv::Vec3b overlayPixel = overlayBGR.at<cv::Vec3b>(i, j);

                // Simply replace the background pixel with the overlay pixel
                backgroundPixel = overlayPixel;
            }
        }
    }
    /**
     * @brief Function to convert OccupancyGrid to OpenCV Mat.
     *
     * This function converts the data from an OccupancyGrid message into an OpenCV matrix.
     * The resulting matrix is a grayscale image where each pixel represents the occupancy
     * probability of the corresponding cell in the grid.
     *
     * @param msg The received OccupancyGrid message.
     * @return cv::Mat The converted OpenCV matrix.
     */
    // cv::Mat convertOccupancyGridToMat(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    cv::Mat convertOccupancyGridToMat(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        size_x = msg->info.width;
        size_y = msg->info.height;
        map_scale_ = msg->info.resolution;
        origin_x = msg->info.origin.position.x;
        origin_y = msg->info.origin.position.y;

        // Allocate an OpenCV matrix to store the map
        cv::Mat map_image(size_y, size_x, CV_8UC1); // Create a blank matrix for map

        // Convert occupancy grid data into OpenCV format
        for (int y = 0; y < size_y; ++y)
        {
            for (int x = 0; x < size_x; ++x)
            {
                int index = x + (size_y - y - 1) * size_x; // Index in the OccupancyGrid data
                int8_t data = msg->data[index];

                if (data == -1)
                {                                    // Unknown space
                    map_image.at<uchar>(y, x) = 127; // Set pixel to gray
                }
                else
                {
                    map_image.at<uchar>(y, x) = 255 - data * 255 / 100; // Set pixel to a scale based on occupancy
                }
            }
        }
        return map_image; // Return the OpenCV matrix
    }

    /**
     * @brief Timer callback to check if /map messages are being received.
     *
     * This function is called periodically by a timer to check if any messages have been
     * received on the /map topic. It logs a warning if no messages have been received yet.
     */
    // void checkMapMessage();
    void checkMapMessage()
    {
        if (!map_received_)
        {
            RCLCPP_WARN(this->get_logger(), "No /map messages received yet.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Map message has been received.");
        }
    }

    // ROS Subs
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_; ///< Subscription for the /map topic.

    // ROS Timer
    rclcpp::TimerBase::SharedPtr timer_; ///< Timer for checking if messages are received.

    // Image variables
    cv::Mat file_image_; ///< Image loaded from file.
    cv::Mat background_image_; ///< Black background image (500x500).

    // Map parameters
    float map_scale_; //< Scale factor of the map.
    float origin_x, origin_y; ///< Origin of the map (x, y coordinates).
    int size_x, size_y; //< Size of the map in pixels (width, height).

    // Flag to track map reception
    bool map_received_; //< Flag indicating if /map messages have been received.

    // Constants for window names
    const std::string WINDOW_GT = "Map Ground Truth"; //< Window name for ground truth map display.
    const std::string WINDOW_SLAM = "Map SLAM"; //< Window name for SLAM map display.
    const std::string WINDOW_OVER = "Map Overlay"; //< Window name for overlay map display.
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageOverlayNode>());
    rclcpp::shutdown();
    return 0;
}
