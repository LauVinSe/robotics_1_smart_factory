#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <iostream>
#include <chrono>

class ImageOverlayNode : public rclcpp::Node {
public:
    ImageOverlayNode() : Node("image_overlay_node"), map_received_(false) {
        // Subscribe to the /map (OccupancyGrid) topic
        occupancy_grid_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&ImageOverlayNode::mapCallback, this, std::placeholders::_1));

        // Initialize the image publisher (use image_transport for better QoS options)
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/map_overlay", 10);

        // Load the second image from file
        file_image_ = cv::imread("/home/vincent/git/warehouse_world/map/warehouse_map.png");
        if (file_image_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Could not load the image from file.");
        }

        // Create a timer to check if /map messages are being received
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&ImageOverlayNode::checkMapMessage, this)
        );
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        occupancyGridToImage(msg);  // Convert occupancy grid to OpenCV image
        map_received_ = true;  // Set the flag indicating we received a map

        // Overlay the images only if both images are available
        if (!file_image_.empty()) {
            // Adjust contrast of both images
            cv::Mat contrast_adjusted_map = adjustContrast(colouredImage_, 2.0, 0);  // Increase contrast of map
            cv::Mat contrast_adjusted_static = adjustContrast(file_image_, 1.5, 20); // Adjust contrast of static image

            // Overlay images
            cv::Mat result = overlayImages(contrast_adjusted_static, contrast_adjusted_map, 0.7);  // Adjust alpha value as needed

            // Display the result locally
            cv::imshow("Overlay Result", result);
            cv::waitKey(1);  // Display the result for a brief moment

            // Publish the result image
            publishOverlayImage(result);
        }
    }

    void checkMapMessage() {
        if (!map_received_) {
            RCLCPP_WARN(this->get_logger(), "No messages received on the /map topic.");
        } else {
            map_received_ = false;  // Reset the flag for the next check
        }
    }

    // Helper function to overlay two images
    cv::Mat overlayImages(const cv::Mat& image1, const cv::Mat& image2, double alpha = 0.5) {
        cv::Mat outputImage;

        // Ensure the images are of the same size
        cv::Mat resizedImage2;
        if (image1.size() != image2.size()) {
            cv::resize(image2, resizedImage2, image1.size());
        } else {
            resizedImage2 = image2;
        }

        // Perform alpha blending
        cv::addWeighted(image1, alpha, resizedImage2, 1.0 - alpha, 0.0, outputImage);

        return outputImage;
    }

    // Adjust contrast and brightness of an image
    cv::Mat adjustContrast(const cv::Mat& input_image, double alpha, int beta) {
        cv::Mat contrast_adjusted_image;
        input_image.convertTo(contrast_adjusted_image, -1, alpha, beta);  // alpha is the contrast factor, beta is brightness
        return contrast_adjusted_image;
    }

    // Convert OccupancyGrid to an OpenCV image (binary image, erosion, and color conversion)
    void occupancyGridToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr grid) {
        int grid_data;
        unsigned int row, col, val;

        // Create a temporary image to store the map
        m_temp_img = cv::Mat::zeros(grid->info.height, grid->info.width, CV_8UC1);

        std::cout << "DataParse started for map: " << grid->header.stamp.sec
                  << " Dim: " << grid->info.height << "x" << grid->info.width << std::endl;

        // Parse the occupancy grid data
        for (row = 0; row < grid->info.height; row++) {
            for (col = 0; col < grid->info.width; col++) {
                grid_data = grid->data[row * grid->info.width + col];
                if (grid_data != -1) {
                    val = 255 - (255 * grid_data) / 100;
                    val = (val == 0) ? 255 : 0;
                    m_temp_img.at<uchar>(grid->info.height - row - 1, col) = val;
                } else {
                    m_temp_img.at<uchar>(grid->info.height - row - 1, col) = 0;
                }
            }
        }

        // Erode the image to remove noise
        map_scale_ = grid->info.resolution;
        origin_x = grid->info.origin.position.x;
        origin_y = grid->info.origin.position.y;
        size_x = grid->info.width;
        size_y = grid->info.height;

        // Create a kernel for erosion
        cv::Mat kernel = (cv::Mat_<uchar>(3, 3) << 0, 0, 0, 0, 1, 0, 0, 0, 0);
        cv::erode(m_temp_img, binaryImage_, kernel);

        // Convert the binary image to a color image and store it
        colouredImage_.create(binaryImage_.size(), CV_8UC3);
        cv::cvtColor(binaryImage_, colouredImage_, cv::COLOR_GRAY2BGR);

        RCLCPP_INFO(this->get_logger(), "Occupancy grid map converted to a binary image");
    }

    // Function to publish the overlaid image
    void publishOverlayImage(const cv::Mat& overlay_image) {
        if (overlay_image.empty()) {
            RCLCPP_WARN(this->get_logger(), "Overlay image is empty, not publishing.");
            return;
        }

        // Convert OpenCV Mat to ROS2 Image message
        cv_bridge::CvImage cv_img;
        cv_img.header.stamp = this->now();  // Set the timestamp
        cv_img.encoding = "bgr8";           // Set the encoding (BGR format)
        cv_img.image = overlay_image;       // Set the image

        // Publish the image
        sensor_msgs::msg::Image::SharedPtr msg = cv_img.toImageMsg();
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;  // Timer for checking if messages are received
    cv::Mat file_image_;
    cv::Mat m_temp_img, binaryImage_, colouredImage_;
    float map_scale_;
    float origin_x, origin_y;
    int size_x, size_y;
    bool map_received_;  // Flag to track if /map topic messages have been received
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageOverlayNode>());
    rclcpp::shutdown();
    return 0;
}
