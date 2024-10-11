#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // Load the GMAP (SLAM-generated map) and Gazebo 2D map
    cv::Mat gmap = cv::imread("/home/vincent/git/warehouse_world/map/rtbmap/rtab_warehouse_map.pgm", cv::IMREAD_GRAYSCALE);
    cv::Mat gazebo_map = cv::imread("/home/vincent/git/warehouse_world/map/warehouse_map.png", cv::IMREAD_GRAYSCALE);
    
    // Check if images are loaded successfully
    if (gmap.empty()) {
        std::cerr << "Error loading GMAP image" << std::endl;
        return -1;
    }
    if (gazebo_map.empty()) {
        std::cerr << "Error loading Gazebo map image" << std::endl;
        return -1;
    }

    cv::resize(gmap, gmap, cv::Size(500, 550), 0, 0, cv::INTER_LINEAR);
    cv::resize(gazebo_map, gazebo_map, cv::Size(500, 550), 0, 0, cv::INTER_LINEAR);

    // GS => BGR
    cv::Mat gmap_color, gazebo_map_color;
    cv::cvtColor(gmap, gmap_color, cv::COLOR_GRAY2BGR);
    cv::cvtColor(gazebo_map, gazebo_map_color, cv::COLOR_GRAY2BGR);

    // Transparency
    double alpha = 0.7;         // GMAP
    double beta = 1.0 - alpha;  // Gazebo map

    cv::Mat overlay, overlay_resized;
    cv::addWeighted(gmap_color, alpha, gazebo_map_color, beta, 0.0, overlay);
    cv::imshow("Overlaid Map", overlay);
    cv::imwrite("/home/vincent/git/warehouse_world/images/overlaid_1.png", overlay);
    cv::waitKey(0);

    cv::resize(overlay, overlay_resized, cv::Size(), 3, 3);
    cv::imshow("Overlaid Map x3", overlay_resized);
    cv::waitKey(0);

    return 0;
}
