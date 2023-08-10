#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("webcam_publisher");

    // Create a publisher for camera frames and camera info
    auto image_pub = node->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);
    auto camera_info_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", 10);

    cv::VideoCapture cap;
    cap.open(2);

    if (!cap.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open webcam");
        return 1;
    }

    cv_bridge::CvImage cv_img;
    cv::Mat frame;

    while (rclcpp::ok()) {
        cap >> frame;

        if (!frame.empty()) {
            // Convert OpenCV image to ROS Image message
            cv_img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame);
            cv_img.header.stamp = node->now();
            cv_img.header.frame_id = "camera_link";
            sensor_msgs::msg::Image out_image;
            cv_img.toImageMsg(out_image);
            image_pub->publish(out_image);

            // Create and publish CameraInfo message (assuming a simple camera setup)
            sensor_msgs::msg::CameraInfo camera_info;
            camera_info.header = cv_img.header;
            camera_info.height = frame.rows;
            camera_info.width = frame.cols;
            camera_info.distortion_model = "plumb_bob";
            camera_info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
            camera_info.k = {600.0, 0.0, frame.cols / 2.0, 0.0, 600.0, frame.rows / 2.0, 0.0, 0.0, 1.0};
            camera_info.p = {600.0, 0.0, frame.cols / 2.0, 0.0, 0.0, 600.0, frame.rows / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0};
            camera_info_pub->publish(camera_info);
        }

        rclcpp::spin_some(node);
    }

    cap.release();
    rclcpp::shutdown();
    return 0;
}
