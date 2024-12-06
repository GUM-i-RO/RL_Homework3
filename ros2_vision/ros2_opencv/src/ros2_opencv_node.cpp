#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

class BlueObjectDetector : public rclcpp::Node {
public:
    BlueObjectDetector() : Node("blue_object_detector") {

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/videocamera", 10, std::bind(&BlueObjectDetector::image_callback, this, _1));
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/processed_image", 10);

    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {

        // Conversion from ROS to OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);    // the type of the image from the camera is bgr8 and it must be converted
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in the conversion: %s", e.what());
            return;
        }

        cv::Mat image = cv_ptr->image;

        // Conversion from BGR to HSV (Hue Saturation Value)
        cv::Mat hsv_image;
        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

        // Creation of the mask for the blue color
        cv::Mat mask;
        cv::inRange(hsv_image, cv::Scalar(100, 150, 50), cv::Scalar(140, 255, 255), mask);

        // Operations on the mask
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

        // Finding contours
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        // Draw of the contours on the original image
        cv::Mat processed_image = image.clone();
        cv::drawContours(processed_image, contours, -1, cv::Scalar(0, 255, 0), 2);

        // Write message to be sent. Member function toImageMsg() converts a CvImage
        // into a ROS image message

        // msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", my_image)
        //        .toImageMsg();

        auto output_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, processed_image).toImageMsg();
        publisher_->publish(*output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BlueObjectDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}