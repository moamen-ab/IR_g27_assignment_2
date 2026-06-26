#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class CubeColorDetector : public rclcpp::Node
{
public:
  CubeColorDetector() : Node("cube_color_detector")
  {
    // Subscribers
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/rgb_camera/image", 10,
      std::bind(&CubeColorDetector::imageCallback, this, std::placeholders::_1));

    detections_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      "/detections", 10,
      std::bind(&CubeColorDetector::detectionsCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Cube color detector started.");
  }

private:
  // ----------------------------------------------------------------
  // Store the latest image
  // ----------------------------------------------------------------
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    latest_image_ = msg;
  }

  // ----------------------------------------------------------------
  // On every detection, sample the color at each tag center
  // ----------------------------------------------------------------
  void detectionsCallback(
    const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
  {
    if (!latest_image_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "No image received yet.");
      return;
    }

    // Convert ROS image to OpenCV
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(latest_image_, "rgb8");
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
      return;
    }

    cv::Mat & image = cv_ptr->image;
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_RGB2HSV);

    std::string result = "";

    for (const auto & detection : msg->detections) {

      // Tag center in pixels (provided directly by apriltag_ros)
      int cx = static_cast<int>(detection.centre.x);
      int cy = static_cast<int>(detection.centre.y);

      // Clamp to image bounds
      int sample_x = std::clamp(cx - 10, 5, image.cols - 6);
      int sample_y = std::clamp(cy + 20, 5, image.rows - 6);

      // Sample a small 6x6 region around the center for robustness
      cv::Rect roi(sample_x - 3, sample_y - 3, 6, 6);
      cv::Mat patch = hsv(roi);
      cv::Scalar mean_hsv = cv::mean(patch);

      std::string color = classifyColor(mean_hsv);

      RCLCPP_INFO(get_logger(),
        "Tag ID %d → center (%d, %d) → HSV (%.1f, %.1f, %.1f) → %s",
        detection.id, cx, cy,
        mean_hsv[0], mean_hsv[1], mean_hsv[2],
        color.c_str());

    }

  }

  // ----------------------------------------------------------------
  // Classify HSV mean into red / blue / unknown
  // ----------------------------------------------------------------
  std::string classifyColor(const cv::Scalar & hsv)
  {
    double h = hsv[0];  // OpenCV hue: 0-180
    double s = hsv[1];  // saturation: 0-255
    double v = hsv[2];  // value: 0-255

    // Reject low saturation (gray/white) or low value (black)
    if (s < 80 || v < 40) {
        return "unknown";
    }

    if ((h >= 0 && h <= 10) || (h >= 170 && h <= 180)) {
        return "red";
    }
    
    if (h >= 100 && h <= 130) {
        return "blue";
    }

    return "unknown";
  }

  // ----------------------------------------------------------------
  // Members
  // ----------------------------------------------------------------
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr         image_sub_;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;
  sensor_msgs::msg::Image::SharedPtr                               latest_image_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CubeColorDetector>());
  rclcpp::shutdown();
  return 0;
}
