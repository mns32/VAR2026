#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

class DepthCameraInfoPublisher final : public rclcpp::Node
{
public:
  DepthCameraInfoPublisher()
  : Node("depth_camera_info_publisher")
  {
    depth_image_topic_ = this->declare_parameter<std::string>(
      "depth_image_topic", "/camera/depth/image_raw");
    camera_info_topic_ = this->declare_parameter<std::string>(
      "camera_info_topic", "/camera/depth/camera_info");
    width_ = this->declare_parameter<int>("width", 640);
    height_ = this->declare_parameter<int>("height", 480);
    hfov_rad_ = this->declare_parameter<double>("hfov", 1.047);
    frame_id_override_ = this->declare_parameter<std::string>("frame_id", "");

    publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::QoS(10).reliable().durability_volatile());

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      depth_image_topic_, rclcpp::QoS(10).reliable().durability_volatile(),
      std::bind(&DepthCameraInfoPublisher::on_depth_image, this, std::placeholders::_1));
  }

private:
  void on_depth_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Compute intrinsics from horizontal FOV and resolution.
    // fx = width / (2 * tan(hfov/2)); fy ~= fx (square pixels assumption)
    const double fx = static_cast<double>(width_) / (2.0 * std::tan(hfov_rad_ / 2.0));
    const double fy = fx;
    const double cx = static_cast<double>(width_) / 2.0;
    const double cy = static_cast<double>(height_) / 2.0;

    sensor_msgs::msg::CameraInfo info;
    info.header = msg->header;  // critical for sync with depth_image_proc
    if (!frame_id_override_.empty()) {
      info.header.frame_id = frame_id_override_;
    }
    info.width = static_cast<uint32_t>(width_);
    info.height = static_cast<uint32_t>(height_);
    info.distortion_model = "plumb_bob";
    info.d = {0.0, 0.0, 0.0, 0.0, 0.0};

    // K (3x3)
    info.k = {
      fx, 0.0, cx,
      0.0, fy, cy,
      0.0, 0.0, 1.0
    };

    // R (identity)
    info.r = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    };

    // P (3x4)
    info.p = {
      fx, 0.0, cx, 0.0,
      0.0, fy, cy, 0.0,
      0.0, 0.0, 1.0, 0.0
    };

    publisher_->publish(std::move(info));
  }

  std::string depth_image_topic_;
  std::string camera_info_topic_;
  int width_{640};
  int height_{480};
  double hfov_rad_{1.047};
  std::string frame_id_override_;

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthCameraInfoPublisher>());
  rclcpp::shutdown();
  return 0;
}
