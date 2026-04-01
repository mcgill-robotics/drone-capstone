#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>

#include <memory>
#include <string>

class GzImageBridge : public rclcpp::Node
{
public:
  GzImageBridge() : Node("gz_image_bridge")
  {
    // ROS 2 publisher
    pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10);

    // Gazebo subscriber
    gz_node_.Subscribe(
      "/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image",
      &GzImageBridge::gz_callback, this);

    RCLCPP_INFO(this->get_logger(), "GzImageBridge started, waiting for Gazebo frames...");
  }

private:
  void gz_callback(const gz::msgs::Image & gz_img)
  {
    auto ros_msg = sensor_msgs::msg::Image();

    // Header
    ros_msg.header.stamp    = this->now();
    ros_msg.header.frame_id = "camera_link";

    // Dimensions
    ros_msg.height = gz_img.height();
    ros_msg.width  = gz_img.width();
    ros_msg.step   = gz_img.step();

    // Pixel format
    switch (gz_img.pixel_format_type())
    {
      case gz::msgs::PixelFormatType::RGB_INT8:
        ros_msg.encoding = "rgb8";
        break;
      case gz::msgs::PixelFormatType::BGR_INT8:
        ros_msg.encoding = "bgr8";
        break;
      case gz::msgs::PixelFormatType::L_INT8:
        ros_msg.encoding = "mono8";
        break;
      case gz::msgs::PixelFormatType::L_INT16:
        ros_msg.encoding = "mono16";
        break;
      default:
        ros_msg.encoding = "rgb8";  // fallback
        break;
    }

    ros_msg.is_bigendian = false;

    // Copy raw pixel data into the ROS message data vector
    const std::string & data = gz_img.data();
    ros_msg.data.assign(data.begin(), data.end());

    pub_->publish(ros_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  gz::transport::Node gz_node_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GzImageBridge>());
  rclcpp::shutdown();
  return 0;
}
