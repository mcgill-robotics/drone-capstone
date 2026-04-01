#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/camera_info.pb.h>

#include <memory>
#include <string>

class GzImageBridge : public rclcpp::Node
{
public:
  GzImageBridge() : Node("gz_image_bridge")
  {
    img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10);

    info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/camera/camera_info", 10);

    gz_node_.Subscribe(
      "/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image",
      &GzImageBridge::gz_image_callback, this);

    gz_node_.Subscribe(
      "/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info",
      &GzImageBridge::gz_info_callback, this);

    RCLCPP_INFO(this->get_logger(), "GzImageBridge started, waiting for Gazebo frames...");
  }

private:
  // ── Image ──────────────────────────────────────────────────────────────────
  void gz_image_callback(const gz::msgs::Image & gz_img)
  {
    auto ros_msg = sensor_msgs::msg::Image();

    ros_msg.header.stamp    = this->now();
    ros_msg.header.frame_id = "camera_link";

    ros_msg.height = gz_img.height();
    ros_msg.width  = gz_img.width();
    ros_msg.step   = gz_img.step();

    switch (gz_img.pixel_format_type())
    {
      case gz::msgs::PixelFormatType::RGB_INT8:  ros_msg.encoding = "rgb8";   break;
      case gz::msgs::PixelFormatType::BGR_INT8:  ros_msg.encoding = "bgr8";   break;
      case gz::msgs::PixelFormatType::L_INT8:    ros_msg.encoding = "mono8";  break;
      case gz::msgs::PixelFormatType::L_INT16:   ros_msg.encoding = "mono16"; break;
      default:                                   ros_msg.encoding = "rgb8";   break;
    }

    ros_msg.is_bigendian = false;

    if (ros_msg.step == 0)
      ros_msg.step = ros_msg.width * 3;

    const std::string & data = gz_img.data();
    ros_msg.data.assign(data.begin(), data.end());

    img_pub_->publish(ros_msg);
  }

  // ── Camera Info ────────────────────────────────────────────────────────────
  void gz_info_callback(const gz::msgs::CameraInfo & gz_info)
  {
    auto ros_msg = sensor_msgs::msg::CameraInfo();

    ros_msg.header.stamp    = this->now();
    ros_msg.header.frame_id = "camera_link";

    ros_msg.width  = gz_info.width();
    ros_msg.height = gz_info.height();

    // Distortion
    if (gz_info.has_distortion())
    {
      const auto & d = gz_info.distortion();
      switch (d.model())
      {
        case gz::msgs::CameraInfo::Distortion::PLUMB_BOB:
          ros_msg.distortion_model = "plumb_bob";
          break;
        case gz::msgs::CameraInfo::Distortion::RATIONAL_POLYNOMIAL:
          ros_msg.distortion_model = "rational_polynomial";
          break;
        default:
          ros_msg.distortion_model = "plumb_bob";
          break;
      }
      for (int i = 0; i < d.k_size(); ++i)
        ros_msg.d.push_back(d.k(i));
    }

    // Intrinsic matrix K (3x3, row-major → 9 elements)
    if (gz_info.has_intrinsics())
    {
      const auto & k = gz_info.intrinsics().k();
      // k is a repeated double with 9 values
      for (int i = 0; i < k.size() && i < 9; ++i)
        ros_msg.k[i] = k.Get(i);
    }

    // Rectification matrix R (identity for a mono camera)
    ros_msg.r[0] = 1.0; ros_msg.r[4] = 1.0; ros_msg.r[8] = 1.0;

    // Projection matrix P (3x4)
    if (gz_info.has_projection())
    {
      const auto & p = gz_info.projection().p();
      for (int i = 0; i < p.size() && i < 12; ++i)
        ros_msg.p[i] = p.Get(i);
    }
    else if (gz_info.has_intrinsics())
    {
      // Fall back: build P from K with Tx=Ty=0
      const auto & k = gz_info.intrinsics().k();
      if (k.size() >= 9) {
        ros_msg.p[0]  = k.Get(0); ros_msg.p[1]  = k.Get(1); ros_msg.p[2]  = k.Get(2); ros_msg.p[3]  = 0.0;
        ros_msg.p[4]  = k.Get(3); ros_msg.p[5]  = k.Get(4); ros_msg.p[6]  = k.Get(5); ros_msg.p[7]  = 0.0;
        ros_msg.p[8]  = k.Get(6); ros_msg.p[9]  = k.Get(7); ros_msg.p[10] = k.Get(8); ros_msg.p[11] = 0.0;
      }
    }

    info_pub_->publish(ros_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr      img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  gz::transport::Node gz_node_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GzImageBridge>());
  rclcpp::shutdown();
  return 0;
}