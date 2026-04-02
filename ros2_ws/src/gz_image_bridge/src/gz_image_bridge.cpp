#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
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
    // SensorDataQoS = best effort, small queue, better for image streams
    auto qos = rclcpp::SensorDataQoS();

    img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/camera/image_raw", qos);

    info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/camera/camera_info", qos);

    gz_node_.Subscribe(
      "/world/apriltag/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image",
      &GzImageBridge::gz_image_callback, this);

    gz_node_.Subscribe(
      "/world/apriltag/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info",
      &GzImageBridge::gz_info_callback, this);

    RCLCPP_INFO(this->get_logger(), "GzImageBridge started, waiting for Gazebo frames...");
  }

private:
  void gz_image_callback(const gz::msgs::Image & gz_img)
  {
    if (!have_camera_info_) {
      return;
    }

    auto stamp = this->now();

    sensor_msgs::msg::Image ros_msg;
    ros_msg.header.stamp = stamp;
    ros_msg.header.frame_id = "camera_link";

    ros_msg.height = gz_img.height();
    ros_msg.width = gz_img.width();
    ros_msg.step = gz_img.step();

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
        ros_msg.encoding = "rgb8";
        break;
    }

    ros_msg.is_bigendian = false;

    if (ros_msg.step == 0)
    {
      if (ros_msg.encoding == "mono8") {
        ros_msg.step = ros_msg.width;
      } else if (ros_msg.encoding == "mono16") {
        ros_msg.step = ros_msg.width * 2;
      } else {
        ros_msg.step = ros_msg.width * 3;
      }
    }

    const std::string & data = gz_img.data();
    ros_msg.data.assign(data.begin(), data.end());

    img_pub_->publish(ros_msg);

    auto info_msg = latest_info_;
    info_msg.header.stamp = stamp;
    info_msg.header.frame_id = "camera_link";
    info_pub_->publish(info_msg);
  }

  void gz_info_callback(const gz::msgs::CameraInfo & gz_info)
  {
    sensor_msgs::msg::CameraInfo ros_msg;

    ros_msg.header.frame_id = "camera_link";
    ros_msg.width = gz_info.width();
    ros_msg.height = gz_info.height();

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

      ros_msg.d.reserve(d.k_size());
      for (int i = 0; i < d.k_size(); ++i)
      {
        ros_msg.d.push_back(d.k(i));
      }
    }

    if (gz_info.has_intrinsics())
    {
      const auto & k = gz_info.intrinsics().k();
      for (int i = 0; i < k.size() && i < 9; ++i)
      {
        ros_msg.k[i] = k.Get(i);
      }
    }

    ros_msg.r[0] = 1.0;
    ros_msg.r[1] = 0.0;
    ros_msg.r[2] = 0.0;
    ros_msg.r[3] = 0.0;
    ros_msg.r[4] = 1.0;
    ros_msg.r[5] = 0.0;
    ros_msg.r[6] = 0.0;
    ros_msg.r[7] = 0.0;
    ros_msg.r[8] = 1.0;

    if (gz_info.has_projection())
    {
      const auto & p = gz_info.projection().p();
      for (int i = 0; i < p.size() && i < 12; ++i)
      {
        ros_msg.p[i] = p.Get(i);
      }
    }
    else if (gz_info.has_intrinsics())
    {
      const auto & k = gz_info.intrinsics().k();
      if (k.size() >= 9)
      {
        ros_msg.p[0]  = k.Get(0);
        ros_msg.p[1]  = k.Get(1);
        ros_msg.p[2]  = k.Get(2);
        ros_msg.p[3]  = 0.0;

        ros_msg.p[4]  = k.Get(3);
        ros_msg.p[5]  = k.Get(4);
        ros_msg.p[6]  = k.Get(5);
        ros_msg.p[7]  = 0.0;

        ros_msg.p[8]  = k.Get(6);
        ros_msg.p[9]  = k.Get(7);
        ros_msg.p[10] = k.Get(8);
        ros_msg.p[11] = 0.0;
      }
    }

    latest_info_ = std::move(ros_msg);
    have_camera_info_ = true;
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  gz::transport::Node gz_node_;

  sensor_msgs::msg::CameraInfo latest_info_;
  bool have_camera_info_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GzImageBridge>());
  rclcpp::shutdown();
  return 0;
}