#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

static constexpr float AXIS_LENGTH_PX = 50.0f;

class AprilTagVisualizer : public rclcpp::Node
{
public:
  AprilTagVisualizer() : Node("apriltag_visualizer")
  {
    rclcpp::QoS qos(10);
    qos.best_effort();

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", qos,
      std::bind(&AprilTagVisualizer::image_callback, this, std::placeholders::_1));

    detections_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      "/detections", 10,
      std::bind(&AprilTagVisualizer::detections_callback, this, std::placeholders::_1));

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/camera/image_processed", 10);

    RCLCPP_INFO(this->get_logger(), "AprilTag Visualizer started");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr latest_detections_;

  void detections_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
  {
    latest_detections_ = msg;
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s", e.what());
      return;
    }

    cv::Mat & frame = cv_ptr->image;
    const int cx = frame.cols / 2;
    const int cy = frame.rows / 2;

    // Crosshair at image centre
    cv::line(frame, {cx-20, cy}, {cx+20, cy}, cv::Scalar(0,255,255), 1, cv::LINE_AA);
    cv::line(frame, {cx, cy-20}, {cx, cy+20}, cv::Scalar(0,255,255), 1, cv::LINE_AA);

    if (latest_detections_ && !latest_detections_->detections.empty()) {
      for (const auto & det : latest_detections_->detections) {

        // Tag outline
        std::vector<cv::Point2f> corners;
        for (const auto & c : det.corners) {
          corners.push_back({static_cast<float>(c.x), static_cast<float>(c.y)});
        }
        for (int i = 0; i < 4; ++i) {
          cv::line(frame, corners[i], corners[(i+1)%4],
            cv::Scalar(0,255,255), 2, cv::LINE_AA);
        }

        const cv::Point2f centre(
          static_cast<float>(det.centre.x),
          static_cast<float>(det.centre.y));

        cv::circle(frame, centre, 4, cv::Scalar(255,255,255), -1);

        // Axes from homography
        if (det.homography.size() == 9) {
          cv::Mat H = cv::Mat(3, 3, CV_64F);
          for (int i = 0; i < 9; ++i)
            H.at<double>(i/3, i%3) = det.homography[i];

          auto project = [&](double lx, double ly) -> cv::Point2f {
            cv::Mat p = H * (cv::Mat_<double>(3,1) << lx, ly, 1.0);
            return {
              static_cast<float>(p.at<double>(0) / p.at<double>(2)),
              static_cast<float>(p.at<double>(1) / p.at<double>(2))
            };
          };

          // Average edge length in pixels to estimate scale
          float span = 0.0f;
          for (int i = 0; i < 4; ++i) {
            float dx = corners[(i+1)%4].x - corners[i].x;
            float dy = corners[(i+1)%4].y - corners[i].y;
            span += std::sqrt(dx*dx + dy*dy);
          }
          span /= 4.0f;

          const double scale = static_cast<double>(AXIS_LENGTH_PX) / (span * 0.5);

          const cv::Point2f origin = project(0.0, 0.0);
          const cv::Point2f x_tip  = project(scale, 0.0);
          const cv::Point2f y_tip  = project(0.0, scale);

          // Approximate Z as perpendicular to X+Y in image space
          const cv::Point2f xv = x_tip - origin;
          const cv::Point2f yv = y_tip - origin;
          cv::Point2f z_dir = { -(xv.y + yv.y), (xv.x + yv.x) };
          const float z_mag = std::sqrt(z_dir.x*z_dir.x + z_dir.y*z_dir.y);
          if (z_mag > 1e-3f) z_dir *= (AXIS_LENGTH_PX * 0.7f / z_mag);
          const cv::Point2f z_tip = origin + z_dir;

          cv::arrowedLine(frame, origin, x_tip, cv::Scalar(0,0,255),   2, cv::LINE_AA, 0, 0.2);
          cv::arrowedLine(frame, origin, y_tip, cv::Scalar(0,255,0),   2, cv::LINE_AA, 0, 0.2);
          cv::arrowedLine(frame, origin, z_tip, cv::Scalar(255,0,0),   2, cv::LINE_AA, 0, 0.2);

          cv::putText(frame, "X", x_tip + cv::Point2f(4,4),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255),   1, cv::LINE_AA);
          cv::putText(frame, "Y", y_tip + cv::Point2f(4,4),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0),   1, cv::LINE_AA);
          cv::putText(frame, "Z", z_tip + cv::Point2f(4,4),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,0),   1, cv::LINE_AA);

        } else {
          // Fallback: fixed pixel-aligned axes
          cv::arrowedLine(frame, centre,
            centre + cv::Point2f(AXIS_LENGTH_PX, 0),
            cv::Scalar(0,0,255), 2, cv::LINE_AA, 0, 0.2);
          cv::arrowedLine(frame, centre,
            centre + cv::Point2f(0, -AXIS_LENGTH_PX),
            cv::Scalar(0,255,0), 2, cv::LINE_AA, 0, 0.2);
          cv::arrowedLine(frame, centre,
            centre + cv::Point2f(-AXIS_LENGTH_PX*0.5f, -AXIS_LENGTH_PX*0.5f),
            cv::Scalar(255,0,0), 2, cv::LINE_AA, 0, 0.2);
        }

        // ID + pixel error label
        const float err_x = static_cast<float>(det.centre.x) - static_cast<float>(cx);
        const float err_y = static_cast<float>(det.centre.y) - static_cast<float>(cy);
        const float dist  = std::sqrt(err_x*err_x + err_y*err_y);

        cv::putText(frame,
          "ID:" + std::to_string(det.id),
          centre + cv::Point2f(8, -12),
          cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,255,255), 2, cv::LINE_AA);

        cv::putText(frame,
          "err(" + std::to_string(static_cast<int>(err_x)) + ","
                 + std::to_string(static_cast<int>(err_y)) + ") "
                 + std::to_string(static_cast<int>(dist)) + "px",
          centre + cv::Point2f(8, 8),
          cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200,200,200), 1, cv::LINE_AA);
      }
    }

    cv_ptr->header.stamp    = this->now();
    cv_ptr->header.frame_id = "camera_link";
    image_pub_->publish(*cv_ptr->toImageMsg());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AprilTagVisualizer>());
  rclcpp::shutdown();
  return 0;
}
