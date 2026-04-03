#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <algorithm>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control")
    {
        offboard_control_mode_publisher_ =
            this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ =
            this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ =
            this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
        qos_profile.best_effort();

        vehicle_local_position_subscriber_ =
            this->create_subscription<VehicleLocalPosition>(
                "/fmu/out/vehicle_local_position_v1",
                qos_profile,
                std::bind(&OffboardControl::vehicle_local_position_callback, this, std::placeholders::_1));

        apriltag_subscriber_ =
            this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
                "/detections",
                10,
                std::bind(&OffboardControl::apriltag_callback, this, std::placeholders::_1));

        offboard_setpoint_counter_ = 0;
        stage_ = Stage::TAKEOFF_TO_APPROACH;
        delay_started_ = false;
        land_command_sent_ = false;

        smooth_x_ = 0.0f;
        smooth_y_ = 0.0f;
        smooth_z_ = -1.0f;

        prev_err_x_px_ = 0.0f;
        prev_err_y_px_ = 0.0f;

        timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timer_callback, this));
    }

private:
    enum class Stage {
        TAKEOFF_TO_APPROACH,
        HOVER_AT_APPROACH,
        VISION_LANDING,
        LAND_SENT
    };

    // ── Approach point — directly above tag 11 ───────────────────────────────
    static constexpr float APPROACH_X = 10.0f;
    static constexpr float APPROACH_Y = 0.0f;
    static constexpr float APPROACH_Z = -3.0f;

    static constexpr int TARGET_TAG_ID = 11;

    // ── Camera ───────────────────────────────────────────────────────────────
    static constexpr float IMAGE_WIDTH  = 640.0f;
    static constexpr float IMAGE_HEIGHT = 480.0f;

    // ── PD controller ────────────────────────────────────────────────────────
    // KP: proportional gain — normalised pixel error → metres
    // KD: derivative gain  — damps oscillation
    // Lower KP if oscillating. Higher KD if still overshooting.
    static constexpr float VISION_KP_XY  = 0.4f;
    static constexpr float VISION_KD_XY  = 0.15f;
    static constexpr float MAX_STEP_XY   = 0.20f;  // hard cap per tick (m)

    // ── Signs: flip if drone moves AWAY from tag ─────────────────────────────
    static constexpr float SIGN_X_FROM_IMG_Y =  1.0f;
    static constexpr float SIGN_Y_FROM_IMG_X =  1.0f;

    // ── Descent ──────────────────────────────────────────────────────────────
    static constexpr float DESCENT_RATE_MPS = 0.25f;
    static constexpr float TIMER_DT         = 0.1f;
    static constexpr float DESCENT_STEP     = DESCENT_RATE_MPS * TIMER_DT;

    // Land command fires below this altitude (m AGL)
    static constexpr float LAND_ALT_M = 0.50f;

    // ── Centering tolerance ───────────────────────────────────────────────────
    // Tag just needs to be roughly in frame to start descending.
    // This is intentionally generous — precision improves as we get lower.
    static constexpr float CENTER_TOL_PX = 120.0f;  // ~19% of frame width

    // ── Approach thresholds ──────────────────────────────────────────────────
    static constexpr float XY_REACHED_THRESH = 0.50f;
    static constexpr float Z_REACHED_THRESH  = 0.40f;

    // ── Tag staleness ────────────────────────────────────────────────────────
    static constexpr double TAG_TIMEOUT_SEC = 0.8;

    // ── Setpoint smoothing ────────────────────────────────────────────────────
    // Higher alpha = more responsive but jerkier
    // Lower alpha  = smoother but laggier
    static constexpr float SMOOTH_ALPHA = 0.35f;

    // ── Members ──────────────────────────────────────────────────────────────
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr     offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr      trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr          vehicle_command_publisher_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_subscriber_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr apriltag_subscriber_;

    uint64_t offboard_setpoint_counter_;
    VehicleLocalPosition vehicle_local_position_{};
    bool has_position_{false};

    Stage stage_;
    bool delay_started_;
    bool land_command_sent_;
    rclcpp::Time approach_hold_start_time_;

    bool has_tag_{false};
    rclcpp::Time last_tag_time_;
    float tag_px_x_{0.0f};
    float tag_px_y_{0.0f};

    float smooth_x_, smooth_y_, smooth_z_;

    // Previous pixel errors for derivative term
    float prev_err_x_px_;
    float prev_err_y_px_;

    // ── Helpers ───────────────────────────────────────────────────────────────
    static float clampf(float v, float lo, float hi)
    {
        return std::max(lo, std::min(v, hi));
    }

    float current_alt_m()
    {
        return -vehicle_local_position_.z;
    }

    bool tag_is_fresh()
    {
        if (!has_tag_) return false;
        return (this->get_clock()->now() - last_tag_time_).seconds() < TAG_TIMEOUT_SEC;
    }

    void publish_smoothed_setpoint(float raw_x, float raw_y, float raw_z)
    {
        smooth_x_ = SMOOTH_ALPHA * raw_x + (1.0f - SMOOTH_ALPHA) * smooth_x_;
        smooth_y_ = SMOOTH_ALPHA * raw_y + (1.0f - SMOOTH_ALPHA) * smooth_y_;
        smooth_z_ = SMOOTH_ALPHA * raw_z + (1.0f - SMOOTH_ALPHA) * smooth_z_;
        publish_trajectory_setpoint(smooth_x_, smooth_y_, smooth_z_);
    }

    // ── Callbacks ─────────────────────────────────────────────────────────────
    void vehicle_local_position_callback(const VehicleLocalPosition::SharedPtr msg)
    {
        vehicle_local_position_ = *msg;
        has_position_ = true;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "POS  x:%.2f  y:%.2f  z:%.2f  alt:%.2f m",
            msg->x, msg->y, msg->z, -msg->z);
    }

    void apriltag_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
    {
        for (const auto & det : msg->detections) {
            if (det.id == TARGET_TAG_ID) {
                tag_px_x_ = static_cast<float>(det.centre.x);
                tag_px_y_ = static_cast<float>(det.centre.y);
                last_tag_time_ = this->get_clock()->now();
                has_tag_ = true;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                    "TAG %d  px_x:%.1f  px_y:%.1f", TARGET_TAG_ID, tag_px_x_, tag_px_y_);
                break;
            }
        }
    }

    // ── Main timer ────────────────────────────────────────────────────────────
    void timer_callback()
    {
        publish_offboard_control_mode();

        if (offboard_setpoint_counter_ < 10) {
            publish_smoothed_setpoint(0.0f, 0.0f, -1.0f);
            offboard_setpoint_counter_++;
            return;
        }

        if (offboard_setpoint_counter_ == 10) {
            if (!has_position_) {
                publish_smoothed_setpoint(0.0f, 0.0f, -1.0f);
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Waiting for position estimate...");
                return;
            }
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
            arm();
            offboard_setpoint_counter_++;
            return;
        }

        float target_x = APPROACH_X;
        float target_y = APPROACH_Y;
        float target_z = APPROACH_Z;

        switch (stage_)
        {
        // ── 1. Fly to approach point ──────────────────────────────────────────
        case Stage::TAKEOFF_TO_APPROACH:
        {
            const float dx     = vehicle_local_position_.x - APPROACH_X;
            const float dy     = vehicle_local_position_.y - APPROACH_Y;
            const float dz     = vehicle_local_position_.z - APPROACH_Z;
            const float xy_err = std::sqrt(dx*dx + dy*dy);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "APPROACH  xy_err:%.2f  z_err:%.2f", xy_err, std::fabs(dz));

            if (has_position_ && xy_err < XY_REACHED_THRESH && std::fabs(dz) < Z_REACHED_THRESH) {
                if (!delay_started_) {
                    delay_started_ = true;
                    approach_hold_start_time_ = this->get_clock()->now();
                    RCLCPP_INFO(this->get_logger(), "AT APPROACH — holding 2 s");
                }
                if ((this->get_clock()->now() - approach_hold_start_time_).seconds() > 2.0) {
                    stage_ = Stage::HOVER_AT_APPROACH;
                    delay_started_ = false;
                    RCLCPP_INFO(this->get_logger(), "→ HOVER_AT_APPROACH");
                }
            }
            break;
        }

        // ── 2. Hover and wait for tag ─────────────────────────────────────────
        case Stage::HOVER_AT_APPROACH:
        {
            if (tag_is_fresh()) {
                // Reset derivative state when entering vision mode
                prev_err_x_px_ = tag_px_x_ - (IMAGE_WIDTH  * 0.5f);
                prev_err_y_px_ = tag_px_y_ - (IMAGE_HEIGHT * 0.5f);
                stage_ = Stage::VISION_LANDING;
                RCLCPP_INFO(this->get_logger(), "TAG ACQUIRED → VISION_LANDING");
            }
            break;
        }

        // ── 3. Vision-guided descent ──────────────────────────────────────────
        case Stage::VISION_LANDING:
        {
            if (!tag_is_fresh()) {
                target_x = APPROACH_X;
                target_y = APPROACH_Y;
                target_z = APPROACH_Z;
                // Reset derivative so there's no spike when tag reappears
                prev_err_x_px_ = 0.0f;
                prev_err_y_px_ = 0.0f;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "TAG LOST — returning to approach point");
                break;
            }

            // Current pixel error
            const float err_x_px = tag_px_x_ - (IMAGE_WIDTH  * 0.5f);
            const float err_y_px = tag_px_y_ - (IMAGE_HEIGHT * 0.5f);

            // Derivative: change in error since last tick
            const float derr_x = err_x_px - prev_err_x_px_;
            const float derr_y = err_y_px - prev_err_y_px_;
            prev_err_x_px_ = err_x_px;
            prev_err_y_px_ = err_y_px;

            // Normalise errors to [-1, 1]
            const float err_x_n  = err_x_px / (IMAGE_WIDTH  * 0.5f);
            const float err_y_n  = err_y_px / (IMAGE_HEIGHT * 0.5f);
            const float derr_x_n = derr_x   / (IMAGE_WIDTH  * 0.5f);
            const float derr_y_n = derr_y   / (IMAGE_HEIGHT * 0.5f);

            // PD correction in local NED frame
            // If drone moves AWAY from tag: flip the relevant SIGN constant
            const float step_x = clampf(
                SIGN_X_FROM_IMG_Y * (VISION_KP_XY * err_y_n + VISION_KD_XY * derr_y_n),
                -MAX_STEP_XY, MAX_STEP_XY);

            const float step_y = clampf(
                SIGN_Y_FROM_IMG_X * (VISION_KP_XY * err_x_n + VISION_KD_XY * derr_x_n),
                -MAX_STEP_XY, MAX_STEP_XY);

            target_x = vehicle_local_position_.x + step_x;
            target_y = vehicle_local_position_.y + step_y;

            const float alt      = current_alt_m();
            const float dist_px  = std::sqrt(err_x_px*err_x_px + err_y_px*err_y_px);

            // If tag is within the hard landing zone — stop correcting, just land now
            static constexpr float LAND_NOW_TOL_PX = 600.0f;  // ~9% of frame, tune this
            if (dist_px < LAND_NOW_TOL_PX && !land_command_sent_) {
                land();
                land_command_sent_ = true;
                stage_ = Stage::LAND_SENT;
                RCLCPP_INFO(this->get_logger(), "TAG IN LANDING ZONE (%.1f px) → LAND NOW", dist_px);
                break;
            }

            const bool  centered = dist_px < CENTER_TOL_PX;

            if (centered) {
                // Tag is reasonably in frame — descend
                target_z = vehicle_local_position_.z + DESCENT_STEP;

                if (alt < LAND_ALT_M && !land_command_sent_) {
                    land();
                    land_command_sent_ = true;
                    stage_ = Stage::LAND_SENT;
                    RCLCPP_INFO(this->get_logger(), "LOW & CENTERED → LAND SENT");
                }
            } else {
                // Tag is far off-centre — hold altitude, correct XY
                target_z = vehicle_local_position_.z;
            }

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                "VISION  err:(%.1f,%.1f) dist:%.1f tol:%.0f centered:%d  alt:%.2f  sp:(%.2f %.2f %.2f)",
                err_x_px, err_y_px, dist_px, CENTER_TOL_PX, (int)centered,
                alt, target_x, target_y, target_z);

            break;
        }

        // ── 4. Landing in progress ────────────────────────────────────────────
        case Stage::LAND_SENT:
        {
            target_x = vehicle_local_position_.x;
            target_y = vehicle_local_position_.y;
            target_z = vehicle_local_position_.z;
            break;
        }
        }

        publish_smoothed_setpoint(target_x, target_y, target_z);
    }

    // ── PX4 helpers ───────────────────────────────────────────────────────────
    void arm()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        RCLCPP_INFO(this->get_logger(), "ARM COMMAND SENT");
    }

    void land()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
        RCLCPP_INFO(this->get_logger(), "LAND COMMAND SENT");
    }

    void publish_offboard_control_mode()
    {
        OffboardControlMode msg{};
        msg.position  = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }

    void publish_trajectory_setpoint(float x, float y, float z)
    {
        TrajectorySetpoint msg{};
        msg.position  = {x, y, z};
        msg.yaw       = 0.0f;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
    {
        VehicleCommand msg{};
        msg.command          = command;
        msg.param1           = param1;
        msg.param2           = param2;
        msg.target_system    = 1;
        msg.target_component = 1;
        msg.source_system    = 1;
        msg.source_component = 1;
        msg.from_external    = true;
        msg.timestamp        = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }
};

int main(int argc, char *argv[])
{
    std::cout << "MAIN STARTED" << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}