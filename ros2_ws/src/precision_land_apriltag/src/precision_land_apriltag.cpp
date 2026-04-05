#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <iostream>
#include <algorithm>

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

        rclcpp::QoS qos_be(rclcpp::KeepLast(10));
        qos_be.best_effort();

        vehicle_local_position_sub_ =
            this->create_subscription<VehicleLocalPosition>(
                "/fmu/out/vehicle_local_position_v1", qos_be,
                std::bind(&OffboardControl::localPositionCallback, this, std::placeholders::_1));

        vehicle_attitude_sub_ =
            this->create_subscription<VehicleAttitude>(
                "/fmu/out/vehicle_attitude", qos_be,
                std::bind(&OffboardControl::attitudeCallback, this, std::placeholders::_1));

        land_detected_sub_ =
            this->create_subscription<VehicleLandDetected>(
                "/fmu/out/vehicle_land_detected", qos_be,
                std::bind(&OffboardControl::landDetectedCallback, this, std::placeholders::_1));

        apriltag_sub_ =
            this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
                "/detections",
                rclcpp::QoS(10).best_effort(),
                std::bind(&OffboardControl::apriltagCallback, this, std::placeholders::_1));

        camera_info_sub_ =
            this->create_subscription<sensor_msgs::msg::CameraInfo>(
                "/camera/camera_info",
                rclcpp::QoS(10).best_effort(),
                std::bind(&OffboardControl::cameraInfoCallback, this, std::placeholders::_1));

        offboard_setpoint_counter_ = 0;
        stage_             = Stage::INIT;
        land_detected_     = false;
        target_lost_prev_  = false;
        land_command_sent_ = false;
        disarm_sent_       = false;

        resetTag();

        timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timerCallback, this));
        RCLCPP_INFO(this->get_logger(), "OffboardControl started");
    }

private:
    // ── Parameters ────────────────────────────────────────────────────────────

    // Tag IDs
    static constexpr int    HOME_TAG_ID      = 9;    // tag at takeoff position (0,0)
    static constexpr int    TARGET_TAG_ID    = 11;   // tag to land on

    // Takeoff
    static constexpr float  TAKEOFF_X        =  0.0f;
    static constexpr float  TAKEOFF_Y        =  0.0f;
    static constexpr float  TAKEOFF_ALT      = -5.0f;   // 5 m AGL in NED

    // Cruise altitude — climb here before flying to target XY
    static constexpr float  CRUISE_ALT       = -10.0f;  // 10 m AGL in NED

    // Target approach waypoint (XY of tag 11, cruise altitude)
    static constexpr float  APPROACH_X       = 11.0f;
    static constexpr float  APPROACH_Y       =  1.0f;

    // Descend from cruise to this altitude before starting precision landing
    static constexpr float  APPROACH_Z       = -5.0f;   // 5 m AGL in NED

    // Arrival radius and speed for waypoint checks
    static constexpr float  WP_ARRIVE_RADIUS = 1.0f;
    static constexpr float  WP_ARRIVE_VEL    = 0.3f;

    // Precision descent
    static constexpr float  DESCENT_VEL_FAST =  0.3f;
    static constexpr float  DESCENT_VEL_SLOW =  0.08f;
    static constexpr float  DESCENT_SLOW_ALT =  1.0f;
    static constexpr float  COMMIT_LAND_ALT  =  0.30f;

    // XY velocity controller
    static constexpr float  VEL_P_GAIN       =  0.4f;
    static constexpr float  VEL_I_GAIN       =  0.0f;
    static constexpr float  MAX_VELOCITY     =  0.6f;

    static constexpr float  TARGET_TIMEOUT_S =  1.0f;
    static constexpr double DISARM_TIMEOUT_S =  5.0;
    static constexpr double CAM_OFFSET_Z     = -0.1;
    static constexpr float  TAG_EMA_ALPHA    =  0.3f;

    // ── Internal tag struct ───────────────────────────────────────────────────
    struct Tag {
        Eigen::Vector3d    position{NAN, NAN, NAN};
        Eigen::Quaterniond orientation{1, 0, 0, 0};
        rclcpp::Time       timestamp{0, 0, RCL_ROS_TIME};
        bool valid() const { return !std::isnan(position.x()); }
    };

    void resetTag()
    {
        tag_.position    = {NAN, NAN, NAN};
        tag_.orientation = Eigen::Quaterniond::Identity();
        tag_.timestamp   = rclcpp::Time(0, 0, RCL_ROS_TIME);
        tag_ema_init_    = false;
        tag_ema_         = {NAN, NAN, NAN};
        vel_x_integral_  = 0.0f;
        vel_y_integral_  = 0.0f;
    }

    // ── State machine ─────────────────────────────────────────────────────────
    enum class Stage {
        INIT,
        TAKEOFF,              // climb to 5 m above home (0,0)
        VERIFY_HOME_TAG,      // hover at 5 m, wait to see tag 9 below
        CLIMB_TO_CRUISE,      // climb to 10 m at same XY
        FLY_TO_APPROACH_XY,   // fly to target XY at 10 m
        DESCEND_TO_APPROACH,  // descend from 10 m to 5 m above target
        WAIT_FOR_TARGET_TAG,  // hover at 5 m above target, wait for tag 11
        DESCEND,              // precision vision-guided descent
        COMMIT_LAND,          // below 0.3 m — send NAV_LAND and disarm
        FINISHED
    };

    std::string stageName(Stage s) const
    {
        switch (s) {
            case Stage::INIT:                 return "INIT";
            case Stage::TAKEOFF:              return "TAKEOFF";
            case Stage::VERIFY_HOME_TAG:      return "VERIFY_HOME_TAG";
            case Stage::CLIMB_TO_CRUISE:      return "CLIMB_TO_CRUISE";
            case Stage::FLY_TO_APPROACH_XY:   return "FLY_TO_APPROACH_XY";
            case Stage::DESCEND_TO_APPROACH:  return "DESCEND_TO_APPROACH";
            case Stage::WAIT_FOR_TARGET_TAG:  return "WAIT_FOR_TARGET_TAG";
            case Stage::DESCEND:              return "DESCEND";
            case Stage::COMMIT_LAND:          return "COMMIT_LAND";
            case Stage::FINISHED:             return "FINISHED";
            default:                          return "UNKNOWN";
        }
    }

    void switchToStage(Stage s)
    {
        RCLCPP_INFO(this->get_logger(), "→ %s", stageName(s).c_str());
        vel_x_integral_    = 0.0f;
        vel_y_integral_    = 0.0f;
        land_command_sent_ = false;
        stage_ = s;
    }

    // ── Helpers ───────────────────────────────────────────────────────────────
    bool arrivedAt(float tx, float ty, float tz) const
    {
        if (!has_position_) return false;
        const float dx   = pos_.x - tx;
        const float dy   = pos_.y - ty;
        const float dz   = pos_.z - tz;
        const float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        const float spd  = std::sqrt(vel_.x*vel_.x + vel_.y*vel_.y + vel_.z*vel_.z);
        return dist < WP_ARRIVE_RADIUS && spd < WP_ARRIVE_VEL;
    }

    bool checkTargetTimeout() const
    {
        if (!tag_.valid()) return true;
        return (this->now() - tag_.timestamp).seconds() > TARGET_TIMEOUT_S;
    }

    Tag getTagWorld(const Eigen::Vector3d & cam_pos, const Eigen::Quaterniond & cam_ori)
    {
        Eigen::Matrix3d R_opt_to_ned;
        R_opt_to_ned <<  0, -1,  0,
                         1,  0,  0,
                         0,  0,  1;
        Eigen::Quaterniond q_opt_to_ned(R_opt_to_ned);

        Eigen::Vector3d    drone_pos(pos_.x, pos_.y, pos_.z);
        Eigen::Quaterniond drone_ori(
            has_attitude_ ? att_.q[0] : 1.0f,
            has_attitude_ ? att_.q[1] : 0.0f,
            has_attitude_ ? att_.q[2] : 0.0f,
            has_attitude_ ? att_.q[3] : 0.0f);

        Eigen::Affine3d drone_tf  = Eigen::Translation3d(drone_pos) * drone_ori;
        Eigen::Affine3d camera_tf = Eigen::Translation3d(0, 0, CAM_OFFSET_Z) * q_opt_to_ned;
        Eigen::Affine3d tag_tf    = Eigen::Translation3d(cam_pos) * cam_ori;
        Eigen::Affine3d world_tf  = drone_tf * camera_tf * tag_tf;

        Tag t;
        t.position    = world_tf.translation();
        t.orientation = Eigen::Quaterniond(world_tf.rotation());
        t.timestamp   = this->now();
        return t;
    }

    void updateTagEMA(const Tag & raw)
    {
        if (!tag_ema_init_) {
            tag_ema_      = raw.position;
            tag_ema_init_ = true;
        } else {
            tag_ema_ = TAG_EMA_ALPHA * raw.position + (1.0 - TAG_EMA_ALPHA) * tag_ema_;
        }
        tag_.position    = tag_ema_;
        tag_.orientation = raw.orientation;
        tag_.timestamp   = raw.timestamp;
    }

    Eigen::Vector2f calculateVelocitySetpointXY()
    {
        float delta_x = pos_.x - static_cast<float>(tag_.position.x());
        float delta_y = pos_.y - static_cast<float>(tag_.position.y());

        vel_x_integral_ += delta_x;
        vel_y_integral_ += delta_y;
        vel_x_integral_ = std::clamp(vel_x_integral_, -MAX_VELOCITY, MAX_VELOCITY);
        vel_y_integral_ = std::clamp(vel_y_integral_, -MAX_VELOCITY, MAX_VELOCITY);

        float vx = -1.f * (delta_x * VEL_P_GAIN + vel_x_integral_ * VEL_I_GAIN);
        float vy = -1.f * (delta_y * VEL_P_GAIN + vel_y_integral_ * VEL_I_GAIN);

        vx = std::clamp(vx, -MAX_VELOCITY, MAX_VELOCITY);
        vy = std::clamp(vy, -MAX_VELOCITY, MAX_VELOCITY);

        return {vx, vy};
    }

    // ── Callbacks ─────────────────────────────────────────────────────────────
    void localPositionCallback(const VehicleLocalPosition::SharedPtr msg)
    {
        pos_   = *msg;
        vel_.x = msg->vx;
        vel_.y = msg->vy;
        vel_.z = msg->vz;
        has_position_ = true;
    }

    void attitudeCallback(const VehicleAttitude::SharedPtr msg)
    {
        att_ = *msg;
        has_attitude_ = true;
    }

    void landDetectedCallback(const VehicleLandDetected::SharedPtr msg)
    {
        land_detected_ = msg->landed;
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (has_camera_info_) return;
        fx_ = static_cast<float>(msg->k[0]);
        fy_ = static_cast<float>(msg->k[4]);
        cx_ = static_cast<float>(msg->k[2]);
        cy_ = static_cast<float>(msg->k[5]);
        has_camera_info_ = true;
        RCLCPP_INFO(this->get_logger(),
            "Camera info: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
            fx_, fy_, cx_, cy_);
    }

    void apriltagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
    {
        if (!has_camera_info_ || !has_position_ || !has_attitude_) return;
        if (stage_ == Stage::COMMIT_LAND || stage_ == Stage::FINISHED) return;

        const float alt = -pos_.z;
        if (alt < 0.05f) return;

        for (const auto & det : msg->detections) {

            // ── Home tag (9) check during VERIFY_HOME_TAG ──────────────────────
            if (det.id == HOME_TAG_ID && stage_ == Stage::VERIFY_HOME_TAG) {
                home_tag_seen_ = true;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Home tag 9 confirmed below");
                continue;
            }

            // ── Target tag (11) — only process when past the cruise transit ────
            if (det.id == TARGET_TAG_ID &&
                (stage_ == Stage::WAIT_FOR_TARGET_TAG || stage_ == Stage::DESCEND))
            {
                const float u  = static_cast<float>(det.centre.x);
                const float v  = static_cast<float>(det.centre.y);
                const float xn = (u - cx_) / fx_;
                const float yn = (v - cy_) / fy_;

                Eigen::Vector3d    cam_pos(xn * alt, yn * alt, alt);
                Eigen::Quaterniond cam_ori = Eigen::Quaterniond::Identity();

                Tag raw = getTagWorld(cam_pos, cam_ori);
                updateTagEMA(raw);

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                    "TAG 11  raw=(%.2f,%.2f)  ema=(%.2f,%.2f)  drone=(%.2f,%.2f)",
                    raw.position.x(), raw.position.y(),
                    tag_.position.x(), tag_.position.y(),
                    pos_.x, pos_.y);
            }
        }
    }

    // ── Main timer ────────────────────────────────────────────────────────────
    void timerCallback()
    {
        publishOffboardControlMode();

        // Pre-arm setpoints
        if (offboard_setpoint_counter_ < 10) {
            publishTrajectorySetpointPos(0.0f, 0.0f, -1.0f);
            offboard_setpoint_counter_++;
            return;
        }

        // Arm + offboard once
        if (offboard_setpoint_counter_ == 10) {
            if (!has_position_) {
                publishTrajectorySetpointPos(0.0f, 0.0f, -1.0f);
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Waiting for position...");
                return;
            }
            publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
            arm();
            offboard_setpoint_counter_++;
            return;
        }

        const bool target_lost = checkTargetTimeout();
        if (target_lost && !target_lost_prev_)
            RCLCPP_WARN(this->get_logger(), "Target lost in %s", stageName(stage_).c_str());
        else if (!target_lost && target_lost_prev_)
            RCLCPP_INFO(this->get_logger(), "Target acquired");
        target_lost_prev_ = target_lost;

        const float alt = -pos_.z;

        switch (stage_)
        {
        // ── 1. Take off to 5 m above home ────────────────────────────────────
        case Stage::INIT:
        case Stage::TAKEOFF:
        {
            publishTrajectorySetpointPos(TAKEOFF_X, TAKEOFF_Y, TAKEOFF_ALT);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "TAKEOFF  pos=(%.1f,%.1f,%.1f)  alt=%.2f",
                pos_.x, pos_.y, pos_.z, alt);

            if (arrivedAt(TAKEOFF_X, TAKEOFF_Y, TAKEOFF_ALT)) {
                switchToStage(Stage::VERIFY_HOME_TAG);
            }
            break;
        }

        // ── 2. Hover at 5 m, confirm tag 9 is below ──────────────────────────
        case Stage::VERIFY_HOME_TAG:
        {
            publishTrajectorySetpointPos(TAKEOFF_X, TAKEOFF_Y, TAKEOFF_ALT);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "VERIFY_HOME_TAG  alt=%.2f  home_tag_seen=%d",
                alt, (int)home_tag_seen_);

            if (home_tag_seen_) {
                RCLCPP_INFO(this->get_logger(), "Tag 9 confirmed — climbing to cruise altitude");
                switchToStage(Stage::CLIMB_TO_CRUISE);
            }
            break;
        }

        // ── 3. Climb to 10 m at same XY ──────────────────────────────────────
        case Stage::CLIMB_TO_CRUISE:
        {
            publishTrajectorySetpointPos(TAKEOFF_X, TAKEOFF_Y, CRUISE_ALT);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "CLIMB_TO_CRUISE  alt=%.2f  target=%.1f", alt, -CRUISE_ALT);

            if (arrivedAt(TAKEOFF_X, TAKEOFF_Y, CRUISE_ALT)) {
                switchToStage(Stage::FLY_TO_APPROACH_XY);
            }
            break;
        }

        // ── 4. Fly to target XY at cruise altitude (10 m) ────────────────────
        case Stage::FLY_TO_APPROACH_XY:
        {
            publishTrajectorySetpointPos(APPROACH_X, APPROACH_Y, CRUISE_ALT);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "FLY_TO_APPROACH_XY  pos=(%.1f,%.1f)  target=(%.1f,%.1f)  alt=%.2f",
                pos_.x, pos_.y, APPROACH_X, APPROACH_Y, alt);

            if (arrivedAt(APPROACH_X, APPROACH_Y, CRUISE_ALT)) {
                switchToStage(Stage::DESCEND_TO_APPROACH);
            }
            break;
        }

        // ── 5. Descend to 5 m above target ───────────────────────────────────
        case Stage::DESCEND_TO_APPROACH:
        {
            publishTrajectorySetpointPos(APPROACH_X, APPROACH_Y, APPROACH_Z);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "DESCEND_TO_APPROACH  alt=%.2f  target=%.1f", alt, -APPROACH_Z);

            if (arrivedAt(APPROACH_X, APPROACH_Y, APPROACH_Z)) {
                resetTag();  // fresh EMA when we start looking for tag 11
                switchToStage(Stage::WAIT_FOR_TARGET_TAG);
            }
            break;
        }

        // ── 6. Hover at 5 m, wait for tag 11 lock ────────────────────────────
        case Stage::WAIT_FOR_TARGET_TAG:
        {
            publishTrajectorySetpointPos(APPROACH_X, APPROACH_Y, APPROACH_Z);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "WAIT_FOR_TARGET_TAG  alt=%.2f  tag_valid=%d",
                alt, (int)tag_.valid());

            if (!target_lost) {
                RCLCPP_INFO(this->get_logger(), "Tag 11 locked — starting precision descent");
                switchToStage(Stage::DESCEND);
            }
            break;
        }

        // ── 7. Precision vision-guided descent ────────────────────────────────
        case Stage::DESCEND:
        {
            if (alt < COMMIT_LAND_ALT) {
                switchToStage(Stage::COMMIT_LAND);
                break;
            }

            if (target_lost) {
                // Hold current position until tag reappears
                publishTrajectorySetpointPos(pos_.x, pos_.y, pos_.z);
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Tag 11 lost — holding (%.2f,%.2f,%.2f)", pos_.x, pos_.y, pos_.z);
                break;
            }

            const float descent_vel = (alt < DESCENT_SLOW_ALT)
                ? DESCENT_VEL_SLOW
                : DESCENT_VEL_FAST;

            Eigen::Vector2f vel_xy = calculateVelocitySetpointXY();
            publishTrajectorySetpointVel(vel_xy.x(), vel_xy.y(), descent_vel);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                "DESCEND  tag=(%.2f,%.2f)  drone=(%.2f,%.2f)  alt=%.2f  vz=%.2f  vxy=(%.2f,%.2f)",
                tag_.position.x(), tag_.position.y(),
                pos_.x, pos_.y, alt, descent_vel,
                vel_xy.x(), vel_xy.y());

            if (land_detected_) {
                switchToStage(Stage::FINISHED);
            }
            break;
        }

        // ── 8. Below 0.3 m — stop tracking, land and disarm ──────────────────
        case Stage::COMMIT_LAND:
        {
            if (!land_command_sent_) {
                publishVehicleCommand(VehicleCommand::VEHICLE_CMD_NAV_LAND);
                land_command_sent_ = true;
                land_cmd_time_     = this->now();
                RCLCPP_INFO(this->get_logger(),
                    "COMMIT_LAND at %.2f m — NAV_LAND sent", alt);
            }

            publishTrajectorySetpointPos(pos_.x, pos_.y, pos_.z);

            const double elapsed = (this->now() - land_cmd_time_).seconds();
            if ((land_detected_ || elapsed > DISARM_TIMEOUT_S) && !disarm_sent_) {
                publishVehicleCommand(
                    VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
                disarm_sent_ = true;
                RCLCPP_INFO(this->get_logger(),
                    "DISARM sent (land_detected=%d, elapsed=%.1fs)",
                    (int)land_detected_, elapsed);
                switchToStage(Stage::FINISHED);
            }
            break;
        }

        case Stage::FINISHED:
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Mission complete — landed and disarmed.");
            break;
        }
        }
    }

    // ── PX4 helpers ───────────────────────────────────────────────────────────
    void arm()
    {
        publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        RCLCPP_INFO(this->get_logger(), "ARM SENT");
    }

    void publishOffboardControlMode()
    {
        OffboardControlMode msg{};
        const bool descending = (stage_ == Stage::DESCEND) && !checkTargetTimeout();
        msg.position     = !descending;
        msg.velocity     = descending;
        msg.acceleration = false;
        msg.attitude     = false;
        msg.body_rate    = false;
        msg.timestamp    = this->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }

    void publishTrajectorySetpointPos(float x, float y, float z)
    {
        TrajectorySetpoint msg{};
        msg.position  = {x, y, z};
        msg.velocity  = {NAN, NAN, NAN};
        msg.yaw       = 0.0f;
        msg.timestamp = this->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);
    }

    void publishTrajectorySetpointVel(float vx, float vy, float vz)
    {
        TrajectorySetpoint msg{};
        msg.position  = {NAN, NAN, NAN};
        msg.velocity  = {vx, vy, vz};
        msg.yaw       = 0.0f;
        msg.timestamp = this->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);
    }

    void publishVehicleCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
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
        msg.timestamp        = this->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }

    // ── Members ───────────────────────────────────────────────────────────────
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr  offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr   trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr       vehicle_command_publisher_;

    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr  vehicle_local_position_sub_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr       vehicle_attitude_sub_;
    rclcpp::Subscription<VehicleLandDetected>::SharedPtr   land_detected_sub_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr apriltag_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    uint64_t offboard_setpoint_counter_{0};

    VehicleLocalPosition pos_{};
    struct { float x{0}, y{0}, z{0}; } vel_;
    VehicleAttitude att_{};
    bool has_position_{false};
    bool has_attitude_{false};

    bool land_detected_{false};
    bool target_lost_prev_{false};
    bool land_command_sent_{false};
    bool disarm_sent_{false};
    bool home_tag_seen_{false};

    Tag   tag_{};
    Stage stage_{Stage::INIT};

    Eigen::Vector3d tag_ema_{NAN, NAN, NAN};
    bool            tag_ema_init_{false};

    rclcpp::Time land_cmd_time_{0, 0, RCL_ROS_TIME};

    float vel_x_integral_{0.0f};
    float vel_y_integral_{0.0f};

    float fx_{554.0f}, fy_{554.0f};
    float cx_{320.0f}, cy_{240.0f};
    bool  has_camera_info_{false};
};

int main(int argc, char * argv[])
{
    std::cout << "MAIN STARTED" << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}