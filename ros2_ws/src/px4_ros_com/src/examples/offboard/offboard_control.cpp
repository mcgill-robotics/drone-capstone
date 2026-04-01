#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <cmath>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{
		RCLCPP_INFO(this->get_logger(), "CONSTRUCTOR STARTED");

		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		// FIX: PX4 publishes /fmu/out/ topics with BEST_EFFORT + VOLATILE durability.
		// Using transient_local() causes a QoS incompatibility — the subscription
		// silently receives nothing. Durability must be left as volatile (default).
		rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
		qos_profile.best_effort();
		// Do NOT call qos_profile.transient_local()

		vehicle_local_position_subscriber_ =
			this->create_subscription<VehicleLocalPosition>(
				"/fmu/out/vehicle_local_position_v1",
				qos_profile,
				std::bind(&OffboardControl::vehicle_local_position_callback, this, std::placeholders::_1));

		RCLCPP_INFO(this->get_logger(), "SUBSCRIBED TO /fmu/out/vehicle_local_position_v1");

		offboard_setpoint_counter_ = 0;
		stage_ = Stage::TAKEOFF;
		delay_started_ = false;

		timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timer_callback, this));

		RCLCPP_INFO(this->get_logger(), "TIMER CREATED");
	}

private:
	enum class Stage {
		TAKEOFF,
		MOVE_TO_SECOND_WAYPOINT
	};

	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_subscriber_;

	uint64_t offboard_setpoint_counter_;

	VehicleLocalPosition vehicle_local_position_{};
	bool has_position_{false};

	Stage stage_;
	bool delay_started_;
	rclcpp::Time first_waypoint_time_;

	void vehicle_local_position_callback(const VehicleLocalPosition::SharedPtr msg)
	{
		vehicle_local_position_ = *msg;
		has_position_ = true;

		RCLCPP_INFO_THROTTLE(
			this->get_logger(),
			*this->get_clock(),
			1000,
			"POSITION -> x: %.2f y: %.2f z: %.2f",
			msg->x, msg->y, msg->z
		);
	}

	void timer_callback()
	{
		publish_offboard_control_mode();

		// Phase 1: Send 10 setpoints first so PX4 accepts offboard mode
		if (offboard_setpoint_counter_ < 10) {
			publish_trajectory_setpoint(0.0, 0.0, -1.0);
			offboard_setpoint_counter_++;
			return;
		}

		// Phase 2: Arm and switch to offboard — but only once we have a valid position
		if (offboard_setpoint_counter_ == 10) {
			if (!has_position_) {
				// Keep streaming setpoints while waiting for position estimate
				publish_trajectory_setpoint(0.0, 0.0, -1.0);
				RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
					"Waiting for valid position estimate before arming...");
				return;
			}
			publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
			arm();
			offboard_setpoint_counter_++;
			RCLCPP_INFO(this->get_logger(), "OFFBOARD + ARM SENT");
			return;
		}

		// Phase 3: Active flight — evaluate stage and publish appropriate setpoint
		float target_x = 0.0f;
		float target_y = 0.0f;
		float target_z = -1.0f;

		if (stage_ == Stage::TAKEOFF && has_position_) {
			float dx = vehicle_local_position_.x - 0.0f;
			float dy = vehicle_local_position_.y - 0.0f;
			float dz = vehicle_local_position_.z - (-1.0f);

			float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

			RCLCPP_INFO_THROTTLE(
				this->get_logger(),
				*this->get_clock(),
				1000,
				"DIST TO WP1 -> %.2f",
				dist
			);

			// FIX: Raised threshold from 0.5 to 0.75 for simulation tolerance
			if (dist < 0.75f) {
				if (!delay_started_) {
					delay_started_ = true;
					first_waypoint_time_ = this->get_clock()->now();
					RCLCPP_INFO(this->get_logger(), "REACHED WP1, STARTING 2s DELAY");
				}

				double elapsed = (this->get_clock()->now() - first_waypoint_time_).seconds();

				RCLCPP_INFO_THROTTLE(
					this->get_logger(),
					*this->get_clock(),
					1000,
					"DELAY TIMER -> %.2f s",
					elapsed
				);

				if (elapsed > 2.0) {
					stage_ = Stage::MOVE_TO_SECOND_WAYPOINT;
					RCLCPP_INFO(this->get_logger(), "MOVING TO WP2 (x=10)");
				}
			}
		}

		if (stage_ == Stage::MOVE_TO_SECOND_WAYPOINT) {
			target_x = 10.0f;
		}

		publish_trajectory_setpoint(target_x, target_y, target_z);
	}

	void arm()
	{
		publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
		RCLCPP_INFO(this->get_logger(), "ARM COMMAND SENT");
	}

	void publish_offboard_control_mode()
	{
		OffboardControlMode msg{};
		msg.position = true;
		msg.velocity = false;
		msg.acceleration = false;
		msg.attitude = false;
		msg.body_rate = false;
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		offboard_control_mode_publisher_->publish(msg);
	}

	void publish_trajectory_setpoint(float x, float y, float z)
	{
		TrajectorySetpoint msg{};
		msg.position = {x, y, z};
		msg.yaw = 0.0;
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		trajectory_setpoint_publisher_->publish(msg);
	}

	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
	{
		VehicleCommand msg{};
		msg.command = command;
		msg.param1 = param1;
		msg.param2 = param2;
		msg.target_system = 1;
		msg.target_component = 1;
		msg.source_system = 1;
		msg.source_component = 1;
		msg.from_external = true;
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
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