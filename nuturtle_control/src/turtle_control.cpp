/// \file
/// \brief Converts turtlebot sensor data into joint states and commanded twists into wheel commands.
///
/// PARAMETERS:
///     track_width (double): The wheel track width in meters (REQUIRED)
///     wheel_radius (double): The wheel radius in meters (REQUIRED)
///     motor_cmd_per_rad_sec (double): Motor command value per rad/sec conversion factor (REQUIRED)
///     motor_cmd_max (int32_t): Maximum possible motor command value (REQUIRED)
///     encoder_ticks_per_rad (double): Motor encoder ticks per radian conversion factor (REQUIRED)
/// PUBLISHERS:
///     wheel_cmd (nuturtlebot_msgs/msg/WheelCommands): wheel commands calculated from received cmd_vel twists
///     joint_states (sensor_msgs/msg/JointState): joint states determined from input sensor data
/// SUBSCRIBERS:
///     cmd_vel (geometry_msgs/msg/Twist): commanded twist to convert into wheel commands
///     sensor_data (nuturtlebot_msgs/msg/SensorData): sensor data from robot
/// SERVERS:
///     none
/// CLIENTS:
///     none

#include <stdexcept>
#include "turtlelib/diff_drive.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"

using turtlelib::DiffDrive;
using turtlelib::Wheel;
using turtlelib::Twist2D;
using turtlelib::PI;
using turtlelib::almost_equal;

/// \brief Enables control of the turtlebot
class TurtleControl : public rclcpp::Node
{
public:
  /// \brief initialize the node
  TurtleControl()
  : Node("turtle_control")
  {
    //Parameters
    auto param = rcl_interfaces::msg::ParameterDescriptor{};

    //Check if required parameters were provided
    bool required_parameters_received = true;

    param.description = "The wheel track width in meters (REQUIRED)";
    declare_parameter("track_width", 0.0, param);
    double wheel_track = get_parameter("track_width").get_parameter_value().get<double>();

    if (wheel_track <= 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid wheel track provided: " << wheel_track);
      required_parameters_received = false;
    }

    param.description = "The wheel radius in meters (REQUIRED)";
    declare_parameter("wheel_radius", 0.0, param);
    double wheel_radius = get_parameter("wheel_radius").get_parameter_value().get<double>();

    if (wheel_radius <= 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid wheel radius provided: " << wheel_radius);
      required_parameters_received = false;
    }

    param.description = "Motor command value per rad/sec conversion factor (REQUIRED)";
    declare_parameter("motor_cmd_per_rad_sec", 0.0, param);
    motor_cmd_per_rad_sec_ = get_parameter(
      "motor_cmd_per_rad_sec").get_parameter_value().get<double>();

    if (motor_cmd_per_rad_sec_ <= 0.0) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Invalid motor command to rad/sec conversion provided: " << motor_cmd_per_rad_sec_);
      required_parameters_received = false;
    }

    param.description = "Maximum possible motor command value (REQUIRED)";
    declare_parameter("motor_cmd_max", 0, param);
    motor_cmd_max_ = get_parameter(
      "motor_cmd_max").get_parameter_value().get<int32_t>();

    if (motor_cmd_max_ <= 0) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Invalid maximum motor command provided: " << motor_cmd_max_);
      required_parameters_received = false;
    }

    param.description = "Motor encoder ticks per radian conversion factor (REQUIRED)";
    declare_parameter("encoder_ticks_per_rad", 0.0, param);
    encoder_ticks_per_rad_ = get_parameter(
      "encoder_ticks_per_rad").get_parameter_value().get<double>();

    if (encoder_ticks_per_rad_ <= 0.0) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Invalid encoder ticks to radian conversion provided: " << encoder_ticks_per_rad_);
      required_parameters_received = false;
    }

    //Abort if any required parameters were not provided
    if (!required_parameters_received) {
      throw std::logic_error(
              "Required parameters were not received or were invalid. Please provide valid parameters."
      );
    }


    //Publishers
    pub_wheel_cmd_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);
    pub_joint_states_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    //Subscribers
    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      10,
      std::bind(&TurtleControl::cmd_vel_callback, this, std::placeholders::_1)
    );
    sub_sensor_data_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data",
      10,
      std::bind(&TurtleControl::sensor_data_callback, this, std::placeholders::_1)
    );


    //Initialize turtlebot with input parameters
    turtlebot_ = DiffDrive {wheel_track, wheel_radius};
    wheel_pos_last_ = turtlebot_.config().wheel_pos;

    //Init joint states
    joint_states_.name = {
      "wheel_left_joint",
      "wheel_right_joint"
    };
    joint_states_.position = {
      wheel_pos_last_.left,
      wheel_pos_last_.right
    };
    joint_states_.velocity = {
      0.0,
      0.0
    };

    sensor_stamp_last_ = get_clock()->now();

    RCLCPP_INFO_STREAM(get_logger(), "turtle_control node started");
  }

private:
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr pub_wheel_cmd_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sub_sensor_data_;

  DiffDrive turtlebot_ {0.16, 0.033}; //Default values, to be overwritten in constructor
  double motor_cmd_per_rad_sec_, encoder_ticks_per_rad_;
  int32_t motor_cmd_max_;
  Wheel wheel_pos_last_ {0.0, 0.0};
  rclcpp::Time sensor_stamp_last_;
  sensor_msgs::msg::JointState joint_states_;

  /// \brief take received cmd_vel twist, convert to wheel_cmd, and publish
  /// \param msg - received cmd_vel message
  void cmd_vel_callback(const geometry_msgs::msg::Twist & msg)
  {

    //Use inverse kinematics to calculate the required wheel velocities for the
    //input twist
    Wheel wheel_vel = turtlebot_.get_required_wheel_vel(
    {
      msg.angular.z,
      msg.linear.x,
      msg.linear.y
    });

    //generate wheel command message
    nuturtlebot_msgs::msg::WheelCommands wheel_cmd;
    wheel_cmd.left_velocity = static_cast<int32_t>(wheel_vel.left * motor_cmd_per_rad_sec_);
    wheel_cmd.right_velocity = static_cast<int32_t>(wheel_vel.right * motor_cmd_per_rad_sec_);

    //Clamp velocities
    if (wheel_cmd.left_velocity > motor_cmd_max_) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "Desired left wheel command (" << wheel_cmd.left_velocity << ") exceeded max, clamping to "
                                       << motor_cmd_max_
      );
      wheel_cmd.left_velocity = motor_cmd_max_;
    } else if (wheel_cmd.left_velocity < -motor_cmd_max_) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "Desired left wheel command (" << wheel_cmd.left_velocity << ") exceeded max, clamping to "
                                       << -motor_cmd_max_
      );
      wheel_cmd.left_velocity = -motor_cmd_max_;
    }

    if (wheel_cmd.right_velocity > motor_cmd_max_) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "Desired right wheel command (" << wheel_cmd.right_velocity << ") exceeded max, clamping to "
                                        << motor_cmd_max_
      );
      wheel_cmd.right_velocity = motor_cmd_max_;
    } else if (wheel_cmd.right_velocity < -motor_cmd_max_) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "Desired right wheel command (" << wheel_cmd.right_velocity << ") exceeded max, clamping to "
                                        << -motor_cmd_max_
      );
      wheel_cmd.right_velocity = -motor_cmd_max_;
    }

    //publish wheel command message
    pub_wheel_cmd_->publish(wheel_cmd);
  }

  /// \brief interpret wheel position and velocity from sensor data, publish to joint states
  /// \param msg - received sensor_data message
  void sensor_data_callback(const nuturtlebot_msgs::msg::SensorData & msg)
  {
    //Convert received wheel position to radians
    Wheel wheel_pos {
      static_cast<double>(msg.left_encoder) / encoder_ticks_per_rad_,
      static_cast<double>(msg.right_encoder) / encoder_ticks_per_rad_
    };

    rclcpp::Time sensor_stamp_current = msg.stamp;

    double elapsed_time =
      static_cast<double>(sensor_stamp_current.nanoseconds() -
      sensor_stamp_last_.nanoseconds()) * 1.0e-9;

    //Calculate wheel velocity
    Wheel wheel_vel {0.0, 0.0};

    if (!almost_equal(elapsed_time, 0.0)) {
      wheel_vel.left = (wheel_pos.left - wheel_pos_last_.left) / elapsed_time;
      wheel_vel.right = (wheel_pos.right - wheel_pos_last_.right) / elapsed_time;
    }

    //Update joint state message
    joint_states_.header.stamp = msg.stamp;
    joint_states_.position[0] = wheel_pos.left;
    joint_states_.position[1] = wheel_pos.right;
    joint_states_.velocity[0] = wheel_vel.left;
    joint_states_.velocity[1] = wheel_vel.right;

    //publish joint state message
    pub_joint_states_->publish(joint_states_);

    //Update stored last values
    wheel_pos_last_ = wheel_pos;
    sensor_stamp_last_ = msg.stamp;
  }
};

/// \brief Run the node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}
