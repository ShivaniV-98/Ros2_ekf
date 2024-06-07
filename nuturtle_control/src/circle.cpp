/// \file
/// \brief Publishes cmd_vel commands to move a differential drive robot in a circle.
///
/// PARAMETERS:
///     frequency (double): The frequency of publishing cmd_vel messages (Hz).
/// PUBLISHERS:
///     cmd_vel (geometry_msgs/msg/Twist): command twist for the differential drive robot to follow
/// SUBSCRIBERS:
///     none
/// SERVERS:
///     control (nuturtle_control/srv/Circle): start publishing cmd_vel commands to follow a circle with given parameters
///     reverse (std_srvs/srv/Empty): reverse the direction of the currently active circle
///     stop (std_srvs/srv/Empty): stop publishing cmd_vel commands after publishing a single 0 twist command
/// CLIENTS:
///     none

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtle_control/srv/circle.hpp"

/// \brief Moves the turtlebot in a circle with a configurable velocity/radius
class Circle : public rclcpp::Node
{
public:
  /// \brief initialize the node
  Circle()
  : Node("circle")
  {
    //Parameters
    auto param = rcl_interfaces::msg::ParameterDescriptor{};

    param.description = "The frequency of publishing cmd_vel messages (Hz).";
    declare_parameter("frequency", 100.0, param);
    sim_freq_ = get_parameter("frequency").get_parameter_value().get<double>();
    sim_interval_ = 1.0 / sim_freq_;

    //Timers
    timer_ = create_wall_timer(
      static_cast<std::chrono::microseconds>(static_cast<int>(sim_interval_ * 1000000.0)),
      std::bind(&Circle::timer_callback, this)
    );

    //Publishers
    pub_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    //Services
    srv_control_ = create_service<nuturtle_control::srv::Circle>(
      "control",
      std::bind(&Circle::control_callback, this, std::placeholders::_1, std::placeholders::_2)
    );
    srv_reverse_ = create_service<std_srvs::srv::Empty>(
      "reverse",
      std::bind(&Circle::reverse_callback, this, std::placeholders::_1, std::placeholders::_2)
    );
    srv_stop_ = create_service<std_srvs::srv::Empty>(
      "stop",
      std::bind(&Circle::stop_callback, this, std::placeholders::_1, std::placeholders::_2)
    );


    //Init cmd_vel twist
    cmd_vel_.linear.y = 0.0;
    cmd_vel_.linear.z = 0.0;
    cmd_vel_.angular.x = 0.0;
    cmd_vel_.angular.y = 0.0;

    RCLCPP_INFO_STREAM(get_logger(), "circle node started");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Service<nuturtle_control::srv::Circle>::SharedPtr srv_control_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_reverse_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_stop_;

  double sim_freq_, sim_interval_;
  bool publish_ = false;
  geometry_msgs::msg::Twist cmd_vel_;

  /// \brief main timer loop for publishing cmd_vel messages
  void timer_callback()
  {
    if (publish_) {
      pub_cmd_vel_->publish(cmd_vel_);
    }
  }

  /// \brief start publishing cmd_vel commands to follow a circle with given parameters
  /// \param request - parameters of the circle to be followed
  void control_callback(
    const std::shared_ptr<nuturtle_control::srv::Circle::Request> request,
    std::shared_ptr<nuturtle_control::srv::Circle::Response>
  )
  {
    cmd_vel_.angular.z = request->velocity;
    cmd_vel_.linear.x = request->radius * request->velocity;

    //start publishing
    publish_ = true;
  }

  /// \brief reverse the direction of the currently active circle
  void reverse_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>
  )
  {
    //reverse circle direction
    cmd_vel_.angular.z *= -1.0;
    cmd_vel_.linear.x *= -1.0;
  }

  /// \brief stop publishing cmd_vel commands after publishing a single 0 twist command
  void stop_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>
  )
  {
    //stop publishing
    publish_ = false;

    //Publish a single stop command
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.linear.y = 0.0;
    stop_cmd.linear.z = 0.0;
    stop_cmd.angular.x = 0.0;
    stop_cmd.angular.y = 0.0;
    stop_cmd.angular.z = 0.0;

    pub_cmd_vel_->publish(stop_cmd);
  }
};

/// \brief Run the node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}
