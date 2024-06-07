/// \file
/// \brief Calculates and publishes odometry from input joint states.
///
/// PARAMETERS:
///     body_id (string): The name of the robot's body frame (REQUIRED)
///     odom_id (string): The name of the robot's odometry frame
///     wheel_left (string): The name of the robot's left wheel joint (REQUIRED)
///     wheel_right (string): The name of the robot's right wheel joint (REQUIRED)
///     track_width (double): The wheel track width in meters (REQUIRED)
///     wheel_radius (double): The wheel radius in meters (REQUIRED)
///     path.rate (double): The rate the path is updated at (Hz).
///     path.num_points (int): Number of path points retained before deleting. Set to 0 to disable limit.
/// PUBLISHERS:
///     odom (nav_msgs/msg/Odometry): the calculated odometry from the input joint states
///     path (nav_msgs/msg/Path): the path data determined from the odometry calculations
/// SUBSCRIBERS:
///     joint_states (sensor_msgs/msg/JointState): the robot's joint states
/// SERVERS:
///     initial_pose (nuturtle_control/srv/InitialPose): set the robot config to the specified config
/// CLIENTS:
///     none

#include <stdexcept>
#include <string>
#include <array>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib_ros/convert.hpp"
#include "nuturtle_control/srv/initial_pose.hpp"

using turtlelib::DiffDrive;
using turtlelib::DiffDriveConfig;
using turtlelib::Wheel;
using turtlelib::Transform2D;
using turtlelib::Vector2D;
using turtlelib::Twist2D;
using turtlelib_ros::tf_to_pose_msg;

/// \brief Calculates odometry for the turtlebot
class Odometry : public rclcpp::Node
{
public:
  /// \brief initialize the node
  Odometry()
  : Node("odometry")
  {
    //Parameters
    auto param = rcl_interfaces::msg::ParameterDescriptor{};

    //Check if required parameters were provided
    bool required_parameters_received = true;

    param.description = "The name of the robot's body frame (REQUIRED).";
    declare_parameter("body_id", "", param);
    body_id_ = get_parameter("body_id").get_parameter_value().get<std::string>();

    if (body_id_ == "") {
      RCLCPP_ERROR_STREAM(get_logger(), "No body frame provided.");
      required_parameters_received = false;
    }

    param.description = "The name of the robot's odometry frame.";
    declare_parameter("odom_id", "odom", param);
    odom_id_ = get_parameter("odom_id").get_parameter_value().get<std::string>();

    param.description = "The name of the robot's left wheel joint (REQUIRED).";
    declare_parameter("wheel_left", "", param);
    wheel_left_joint_ = get_parameter("wheel_left").get_parameter_value().get<std::string>();

    if (wheel_left_joint_ == "") {
      RCLCPP_ERROR_STREAM(get_logger(), "No left wheel joint provided.");
      required_parameters_received = false;
    }

    param.description = "The name of the robot's right wheel joint (REQUIRED).";
    declare_parameter("wheel_right", "", param);
    wheel_right_joint_ = get_parameter("wheel_right").get_parameter_value().get<std::string>();

    if (wheel_right_joint_ == "") {
      RCLCPP_ERROR_STREAM(get_logger(), "No right wheel joint provided.");
      required_parameters_received = false;
    }

    param.description = "The wheel track width in meters (REQUIRED).";
    declare_parameter("track_width", 0.0, param);
    double wheel_track = get_parameter("track_width").get_parameter_value().get<double>();

    if (wheel_track <= 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid wheel track provided: " << wheel_track);
      required_parameters_received = false;
    }

    param.description = "The wheel radius in meters (REQUIRED).";
    declare_parameter("wheel_radius", 0.0, param);
    double wheel_radius = get_parameter("wheel_radius").get_parameter_value().get<double>();

    if (wheel_radius <= 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid wheel radius provided: " << wheel_radius);
      required_parameters_received = false;
    }

    param.description = "The rate the path is updated at (Hz).";
    declare_parameter("path.rate", 5.0, param);
    auto path_interval = 1.0 / get_parameter("path.rate").get_parameter_value().get<double>();

    param.description =
      "Number of path points retained before deleting. Set to 0 to disable limit.";
    declare_parameter("path.num_points", 100, param);
    path_num_points_ = get_parameter(
      "path.num_points").get_parameter_value().get<size_t>();

    //Abort if any required parameters were not provided
    if (!required_parameters_received) {
      throw std::logic_error(
              "Required parameters were not received or were invalid. Please provide valid parameters."
      );
    }

    //Timers
    timer_path_ = create_wall_timer(
      static_cast<std::chrono::milliseconds>(static_cast<int>(path_interval * 1000.0)),
      std::bind(&Odometry::timer_path_callback, this)
    );

    //Publishers
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    pub_path_ = create_publisher<nav_msgs::msg::Path>("path", 10);

    //Subscribers
    sub_joint_states_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states",
      10,
      std::bind(&Odometry::joint_states_callback, this, std::placeholders::_1)
    );

    //Services
    srv_initial_pose_ = create_service<nuturtle_control::srv::InitialPose>(
      "initial_pose",
      std::bind(
        &Odometry::initial_pose_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );

    //Broadcasters
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    //Initialize turtlebot with input parameters and at q(0,0,0)
    turtlebot_ = DiffDrive {wheel_track, wheel_radius};

    //Init odom message
    odom_msg_.header.frame_id = odom_id_;
    odom_msg_.child_frame_id = body_id_;
    odom_msg_.pose.pose.position.z = 0.0;
    odom_msg_.pose.covariance = std::array<double, 36> {};
    odom_msg_.twist.twist.linear.z = 0.0;
    odom_msg_.twist.twist.angular.x = 0.0;
    odom_msg_.twist.twist.angular.y = 0.0;
    odom_msg_.twist.covariance = std::array<double, 36> {};

    //Init odom transform
    odom_tf_.header.frame_id = odom_id_;
    odom_tf_.child_frame_id = body_id_;
    odom_tf_.transform.translation.z = 0.0;

    //Init config pose message and path message
    config_pose_msg_.header.frame_id = odom_id_;
    path_.header.frame_id = odom_id_;

    config_pose_msg_.pose = tf_to_pose_msg(turtlebot_.config().location);
    config_pose_msg_.header.stamp = get_clock()->now();
    path_.poses.push_back(config_pose_msg_);


    RCLCPP_INFO_STREAM(get_logger(), "odometry node started");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_path_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
  rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr srv_initial_pose_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  std::string body_id_, odom_id_, wheel_left_joint_, wheel_right_joint_;
  DiffDrive turtlebot_ {0.16, 0.033}; //Default values, to be overwritten in constructor
  nav_msgs::msg::Odometry odom_msg_;
  geometry_msgs::msg::TransformStamped odom_tf_;
  nav_msgs::msg::Path path_;
  geometry_msgs::msg::PoseStamped config_pose_msg_;
  size_t path_num_points_;
  bool first_joint_states_ = true;


  /// \brief publish odometry estimate path
  void timer_path_callback()
  {
    static Transform2D last_published_tf = turtlebot_.config().location;

    //Only add a new pose to the path if the turtlebot has moved
    if (!almost_equal(last_published_tf, turtlebot_.config().location)) {

      config_pose_msg_.pose = odom_msg_.pose.pose;
      config_pose_msg_.header.stamp = odom_msg_.header.stamp;

      //Remove oldest element if we've reached the max number of path points
      if (path_num_points_ != 0 && path_.poses.size() >= path_num_points_) {
        path_.poses.erase(path_.poses.begin());
      }

      path_.poses.push_back(config_pose_msg_);

      last_published_tf = turtlebot_.config().location;
    }

    //Update stamp
    path_.header.stamp = get_clock()->now();

    //Publish path
    pub_path_->publish(path_);
  }

  /// \brief update internal odometry from received joint states
  /// \param msg - joint states
  void joint_states_callback(const sensor_msgs::msg::JointState & msg)
  {
    Wheel wheel_pos;
    Wheel wheel_vel;
    int wheel_count = 0;

    //Search for joints and wheel values
    for (unsigned int i = 0; i < msg.name.size(); i++) {
      if (msg.name.at(i) == wheel_left_joint_) {
        wheel_pos.left = msg.position.at(i);
        wheel_count++;
        if (msg.velocity.size() > i) {
          wheel_vel.left = msg.velocity.at(i);
        }
      } else if (msg.name.at(i) == wheel_right_joint_) {
        wheel_pos.right = msg.position.at(i);
        wheel_count++;
        if (msg.velocity.size() > i) {
          wheel_vel.right = msg.velocity.at(i);
        }
      }
    }

    //No point in doing odometry if both wheels have not been detected
    if (wheel_count != 2) {return;}

    //init wheel pos with first received states
    if (first_joint_states_) {
      turtlebot_.set_wheel_pos(wheel_pos);
      first_joint_states_ = false;
      return;
    }

    //Get body twist
    Twist2D body_twist = turtlebot_.get_body_twist(wheel_pos);

    //Calculate new configuration
    turtlebot_.update_config(wheel_pos);

    // build odometry message
    odom_msg_.pose.pose = tf_to_pose_msg(turtlebot_.config().location);

    odom_msg_.header.stamp = msg.header.stamp;

    odom_msg_.twist.twist.angular.z = body_twist.w;
    odom_msg_.twist.twist.linear.x = body_twist.x;
    odom_msg_.twist.twist.linear.y = body_twist.y;

    //Publish odometry message
    pub_odom_->publish(odom_msg_);

    //Build transform
    odom_tf_.transform.translation.x = odom_msg_.pose.pose.position.x;
    odom_tf_.transform.translation.y = odom_msg_.pose.pose.position.y;
    odom_tf_.transform.rotation = odom_msg_.pose.pose.orientation;
    odom_tf_.header.stamp = odom_msg_.header.stamp;

    //Broadcast transform
    broadcaster_->sendTransform(odom_tf_);
  }

  /// \brief set the robot config to the specified config
  /// \param request - contains a configuration to set
  void initial_pose_callback(
    const std::shared_ptr<nuturtle_control::srv::InitialPose::Request> request,
    std::shared_ptr<nuturtle_control::srv::InitialPose::Response>
  )
  {
    //Example call
    //ros2 service call /initial_pose nuturtle_control/srv/InitialPose "{x: 0., y: 0., theta: 0.}"

    //Update configuration, but retain current wheel positions
    turtlebot_.set_location(
    {
      {
        request->config.x,
        request->config.y
      },
      request->config.theta
    });
  }
};

/// \brief Run the node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}
