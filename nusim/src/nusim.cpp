/// \file
/// \brief Runs the simulation for the NUTurtle.
///
/// PARAMETERS:
///     draw_only (bool): Only display walls, do not perform any simulation.
///     track_width (double): The wheel track width in meters (REQUIRED).
///     wheel_radius (double): The wheel radius in meters (REQUIRED).
///     motor_cmd_per_rad_sec (double): Motor command value per rad/sec conversion factor (REQUIRED).
///     motor_cmd_max (int32_t): Maximum possible motor command value (REQUIRED).
///     encoder_ticks_per_rad (double): Motor encoder ticks per radian conversion factor (REQUIRED).
///     rate (double): The rate the simulation runs at (Hz).
///     path.rate (double): The rate the path is updated at (Hz).
///     path.num_points (int): Number of path points retained before deleting. Set to 0 to disable limit.
///     x0 (double): Initial x position of the robot (m).
///     y0 (double): Initial y position of the robot (m).
///     theta0 (double): Initial rotation of the robot (rad).
///     collision_radius (double): Collision radius of the robot (m).
///     display_collision_cylinder (bool): Activates a marker to display collision cylinder of the robot.
///     allow_collision_rotation (bool): Allow rotation when colliding with an obstacle.
///     allow_collision_sliding (bool): Allow sliding when colliding with an obstacle.
///     obstacles.x (std::vector<double>): List of x starting positions of obstacles (m). Arbitrary length, but must match length of `y`.
///     obstacles.y (std::vector<double>): List of y starting positions of obstacles (m). Arbitray length, but must match length of `x`.
///     obstacles.r (double): Radius of all cylinder obstacles (m). Single value applies to all obstacles.
///     x_length (double): Length of the arena in the x direction (m).
///     y_length (double): Length of the arena in the y direction (m).
///     input_noise (double): Standard deviation for noise on input wheel commands (rad/s). Must be nonnegative.
///     slip_fraction (double): Bound of fraction of slip experienced by the wheels during motion (decimal fraction). Must be nonnegative.
///     basic_sensor_variance (double): Standard deviation for noise in obstacle sensing (m). Must be nonnegative.
///     max_range (double): Max range for obstacle sensing (m).
///     lidar.range_min (double): Min range for lidar scanning (m).
///     lidar.range_max (double): Max range for lidar scanning (m).
///     lidar.angle_min (double): Min angle for lidar scanning (rad).
///     lidar.angle_max (double): Max angle for lidar scanning (rad).
///     lidar.angle_incr (double): Angle increment for lidar scanning (rad).
///     lidar.resolution (double): Resolution for lidar scan (m). Set to 0 for perfect resolution.
///     lidar.noise (double): Standard deviation for noise in lidar scan (m). Must be nonnegative.
/// PUBLISHERS:
///     ~/obstacles (visualization_msgs/msg/MarkerArray): marker array containing cylindrical obstacles in the world.
///     ~/timestep (std_msgs/msg/UInt64): current timestep of the simulation
///     ~/collision_cylinder (visualization_msgs/msg/Marker): marker representing the robot's collision cylinder
///     ~/path (nav_msgs/msg/Path): the ground truth path data from the simulated robot
///     sensor_data (nuturtlebot_msgs/msg/SensorData): simulated turtlebot sensor data
///     fake_sensor (visualization_msgs/msg/MarkerArray): fake sensor data telling the relative location of the obstacles to the robot
///     scan (sensor_msgs/msg/LaserScan): simulated lidar scan data
/// SUBSCRIBERS:
///     wheel_cmd (nuturtlebot_msgs/msg/WheelCommands): wheel commands from control nodes
/// SERVERS:
///     ~/reset (std_srvs/srv/Empty): resets the simulation to its starting state
///     ~/teleport (nusim/srv/Teleport): teleports the actual turtlebot to a provided location
/// CLIENTS:
///     none

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <vector>
#include <exception>
#include <algorithm>
#include <random>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib_ros/convert.hpp"
#include "nusim/srv/teleport.hpp"

using namespace std::chrono_literals;
using turtlelib::PI;
using turtlelib::almost_equal;
using turtlelib::normalize_angle;
using turtlelib::Vector2D;
using turtlelib::Transform2D;
using turtlelib::Line2D;
using turtlelib::Circle2D;
using turtlelib::find_intersection;
using turtlelib::DiffDrive;
using turtlelib::DiffDriveConfig;
using turtlelib::Wheel;
using turtlelib_ros::tf_to_tf_msg;
using turtlelib_ros::tf_to_pose_msg;

// Constants
constexpr std::string_view WORLD_FRAME = "nusim/world";
constexpr std::string_view ROBOT_GROUND_TRUTH_FRAME = "red/base_footprint";
constexpr std::string_view LIDAR_GROUND_TRUTH_FRAME = "red/base_scan";
const Transform2D ROBOT_LIDAR_TF {{-0.032, 0.0}};
constexpr double OBSTACLE_HEIGHT = 0.25;
constexpr double WALL_HEIGHT = 0.25;
constexpr double WALL_WIDTH = 0.1;
constexpr int32_t MARKER_ID_WALL = 0;
constexpr int32_t MARKER_ID_COLLISION_CYLINDER = 1;
constexpr int32_t MARKER_ID_OFFSET_OBSTACLES = 100;


//Function prototypes
std::mt19937 & get_random();


/// \brief Runs the simulation for the NUTurtle.
class NuSim : public rclcpp::Node
{
public:
  /// \brief initialize the node
  NuSim()
  : Node("nusim")
  {

    //Parameters
    auto param = rcl_interfaces::msg::ParameterDescriptor{};

    //Check if required parameters were provided
    bool required_parameters_received = true;

    param.description = "Only display walls, do not perform any simulation.";
    declare_parameter("draw_only", false, param);
    draw_only_ = get_parameter("draw_only").get_parameter_value().get<bool>();

    param.description = "The wheel track width in meters (REQUIRED).";
    declare_parameter("track_width", 0.0, param);
    double wheel_track = get_parameter("track_width").get_parameter_value().get<double>();

    if (!draw_only_ && wheel_track <= 0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid wheel track provided: " << wheel_track);
      required_parameters_received = false;
    }

    param.description = "The wheel radius in meters (REQUIRED).";
    declare_parameter("wheel_radius", 0.0, param);
    double wheel_radius = get_parameter("wheel_radius").get_parameter_value().get<double>();

    if (!draw_only_ && wheel_radius <= 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid wheel radius provided: " << wheel_radius);
      required_parameters_received = false;
    }

    param.description = "Motor command value per rad/sec conversion factor (REQUIRED).";
    declare_parameter("motor_cmd_per_rad_sec", 0.0, param);
    motor_cmd_per_rad_sec_ = get_parameter(
      "motor_cmd_per_rad_sec").get_parameter_value().get<double>();

    if (!draw_only_ && motor_cmd_per_rad_sec_ <= 0.0) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Invalid motor command to rad/sec conversion provided: " << motor_cmd_per_rad_sec_);
      required_parameters_received = false;
    }

    param.description = "Maximum possible motor command value (REQUIRED).";
    declare_parameter("motor_cmd_max", 0, param);
    motor_cmd_max_ = get_parameter(
      "motor_cmd_max").get_parameter_value().get<int32_t>();

    if (!draw_only_ && motor_cmd_max_ <= 0) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Invalid maximum motor command provided: " << motor_cmd_max_);
      required_parameters_received = false;
    }

    param.description = "Motor encoder ticks per radian conversion factor (REQUIRED).";
    declare_parameter("encoder_ticks_per_rad", 0.0, param);
    encoder_ticks_per_rad_ = get_parameter(
      "encoder_ticks_per_rad").get_parameter_value().get<double>();

    if (!draw_only_ && encoder_ticks_per_rad_ <= 0.0) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Invalid encoder ticks to radian conversion provided: " << encoder_ticks_per_rad_);
      required_parameters_received = false;
    }

    param.description = "The rate the simulation runs at (Hz).";
    declare_parameter("rate", 200.0, param);
    sim_rate_ = get_parameter("rate").get_parameter_value().get<double>();
    sim_interval_ = 1.0 / sim_rate_;

    param.description = "The rate the path is updated at (Hz).";
    declare_parameter("path.rate", 5.0, param);
    auto path_interval = 1.0 / get_parameter("path.rate").get_parameter_value().get<double>();

    param.description =
      "Number of path points retained before deleting. Set to 0 to disable limit.";
    declare_parameter("path.num_points", 100, param);
    path_num_points_ = get_parameter(
      "path.num_points").get_parameter_value().get<size_t>();

    Vector2D translation_initial;

    param.description = "Initial x position of the robot (m).";
    declare_parameter("x0", 0.0, param);
    translation_initial.x = get_parameter("x0").get_parameter_value().get<double>();

    param.description = "Initial y position of the robot (m).";
    declare_parameter("y0", 0.0, param);
    translation_initial.y = get_parameter("y0").get_parameter_value().get<double>();

    param.description = "Initial rotation of the robot (rad).";
    declare_parameter("theta0", 0.0, param);
    auto rotation_initial = get_parameter("theta0").get_parameter_value().get<double>();

    param.description = "Collision radius of the robot (m).";
    declare_parameter("collision_radius", 0.0, param);
    collision_radius_ = get_parameter("collision_radius").get_parameter_value().get<double>();

    param.description = "Activates a marker to display collision cylinder of the robot.";
    declare_parameter("display_collision_cylinder", false, param);
    display_collision_cylinder_ =
      get_parameter("display_collision_cylinder").get_parameter_value().get<bool>();

    param.description = "Allow rotation when colliding with an obstacle.";
    declare_parameter("allow_collision_rotation", true, param);
    allow_collision_rotation_ =
      get_parameter("allow_collision_rotation").get_parameter_value().get<bool>();

    param.description = "Allow sliding when colliding with an obstacle.";
    declare_parameter("allow_collision_sliding", false, param);
    allow_collision_sliding_ =
      get_parameter("allow_collision_sliding").get_parameter_value().get<bool>();

    param.description =
      "List of x starting positions of obstacles (m). Arbitrary length, but must match length of y.";
    declare_parameter("obstacles.x", std::vector<double> {}, param);
    obstacles_x_ = get_parameter("obstacles.x").as_double_array();

    param.description =
      "List of y starting positions of obstacles (m). Arbitray length, but must match length of x.";
    declare_parameter("obstacles.y", std::vector<double> {}, param);
    obstacles_y_ = get_parameter("obstacles.y").as_double_array();

    //If vectors differ in size, exit node
    if (obstacles_x_.size() != obstacles_y_.size()) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Size mismatch between input obstacle coordinate lists (x: " << obstacles_x_.size() <<
          " elements, y: " << obstacles_y_.size() << " elements)"
      );
      required_parameters_received = false;
    }

    param.description =
      "Radius of all cylinder obstacles (m). Single value applies to all obstacles.";
    declare_parameter("obstacles.r", 0.015, param);
    obstacles_r_ = get_parameter("obstacles.r").get_parameter_value().get<double>();

    collision_dist_ = collision_radius_ + obstacles_r_;

    param.description =
      "Length of the arena in the x direction (m).";
    declare_parameter("x_length", 5.0, param);
    x_length_ = get_parameter("x_length").get_parameter_value().get<double>();

    param.description =
      "Length of the arena in the y direction (m).";
    declare_parameter("y_length", 5.0, param);
    y_length_ = get_parameter("y_length").get_parameter_value().get<double>();

    param.description =
      "Standard deviation for noise on input wheel commands (rad/s). Must be nonnegative.";
    declare_parameter("input_noise", 0.0, param);
    auto input_noise = get_parameter("input_noise").get_parameter_value().get<double>();

    if (!draw_only_ && input_noise < 0.0) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Invalid input noise provided: " << input_noise);
      required_parameters_received = false;
    }

    wheel_vel_dist_ = std::normal_distribution<> {0.0, input_noise};

    param.description =
      "Bound of fraction of slip experienced by the wheels during motion (decimal fraction)."
      " Must be nonnegative.";
    declare_parameter("slip_fraction", 0.0, param);
    auto slip_fraction = get_parameter("slip_fraction").get_parameter_value().get<double>();

    if (!draw_only_ && slip_fraction < 0.0) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Invalid slip fraction provided: " << slip_fraction);
      required_parameters_received = false;
    }

    slip_dist_ = std::uniform_real_distribution<> {-slip_fraction, slip_fraction};

    param.description =
      "Standard deviation for noise in obstacle sensing (m). Must be nonnegative.";
    declare_parameter("basic_sensor_variance", 0.0, param);
    auto basic_sensor_variance =
      get_parameter("basic_sensor_variance").get_parameter_value().get<double>();

    if (!draw_only_ && basic_sensor_variance < 0.0) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Invalid basic sensor variance provided: " << basic_sensor_variance);
      required_parameters_received = false;
    }

    fake_sensor_dist_ = std::normal_distribution<> {0.0, basic_sensor_variance};

    param.description = "Max range for obstacle sensing (m).";
    declare_parameter("max_range", 1.0, param);
    max_range_ = get_parameter("max_range").get_parameter_value().get<double>();

    param.description = "Min range for lidar scanning (m).";
    declare_parameter("lidar.range_min", 0.12, param);
    lidar_scan_.range_min = get_parameter("lidar.range_min").get_parameter_value().get<double>();

    param.description = "Max range for lidar scanning (m).";
    declare_parameter("lidar.range_max", 3.5, param);
    lidar_scan_.range_max = get_parameter("lidar.range_max").get_parameter_value().get<double>();

    param.description = "Min angle for lidar scanning (rad).";
    declare_parameter("lidar.angle_min", 0.0, param);
    lidar_scan_.angle_min = get_parameter("lidar.angle_min").get_parameter_value().get<double>();

    param.description = "Max angle for lidar scanning (rad).";
    declare_parameter("lidar.angle_max", 2.0 * PI, param);
    lidar_scan_.angle_max = get_parameter("lidar.angle_max").get_parameter_value().get<double>();

    param.description = "Angle increment for lidar scanning (rad).";
    declare_parameter("lidar.angle_incr", PI / 180.0, param);
    lidar_scan_.angle_increment =
      get_parameter("lidar.angle_incr").get_parameter_value().get<double>();

    param.description = "Resolution for lidar scan (m). Set to 0 for perfect resolution.";
    declare_parameter("lidar.resolution", 0.0, param);
    lidar_resolution_ = get_parameter("lidar.resolution").get_parameter_value().get<double>();

    param.description =
      "Standard deviation for noise in lidar scan (m). Must be nonnegative.";
    declare_parameter("lidar.noise", 0.0, param);
    auto lidar_noise =
      get_parameter("lidar.noise").get_parameter_value().get<double>();

    if (!draw_only_ && lidar_noise < 0.0) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Invalid lidar noise provided: " << lidar_noise);
      required_parameters_received = false;
    }

    lidar_dist_ = std::normal_distribution<> {0.0, lidar_noise};

    //Abort if any required parameters were not provided
    if (!required_parameters_received) {
      throw std::logic_error(
              "Required parameters were not received or were invalid. Please provide valid parameters."
      );
    }

    //Timers
    timer_main_ = create_wall_timer(
      static_cast<std::chrono::microseconds>(static_cast<int>(sim_interval_ * 1000000.0)),
      std::bind(&NuSim::timer_main_callback, this)
    );

    if (!draw_only_) {
      timer_sensors_ = create_wall_timer(
        static_cast<std::chrono::milliseconds>(static_cast<int>(1000.0 / 5.0)), //5 Hz
        std::bind(&NuSim::timer_sensors_callback, this)
      );
      timer_path_ = create_wall_timer(
        static_cast<std::chrono::milliseconds>(static_cast<int>(path_interval * 1000.0)),
        std::bind(&NuSim::timer_path_callback, this)
      );
    }

    //Publishers
    pub_obstacles_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);

    if (!draw_only_) {
      pub_timestep_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
      pub_collision_cylinder_ =
        create_publisher<visualization_msgs::msg::Marker>("~/collision_cylinder", 10);
      pub_sensor_data_ = create_publisher<nuturtlebot_msgs::msg::SensorData>("sensor_data", 10);
      pub_fake_sensor_ = create_publisher<visualization_msgs::msg::MarkerArray>("fake_sensor", 10);
      pub_lidar_scan_ = create_publisher<sensor_msgs::msg::LaserScan>(
        "scan",
        rclcpp::SensorDataQoS());
      pub_path_ = create_publisher<nav_msgs::msg::Path>("~/path", 10);
    }

    //Subscribers
    if (!draw_only_) {
      sub_wheel_cmd_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
        "wheel_cmd",
        10,
        std::bind(&NuSim::wheel_cmd_callback, this, std::placeholders::_1)
      );
    }

    //Services
    if (!draw_only_) {
      srv_reset_ = create_service<std_srvs::srv::Empty>(
        "~/reset",
        std::bind(&NuSim::reset_callback, this, std::placeholders::_1, std::placeholders::_2)
      );
      srv_teleport_ = create_service<nusim::srv::Teleport>(
        "~/teleport",
        std::bind(&NuSim::teleport_callback, this, std::placeholders::_1, std::placeholders::_2)
      );
    }

    //Broadcasters
    if (!draw_only_) {
      broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    //Initialize other variables
    if (!draw_only_) {
      turtlebot_ = DiffDrive {
        wheel_track,
        wheel_radius,
        {
          translation_initial,
          rotation_initial
        }
      };

      turtlebot_last_config_ = turtlebot_.config();

      config_tf_msg_.header.frame_id = WORLD_FRAME;
      config_tf_msg_.child_frame_id = ROBOT_GROUND_TRUTH_FRAME;
      config_pose_msg_.header.frame_id = WORLD_FRAME;


      lidar_scan_.header.frame_id = LIDAR_GROUND_TRUTH_FRAME;
      //Reserve enough size in the ranges vector for all the angle increments
      lidar_scan_.ranges.reserve(
        static_cast<size_t>(
          std::ceil(
            (lidar_scan_.angle_max - lidar_scan_.angle_min) /
            lidar_scan_.angle_increment)));
      //Maximum possible detected items at a certain angle is number of obstacles + 4 walls
      possible_lidar_ranges_.reserve(obstacles_x_.size() + 4);

      path_.header.frame_id = WORLD_FRAME;

      config_pose_msg_.pose = tf_to_pose_msg(turtlebot_.config().location);
      config_pose_msg_.header.stamp = get_clock()->now();
      path_.poses.push_back(config_pose_msg_);
    }

    init_obstacles();

    if (!draw_only_) {
      RCLCPP_INFO_STREAM(get_logger(), "nusim node started");
    } else {
      RCLCPP_INFO_STREAM(get_logger(), "nuwall node started");
    }
  }

private:
  rclcpp::TimerBase::SharedPtr timer_main_;
  rclcpp::TimerBase::SharedPtr timer_sensors_;
  rclcpp::TimerBase::SharedPtr timer_path_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr pub_timestep_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_obstacles_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_collision_cylinder_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr pub_sensor_data_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_fake_sensor_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_lidar_scan_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr sub_wheel_cmd_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_reset_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr srv_teleport_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  double sim_rate_, sim_interval_;
  uint64_t timestep_ = 0;
  DiffDrive turtlebot_ {0.16, 0.033}; //Default values, to be overwritten in constructor
  DiffDriveConfig turtlebot_last_config_;
  geometry_msgs::msg::TransformStamped config_tf_msg_;
  Wheel wheel_vel_ {0.0, 0.0};
  std::vector<double> obstacles_x_, obstacles_y_;
  double obstacles_r_, x_length_, y_length_, max_range_, collision_radius_, collision_dist_;
  visualization_msgs::msg::MarkerArray detected_obstacles_, obstacle_and_wall_markers_;
  bool display_collision_cylinder_, allow_collision_rotation_, allow_collision_sliding_;
  visualization_msgs::msg::Marker collision_cylinder_marker_;
  std::vector<Transform2D> obstacle_abs_tfs_, obstacle_rel_tfs_;
  double motor_cmd_per_rad_sec_, encoder_ticks_per_rad_;
  int32_t motor_cmd_max_;
  rclcpp::Time current_time_;
  std::normal_distribution<> wheel_vel_dist_, fake_sensor_dist_, lidar_dist_;
  std::uniform_real_distribution<> slip_dist_;
  double lidar_resolution_;
  sensor_msgs::msg::LaserScan lidar_scan_;
  std::vector<double> possible_lidar_ranges_;
  std::vector<Line2D> wall_lines_;
  std::vector<Circle2D> obstacle_circles_;
  nav_msgs::msg::Path path_;
  geometry_msgs::msg::PoseStamped config_pose_msg_;
  size_t path_num_points_;
  bool draw_only_;

  /// \brief main simulation timer loop
  void timer_main_callback()
  {
    //Use a single time value for all publishing
    current_time_ = get_clock()->now();

    if (!draw_only_) {
      //Publish timestep and increment
      auto timestep_msg = std_msgs::msg::UInt64();
      timestep_msg.data = timestep_++;
      pub_timestep_->publish(timestep_msg);

      //Update wheel positions and publish
      update_wheel_pos_and_config();
    }

    //Publish markers
    publish_walls_and_obstacles();
  }

  /// \brief update the robot's wheel positions based on the current
  /// commanded velocity and update the robot's configuration based
  /// on the new wheel position
  void update_wheel_pos_and_config()
  {

    Wheel wheel_slip_vel = wheel_vel_;

    //Inject noise if our slip fraction is not 0
    if (slip_dist_.max() != 0.0) {
      wheel_slip_vel.left *= (1.0 + slip_dist_(get_random()));
      wheel_slip_vel.right *= (1.0 + slip_dist_(get_random()));
    }

    //Determine new actual wheel position using actual wheel velocity
    Wheel new_actual_wheel_pos {
      turtlebot_.config().wheel_pos.left + wheel_vel_.left * sim_interval_,
      turtlebot_.config().wheel_pos.right + wheel_vel_.right * sim_interval_,
    };

    //Determine artificial "slipped" wheel position using injected slip noise
    Wheel new_slip_wheel_pos {
      turtlebot_.config().wheel_pos.left + wheel_slip_vel.left * sim_interval_,
      turtlebot_.config().wheel_pos.right + wheel_slip_vel.right * sim_interval_,
    };

    //Store last configuration for collision detection
    turtlebot_last_config_ = turtlebot_.config();


    //Update configuration based on new slipped wheel position
    turtlebot_.update_config(new_slip_wheel_pos);

    //Replace artificial "slipped" wheel position with actual wheel position
    //Because slip only actually affects the location of the robot,
    //not the wheel positions
    turtlebot_.set_wheel_pos(new_actual_wheel_pos);

    //Detect and handle collisions
    collision_detection();

    //Publish new actual wheel position
    nuturtlebot_msgs::msg::SensorData sensor_data;
    sensor_data.stamp = current_time_;
    sensor_data.left_encoder = static_cast<int32_t>(
      turtlebot_.config().wheel_pos.left * encoder_ticks_per_rad_);
    sensor_data.right_encoder = static_cast<int32_t>(
      turtlebot_.config().wheel_pos.right * encoder_ticks_per_rad_);

    pub_sensor_data_->publish(sensor_data);

    //Broadcast updated transform of robot
    config_tf_msg_.transform = tf_to_tf_msg(turtlebot_.config().location);
    config_tf_msg_.header.stamp = current_time_;
    broadcaster_->sendTransform(config_tf_msg_);
  }

  /// \brief detect collisions and update robot position accordingly
  void collision_detection()
  {

    // Iterate through all obstacles and calculate relative transforms
    for (std::size_t i = 0; i < obstacle_abs_tfs_.size(); i++) {
      const auto & Tabs = obstacle_abs_tfs_.at(i);
      auto & Trel = obstacle_rel_tfs_.at(i);

      //Calculate relative transformation of obstacle to robot
      Trel = turtlebot_.config().location.inv() * Tabs;
    }

    //Check if we are colliding with any obstacle
    for (std::size_t i = 0; i < obstacle_rel_tfs_.size(); i++) {
      //If the magnitude of the translation between the obstacle and the robot is less
      //than the collision distance, a collision has occurred.
      if (obstacle_rel_tfs_.at(i).translation().magnitude() < collision_dist_) {
        const auto & Tabs = obstacle_abs_tfs_.at(i);

        //Find vector from obstacle to robot
        Vector2D v = turtlebot_last_config_.location.translation() - Tabs.translation();

        //New position should be the collision distance away from the obstacle in the direction
        //of this vector
        if (allow_collision_sliding_) {
          if (allow_collision_rotation_) {
            turtlebot_.set_location(
            {
              turtlebot_.config().location.translation() + normalize(v) *
              (collision_dist_ - obstacle_rel_tfs_.at(i).translation().magnitude()),                                                              //new location
              turtlebot_.config().location.rotation()             //new rotation
            });
          } else {
            turtlebot_.set_location(
            {
              turtlebot_.config().location.translation() + normalize(v) *
              (collision_dist_ - obstacle_rel_tfs_.at(i).translation().magnitude()),                                                              //new location
              turtlebot_last_config_.location.rotation()          //retain rotation
            });
          }
        } else {
          if (allow_collision_rotation_) {
            turtlebot_.set_location(
            {
              Tabs.translation() + normalize(v) * collision_dist_,  //new location
              turtlebot_.config().location.rotation()             //new rotation
            });
          } else {
            turtlebot_.set_location(
            {
              Tabs.translation() + normalize(v) * collision_dist_,  //new location
              turtlebot_last_config_.location.rotation()          //retain rotation
            });
          }
        }


        break;
      }
    }
  }

  /// \brief initialize the obstacle marker array.
  void init_obstacles()
  {

    visualization_msgs::msg::Marker marker;

    //Add walls
    marker.header.frame_id = WORLD_FRAME;
    marker.id = MARKER_ID_WALL;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = WALL_WIDTH;
    marker.scale.y = WALL_WIDTH;
    marker.scale.z = WALL_HEIGHT;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.points = {};

    auto x_bound = (x_length_ + WALL_WIDTH) / 2.0;
    auto y_bound = (y_length_ + WALL_WIDTH) / 2.0;

    geometry_msgs::msg::Point point;
    point.z = WALL_HEIGHT / 2.0;

    //Add top and bottom wall points
    for (double x = -x_bound; x <= x_bound; x += WALL_WIDTH) {
      //Bottom wall
      point.x = x;
      point.y = -y_bound;
      marker.points.push_back(point);

      //Top wall
      point.y = y_bound;
      marker.points.push_back(point);
    }

    //Add left and right wall points
    for (double y = -y_bound; y <= y_bound; y += WALL_WIDTH) {
      //Left wall
      point.x = -x_bound;
      point.y = y;
      marker.points.push_back(point);

      //Right wall
      point.x = x_bound;
      marker.points.push_back(point);
    }

    //Init wall lines
    auto x_half_length = x_length_ / 2.0;
    auto y_half_length = y_length_ / 2.0;

    wall_lines_.reserve(4);
    //bottom wall
    wall_lines_.push_back({{-x_half_length, -y_half_length}, {x_half_length, -y_half_length}});
    //top wall
    wall_lines_.push_back({{-x_half_length, y_half_length}, {x_half_length, y_half_length}});
    //left wall
    wall_lines_.push_back({{-x_half_length, -y_half_length}, {-x_half_length, y_half_length}});
    //right wall
    wall_lines_.push_back({{x_half_length, -y_half_length}, {x_half_length, y_half_length}});

    obstacle_and_wall_markers_.markers.push_back(marker);

    //Create markers from input lists
    for (size_t i = 0; i < obstacles_x_.size(); i++) {
      //Reset marker
      marker = visualization_msgs::msg::Marker {};

      marker.header.frame_id = WORLD_FRAME;
      marker.id = i + MARKER_ID_OFFSET_OBSTACLES;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = obstacles_x_.at(i);
      marker.pose.position.y = obstacles_y_.at(i);
      marker.pose.position.z = OBSTACLE_HEIGHT / 2.0;
      marker.scale.x = obstacles_r_ * 2.0;
      marker.scale.y = obstacles_r_ * 2.0;
      marker.scale.z = OBSTACLE_HEIGHT;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;

      //Append to marker array
      obstacle_and_wall_markers_.markers.push_back(marker);
      detected_obstacles_.markers.push_back(marker);

      //Store transform in world frame
      obstacle_abs_tfs_.push_back(
      {
        {
          marker.pose.position.x,
          marker.pose.position.y
        },
        0.0
      });

      //Store as circles for lidar sim
      obstacle_circles_.push_back(
      {
        {
          marker.pose.position.x,
          marker.pose.position.y
        },
        obstacles_r_
      });
    }

    //Make detected obstacles yellow and change their frame
    for (auto & marker : detected_obstacles_.markers) {
      marker.header.frame_id = ROBOT_GROUND_TRUTH_FRAME;
      marker.color.g = 1.0;
    }

    //Init relative transforms
    obstacle_rel_tfs_ = obstacle_abs_tfs_;

    //Init collision cylinder marker
    collision_cylinder_marker_.header.frame_id = ROBOT_GROUND_TRUTH_FRAME;
    collision_cylinder_marker_.id = MARKER_ID_COLLISION_CYLINDER;
    collision_cylinder_marker_.type = visualization_msgs::msg::Marker::CYLINDER;
    collision_cylinder_marker_.action = visualization_msgs::msg::Marker::ADD;
    collision_cylinder_marker_.pose.position.x = 0.0;
    collision_cylinder_marker_.pose.position.y = 0.0;
    collision_cylinder_marker_.pose.position.z = OBSTACLE_HEIGHT / 2.0;
    collision_cylinder_marker_.scale.x = collision_radius_ * 2.0;
    collision_cylinder_marker_.scale.y = collision_radius_ * 2.0;
    collision_cylinder_marker_.scale.z = OBSTACLE_HEIGHT;
    collision_cylinder_marker_.color.r = 1.0;
    collision_cylinder_marker_.color.g = 0.0;
    collision_cylinder_marker_.color.b = 0.0;
    collision_cylinder_marker_.color.a = 0.5;
    collision_cylinder_marker_.frame_locked = true;
  }

  /// \brief publish the obstacle marker array
  void publish_walls_and_obstacles()
  {

    //Update timestamps of all markers
    for (auto & marker : obstacle_and_wall_markers_.markers) {
      marker.header.stamp = current_time_;
    }

    //Publish marker array
    pub_obstacles_->publish(obstacle_and_wall_markers_);

    //Publish collision cylinder
    if (!draw_only_ && display_collision_cylinder_) {
      collision_cylinder_marker_.header.stamp = current_time_;
      pub_collision_cylinder_->publish(collision_cylinder_marker_);
    }
  }

  /// \brief publish fake sensor and lidar data with noise
  void timer_sensors_callback()
  {
    //fake sensor
    publish_fake_sensor();

    publish_lidar_scan();
  }

  /// \brief publish fake sensor data with noise
  void publish_fake_sensor()
  {
    for (std::size_t i = 0; i < obstacle_rel_tfs_.size(); i++) {
      const auto & Trel = obstacle_rel_tfs_.at(i);
      auto & marker = detected_obstacles_.markers.at(i);

      //Update stamp, match last simulation timestamp
      marker.header.stamp = current_time_;

      //Update marker position
      marker.pose.position.x = Trel.translation().x;
      marker.pose.position.y = Trel.translation().y;

      //If the obstacle is outside of sensor range, delete it
      if (Trel.translation().magnitude() > max_range_) {
        marker.action = visualization_msgs::msg::Marker::DELETE;
        continue;
      }

      //Otherwise, add marker back
      marker.action = visualization_msgs::msg::Marker::ADD;

      //Add sensor noise if there is nonzero standard deviation
      if (fake_sensor_dist_.stddev() != 0.0) {
        marker.pose.position.x += fake_sensor_dist_(get_random());
        marker.pose.position.y += fake_sensor_dist_(get_random());
      }
    }

    //Publish marker array
    pub_fake_sensor_->publish(detected_obstacles_);
  }

  /// \brief publish lidar sensor data with noise
  void publish_lidar_scan()
  {

    //Update stamp to match simulation
    lidar_scan_.header.stamp = current_time_;

    //Clear range list
    lidar_scan_.ranges.clear();

    //Temp variables
    double range;
    double world_angle;
    Vector2D scan_start;
    Vector2D scan_end;
    Line2D scan {scan_start, scan_end};
    std::vector<Vector2D> intersection_points;

    //calculate 2D location of lidar frame
    auto Tlidar = turtlebot_.config().location * ROBOT_LIDAR_TF;

    for (auto angle = lidar_scan_.angle_min; angle < lidar_scan_.angle_max;
      angle += lidar_scan_.angle_increment)
    {
      //Clear list of possible ranges
      possible_lidar_ranges_.clear();

      //Find angle of scan in world frame
      world_angle = Tlidar.rotation() + angle;

      //Construct laser scan line segment using trigonometry and the current position of the turtlebot
      //Start point of scan is at min scan range at the world angle away from the robot's position
      scan_start = Vector2D {
        Tlidar.translation().x + lidar_scan_.range_min * std::cos(world_angle),
        Tlidar.translation().y + lidar_scan_.range_min * std::sin(world_angle)
      };

      //End point of scan is at max scan range at the world angle away from the robot's position
      scan_end = Vector2D {
        Tlidar.translation().x + lidar_scan_.range_max * std::cos(world_angle),
        Tlidar.translation().y + lidar_scan_.range_max * std::sin(world_angle)
      };

      //Construct line segment representing the scan
      scan = Line2D {scan_start, scan_end};

      //Determine if walls intersect
      for (const auto & wall : wall_lines_) {
        intersection_points = find_intersection(scan, wall);

        if (intersection_points.size() != 0) { //intersection occurred
          //Calculate possible range by finding the magnitude of the vector between the
          //intersection point and the turtlebot's current location. Append to array
          possible_lidar_ranges_.push_back(
            (intersection_points.at(0) - Tlidar.translation()).magnitude()
          );
        }
      }

      //Determine if obstacles intersect
      for (const auto & obstacle : obstacle_circles_) {
        intersection_points = find_intersection(scan, obstacle);

        if (intersection_points.size() != 0) { //intersection occurred
          std::vector<double> point_ranges;
          for (const auto & point : intersection_points) {
            point_ranges.push_back(
              (point - Tlidar.translation()).magnitude()
            );
          }

          //Add minimum range to possible lidar ranges
          possible_lidar_ranges_.push_back(
            *std::min_element(
              point_ranges.begin(),
              point_ranges.end()));
        }
      }

      //If there are any possible ranges, find the minimum and use it for
      // the detected range at this angle
      if (possible_lidar_ranges_.size() > 0) {
        range = *std::min_element(possible_lidar_ranges_.begin(), possible_lidar_ranges_.end());

        //Add sensor noise if there is nonzero standard deviation
        if (lidar_dist_.stddev() != 0.0) {
          range += lidar_dist_(get_random());
        }

      } else {
        //Default range is 0
        range = 0.0;
      }

      //Round to nearest resolution mark
      if (lidar_resolution_ > 0.0) {
        range = std::round(range / lidar_resolution_) * lidar_resolution_;
      }

      lidar_scan_.ranges.push_back(range);
    }


    //Publish message
    pub_lidar_scan_->publish(lidar_scan_);
  }

  /// \brief publish ground truth path
  void timer_path_callback()
  {
    static Transform2D last_published_tf = turtlebot_.config().location;

    //Only add a new pose to the path if the turtlebot has moved
    if (!almost_equal(last_published_tf, turtlebot_.config().location)) {

      config_pose_msg_.pose = tf_to_pose_msg(turtlebot_.config().location);
      config_pose_msg_.header.stamp = current_time_;

      //Remove oldest element if we've reached the max number of path points
      if (path_num_points_ != 0 && path_.poses.size() >= path_num_points_) {
        path_.poses.erase(path_.poses.begin());
      }

      path_.poses.push_back(config_pose_msg_);

      last_published_tf = turtlebot_.config().location;
    }

    //Update stamp
    path_.header.stamp = current_time_;

    //Publish path
    pub_path_->publish(path_);
  }

  /// \brief convert and store received wheel commands
  /// \param msg - received wheel command message
  void wheel_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands & msg)
  {
    //Clamp velocities to max allowed and
    //store wheel velocities in rad/s
    wheel_vel_.left = static_cast<double>(
      std::clamp(msg.left_velocity, -motor_cmd_max_, motor_cmd_max_)
      ) / motor_cmd_per_rad_sec_;
    wheel_vel_.right = static_cast<double>(
      std::clamp(msg.right_velocity, -motor_cmd_max_, motor_cmd_max_)
      ) / motor_cmd_per_rad_sec_;

    //If wheel command is not 0 and we have nonzero standard deviation, inject input noise
    if (wheel_vel_dist_.stddev() != 0.0) {
      if (!almost_equal(wheel_vel_.left, 0.0)) {
        wheel_vel_.left += wheel_vel_dist_(get_random());
      }

      if (!almost_equal(wheel_vel_.right, 0.0)) {
        wheel_vel_.right += wheel_vel_dist_(get_random());
      }
    }
  }

  /// \brief reset simulation back to initial parameters. callback for ~/reset service.
  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>
  )
  {
    //Reset simulation timestep
    timestep_ = 0;

    //Reset current pose
    turtlebot_.reset();
  }

  /// \brief teleport the actual robot to a specified pose. callback for ~/teleport service.
  /// \param request - pose data to which to teleport the robot.
  void teleport_callback(
    const std::shared_ptr<nusim::srv::Teleport::Request> request,
    std::shared_ptr<nusim::srv::Teleport::Response>
  )
  {
    //Teleport current pose, retain current wheel positions
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


/// \brief set up and return a reference to a psuedo-random number generator object
/// \return the reference to the psuedo-random number generator object
std::mt19937 & get_random()
{
  // static variables inside a function are created once and persist for the remainder of the program
  static std::random_device rd{};
  static std::mt19937 mt{rd()};
  // we return a reference to the pseudo-random number generator object. This is always the
  // same object every time get_random is called
  return mt;
}

/// \brief Run the node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NuSim>());
  rclcpp::shutdown();
  return 0;
}
