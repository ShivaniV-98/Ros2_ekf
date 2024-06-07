/// \file
/// \brief Calculates extended Kalman Filter SLAM based on odometry and fake sensor data.
///
/// PARAMETERS:
///     body_id (string): The name of the robot's body frame (REQUIRED)
///     odom_id (string): The name of the robot's odometry frame
///     wheel_left (string): The name of the robot's left wheel joint (REQUIRED)
///     wheel_right (string): The name of the robot's right wheel joint (REQUIRED)
///     track_width (double): The wheel track width in meters (REQUIRED)
///     wheel_radius (double): The wheel radius in meters (REQUIRED)
///     path.num_points (int): Number of path points retained before deleting. Set to 0 to disable limit.
///     kalman.process_noise.theta (double): Kalman filter process noise for theta coordinate.
///     kalman.process_noise.x (double): Kalman filter process noise for x coordinate.
///     kalman.process_noise.y (double): Kalman filter process noise for y coordinate.
///     kalman.sensor_noise (double): Kalman filter sensor noise.
///     use_fake_sensor (bool): Use fake sensor data instead of lidar data.
///     mahalanobis.threshold (double): Mahalanobis distance threshold for a new landmark.
/// PUBLISHERS:
///     odom (nav_msgs/msg/Odometry): the calculated odometry from the input joint states
///     path (nav_msgs/msg/Path): the path data determined from the EKF SLAM calculations
///     estimated_landmarks (visualization_msgs/msg/MarkerArray): the estimated map locations of landmarks from the EKF SLAM calculations
/// SUBSCRIBERS:
///     joint_states (sensor_msgs/msg/JointState): the robot's joint states
///     fake_sensor (visualization_msgs/msg/MarkerArray): fake sensor data for testing EKF SLAM
///     landmarks (nuslam::msg::Landmarks): detected landmarks for doing EKF SLAM with lidar data
/// SERVERS:
///     none
/// CLIENTS:
///     none


#include <stdexcept>
#include <string>
#include <array>
#include <armadillo>
#include <cmath>
#include <tuple>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nuslam/msg/landmarks.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib_ros/convert.hpp"

using turtlelib::DiffDrive;
using turtlelib::DiffDriveConfig;
using turtlelib::Wheel;
using turtlelib::Transform2D;
using turtlelib::Vector2D;
using turtlelib::Twist2D;
using turtlelib::almost_equal;
using turtlelib::normalize_angle;
using turtlelib_ros::tf_to_pose_msg;
using turtlelib_ros::tf_to_tf_msg;

using std::sin;
using std::cos;
using arma::mat;
using arma::vec;
using arma::eye;
using arma::fill::zeros;
using arma::as_scalar;

//Constants
constexpr std::string_view MAP_FRAME = "map";
constexpr int MAX_LANDMARKS = 20; //adjust size here if necessary, would be good to make a parameter
constexpr int STATE_SIZE = 2 * MAX_LANDMARKS + 3;
constexpr double VERY_LARGE_NUMBER = 1e10;
constexpr double LANDMARK_HEIGHT = 0.25;
constexpr double LANDMARK_RADIUS = 0.038;
constexpr int32_t MARKER_ID_OFFSET_LANDMARKS = 100;
const Transform2D ROBOT_LIDAR_TF {{-0.032, 0.0}};

//Function prototypes
std::tuple<double, double> relative_to_range_bearing(double x, double y);

class NuSlam : public rclcpp::Node
{
public:
  /// \brief initialize the node
  NuSlam()
  : Node("nuslam")
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

    param.description =
      "Number of path points retained before deleting. Set to 0 to disable limit.";
    declare_parameter("path.num_points", 100, param);
    path_num_points_ = get_parameter(
      "path.num_points").get_parameter_value().get<size_t>();

    param.description = "Kalman filter process noise for theta coordinate.";
    declare_parameter("kalman.process_noise.theta", 0.001, param);
    auto kalman_process_noise_theta = get_parameter(
      "kalman.process_noise.theta").get_parameter_value().get<double>();
    param.description = "Kalman filter process noise for x coordinate.";
    declare_parameter("kalman.process_noise.x", 0.001, param);
    auto kalman_process_noise_x = get_parameter(
      "kalman.process_noise.x").get_parameter_value().get<double>();
    param.description = "Kalman filter process noise for y coordinate.";
    declare_parameter("kalman.process_noise.y", 0.001, param);
    auto kalman_process_noise_y = get_parameter(
      "kalman.process_noise.y").get_parameter_value().get<double>();

    param.description = "Kalman filter sensor noise.";
    declare_parameter("kalman.sensor_noise", 0.1, param);
    auto kalman_sensor_noise = get_parameter(
      "kalman.sensor_noise").get_parameter_value().get<double>();

    param.description = "Use fake sensor data instead of lidar data.";
    declare_parameter("use_fake_sensor", true, param);
    auto use_fake_sensor = get_parameter("use_fake_sensor").get_parameter_value().get<bool>();

    param.description = "Mahalanobis distance threshold for a new landmark.";
    declare_parameter("mahalanobis.threshold", 0.1, param);
    mahalanobis_threshold_ = get_parameter(
      "mahalanobis.threshold").get_parameter_value().get<double>();


    //Abort if any required parameters were not provided
    if (!required_parameters_received) {
      throw std::logic_error(
              "Required parameters were not received or were invalid. Please provide valid parameters."
      );
    }

    //Publishers
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    pub_path_ = create_publisher<nav_msgs::msg::Path>("path", 10);
    pub_estimated_landmarks_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "estimated_landmarks", 10);

    //Subscribers
    sub_joint_states_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states",
      10,
      std::bind(&NuSlam::joint_states_callback, this, std::placeholders::_1)
    );
    if (use_fake_sensor) {
      sub_fake_sensor_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "fake_sensor",
        10,
        std::bind(&NuSlam::fake_sensor_callback, this, std::placeholders::_1)
      );
    } else {
      sub_landmarks_ = create_subscription<nuslam::msg::Landmarks>(
        "landmarks",
        10,
        std::bind(&NuSlam::landmarks_callback, this, std::placeholders::_1)
      );
    }

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

    //SLAM
    slam_last_odom_location_ = turtlebot_.config().location;

    //Leave initial robot covariance estimate as 0s
    //update object covariance estimates as very large numbers (infinity)
    for (int i = 3; i < STATE_SIZE; i++) {
      slam_last_covariance_(i, i) = VERY_LARGE_NUMBER;
    }

    //Q_bar matrix
    slam_process_noise_(0, 0) = kalman_process_noise_theta;
    slam_process_noise_(1, 1) = kalman_process_noise_x;
    slam_process_noise_(2, 2) = kalman_process_noise_y;

    //R matrix
    for (int i = 0; i < 2 * MAX_LANDMARKS; i++) {
      slam_sensor_noise_(i, i) = kalman_sensor_noise;
    }

    //SLAM TFs
    map_odom_tf_.header.frame_id = MAP_FRAME;
    map_odom_tf_.child_frame_id = odom_id_;
    map_odom_tf_.transform.translation.z = 0.0;

    //Default landmark marker
    default_landmark_.header.frame_id = MAP_FRAME;
    default_landmark_.type = visualization_msgs::msg::Marker::CYLINDER;
    default_landmark_.action = visualization_msgs::msg::Marker::ADD;
    default_landmark_.pose.position.z = LANDMARK_HEIGHT / 2.0;
    default_landmark_.scale.x = LANDMARK_RADIUS * 2.0;
    default_landmark_.scale.y = LANDMARK_RADIUS * 2.0;
    default_landmark_.scale.z = LANDMARK_HEIGHT;
    default_landmark_.color.r = 0.0;
    default_landmark_.color.g = 1.0;
    default_landmark_.color.b = 0.0;
    default_landmark_.color.a = 1.0;

    //Path from SLAM
    path_.header.frame_id = MAP_FRAME;

    config_pose_msg_.pose = tf_to_pose_msg(turtlebot_.config().location);
    config_pose_msg_.header.stamp = get_clock()->now();
    path_.poses.push_back(config_pose_msg_);

    RCLCPP_INFO_STREAM(get_logger(), "nuslam node started");
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_estimated_landmarks_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_fake_sensor_;
  rclcpp::Subscription<nuslam::msg::Landmarks>::SharedPtr sub_landmarks_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  std::string body_id_, odom_id_, wheel_left_joint_, wheel_right_joint_;
  DiffDrive turtlebot_ {0.16, 0.033}; //Default values, to be overwritten in constructor
  nav_msgs::msg::Odometry odom_msg_;
  geometry_msgs::msg::TransformStamped odom_tf_, map_odom_tf_;
  bool first_joint_states_ = true;
  nav_msgs::msg::Path path_;
  geometry_msgs::msg::PoseStamped config_pose_msg_;
  size_t path_num_points_;
  Transform2D slam_last_odom_location_;
  Transform2D slam_map_odom_tf_;
  vec slam_last_state_ {STATE_SIZE, zeros}; //Init robot state guess to (0,0,0)
  mat slam_last_covariance_ {STATE_SIZE, STATE_SIZE, zeros}; //epsilon_t-1
  mat slam_process_noise_ {STATE_SIZE, STATE_SIZE, zeros}; //Q_bar
  mat slam_sensor_noise_ {2 * MAX_LANDMARKS, 2 * MAX_LANDMARKS, zeros}; //R
  std::vector<bool> slam_landmark_seen_ = std::vector<bool>(MAX_LANDMARKS, false);
  size_t slam_landmark_count_ = 0;
  double mahalanobis_threshold_;
  visualization_msgs::msg::Marker default_landmark_;


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

  /// \brief Subscribe to fake sensor and compute extended Kalman filter SLAM based on measurements
  /// \param msg - contains markers which represent the fake sensor measurements
  void fake_sensor_callback(const visualization_msgs::msg::MarkerArray & msg)
  {

    //Calculate new location of robot in map frame
    auto map_robot_tf = slam_map_odom_tf_ * turtlebot_.config().location;

    //Construct identity matrix of the state size
    mat I = eye<mat>(STATE_SIZE, STATE_SIZE);

    //KALMAN FILTER PREDICTION
    //Calculate state prediction
    vec state_prediction = slam_last_state_;

    //Temporary references for readability
    auto & state_prediction_theta = state_prediction(0);
    auto & state_prediction_x = state_prediction(1);
    auto & state_prediction_y = state_prediction(2);

    state_prediction_theta = normalize_angle(map_robot_tf.rotation());
    state_prediction_x = map_robot_tf.translation().x;
    state_prediction_y = map_robot_tf.translation().y;

    //Construct A matrix
    mat A = I;
    A(1, 0) += -(state_prediction_y - slam_last_state_(2));
    A(2, 0) += state_prediction_x - slam_last_state_(1);

    //Calculate covariance prediction
    mat covariance_prediction = A * slam_last_covariance_ * A.t() + slam_process_noise_;


    //KALMAN FILTER CORRECTION
    //iterate through sensor measurements

    for (size_t i = 0; i < msg.markers.size(); i++) {
      const auto & marker = msg.markers.at(i);

      //Ignore any markers that are marked to "DELETE", they are out of range
      if (marker.action == visualization_msgs::msg::Marker::DELETE) {
        continue;
      }

      //Temporary references for readability
      auto & state_prediction_m_x = state_prediction(3 + 2 * i);
      auto & state_prediction_m_y = state_prediction(3 + 2 * i + 1);

      //Get range bearing measurement out of marker
      auto [range, bearing] = relative_to_range_bearing(
        marker.pose.position.x,
        marker.pose.position.y);

      bearing = normalize_angle(bearing);

      if (!slam_landmark_seen_.at(i)) {
        slam_landmark_seen_.at(i) = true;

        //Initialize landmark measurement
        state_prediction_m_x = state_prediction_x + range * cos(bearing + state_prediction_theta);
        state_prediction_m_y = state_prediction_y + range * sin(bearing + state_prediction_theta);
      }

      //Compute quantities for later use
      const auto del_x = state_prediction_m_x - state_prediction_x;
      const auto del_y = state_prediction_m_y - state_prediction_y;
      const auto d = std::pow(del_x, 2) + std::pow(del_y, 2);
      const auto sqrt_d = std::sqrt(d);

      //Construct Hi matrix
      mat Hi {2, STATE_SIZE, zeros};
      Hi(1, 0) = -1.0;
      Hi(0, 1) = -del_x / sqrt_d;
      Hi(1, 1) = del_y / d;
      Hi(0, 2) = -del_y / sqrt_d;
      Hi(1, 2) = -del_x / d;
      Hi(0, 3 + 2 * i) = del_x / sqrt_d;
      Hi(1, 3 + 2 * i) = -del_y / d;
      Hi(0, 3 + 2 * i + 1) = del_y / sqrt_d;
      Hi(1, 3 + 2 * i + 1) = del_x / d;

      //Get transpose
      mat Hi_t = Hi.t();

      //Compute the theoretical landmark measurement given the current state estimate
      vec meas_theo {sqrt_d, normalize_angle(std::atan2(del_y, del_x) - state_prediction_theta)};

      //Construct actual landmark measurement vector
      vec meas_act {range, bearing};

      //Calculate Kalman gain for this landmark
      mat Ki = covariance_prediction * Hi_t *
        (Hi * covariance_prediction * Hi_t +
        slam_sensor_noise_.submat(2 * i, 2 * i, 2 * i + 1, 2 * i + 1)).i();

      //calculate the difference in the measurements
      mat meas_delta = meas_act - meas_theo;
      meas_delta(1) = normalize_angle(meas_delta(1));

      //Update the state prediction
      state_prediction += Ki * (meas_delta);

      //Normalize angle again
      state_prediction_theta = normalize_angle(state_prediction_theta);

      //Update the covariance prediction
      covariance_prediction = (I - Ki * Hi) * covariance_prediction;
    }

    auto slam_time = get_clock()->now();

    //Only add a new pose to the path if the turtlebot has moved
    if (!(
        almost_equal(state_prediction(0), slam_last_state_(0)) &&
        almost_equal(state_prediction(1), slam_last_state_(1)) &&
        almost_equal(state_prediction(2), slam_last_state_(2))
    ))
    {

      config_pose_msg_.pose.position.x = state_prediction_x;
      config_pose_msg_.pose.position.y = state_prediction_y;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, state_prediction_theta);
      config_pose_msg_.pose.orientation = tf2::toMsg(q);
      config_pose_msg_.header.stamp = slam_time;

      //Remove oldest element if we've reached the max number of path points
      if (path_num_points_ != 0 && path_.poses.size() >= path_num_points_) {
        path_.poses.erase(path_.poses.begin());
      }

      path_.poses.push_back(config_pose_msg_);
    }

    //Update stamp
    path_.header.stamp = slam_time;

    //Publish path
    pub_path_->publish(path_);

    //Determine the map to odometry transformation
    map_robot_tf = Transform2D {{state_prediction_x, state_prediction_y}, state_prediction_theta};

    slam_map_odom_tf_ = map_robot_tf * turtlebot_.config().location.inv();

    //Build transform
    map_odom_tf_.transform = tf_to_tf_msg(slam_map_odom_tf_);
    map_odom_tf_.header.stamp = slam_time;

    //Broadcast transform
    broadcaster_->sendTransform(map_odom_tf_);

    //Build marker array
    visualization_msgs::msg::MarkerArray estimated_landmarks;

    for (size_t i = 0; i < MAX_LANDMARKS; i++) {

      //Skip marker if it hasn't been seen
      if (!slam_landmark_seen_.at(i)) {continue;}

      auto marker = default_landmark_;
      marker.id = i + MARKER_ID_OFFSET_LANDMARKS;
      marker.pose.position.x = state_prediction(3 + 2 * i);
      marker.pose.position.y = state_prediction(3 + 2 * i + 1);

      estimated_landmarks.markers.push_back(marker);
    }

    pub_estimated_landmarks_->publish(estimated_landmarks);

    //Save last odom config
    slam_last_odom_location_ = turtlebot_.config().location;

    //Save last state prediction
    slam_last_state_ = state_prediction;

    //Save last covariance prediction;
    slam_last_covariance_ = covariance_prediction;
  }

  void landmarks_callback(const nuslam::msg::Landmarks & msg)
  {
    //DEBUG //RCLCPP_INFO_STREAM(get_logger(), "Landmark callback start");

    //Calculate new location of robot in map frame
    auto map_robot_tf = slam_map_odom_tf_ * turtlebot_.config().location;

    //Construct identity matrix of the state size
    mat I = eye<mat>(STATE_SIZE, STATE_SIZE);

    //KALMAN FILTER PREDICTION
    //Calculate state prediction
    vec state_prediction = slam_last_state_;

    //Temporary references for readability
    auto & state_prediction_theta = state_prediction(0);
    auto & state_prediction_x = state_prediction(1);
    auto & state_prediction_y = state_prediction(2);

    state_prediction_theta = normalize_angle(map_robot_tf.rotation());
    state_prediction_x = map_robot_tf.translation().x;
    state_prediction_y = map_robot_tf.translation().y;

    //Construct A matrix
    mat A = I;
    A(1, 0) += -(state_prediction_y - slam_last_state_(2));
    A(2, 0) += state_prediction_x - slam_last_state_(1);

    //Calculate covariance prediction
    mat covariance_prediction = A * slam_last_covariance_ * A.t() + slam_process_noise_;

    //iterate through sensor measurements
    for (const auto & landmark : msg.landmarks) {

      //DATA ASSOCIATION
      //Init landmark_index to highest index + 1 (N - 1 + 1 = N)
      auto landmark_index = slam_landmark_count_;
      //Init min_distance of any landmark to the new measurement to
      //the mahalanobis threshold distance
      auto min_distance = mahalanobis_threshold_;

      //Correct for lidar offset
      const auto Tlandmark = ROBOT_LIDAR_TF * Transform2D{{landmark.center.x, landmark.center.y}};


      const auto Tlandmark_abs = map_robot_tf * Tlandmark;
      //DEBUG //RCLCPP_INFO_STREAM(get_logger(), "X: " << Tlandmark_abs.translation().x << " Y: " << Tlandmark_abs.translation().y);

      //Get range bearing measurement out of marker
      auto [range, bearing] = relative_to_range_bearing(
        Tlandmark.translation().x,
        Tlandmark.translation().y);

      bearing = normalize_angle(bearing);

      //Construct actual landmark measurement vector
      vec meas_act {range, bearing};

      //Iterate through previously found landmarks
      for (size_t k = 0; k < slam_landmark_count_; k++) {

        //Temporary references for readability
        auto & state_prediction_m_x = state_prediction(3 + 2 * k);
        auto & state_prediction_m_y = state_prediction(3 + 2 * k + 1);

        //Compute quantities for later use
        const auto del_x = state_prediction_m_x - state_prediction_x;
        const auto del_y = state_prediction_m_y - state_prediction_y;
        const auto d = std::pow(del_x, 2) + std::pow(del_y, 2);
        const auto sqrt_d = std::sqrt(d);

        //Construct Hk matrix
        mat Hk {2, STATE_SIZE, zeros};
        Hk(1, 0) = -1.0;
        Hk(0, 1) = -del_x / sqrt_d;
        Hk(1, 1) = del_y / d;
        Hk(0, 2) = -del_y / sqrt_d;
        Hk(1, 2) = -del_x / d;
        Hk(0, 3 + 2 * k) = del_x / sqrt_d;
        Hk(1, 3 + 2 * k) = -del_y / d;
        Hk(0, 3 + 2 * k + 1) = del_y / sqrt_d;
        Hk(1, 3 + 2 * k + 1) = del_x / d;

        //Compute the covariance psi_k
        mat covariance = Hk * covariance_prediction * Hk.t() +
          slam_sensor_noise_.submat(2 * k, 2 * k, 2 * k + 1, 2 * k + 1);

        //Compute the theoretical sensor measurement zhat_k given the current state estimate
        vec meas_theo {sqrt_d, normalize_angle(std::atan2(del_y, del_x) - state_prediction_theta)};

        //Compute the difference between the measurements
        mat meas_delta = meas_act - meas_theo;
        meas_delta(1) = normalize_angle(meas_delta(1));

        //Compute the mahalanobis distance
        const auto mahalanobis_distance = as_scalar((meas_delta.t() * covariance.i() * meas_delta));

        //DEBUG //RCLCPP_INFO_STREAM(get_logger(), "Mahalanobis Distance: " << mahalanobis_distance);

        //If this mahalanobis distance is less than the stored minimum distance,
        //update the stored minimum distance and store the index of the corresponding landmark
        if (mahalanobis_distance < min_distance) {
          min_distance = mahalanobis_distance;
          landmark_index = k;
        }
      }


      //DEBUG //RCLCPP_INFO_STREAM(get_logger(), "Landmark count: " << slam_landmark_count_);

      //If our landmark index is still its initial value (N), we have a new landmark
      if (landmark_index == slam_landmark_count_) {

        RCLCPP_INFO_STREAM(
          get_logger(),
          "New landmark found at (" << Tlandmark_abs.translation().x << ", " << Tlandmark_abs.translation().y <<
            ")");

        //Throw an error if we have too many landmarks
        if (slam_landmark_count_ == MAX_LANDMARKS) {
          RCLCPP_ERROR_STREAM(
            get_logger(), "Landmark count exceed maximum but new landmark detected, ignoring.");
          RCLCPP_INFO_STREAM(get_logger(), "Landmark count: " << slam_landmark_count_);
          continue;
        }

        //Increment landmark counter
        slam_landmark_count_++;

        RCLCPP_INFO_STREAM(get_logger(), "Landmark count: " << slam_landmark_count_);

        //Initialize landmark measurement
        state_prediction(3 + 2 * landmark_index) = Tlandmark.translation().x;
        state_prediction(3 + 2 * landmark_index + 1) = Tlandmark.translation().y;
      }

      //KALMAN FILTER CORRECTION
      //perform correction with associated landmark
      //Temporary references for readability
      auto & state_prediction_m_x = state_prediction(3 + 2 * landmark_index);
      auto & state_prediction_m_y = state_prediction(3 + 2 * landmark_index + 1);

      //Compute quantities for later use
      const auto del_x = state_prediction_m_x - state_prediction_x;
      const auto del_y = state_prediction_m_y - state_prediction_y;
      const auto d = std::pow(del_x, 2) + std::pow(del_y, 2);
      const auto sqrt_d = std::sqrt(d);

      //Construct Hk matrix
      mat Hk {2, STATE_SIZE, zeros};
      Hk(1, 0) = -1.0;
      Hk(0, 1) = -del_x / sqrt_d;
      Hk(1, 1) = del_y / d;
      Hk(0, 2) = -del_y / sqrt_d;
      Hk(1, 2) = -del_x / d;
      Hk(0, 3 + 2 * landmark_index) = del_x / sqrt_d;
      Hk(1, 3 + 2 * landmark_index) = -del_y / d;
      Hk(0, 3 + 2 * landmark_index + 1) = del_y / sqrt_d;
      Hk(1, 3 + 2 * landmark_index + 1) = del_x / d;

      //Get transpose
      mat Hk_t = Hk.t();

      //Compute the theoretical landmark measurement given the current state estimate
      vec meas_theo {sqrt_d, normalize_angle(std::atan2(del_y, del_x) - state_prediction_theta)};

      //Calculate Kalman gain for this landmark
      mat Kk = covariance_prediction * Hk_t *
        (Hk * covariance_prediction * Hk_t +
        slam_sensor_noise_.submat(
          2 * landmark_index, 2 * landmark_index, 2 * landmark_index + 1,
          2 * landmark_index + 1)).i();

      //calculate the difference in the measurements
      mat meas_delta = meas_act - meas_theo;
      meas_delta(1) = normalize_angle(meas_delta(1));

      //Update the state prediction
      state_prediction += Kk * (meas_delta);

      //Normalize angle again
      state_prediction_theta = normalize_angle(state_prediction_theta);

      //Update the covariance prediction
      covariance_prediction = (I - Kk * Hk) * covariance_prediction;

    }


    auto slam_time = get_clock()->now();

    //Only add a new pose to the path if the turtlebot has moved
    if (!(
        almost_equal(state_prediction(0), slam_last_state_(0)) &&
        almost_equal(state_prediction(1), slam_last_state_(1)) &&
        almost_equal(state_prediction(2), slam_last_state_(2))
    ))
    {

      config_pose_msg_.pose.position.x = state_prediction_x;
      config_pose_msg_.pose.position.y = state_prediction_y;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, state_prediction_theta);
      config_pose_msg_.pose.orientation = tf2::toMsg(q);
      config_pose_msg_.header.stamp = slam_time;

      //Remove oldest element if we've reached the max number of path points
      if (path_num_points_ != 0 && path_.poses.size() >= path_num_points_) {
        path_.poses.erase(path_.poses.begin());
      }

      path_.poses.push_back(config_pose_msg_);
    }

    //Update stamp
    path_.header.stamp = slam_time;

    //Publish path
    pub_path_->publish(path_);

    //Determine the map to odometry transformation
    map_robot_tf = Transform2D {{state_prediction_x, state_prediction_y}, state_prediction_theta};

    slam_map_odom_tf_ = map_robot_tf * turtlebot_.config().location.inv();

    //Build transform
    map_odom_tf_.transform = tf_to_tf_msg(slam_map_odom_tf_);
    map_odom_tf_.header.stamp = slam_time;

    //Broadcast transform
    broadcaster_->sendTransform(map_odom_tf_);

    //Build marker array
    visualization_msgs::msg::MarkerArray estimated_landmarks;

    for (size_t k = 0; k < slam_landmark_count_; k++) {

      auto marker = default_landmark_;
      marker.id = k + MARKER_ID_OFFSET_LANDMARKS;
      marker.pose.position.x = state_prediction(3 + 2 * k);
      marker.pose.position.y = state_prediction(3 + 2 * k + 1);

      estimated_landmarks.markers.push_back(marker);
    }

    pub_estimated_landmarks_->publish(estimated_landmarks);

    //Save last odom config
    slam_last_odom_location_ = turtlebot_.config().location;

    //Save last state prediction
    slam_last_state_ = state_prediction;

    //Save last covariance prediction;
    slam_last_covariance_ = covariance_prediction;
  }
};


/// \brief convert a relative x and y measurement into a range bearing measurement
/// \param x - relative x coordinate of measured object
/// \param y - relative y coordinate of measured object
/// \return a tuple where the first element is the range of the measured object
/// and the second element is the bearing of the measured object
std::tuple<double, double> relative_to_range_bearing(double x, double y)
{
  return {
    std::sqrt(std::pow(x, 2) + std::pow(y, 2)),
    std::atan2(y, x)
  };
}

/// \brief Run the node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NuSlam>());
  rclcpp::shutdown();
  return 0;
}
