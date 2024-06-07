/// \file
/// \brief Detects landmarks from lidar data.
///
/// PARAMETERS:
///     clusters.visualize (bool): Controls whether clustered points are published as markers.
///     clusters.threshold (double): Euclidean distance between points to be considered part of the same cluster.
///     circles.visualize (bool): Controls whether fit circles are published as markers.
///     circles.classification.mean_min (double): Minimum mean angle for considering a cluster a circle (deg).
///     circles.classification.mean_max (double): Maximum mean angle for considering a cluster a circle (deg).
///     circles.classification.std_dev_max (double): Maximum standard deviation for considering a cluster a circle (deg).
///     circles.radius_min (double): Radius minimum for considering a circle fit as a legitimate circle.
///     circles.radius_max (double): Radius maximum for considering a circle fit as a legitimate circle.
/// PUBLISHERS:
///     clusters (visualization_msgs/msg/MarkerArray): cylinder markers representing the detected and filtered clusters
///     circles (visualization_msgs/msg/MarkerArray): cylinder markers representing the detected and filtered circles
///     landmarks (nuslam/msg/landmarks): an array of detected landmark locations.
/// SUBSCRIBERS:
///     scan (sensor_msgs/msg/LaserScan): lidar data to use for landmark detection
/// SERVERS:
///     none
/// CLIENTS:
///     none

#include <cmath>
#include <vector>
#include <tuple>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nuslam/msg/landmark.hpp"
#include "nuslam/msg/landmarks.hpp"
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/circle_detection.hpp"

using turtlelib::Vector2D;
using turtlelib::Circle2D;
using turtlelib::almost_equal;
using turtlelib::deg2rad;
using turtlelib::rad2deg;
using turtlelib::get_mean_and_std_dev;
using turtlelib::angle_between;
using turtlelib::fit_circle;

constexpr double LANDMARK_HEIGHT = 0.125;
constexpr size_t CLUSTER_NUMBER_THRESHOLD = 4;

/// \brief Performs landmark detections from input lidar data
class Landmarks : public rclcpp::Node
{
public:
  /// \brief initialize the node
  Landmarks()
  : Node("landmarks")
  {
    //Parameters
    auto param = rcl_interfaces::msg::ParameterDescriptor{};

    param.description = "Controls whether clustered points are published as markers.";
    declare_parameter("clusters.visualize", false, param);
    clusters_visualize_ = get_parameter("clusters.visualize").get_parameter_value().get<bool>();

    param.description =
      "Euclidean distance between points to be considered part of the same cluster.";
    declare_parameter("clusters.threshold", 0.1, param);
    clusters_threshold_ = get_parameter("clusters.threshold").get_parameter_value().get<double>();

    param.description = "Controls whether fit circles are published as markers.";
    declare_parameter("circles.visualize", false, param);
    circles_visualize_ = get_parameter("circles.visualize").get_parameter_value().get<bool>();

    param.description = "Minimum mean angle for considering a cluster a circle (deg).";
    declare_parameter("circles.classification.mean_min", 90.0, param);
    circles_classification_mean_min_ = deg2rad(
      get_parameter(
        "circles.classification.mean_min").get_parameter_value().get<double>());

    param.description = "Maximum mean angle for considering a cluster a circle (deg).";
    declare_parameter("circles.classification.mean_max", 135.0, param);
    circles_classification_mean_max_ = deg2rad(
      get_parameter(
        "circles.classification.mean_max").get_parameter_value().get<double>());

    param.description = "Maximum standard deviation for considering a cluster a circle (deg).";
    declare_parameter("circles.classification.std_dev_max", 8.6, param);
    circles_classification_std_dev_max_ = deg2rad(
      get_parameter(
        "circles.classification.std_dev_max").get_parameter_value().get<double>());

    param.description = "Radius minimum for considering a circle fit as a legitimate circle.";
    declare_parameter("circles.radius_min", 0.01, param);
    circles_radius_min_ = get_parameter("circles.radius_min").get_parameter_value().get<double>();

    param.description = "Radius maximum for considering a circle fit as a legitimate circle.";
    declare_parameter("circles.radius_max", 0.1, param);
    circles_radius_max_ = get_parameter("circles.radius_max").get_parameter_value().get<double>();


    //Publishers
    if (clusters_visualize_) {
      pub_clusters_ = create_publisher<visualization_msgs::msg::MarkerArray>("clusters", 10);
    }
    if (circles_visualize_) {
      pub_circles_ = create_publisher<visualization_msgs::msg::MarkerArray>("circles", 10);
    }

    pub_landmarks_ = create_publisher<nuslam::msg::Landmarks>("landmarks", 10);

    //Subscribers
    sub_lidar_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan",
      rclcpp::SensorDataQoS(),
      std::bind(&Landmarks::lidar_scan_callback, this, std::placeholders::_1)
    );

    //Init markers
    if (clusters_visualize_) {
      cluster_default_marker_.type = visualization_msgs::msg::Marker::CYLINDER;
      cluster_default_marker_.action = visualization_msgs::msg::Marker::ADD;
      cluster_default_marker_.pose.position.z = LANDMARK_HEIGHT * 2.0;
      cluster_default_marker_.scale.x = clusters_threshold_;
      cluster_default_marker_.scale.y = clusters_threshold_;
      cluster_default_marker_.scale.z = LANDMARK_HEIGHT;
      cluster_default_marker_.color.r = 1.0;
      cluster_default_marker_.color.g = 0.647;
      cluster_default_marker_.color.b = 0.0;
      cluster_default_marker_.color.a = 1.0;
    }

    if (circles_visualize_) {
      circle_default_marker_.type = visualization_msgs::msg::Marker::CYLINDER;
      circle_default_marker_.action = visualization_msgs::msg::Marker::ADD;
      circle_default_marker_.pose.position.z = LANDMARK_HEIGHT;
      circle_default_marker_.scale.z = LANDMARK_HEIGHT;
      circle_default_marker_.color.r = 1.0;
      circle_default_marker_.color.g = 1.0;
      circle_default_marker_.color.b = 0.0;
      circle_default_marker_.color.a = 1.0;
    }

    RCLCPP_INFO_STREAM(get_logger(), "landmarks node started");
  }

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_clusters_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_circles_;
  rclcpp::Publisher<nuslam::msg::Landmarks>::SharedPtr pub_landmarks_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar_scan_;


  double clusters_threshold_;
  bool clusters_visualize_;
  visualization_msgs::msg::MarkerArray cluster_markers_;
  visualization_msgs::msg::Marker cluster_default_marker_;
  size_t max_cluster_markers_ = 0;

  double circles_classification_mean_min_, circles_classification_mean_max_,
    circles_classification_std_dev_max_;
  double circles_radius_min_, circles_radius_max_;
  bool circles_visualize_;
  visualization_msgs::msg::MarkerArray circle_markers_;
  visualization_msgs::msg::Marker circle_default_marker_;
  size_t max_circle_markers_ = 0;

  /// \brief detect landmarks from lidar data
  /// \param msg - lidar data
  void lidar_scan_callback(const sensor_msgs::msg::LaserScan & msg)
  {
    std::vector<Vector2D> measurements {};
    measurements.reserve(msg.ranges.size()); //max number of measurements equals the number of ranges

    std::vector<std::vector<Vector2D>> clusters {};
    clusters.reserve(msg.ranges.size()); //max number of clusters equals the number of ranges

    std::vector<Vector2D> current_cluster;

    Vector2D last_measurement;

    //Cluster points
    for (size_t i = 0; i < msg.ranges.size(); i++) {

      //Get range of point from message
      const auto & range = msg.ranges.at(i);

      //Do not consider out of range measurements in clustering
      if (range < msg.range_min) {continue;}

      //Calculate bearing from message
      const auto bearing = msg.angle_min + i * msg.angle_increment;

      //Convert into point as Vector2D
      const Vector2D measurement = {
        range * std::cos(bearing),
        range * std::sin(bearing)
      };

      //Store measurement
      measurements.push_back(measurement);

      //Do not perform distance comparison for first point
      if (measurements.size() > 1) {
        //calculate distance between this and last point
        const auto distance = (measurement - last_measurement).magnitude();

        //if measurement is above current threshold, store current cluster and start a new one
        if (distance > clusters_threshold_) {
          clusters.push_back(current_cluster);
          current_cluster.clear();
        }
      }

      //Add measurement to current cluster
      current_cluster.push_back(measurement);

      //Store last measurement for next iteration
      last_measurement = measurement;
    }

    //Add last cluster
    clusters.push_back(current_cluster);

    // Wrap clusters
    if (clusters.size() > 1) {
      //if first and last measurements are within the threshold of each other,
      //add the last cluster to the first cluster
      const auto distance = (clusters.front().front() - clusters.back().back()).magnitude();
      if (distance <= clusters_threshold_) {

        //Add last cluster to first cluster
        //https://stackoverflow.com/questions/3177241/what-is-the-best-way-to-concatenate-two-vectors
        clusters.front().reserve(clusters.front().size() + clusters.back().size());
        clusters.front().insert(
          clusters.front().end(), clusters.back().begin(),
          clusters.back().end());

        //Remove last cluster
        clusters.pop_back();
      }
    }

    //filter clusters
    std::vector<std::vector<Vector2D>> filtered_clusters {};
    filtered_clusters.reserve(clusters.size()); //max size is all clusters

    //throw out clusters with less than a certain threshold number of points
    for (const auto & cluster : clusters) {
      if (cluster.size() >= CLUSTER_NUMBER_THRESHOLD) {

        //classify as circle or not circl

        //Make a vector of angles between points in cluster and endpoints
        std::vector<double> angles;
        angles.reserve(cluster.size() - 2);
        for (auto itr = (cluster.cbegin() + 1); itr != (cluster.cend() - 1); ++itr) {
          const auto v_start = cluster.front() - *itr;
          const auto v_end = cluster.back() - *itr;

          angles.push_back(std::abs(angle_between(v_start, v_end)));
        }

        //Calculate the mean and standard deviation of the calculated angles
        const auto [mean, std_dev] = get_mean_and_std_dev(angles);

        //Filter out clusters based on thresholds for mean and standard deviation
        if ((mean > circles_classification_mean_min_) &&
          (mean < circles_classification_mean_max_) &&
          (std_dev < circles_classification_std_dev_max_))
        {
          filtered_clusters.push_back(cluster);
        }
      }
    }

    //fit circles to filtered clusters
    std::vector<std::tuple<Circle2D, double>> circles {};
    circles.reserve(filtered_clusters.size());

    for (const auto & cluster : filtered_clusters) {
      circles.push_back(fit_circle(cluster));
    }

    //Filter circles
    std::vector<std::tuple<Circle2D, double>> filtered_circles {};
    filtered_circles.reserve(circles.size());

    //Filter circles based on maximum and minimum radius thresholds
    for (const auto & circle_data : circles) {
      const auto & circle = std::get<0>(circle_data);
      if ((circle.radius > circles_radius_min_) && (circle.radius < circles_radius_max_)) {
        filtered_circles.push_back(circle_data);
      }
    }

    //Publish cluster markers
    if (clusters_visualize_) {
      const auto markers_to_publish = filtered_clusters.size();

      if (max_cluster_markers_ < markers_to_publish) {
        max_cluster_markers_ = markers_to_publish;
      }

      cluster_markers_.markers.clear();

      cluster_default_marker_.header = msg.header;

      for (size_t i = 0; i < max_cluster_markers_; i++) {
        auto marker = cluster_default_marker_;

        marker.id = i;

        if (i < markers_to_publish) {
          const auto & cluster = filtered_clusters.at(i);

          double x_mean = 0.0;
          double y_mean = 0.0;
          for (auto & point : cluster) {
            x_mean += point.x;
            y_mean += point.y;
          }

          x_mean /= cluster.size();
          y_mean /= cluster.size();

          marker.pose.position.x = x_mean;
          marker.pose.position.y = y_mean;
        } else { //delete markers beyond what we see
          marker.action = visualization_msgs::msg::Marker::DELETE;
        }

        cluster_markers_.markers.push_back(marker);
      }

      //Publish markers
      pub_clusters_->publish(cluster_markers_);
    }

    //Publish circle markers
    if (circles_visualize_) {
      const auto markers_to_publish = filtered_circles.size();

      if (max_circle_markers_ < markers_to_publish) {
        max_circle_markers_ = markers_to_publish;
      }

      circle_markers_.markers.clear();

      circle_default_marker_.header = msg.header;

      for (size_t i = 0; i < max_circle_markers_; i++) {
        auto marker = circle_default_marker_;

        marker.id = i;

        if (i < markers_to_publish) {
          const auto & circle = std::get<0>(filtered_circles.at(i));

          marker.pose.position.x = circle.center.x;
          marker.pose.position.y = circle.center.y;
          marker.scale.x = circle.radius * 2;
          marker.scale.y = circle.radius * 2;
        } else { //delete markers beyond what we see
          marker.action = visualization_msgs::msg::Marker::DELETE;
        }

        circle_markers_.markers.push_back(marker);
      }

      //Publish markers
      pub_circles_->publish(circle_markers_);
    }

    //Publish landmarks
    nuslam::msg::Landmarks landmarks;

    for (const auto & circle_data : filtered_circles) {
      nuslam::msg::Landmark landmark;
      landmark.center.x = std::get<0>(circle_data).center.x;
      landmark.center.y = std::get<0>(circle_data).center.y;
      landmarks.landmarks.push_back(landmark);
    }

    pub_landmarks_->publish(landmarks);
  }
};

/// \brief Run the node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Landmarks>());
  rclcpp::shutdown();
  return 0;
}
