/// \file
/// \brief A passthrough node for lidar data that removes the problematic
/// time_increment field so it can be displayed in RVIZ.
/// PARAMETERS:
///     none
/// PUBLISHERS:
///     real_lidar_visualize (sensor_msgs/msg/LaserScan): lidar data with the time_increment field removed
/// SUBSCRIBERS:
///     scan (sensor_msgs/msg/LaserScan): lidar data to process
/// SERVERS:
///     none
/// CLIENTS:
///     none

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

/// \brief Performs landmark detections from input lidar data
class RealLidarVisualize : public rclcpp::Node
{
public:
  /// \brief initialize the node
  RealLidarVisualize()
  : Node("real_lidar_visualize")
  {

    //Publishers
    pub_real_lidar_visualize_ = create_publisher<sensor_msgs::msg::LaserScan>(
      "real_lidar_visualize", 10);

    //Subscribers
    sub_lidar_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan",
      rclcpp::SensorDataQoS(),
      std::bind(&RealLidarVisualize::lidar_scan_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO_STREAM(get_logger(), "real_lidar_visualize node started");
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_real_lidar_visualize_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar_scan_;

  /// \brief remove the time_increment field from lidar data and republish
  /// \param msg lidar data
  void lidar_scan_callback(const sensor_msgs::msg::LaserScan & msg)
  {
    auto edited_msg = msg;

    edited_msg.time_increment = 0.0;

    pub_real_lidar_visualize_->publish(edited_msg);
  }
};

/// \brief Run the node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealLidarVisualize>());
  rclcpp::shutdown();
  return 0;
}
