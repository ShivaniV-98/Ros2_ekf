#ifndef TURTLELIB_ROS_CONVERT_INCLUDE_GUARD_HPP
#define TURTLELIB_ROS_CONVERT_INCLUDE_GUARD_HPP
/// \file
/// \brief Conversions between the turtlelib library and ROS messages

#include "turtlelib/rigid2d.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform.hpp"

namespace turtlelib_ros
{
    /// \brief convert a Transform2D into a ROS geometry_msgs::msg::Transform
    /// \param tf - the Transform2D to convert
    /// \return the resulting geometry_msgs::msg::Transform
    geometry_msgs::msg::Transform tf_to_tf_msg(const turtlelib::Transform2D & tf);

    /// \brief convert a Transform2D into a ROS geometry_msgs::msg::Pose
    /// \param tf - the Transform2D to convert
    /// \return the resulting geometry_msgs::msg::Pose
    geometry_msgs::msg::Pose tf_to_pose_msg(const turtlelib::Transform2D & tf);

    /// \brief convert a geometry_msgs::msg::Pose to a geometry_msgs::msg::Transform
    /// \param pose_msg - the geometry_msgs::msg::Pose to convert
    /// \return the resulting geometry_msgs::msg::Transform
    geometry_msgs::msg::Transform pose_msg_to_tf_msg(const geometry_msgs::msg::Pose & pose_msg);

    /// \brief convert a geometry_msgs::msg::Transform to a geometry_msgs::msg::Pose
    /// \param tf_msg - the geometry_msgs::msg::Transform to convert
    /// \return the resulting geometry_msgs::msg::Pose
    geometry_msgs::msg::Pose tf_msg_to_pose_msg(const geometry_msgs::msg::Transform & tf_msg);
}


#endif