#include "turtlelib_ros/convert.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace turtlelib_ros
{

    geometry_msgs::msg::Transform tf_to_tf_msg(const turtlelib::Transform2D & tf)
    {
        geometry_msgs::msg::Transform msg;

        msg.translation.x = tf.translation().x;
        msg.translation.y = tf.translation().y;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, tf.rotation());
        msg.rotation = tf2::toMsg(q);

        return msg;
    }

    geometry_msgs::msg::Pose tf_to_pose_msg(const turtlelib::Transform2D & tf)
    {
        auto tf_msg = tf_to_tf_msg(tf);

        return tf_msg_to_pose_msg(tf_msg);
    }

    geometry_msgs::msg::Transform pose_msg_to_tf_msg(const geometry_msgs::msg::Pose & pose_msg)
    {
        geometry_msgs::msg::Transform tf_msg;

        tf_msg.translation.x = pose_msg.position.x;
        tf_msg.translation.y = pose_msg.position.y;
        tf_msg.translation.z = pose_msg.position.z;
        tf_msg.rotation = pose_msg.orientation;

        return tf_msg;
    }

    geometry_msgs::msg::Pose tf_msg_to_pose_msg(const geometry_msgs::msg::Transform & tf_msg)
    {
        geometry_msgs::msg::Pose pose_msg;

        pose_msg.position.x = tf_msg.translation.x;
        pose_msg.position.y = tf_msg.translation.y;
        pose_msg.position.z = tf_msg.translation.z;
        pose_msg.orientation = tf_msg.rotation;

        return pose_msg;
    }
}