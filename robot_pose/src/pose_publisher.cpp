/*
  @file pose_publisher.cpp

  @author Procópio Stein

  @copyright (c) 2018 PAL Robotics SL. All Rights Reserved
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_pose_publisher");
  ros::NodeHandle node;
  ros::Publisher pose_pub =
      node.advertise<geometry_msgs::PoseWithCovarianceStamped>("robot_pose", 1);

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener listener(tf_buffer);

  ros::Rate rate(50);

  geometry_msgs::PoseWithCovarianceStamped robot_pose_msg;

  while (node.ok())
  {
    geometry_msgs::TransformStamped map_to_odom;
    geometry_msgs::TransformStamped odom_to_base;

    try
    {
      // this may not always exist, use the last tf available
      map_to_odom = tf_buffer.lookupTransform("map", "odom",
                                              ros::Time(0),
                                              ros::Duration(0.1));

      // this should always exist, use the tf from now
      odom_to_base = tf_buffer.lookupTransform("odom", "base_footprint",
                                              ros::Time::now(),
                                              ros::Duration(0.1));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(0.1).sleep();
      continue;
    }

    tf2::Transform map_to_odom_tf;
    tf2::Transform odom_to_base_tf;
    tf2::Transform map_to_base_tf;

    tf2::fromMsg(map_to_odom.transform, map_to_odom_tf);
    tf2::fromMsg(odom_to_base.transform, odom_to_base_tf);

    map_to_base_tf = map_to_odom_tf * odom_to_base_tf;

    tf2::toMsg(map_to_base_tf, robot_pose_msg.pose.pose);

    robot_pose_msg.header.frame_id = "/map";
    robot_pose_msg.header.stamp = ros::Time::now();

    pose_pub.publish(robot_pose_msg);

    rate.sleep();
  }
  return EXIT_SUCCESS;
}
