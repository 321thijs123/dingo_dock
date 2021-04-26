#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nav_goal_sender");

  ros::NodeHandle node;

  tf::TransformListener listener;

  bool found = false;

  tf::Vector3 v;
  tf::Quaternion q;

  ros::Rate rate(1.0);
  while (ros::ok())
  {
    tf::StampedTransform transform;
    try
    {

      listener.lookupTransform("odom", "leg_pair_0", ros::Time(0), transform);

      v = transform.getOrigin();
      q = transform.getRotation();

      ROS_INFO("Translation platform: %.3f, %.3f, %.3f", v.getX(), v.getY(), v.getZ());

      //tell the action client that we want to spin a thread by default
      MoveBaseClient ac("move_base", true);

      //wait for the action server to come up
      while (!ac.waitForServer(ros::Duration(5.0)))
      {
        ROS_INFO("Waiting for the move_base action server to come up");
      }

      move_base_msgs::MoveBaseGoal goal;

      goal.target_pose.header.frame_id = "odom";
      goal.target_pose.header.stamp = ros::Time::now();

      double roll, pitch, yaw;

      tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

      goal.target_pose.pose.position.x = v.getX() - cos(yaw) * 1.0;
      goal.target_pose.pose.position.y = v.getY() - sin(yaw) * 1.0;
      goal.target_pose.pose.position.z = v.getZ();
      goal.target_pose.pose.orientation.x = q[0];
      goal.target_pose.pose.orientation.y = q[1];
      goal.target_pose.pose.orientation.z = q[2];
      goal.target_pose.pose.orientation.w = q[3];

      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      ac.waitForResult();

      if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the robot moved to the goal");
      else
        ROS_INFO("The base failed to move to the goal for some reason");

      ros::spin();
    }

    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    rate.sleep();
  }

  return 0;
}