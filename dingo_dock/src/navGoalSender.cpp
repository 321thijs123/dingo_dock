#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_listener");

  ros::NodeHandle node;

  tf::TransformListener listener;

  bool found = false;

  tf::Vector3 v;

  ros::Rate rate(1.0);
  while (!found)
  {
    tf::StampedTransform transform;
    try
    {

      listener.lookupTransform("odom", "platform_0", ros::Time(0), transform);

      v = transform.getOrigin();
      ROS_INFO("Translation platform: %.3f, %.3f, %.3f", v.getX(), v.getY(), v.getZ());

      listener.lookupTransform("odom", "velodyne", ros::Time(0), transform);

      found = true;
    }

    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    rate.sleep();
  }

  ROS_INFO("COMMAND SENDED!!!");
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

  goal.target_pose.pose.position.x = v.getX();
  goal.target_pose.pose.position.y = v.getY();
  goal.target_pose.pose.position.z = v.getZ();
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the robot moved to the goal");
  else
    ROS_INFO("The base failed to move to the goal for some reason");

  return 0;
}