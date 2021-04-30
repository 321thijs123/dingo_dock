#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

#define M_PI 3.14159265358979323846

double determinePlatformDirectionToRobot(tf::StampedTransform tf_leg_pair, tf::StampedTransform tf_robot)
{
	// to do: niet allemaal tf's sturen naar de functies.

	tf::Quaternion q_leg_pair = tf_leg_pair.getRotation();

	double roll_platform, pitch_platform, yaw_platform;

	tf::Matrix3x3(q_leg_pair).getRPY(roll_platform, pitch_platform, yaw_platform);
	
	tf::Vector3 v_leg_pair = tf_leg_pair.getOrigin();
	tf::Vector3 v_robot = tf_robot.getOrigin();

	double deltaX = v_leg_pair.getX() - v_robot.getX();
	double deltaY = v_leg_pair.getY() - v_robot.getY();

	ROS_INFO("%f",atan2(deltaY, deltaX)-yaw_platform);
	return atan2(deltaY, deltaX)-yaw_platform;
}

double determineYawDifference(tf::StampedTransform tf_leg_pair, tf::StampedTransform tf_robot)
{
	tf::Quaternion q_leg_pair = tf_leg_pair.getRotation();
	tf::Quaternion q_robot = tf_robot.getRotation();

	double roll_platform, pitch_platform, yaw_platform;
	double roll_robot, pitch_robot, yaw_robot;

	tf::Matrix3x3(q_leg_pair).getRPY(roll_platform, pitch_platform, yaw_platform);
	tf::Matrix3x3(q_robot).getRPY(roll_robot, pitch_robot, yaw_robot);

	ROS_INFO("%f",(yaw_platform - yaw_robot));
	return (yaw_platform - yaw_robot);
}

double determinePlatformDistance( tf::StampedTransform tf_leg_pair, tf::StampedTransform tf_robot)
{
	tf::Vector3 v_leg_pair = tf_leg_pair.getOrigin();
	tf::Vector3 v_robot = tf_robot.getOrigin();

	double deltaX = v_leg_pair.getX() - v_robot.getX();
	double deltaY = v_leg_pair.getY() - v_robot.getY();

	return sqrt((deltaY * deltaY) + (deltaX * deltaX));
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "drivetest");

	ros::NodeHandle n;

	ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 20);

	tf::TransformListener listener;

	
	double yaw_difference;

	bool found = false;

	tf::Vector3 v;
	tf::Quaternion q;

	tf::StampedTransform tf_leg_pair;

	ros::Rate rate(20.0);
	while (!found)
	{
		try
		{

			listener.lookupTransform("odom", "leg_pair_0", ros::Time(0), tf_leg_pair);

			tf::Vector3 v_leg_pair = tf_leg_pair.getOrigin();

			ROS_INFO("Translation legpair: %.3f, %.3f, %.3f", v_leg_pair.getX(), v_leg_pair.getY(), v_leg_pair.getZ());

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

	while (ros::ok())
	{

		tf::StampedTransform tf_robot;
		geometry_msgs::Twist msg;

		try
		{

			listener.lookupTransform("odom", "base_link", ros::Time(0), tf_robot);

			tf::Vector3 v_robot = tf_robot.getOrigin();
			
			ROS_INFO("Translation robot: %.3f, %.3f, %.3f", v_robot.getX(), v_robot.getY(), v_robot.getZ());
		}

		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			continue;
		}

		double direction_platform = determinePlatformDirectionToRobot(tf_leg_pair, tf_robot);

		tf::Quaternion q_leg_pair = tf_leg_pair.getRotation();
		tf::Quaternion q_robot = tf_robot.getRotation();

		double roll_platform, pitch_platform, yaw_platform;
		double roll_robot, pitch_robot, yaw_robot;

		tf::Matrix3x3(q_leg_pair).getRPY(roll_platform, pitch_platform, yaw_platform);
		tf::Matrix3x3(q_robot).getRPY(roll_robot, pitch_robot, yaw_robot);

		double targetDirection = yaw_platform - direction_platform * 1.0;


		msg.angular.z = (yaw_robot - targetDirection) * 1.0;
		msg.linear.x = determinePlatformDistance( tf_robot, tf_leg_pair) / 1.0;
		
		cmd_vel_pub.publish(msg);

		ros::spinOnce();

		rate.sleep();
	}
}
