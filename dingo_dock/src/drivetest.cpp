#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include "helpers/dingo_exception.cpp"

#define M_PI 3.14159265358979323846

double determineDirection(tf::StampedTransform tf_platform, tf::StampedTransform tf_robot)
{
	// to do: niet allemaal tf's sturen naar de functies.

	tf::Quaternion q_leg_pair = tf_platform.getRotation();

	double roll_platform, pitch_platform, yaw_platform;

	tf::Matrix3x3(q_leg_pair).getRPY(roll_platform, pitch_platform, yaw_platform);
	
	tf::Vector3 v_leg_pair = tf_platform.getOrigin();
	tf::Vector3 v_robot = tf_robot.getOrigin();

	double deltaX = v_leg_pair.getX() - v_robot.getX();
	double deltaY = v_leg_pair.getY() - v_robot.getY();

	//ROS_INFO("%f",atan2(deltaY, deltaX)-yaw_platform);
	return atan2(deltaY, deltaX)-yaw_platform;
}

double determineYawDifference(tf::StampedTransform tf_platform, tf::StampedTransform tf_robot)
{
	tf::Quaternion q_leg_pair = tf_platform.getRotation();
	tf::Quaternion q_robot = tf_robot.getRotation();

	double roll_platform, pitch_platform, yaw_platform;
	double roll_robot, pitch_robot, yaw_robot;

	tf::Matrix3x3(q_leg_pair).getRPY(roll_platform, pitch_platform, yaw_platform);
	tf::Matrix3x3(q_robot).getRPY(roll_robot, pitch_robot, yaw_robot);

	//ROS_INFO("%f",(yaw_platform - yaw_robot));
	return (yaw_platform - yaw_robot);
}

double determineDistance( tf::StampedTransform tf_platform, tf::StampedTransform tf_robot)
{
	tf::Vector3 v_leg_pair = tf_platform.getOrigin();
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

	tf::StampedTransform tf_platform;
	tf::StampedTransform tf_robot;
	tf::StampedTransform tf_chassis;

	ros::Rate rate(20.0);
	while (!found)
	{
		try
		{

			listener.lookupTransform("odom", "leg_pair_0", ros::Time(0), tf_platform);

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

		geometry_msgs::Twist msg;

		try
		{
			listener.lookupTransform("odom", "chassis_link", ros::Time(0), tf_chassis);
			listener.lookupTransform("odom", "base_link", ros::Time(0), tf_robot);
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			continue;
		}
		try
		{
			listener.lookupTransform("odom", "platform_0", ros::Time(0), tf_platform);
			
		}
		catch (tf::TransformException &ex){}


		tf::Quaternion q_leg_pair = tf_platform.getRotation();
		tf::Quaternion q_robot = tf_robot.getRotation();

		double roll_platform, pitch_platform, yaw_platform;
		double roll_robot, pitch_robot, yaw_robot;

		tf::Matrix3x3(q_leg_pair).getRPY(roll_platform, pitch_platform, yaw_platform);
		tf::Matrix3x3(q_robot).getRPY(roll_robot, pitch_robot, yaw_robot);

		double platformDistance = determineDistance(tf_robot, tf_platform);
		double platform_direction = determineDirection(tf_platform, tf_robot);
		double platformCenterDistance = determineDistance(tf_chassis, tf_platform);
		double targetDirection;

		if (platformDistance > 0.12)
		{
			targetDirection = yaw_platform + platform_direction * 2.0;
		}
		else
		{
			targetDirection = yaw_platform;
		}
		
		double yawDiff = targetDirection - yaw_robot;

		msg.angular.z = (yawDiff) * 10.0;

		if (platformDistance > 0.12)
		{
			if (abs(yawDiff) < 1.0/50.0)
			{
				msg.linear.x =  platformDistance * 1.0 / (1.0-50.0*abs(yawDiff));
			}
			else {
				msg.linear.x = 0.0;
			}
		}
		else
		{
			msg.linear.x = platformCenterDistance * 2.0;
		}


		if (msg.linear.x > 0.5) msg.linear.x = 0.5;

		system("clear");

		std::cout << "\n----------------\n"
				"platform dir:    " << platform_direction << "\n" <<
				"target dir:      " << targetDirection << "\n" <<
				"cur dir:         " << yaw_robot << "\n" <<
				"distance:        " << platformDistance << "\n" <<
				"center distance: " << platformCenterDistance << "\n" <<
				"angular:         " << msg.angular.z << "\n" <<
				"linear:          " << msg.linear.x << "\n" <<
				"----------------\n";

		cmd_vel_pub.publish(msg);

		ros::spinOnce();

		rate.sleep();
	}
}
