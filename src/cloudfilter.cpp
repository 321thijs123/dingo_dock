#include "ros/ros.h"
#include <vector>
#include "sensor_msgs/point_cloud_conversion.h"

class CloudFilter
{

public:
	CloudFilter()
	{
		cloud_pub = n.advertise<sensor_msgs::PointCloud>("filtered_cloud", 100);

		sub = n.subscribe("velodyne_points", 100, &CloudFilter::cloudCallback, this);
	}

	// Only keep points between a height of -5 and 5 centimeters
	std::vector<geometry_msgs::Point32> filterLayers(sensor_msgs::PointCloud cloud)
	{
		std::vector<geometry_msgs::Point32> layer_points = {};

		for (int i = 0; i < cloud.points.size(); i++)
		{
			if (cloud.points[i].z < max_height && cloud.points[i].z > min_height)
			{
				layer_points.push_back(cloud.points[i]);
			}
		}

		return layer_points;
	}

	// Only keep objects smaller than 15cm with 30cm of clearance from its center
	std::vector<geometry_msgs::Point32> filterObjects(std::vector<geometry_msgs::Point32> layer_points)
	{
		std::vector<geometry_msgs::Point32> object_points = {};

		for (int i = 0; i < layer_points.size(); i++)
		{
			bool keep = true;

			for (int j = 0; j < layer_points.size(); j++)
			{
				float distance = (layer_points[i].x - layer_points[j].x) *
									 (layer_points[i].x - layer_points[j].x) +
								 (layer_points[i].y - layer_points[j].y) *
									 (layer_points[i].y - layer_points[j].y);

				if (distance > 0.15 * 0.15 && distance < 0.30 * 0.30)
				{
					keep = false;
					break;
				}
			}

			if (keep)
				object_points.push_back(layer_points[i]);
		}

		return object_points;
	}

	// Remove groups of less than 5 points
	std::vector<geometry_msgs::Point32> removeGroups(std::vector<geometry_msgs::Point32> object_points)
	{
		std::vector<geometry_msgs::Point32> size_object_points = {};

		for (int i = 0; i < object_points.size(); i++)
		{
			int count = 0;

			for (int j = 0; j < object_points.size(); j++)
			{
				float distance = (object_points[i].x - object_points[j].x) *
									 (object_points[i].x - object_points[j].x) +
								 (object_points[i].y - object_points[j].y) *
									 (object_points[i].y - object_points[j].y);

				if (distance < 0.15 * 0.15)
				{
					count++;
				}
			}

			if (count > 5)
				size_object_points.push_back(object_points[i]);
		}

		return size_object_points;
	}
	
	void cloudCallback(const sensor_msgs::PointCloud2 &msg)
	{
		double startTime = ros::Time::now().toSec();
		sensor_msgs::PointCloud cloud;

		convertPointCloud2ToPointCloud(msg, cloud);

		std::vector<geometry_msgs::Point32> layer_points = filterLayers(cloud);

		std::vector<geometry_msgs::Point32> object_points = filterObjects(layer_points);

		std::vector<geometry_msgs::Point32> size_object_points = removeGroups(object_points);

		filtered_cloud.header = cloud.header;
		filtered_cloud.points = size_object_points;

		cloud_pub.publish(filtered_cloud);

		ROS_INFO("%lu points in %.2f seconds", size_object_points.size(), ros::Time::now().toSec() - startTime);
	}

private:
	ros::NodeHandle n;
	ros::Publisher cloud_pub;
	ros::Subscriber sub;

	sensor_msgs::PointCloud filtered_cloud;

	const float max_height = 0.05;
	const float min_height = -0.05;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cloud_filter");

	CloudFilter filterObject;

	ros::spin();

	return 0;
}
