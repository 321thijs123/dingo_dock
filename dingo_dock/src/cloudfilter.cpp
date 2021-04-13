#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <vector>

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

	// Orientation filter
	std::vector<geometry_msgs::Point32> filterOrientation(std::vector<geometry_msgs::Point32> layer_points)
	{
		std::vector<geometry_msgs::Point32> orientation_points = {};

		for (int i = 0; i < layer_points.size(); i++)
		{
		
			if (layer_points[i].x > 0 )
			{
				orientation_points.push_back(layer_points[i]);
			}
		}

		return orientation_points;
	}
	
	// Only keep objects smaller than 15cm with 30cm of clearance from its center
	std::vector<geometry_msgs::Point32> filterObjects(std::vector<geometry_msgs::Point32> orientation_points)
	{
		std::vector<geometry_msgs::Point32> object_points = {};

		for (int i = 0; i < orientation_points.size(); i++)
		{
			bool keep = true;

			for (int j = 0; j < orientation_points.size(); j++)
			{
				float distance = (orientation_points[i].x - orientation_points[j].x) *
									 (orientation_points[i].x - orientation_points[j].x) +
								 (orientation_points[i].y - orientation_points[j].y) *
									 (orientation_points[i].y - orientation_points[j].y);

				if (distance > 0.06 * 0.06 && distance < 0.1 * 0.1)
				{
					keep = false;
					break;
				}
			}

			if (keep)
				object_points.push_back(orientation_points[i]);
		}

		return object_points;
	}

	// Remove groups of less than 5 points and locate groups
	std::vector<geometry_msgs::Point32> getGroups(std::vector<geometry_msgs::Point32> object_points)
	{
		std::vector<geometry_msgs::Point32> group_points = {};
		
		for (int i = 0; i < object_points.size(); i++)
		{
			int count = 0;
			geometry_msgs::Point32 current_group; 
			
			for (int j = 0; j < object_points.size(); j++)
			{
				float distance = (object_points[i].x - object_points[j].x) *
									 (object_points[i].x - object_points[j].x) +
								 (object_points[i].y - object_points[j].y) *
									 (object_points[i].y - object_points[j].y);

				if (distance < 0.13 * 0.13)
				{
					
					current_group.x += object_points[j].x;
					current_group.y += object_points[j].y;
				
					count++;
				}
			}

			if (count > 5)
			{
			
				current_group.x = current_group.x / count;
				current_group.y = current_group.y / count;
				
				bool is_new = true;
				
				for(int k = 0; k < group_points.size(); k++)
				{
					if( current_group == group_points[k] )
					{
						is_new = false;
						break;
					}
				}
				if (is_new)
				{
					group_points.push_back(current_group);
				}
			}
		}
		return group_points;
	}

	// Find legs and determine platform location(s)
	int getPlatforms(std::vector<geometry_msgs::Point32> group_points)
	{
		static tf::TransformBroadcaster br;
		std::vector<geometry_msgs::Point32> platforms = {};

		int n = 0;
		
		for (int i = 0; i < group_points.size(); i++)
		{
			for (int j = 0; j < group_points.size(); j++){
				float distance = (group_points[i].x - group_points[j].x) *
									 (group_points[i].x - group_points[j].x) +
								 (group_points[i].y - group_points[j].y) *
									 (group_points[i].y - group_points[j].y);
				
				
				if (distance > min_leg_distance && distance < max_leg_distance){
					tf::Transform transform;
					
					float platform_x = (group_points[i].x + group_points[j].x)/2;
					float platform_y = (group_points[i].y + group_points[j].y)/2;
					float platform_z = (group_points[i].z + group_points[j].z)/2;
					ROS_INFO("Platform at %.2f %.2f %.2f", platform_x, platform_y, platform_z);
					transform.setOrigin(tf::Vector3(platform_x, platform_y, platform_z));
					
					tf::Quaternion q;
					float yaw = atan2(group_points[i].x-group_points[j].x,-group_points[i].y+group_points[j].y);
					q.setRPY(0, 0 , yaw);
					transform.setRotation(q);

					br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "velodyne", "platform_" + std::to_string(n)));

					n++;

					group_points.erase(group_points.begin()+i);
					group_points.erase(group_points.begin()+j-1);

					i--;
					break;
				}
			}
		}
		
		
		return n;
	}
	
	void cloudCallback(const sensor_msgs::PointCloud2 &msg)
	{
		double startTime = ros::Time::now().toSec();
		sensor_msgs::PointCloud cloud;

		convertPointCloud2ToPointCloud(msg, cloud);

		std::vector<geometry_msgs::Point32> layer_points = filterLayers(cloud);

		std::vector<geometry_msgs::Point32> orientation_points = filterOrientation(layer_points);

		std::vector<geometry_msgs::Point32> object_points = filterObjects(orientation_points);

		std::vector<geometry_msgs::Point32> group_points = getGroups(object_points);

		int n = getPlatforms(group_points);

		ROS_INFO("Found %i platforms", n);

		filtered_cloud.header = cloud.header;
		filtered_cloud.points = object_points;

		cloud_pub.publish(filtered_cloud);

		//ROS_INFO("%lu points in %.2f seconds", group_points.size(), ros::Time::now().toSec() - startTime);
	}

private:
	ros::NodeHandle n;
	ros::Publisher cloud_pub;
	ros::Subscriber sub;

	sensor_msgs::PointCloud filtered_cloud;

	const float max_height = 0.025;
	const float min_height = -0.025;

	const float min_leg_distance = 0.600*0.600;
	const float max_leg_distance = 0.620*0.620;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cloud_filter");

	CloudFilter filterObject;

	ros::spin();

	return 0;
}
