#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <vector>

class CloudFilter
{

public:
	//Constructor (Called when the CloudFilter object is created)
	CloudFilter()
	{
		layer_pub = n.advertise<sensor_msgs::PointCloud>("cloudfilter/layer", 100);		//Setup publishers for output point clouds
		direction_pub = n.advertise<sensor_msgs::PointCloud>("cloudfilter/direction", 100);
		object_pub = n.advertise<sensor_msgs::PointCloud>("cloudfilter/object", 100);
		group_pub = n.advertise<sensor_msgs::PointCloud>("cloudfilter/group", 100);

		sub = n.subscribe("velodyne_points", 100, &CloudFilter::cloudCallback, this);	//Subscribe to velodyne pointcloud, call CloudFilter::cloudCallback if a new pointcloud is published
	}

	// Only keep points between min_height and max_height
	std::vector<geometry_msgs::Point32> filterLayers(std::vector<geometry_msgs::Point32> points)
	{
		std::vector<geometry_msgs::Point32> layer_points = {};

		for (int i = 0; i < points.size(); i++)
		{
			if (points[i].z < max_height && points[i].z > min_height)
			{
				layer_points.push_back(points[i]);
			}
		}

		return layer_points;
	}

	// Only keep points in front of the robot
	std::vector<geometry_msgs::Point32> filterDirection(std::vector<geometry_msgs::Point32> points)
	{
		std::vector<geometry_msgs::Point32> direction_points = {};

		for (int i = 0; i < points.size(); i++)
		{
		
			if (points[i].x > 0 )
			{
				direction_points.push_back(points[i]);
			}
		}

		return direction_points;
	}
	
	// Only keep objects smaller than [max_size] with [clearance] of clearance from its center
	std::vector<geometry_msgs::Point32> filterObjects(std::vector<geometry_msgs::Point32> points)
	{
		std::vector<geometry_msgs::Point32> object_points = {};

		for (int i = 0; i < points.size(); i++)
		{
			bool keep = true;

			for (int j = 0; j < points.size(); j++)
			{
				float distance =(points[i].x - points[j].x) *
								(points[i].x - points[j].x) +
								(points[i].y - points[j].y) *
								(points[i].y - points[j].y);

				if (distance > max_size && distance < clearance)
				{
					keep = false;
					break;
				}
			}

			if (keep)
				object_points.push_back(points[i]);
		}

		return object_points;
	}

	// Remove groups of less than [min_points] points then calculate group centers
	std::vector<geometry_msgs::Point32> getGroups(std::vector<geometry_msgs::Point32> points)
	{
		std::vector<geometry_msgs::Point32> group_points = {};
		
		for (int i = 0; i < points.size(); i++)
		{
			int count = 0;
			geometry_msgs::Point32 current_group; 
			
			for (int j = 0; j < points.size(); j++)
			{
				float distance =(points[i].x - points[j].x) *
								(points[i].x - points[j].x) +
								(points[i].y - points[j].y) *
								(points[i].y - points[j].y);

				if (distance < clearance)
				{
					current_group.x += points[j].x;
					current_group.y += points[j].y;
				
					count++;
				}
			}

			if (count > min_points)
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
	int getPlatforms(std::vector<geometry_msgs::Point32> points)
	{
		static tf::TransformBroadcaster br;
		std::vector<geometry_msgs::Point32> platforms = {};

		int n = 0;
		
		for (int i = 0; i < points.size(); i++)
		{
			for (int j = 0; j < points.size(); j++)
			{
				float distance =(points[i].x - points[j].x) *
								(points[i].x - points[j].x) +
								(points[i].y - points[j].y) *
								(points[i].y - points[j].y);
				
				
				if (distance > min_leg_distance && distance < max_leg_distance)
				{
					tf::Transform transform;
					
					float platform_x = (points[i].x + points[j].x)/2;
					float platform_y = (points[i].y + points[j].y)/2;
					float platform_z = (points[i].z + points[j].z)/2;
					transform.setOrigin(tf::Vector3(platform_x, platform_y, platform_z));
					
					tf::Quaternion q;
					float yaw = atan2(points[i].x-points[j].x,-points[i].y+points[j].y);
					q.setRPY(0, 0 , yaw);
					transform.setRotation(q);

					br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "velodyne", "platform_" + std::to_string(n)));

					n++;

					points.erase(points.begin()+i);
					points.erase(points.begin()+j-1);

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

		sensor_msgs::PointCloud layer_cloud;
		sensor_msgs::PointCloud direction_cloud;
		sensor_msgs::PointCloud object_cloud;
		sensor_msgs::PointCloud group_cloud;

		layer_cloud.header = direction_cloud.header = object_cloud.header = group_cloud.header = cloud.header;

		layer_cloud.points = filterLayers(cloud.points);
		direction_cloud.points = filterDirection(layer_cloud.points);
		object_cloud.points = filterObjects(direction_cloud.points);
		group_cloud.points = getGroups(object_cloud.points);

		int n = getPlatforms(group_cloud.points);

		layer_pub.publish(layer_cloud);
		direction_pub.publish(direction_cloud);
		object_pub.publish(object_cloud);
		group_pub.publish(group_cloud);

		ROS_INFO("Found %i platform(s) in %.3f seconds", n, ros::Time::now().toSec() - startTime);
	}

private:
	ros::NodeHandle n;
	ros::Publisher layer_pub;
	ros::Publisher direction_pub;
	ros::Publisher object_pub;
	ros::Publisher group_pub;
	ros::Subscriber sub;

	const float max_height = 0.025;			//Height of platform legs relative to lidar
	const float min_height = -0.025;

	const float max_size = 0.06 * 0.06;		//Maximum size of legs, larger objects will be disgarded (Squared to avoid square root for pythagoras)
	const float clearance = 0.1 * 0.1;		//Minimum distance to other objects to be regarded a seperate object (Squared to avoid square root for pythagoras) 

	const int min_points = 5;				//Minimum poins per object

	const float min_leg_distance = 0.600*0.600;		//Minimum distance between two possible legs to be considered the same platform (Squared to avoid square root for pythagoras)
	const float max_leg_distance = 0.620*0.620;		//Maximum distance between two possible legs to be considered the same platform (Squared to avoid square root for pythagoras)
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cloud_filter");

	CloudFilter filterObject;	//Create cloud filter object

	ros::spin();

	return 0;
}
