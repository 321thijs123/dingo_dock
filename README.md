# Nodes
## cloudfilter
### Subscribed Topics
velodyne_points (sensor_msgs/PointCloud2)

### Published Topics
layer_cloud (sensor_msgs/PointCloud)\
direction_cloud (sensor_msgs/PointCloud)\
object_cloud (sensor_msgs/PointCloud)\
group_cloud (sensor_msgs/PointCloud)

### Published transforms
velodyne -> leg_pair_x\
velodyne -> platform_x

## navGoalSender
#### Transforms Listeners
world -> platform_x

### MoveBase
This node uses a movebase action client for the action named "move_base"

# Launch Files
## rviz.launch
This launch file launches RVIZ with a configuration for visualizing the platform recognition.
## rviz_nav.launch
This launch file launches RVIZ with a configuration for visualizing the platform recognition and sending navgoals.
