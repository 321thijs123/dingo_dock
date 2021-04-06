# Usage
## Cloudfilter
The cloudfilter node is used to filter out all points except the tube.
```
rosrun dingo_dock cloudfilter
```

## rviz.launch
This launch file runs rviz with the correct configuration for visualizing the cloudfilter node.
```
roslaunch dingo_dock rviz.launch
```

## track_test.launch
This launch file runs a rosbag, rviz and the cloudfilter node. The rosbag needs to be placed in the package at `rosbag/tube.bag`.
```
roslaunch dingo_dock track_test.launch
```
