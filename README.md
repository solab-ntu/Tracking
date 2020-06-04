# Model-free detection and tracking of dynamic objects with 2D lidar

## Install
Requirements:
```
sudo apt-get install ros-kinetic-navigation
sudo apt-get install ros-kinetic-tf2-sensor-msgs
sudo apt-get install ros-kinetic-teb-local-planner
pip install -U scikit-learn  # may need to find a version to support python 2.7
```

## Published Topics
### /tracks (tracking_2d_lidar/TracksArrayMsg)
Array of tracking_2d_lidar/TracksMsg. Where tracking_2d_lidar/TracksMsg contains:
```
std_msgs/Float32 x
std_msgs/Float32 y
std_msgs/Float32 vx
std_msgs/Float32 vy
```
of a tracked dynamic obstacle.

## new_layers
### smartobstacle_layer
A duplicate of obstacle_layer with minor modification.
Will clear every cost when new sensor readings arrived. (Whereas obstacle_layer will do ray tracing before clearing)

### predict_layer
Get topic `/tracks` (tracking_2d_lidar/TracksArrayMsg) and put on costmap. 