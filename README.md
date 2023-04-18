#FRUC ROS Utils

## Transform PCL 

### Description
Transform PCL subscribes to the original point cloud with its source frame and republishes it into the target frame instead.

Tested on Ubuntu 18.04 and ROS Melodic

### Installation

1. Add all ROS dependencies using the following command:
```
cd your_work_space
rosdep install --from-paths src --ignore-src -y -r
```

### Compiling

```
cd your_work_space
catkin_make 
```

### Example Usage

#### Transform PCL

**Parameters**

`cloud_topic` (`string`, `default: /fused_point_cloud`)

Topic to subscribe. Defaults at fused_point_cloud.

`newFrame_id` (`string`, `default: front_lslidar`)

Target frame to transform the pointcloud. Defaults at front_lslidar

**Topics**

`transformed_pcl` (`sensor_msgs/PointCloud2`)

Publishes a PointCloud2.

**Node**

```
rosrun ros_bags_utils transform_pcl.py _cloud_topic:=value _newFrame_id:=value
```
*Note*: Only needs to pass cloud_topic and newFrame_id arguments if you want values **different** than the default
