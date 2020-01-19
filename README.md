# Package for MOVO Object Search

### Creating this package
```
catkin_create_pkg movo_object_search ar_track_alvar std_msgs rospy roscpp pcl_ros pcl_msgs
catkin_make -DCATKIN_WHITELIST_PACKAGES="movo_object_search"
```

### Start up movo

```
roslaunch movo_bringup movo_system.launch
```

### Run movo RVIZ

```
roslaunch movo_viz view_robot.launch
```

Note that to visualize the point cloud, go to Sensing-> PointCloud2, and make sure
the Topic is `/movo_camera/point_cloud/points


### Processing point cloud

Converting from scaled values to meters: 

Good answer from [ROS Ask](https://answers.ros.org/question/236223/piontcloud-to-depth-image-codification/):

You should convert your depth image from scaled values to meters (as 32-bit float), as described in REP- 0118, and publish this as a sensor_msgs/Float .
Then run the depth_image_proc/point_cloud_xyz node from depth_image_proc, and remap its image_rect topic to the topic where you are publishing your depth image. This will publish sensor_msgs/PointCloud2 messages on the points topic.
You can then view these messages with the PointCloud2 display type in rviz. (make sure your fixed frame in rviz matches the frame_id)`

USE PCL library to convert it.