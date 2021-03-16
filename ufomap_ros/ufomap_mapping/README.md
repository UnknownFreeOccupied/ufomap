## Dynamic Reconfigure API
ufomap_server offer a [dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure) inferance to change a number of settings on the fly.

## ufomap_server
ufomap_server builds and distributes volumetric 3D occupancy maps as UFOMap binary stream.

### Subscribed Topics
* cloud_in ([sensor_msgs/PointCloud2](http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html))  
   Incoming point cloud for integration. You need to remap this topic to your sensor data topic and provide a TF transform between the sensor data and the static map frame.

### Published Topics
* map  ([ufomap_msgs/UFOMapStamped](https://github.com/UnknownFreeOccupied/ufomap/blob/master/ufomap_ros/ufomap_msgs/msg/UFOMapStamped.msg))  
   The complete UFOMap as a binary stream, encoding unknown, free, and occupied space, together with [meta data](https://github.com/UnknownFreeOccupied/ufomap/blob/master/ufomap_ros/ufomap_msgs/msg/UFOMapMetaData.msg).
* map_depth_X (where X is [1, 21], depending on the parameter `publish_depth`) [OPTIONAL] ([ufomap_msgs/UFOMapStamped](https://github.com/UnknownFreeOccupied/ufomap/blob/master/ufomap_ros/ufomap_msgs/msg/UFOMapStamped.msg))  
   Same as map, except only nodes down to depth X. This substantially reduces the message size and the time it takes to serialize and deserialize the message. Since many nodes does not require a map at finest resolution this is a good way to achieve additional performance.
   
### Services

### Parameters
* ~frame_id (string, default: map)
* ~resolution (double, default: 0.05)
* ~depth_levels (int, default: 16)
* ~color_map (bool, default: false)
* ~max_range (double, default: -1 (unlimited))
* ~insert_depth (int, default: 0)
* ~simple_ray_casting (bool, default: false)
* ~early_stopping (int, default: 0)
* ~clear_robot (bool, default: false)
* ~robot_height (double, default: )
* ~robot_radius (double, default: )
* ~clearing_depth (int, default: 0)
* ~compress (bool, default: false)
* ~update_part_of_map (bool, default: true)
* ~publish_depth (int, default: 4)
* ~prob_hit (double, default: 0.7)
* ~prob_miss (double, default: 0.4)
* ~clamping_thres_min (double, default: )
* ~clamping_thres_max (double, default: )
* ~transform_timeout (double, default: )
* ~map_latch (bool, default: false)
* ~map_queue_size (int, default: 10)
* ~cloud_in_queue_size (int, default: 10)
* ~pub_rate (double, default: 0)
* ~update_rate (double, default: 0)
* ~verbose (bool, default: false)

### Required TF Transforms
* sensor data frame -> map  
   Required transform of sensor data into the static map frame. It has to be possible to transform the point cloud coming from `cloud_in` to `frame_id`. You need to supply this transform.
