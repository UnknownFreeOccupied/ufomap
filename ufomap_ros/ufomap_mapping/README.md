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

### Required TF Transforms
