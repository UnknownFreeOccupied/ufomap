## Dynamic Reconfigure API
ufomap_server offer a [dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure) inferance to change a number of settings on the fly.

## ufomap_server
ufomap_server builds and distributes volumetric 3D occupancy maps as UFOMap binary stream.

See [launch/server.launch](https://github.com/UnknownFreeOccupied/ufomap/blob/master/ufomap_ros/ufomap_mapping/launch/server.launch) for an example launch file. The rest of the parameters can be changed using dynamic_reconfigure.

### Subscribed Topics
* **cloud_in** ([sensor_msgs/PointCloud2](http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html))  
   Incoming point cloud for integration. You need to remap this topic to your sensor data topic and provide a TF transform between the sensor data and the static map frame.

### Published Topics
* **~map**  ([ufomap_msgs/UFOMapStamped](https://github.com/UnknownFreeOccupied/ufomap/blob/master/ufomap_ros/ufomap_msgs/msg/UFOMapStamped.msg))  
   The complete UFOMap as a binary stream, encoding unknown, free, and occupied space, together with [meta data](https://github.com/UnknownFreeOccupied/ufomap/blob/master/ufomap_ros/ufomap_msgs/msg/UFOMapMetaData.msg).
* **~map_depth_X** (where X is [1, 21], depending on the parameter `publish_depth`) [OPTIONAL] ([ufomap_msgs/UFOMapStamped](https://github.com/UnknownFreeOccupied/ufomap/blob/master/ufomap_ros/ufomap_msgs/msg/UFOMapStamped.msg))  
   Same as map, except only nodes down to depth X. This substantially reduces the message size and the time it takes to serialize and deserialize the message. Since many nodes does not require a map at finest resolution this is a good way to achieve additional performance.
   
### Services
* **~get_map** ([ufomap_srvs/GetMap](https://github.com/UnknownFreeOccupied/ufomap_ros/blob/master/ufomap_srvs/srv/GetMap.srv))  
   The complete UFOMap as a binary stream, encoding unknown, free, and occupied space, together with [meta data](https://github.com/UnknownFreeOccupied/ufomap/blob/master/ufomap_ros/ufomap_msgs/msg/UFOMapMetaData.msg).
* **~clear_volume** ([ufomap_srvs/ClearVolume](https://github.com/UnknownFreeOccupied/ufomap_ros/blob/master/ufomap_srvs/srv/ClearVolume.srv))  
   Clear a part of the map, specified by a bounding volume. Sets all voxels inside the bounding volume to `~clamping_thres_min`/free space.
* **~reset**  
   Resets the complete map.

### Parameters
* **~frame_id** (string, default: map)  
   Frame in which the map will be published. A transform from sensor data to this frame needs to be available when building maps.
* **~resolution** (double, default: 0.05)  
   Resolution in meter for the map when starting with an empty map; otherwise, the loaded file's resolution is used.
* **~depth_levels** (int, default: 16)  
   Number of depth levels for the octree when starting with an empty map; otherwise, the loaded file's depth_levels is used.
* **~color_map** (bool, default: false)  
   If a colored map should be built when starting with an empty map; otherwise, the loaded file's type is used.
* **~max_range** (double, default: -1.0 (unlimited))  
   Maximum range in meter for inserting point cloud data. Limiting the range to something useful (e.g. 5 m) prevents spurious erroneous points far away from the robot. It also increases the integration speed.
* **~insert_depth** (int, default: 0)  
   The depth of the octree that free space should be cleared at. 0 means at leaf level (finest resolution), higher value moves higher up to coarser resolutions. This significantly increases the integration speed, making it possible to do mapping at millimeter resolution. It is usually good to keep this at a value such that free space is cleared at around 16 cm. So, if your map has a resolution of 2 cm, you would use a value of 3, since 2^3 is 16. Occupied space is always integrated at the finest resolution.
* **~simple_ray_casting** (bool, default: false)  
   If the integration of point clouds should use the simple ray casting method. It can be a good idea to enable this if `~insert_depth != 0`.
* **~early_stopping** (int, default: 0 (disabled))  
   When a ray is cast while being integrated it detects if any other ray has already passed through the current voxel for the same point cloud. If it passes through more than `~early_stopping` voxels other rays have seen in a row, it is taken to be adding no new information and the casting stops. If this is set to 0 it is disabled.
* **~clear_robot** (bool, default: false)  
   Sets all space at the robots current position to free using the four parameters below. This ensures that the robot never be seen as being inside occupied or unknown space, which can otherwise be a problem for path/trajectory planners.
* **~robot_frame_id** (string, default: base_link)  
   The frame_id of the robot. Used when clearing space.
* **~robot_height** (double, default: 0.2)  
   The robot heigh used when clearing space, all space with z [-`~robot_height`, `~robot_height`] relative to `~robot_frame_id` will be cleared.
* **~robot_radius** (double, default: 0.5)  
   The robot heigh used when clearing space, all space with x/y [-`~robot_radius`, `~robot_radius`] relative to `~robot_frame_id` will be cleared.
* **~clearing_depth** (int, default: 0)  
   The depth of the octree that space should be cleared around the robot's current pose.
* **~compress** (bool, default: false)  
   If the published UFOMap ROS messages should be compressed or not. It is a good idea to enable this if transfering the messages between computers.
* **~update_part_of_map** (bool, default: true)  
   If only the updated part of the map should be published instead of the whole map. It is recommended that this is enabled, as it reduces the amount of data that has to be transfered significantly. It is also a lot faster since only a small amount of data has to be serialized/deserialized.
   
   If a node subscribes to the map in the middle of the mapping process, the whole map is sent only to that node. This is to ensure that every nodes that subscribe to the server has a complete map.
* **~update_rate** (double, default: 0.0 (immediately))  
   How often the updated part of the map should be published. Setting this to 0 means it will be published as soon as the map has been updated.
* **~publish_depth** (int, default: 4)  
   What depths should be publish on the `~map_depth_X` topics.
   
   A value of 0 means no `~map_depth_X` will be published.
   
   A value of 2 means, `~map`, `~map_depth_1`, and `~map_depth_2` containing all data, all data down to depth level 1, and all data down to depth level 2, respectively, will be published.
   
   This is useful if you have a system with many nodes that require different map resolutions. By subscribing to the `~map_depth_X` topics, less data has to be transfered.
* **~prob_hit** (double, default: 0.7)  
   Probabilities for hits in the sensor model when integrating a point cloud.
* **~prob_miss** (double, default: 0.4)
   Probabilities for misses in the sensor model when integrating a point cloud.
* **~clamping_thres_min** (double, default: 0.1192)  
   Minimum probability for clamping when building a map.
* **~clamping_thres_max** (double, default: 0.971)  
   Maximum probability for clamping when building a map.
* **~transform_timeout** (double, default: 0.1)  
   Maxmimum time in seconds to wait for the TF transform between the sensor data frame and map frame.
* **~map_queue_size** (int, default: 10)  
   Queue size for the published maps.
* **~cloud_in_queue_size** (int, default: 10)  
   Queue size for the subscribed point cloud.
* **~pub_rate** (double, default: 0.0 (never))  
   How often the whole map should be published. A value of 0.0 means never, then only the updated part of the map will be published (if enabled). This can be good to enable to ensure that every node has a complete map. However, it can take a very long time to publish and process the whole map if using a fine resolution (below 2 cm) and in a big environment.
* **~map_latch** (bool, default: false)  
   Whether the published topics should be latched or not. For maximum performance, set to false.
* **~verbose** (bool, default: false)  
   If enable, information, such as statistics, are outputted.

### Required TF Transforms
* **sensor data frame -> map**  
   Required transform of sensor data into the static map frame. It has to be possible to transform the point cloud coming from `cloud_in` to `frame_id`. You need to supply this transform.
* **robot frame -> map**  
   Only required if `~clear_robot` is enabled. It has to be possible to transform between the robots frame and the map, in order to get the robot's current pose.
   

**NOTE: '~' means it is in the ufomap_server's private namespace.**
