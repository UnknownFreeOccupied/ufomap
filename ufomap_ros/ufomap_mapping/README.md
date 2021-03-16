## Dynamic Reconfigure API
ufomap_server offer a [dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure) inferance to change a number of settings on the fly.

## ufomap_server
ufomap_server builds and distributes volumetric 3D occupancy maps as UFOMap binary stream.

### Subscribed Topics
* cloud_in ([sensor_msgs/PointCloud2](http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html))  
   Incoming point cloud for integration. You need to remap this topic to your sensor data topic and provide a TF transform between the sensor data and the static map frame.

### Published Topics

### Services

### Parameters

### Required TF Transforms
