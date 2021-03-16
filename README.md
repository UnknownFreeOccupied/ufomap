# UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown

UFOMap is a mapping framework with an explicit representation of unknown space.

## Table of Contents
### [Wiki](https://github.com/UnknownFreeOccupied/ufomap/wiki)
### [Setup](https://github.com/UnknownFreeOccupied/ufomap/wiki/Setup)
* [Packages](https://github.com/UnknownFreeOccupied/ufomap/wiki/Setup#packages)
* [Dependencies](https://github.com/UnknownFreeOccupied/ufomap/wiki/Setup#dependencies)
* [Installation](https://github.com/UnknownFreeOccupied/ufomap/wiki/Setup#installation)
* [ROS Integration](https://github.com/UnknownFreeOccupied/ufomap/wiki/Setup#ros-integration)
* [Next Step](https://github.com/UnknownFreeOccupied/ufomap/wiki/Setup#next-step)
### [Tutorials](https://github.com/UnknownFreeOccupied/ufomap/wiki/Tutorials)
* [Creating a UFOMap](https://github.com/UnknownFreeOccupied/ufomap/wiki/Tutorials#creating-a-ufomap)
* [Integrate Point Cloud into UFOMap](https://github.com/UnknownFreeOccupied/ufomap/wiki/Tutorials#integrate-point-cloud-into-ufomap)
* [Collision Checking](https://github.com/UnknownFreeOccupied/ufomap/wiki/Tutorials#collision-checking)
* [K-Nearest Neighbor Search](https://github.com/UnknownFreeOccupied/ufomap/wiki/Tutorials#k-nearest-neighbor-search)
* [ROS Tutorials](https://github.com/UnknownFreeOccupied/ufomap/wiki/Tutorials#ros-tutorials)
### [ROS Tutorials](https://github.com/UnknownFreeOccupied/ufomap/wiki/ROS-Tutorials)
* [Introduction](https://github.com/UnknownFreeOccupied/ufomap/wiki/ROS-Tutorials#introduction)
* [UFOMap Publisher](https://github.com/UnknownFreeOccupied/ufomap/wiki/ROS-Tutorials#ufomap-publisher)
* [UFOMap Subscriber](https://github.com/UnknownFreeOccupied/ufomap/wiki/ROS-Tutorials#ufomap-subscriber)
* [Integrate sensor_msgs/PointCloud2](https://github.com/UnknownFreeOccupied/ufomap/wiki/ROS-Tutorials#integrate-sensor_msgspointcloud2)
* [Perform Mapping](https://github.com/UnknownFreeOccupied/ufomap/wiki/ROS-Tutorials#perform-mapping)
* [Visualize UFOMap in RViz](https://github.com/UnknownFreeOccupied/ufomap/wiki/ROS-Tutorials#visualize-ufomap-in-rviz)
* [General UFOMap Usage](https://github.com/UnknownFreeOccupied/ufomap/wiki/ROS-Tutorials#general-ufomap-usage)
* [Advanced ROS Tutorials](https://github.com/UnknownFreeOccupied/ufomap/wiki/ROS-Tutorials#advanced-ros-tutorials)
### [Advanced ROS Tutorials](https://github.com/UnknownFreeOccupied/ufomap/wiki/Advanced-ROS-Tutorials)
* [Introduction](https://github.com/UnknownFreeOccupied/ufomap/wiki/Advanced-ROS-Tutorials#introduction)
* [Publish Only Updated Part of Map](https://github.com/UnknownFreeOccupied/ufomap/wiki/Advanced-ROS-Tutorials#publish-only-updated-part-of-map)
* [Publish Full Map to New Subscribers](https://github.com/UnknownFreeOccupied/ufomap/wiki/Advanced-ROS-Tutorials#publish-full-map-to-new-subscribers)
* [Publish Multiple Depths](https://github.com/UnknownFreeOccupied/ufomap/wiki/Advanced-ROS-Tutorials#publish-multiple-depths)
* [Support Vanilla and Colored UFOMap in the Same Node](https://github.com/UnknownFreeOccupied/ufomap/wiki/Advanced-ROS-Tutorials#support-vanilla-and-colored-ufomap-in-the-same-node)
### [Performance](https://github.com/UnknownFreeOccupied/ufomap/wiki/Performance)
* [Integrator](https://github.com/UnknownFreeOccupied/ufomap/wiki/Performance#integrator)
* [Memory Consumption](https://github.com/UnknownFreeOccupied/ufomap/wiki/Performance#memory-consumption)
* [Number of Nodes](https://github.com/UnknownFreeOccupied/ufomap/wiki/Performance#number-of-nodes)
* [Exploration](https://github.com/UnknownFreeOccupied/ufomap/wiki/Performance#exploration)
* [More in the Paper](https://github.com/UnknownFreeOccupied/ufomap/wiki/Performance#more-in-the-paper)
### [Example Outputs](https://github.com/UnknownFreeOccupied/ufomap/wiki/Example-Outputs)
### [Data Repository](https://github.com/UnknownFreeOccupied/ufomap/wiki/Data-Repository)
### [API](https://github.com/UnknownFreeOccupied/ufomap/wiki/API)

## Credits
### Paper
* [IEEE](https://ieeexplore.ieee.org/abstract/document/9158399)
* [ArXiv](https://arxiv.org/abs/2003.04749)
### Cite
If you use UFOMap in a scientific publication, please cite the following paper:
* Daniel Duberg and Patric Jensfelt, "UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown," in IEEE Robotics and Automation Letters, vol. 5, no. 4, pp. 6411-6418, Oct. 2020, doi: 10.1109/LRA.2020.3013861.
```
@article{duberg2020ufomap,
  author={Daniel Duberg and Patric Jensfelt},
  journal={IEEE Robotics and Automation Letters}, 
  title={{UFOMap}: An Efficient Probabilistic {3D} Mapping Framework That Embraces the Unknown}, 
  year={2020},
  volume={5},
  number={4},
  pages={6411-6418},
  doi={10.1109/LRA.2020.3013861}
}
```
### Videos
* [Youtube playlist](https://youtube.com/playlist?list=PLoZnKRp2UVom4bv2fUVXgI5VCbuTrfrU3)
