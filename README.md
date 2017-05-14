# Intel&reg; Euclid&trade; Robot movement controller sample.

This nodelet register to pointcloud messages and publishes a goal for the robot movement control to handle. The algorithm is such that the robot should advance as long as it sees there is where to advance. if an obstacle is detected, the robot will find the direction in which it is most likely not to have obstacle, and will continue there. If the robot is too close to an obstacle, it will reverse. 

http://www.intel.com/Euclid_XXX

http://wiki.ros.org/EuclidWiki_XXX

## Subscribed Topics

    camera/depth/points (sensor_msgs/PointCloud2)
        Registered XYZ point cloud.
	
## Published Topics

Point Cloud

    depth_follower/goal (geometry_msgs::PointStamped)
		X,Y,Z goal of where to follow
	depth_follower/marker (visualization_msgs::Marker)
		X,Y,Z for rviz visualization 

## Parameters

    MinY(double, default: 0) 
         The minimum y position of the points in the box
    MaxY(double, default: 20000) 
         The maximum y position of the points in the box
    MinX(double, default: 0) 
         The minimum x position of the points in the box
    MaxX(double, default: 20000) 
         The maximum x position of the points in the box
	MinZ(double, default: 0) 
         The minimum z position of the points in the box
    MaxZ(double, default: 20000) 
         The maximum z position of the points in the box		
	MinBlobSize(int, default: 4000)
		Minimum number of points to consider as a blob
	Enabled(bool, default: true)
		Enable flag for the algorithm
		
## Contributing to the Project

The Intel&reg; Euclid&trade; Robot movement controller sample is developed and distributed under
a BSD-3 license as noted in [licenses/License.txt](licenses/License.txt).

By making a contribution to this project, I certify that:

(a) The contribution was created in whole or in part by me and I
have the right to submit it under the open source license
indicated in the file; or

(b) The contribution is based upon previous work that, to the best
of my knowledge, is covered under an appropriate open source
license and I have the right under that license to submit that
work with modifications, whether created in whole or in part
by me, under the same open source license (unless I am
permitted to submit under a different license), as indicated
in the file; or

(c) The contribution was provided directly to me by some other
person who certified (a), (b) or (c) and I have not modified
it.

(d) I understand and agree that this project and the contribution
are public and that a record of the contribution (including all
personal information I submit with it, including my sign-off) is
maintained indefinitely and may be redistributed consistent with
this project or the open source license(s) involved.

#Configuration:

| Version        | Best Known           |
|:-------------- |:---------------------|
| OS             | Ubuntu 16.04 LTS     |
| ROS            | Kinetic              |

