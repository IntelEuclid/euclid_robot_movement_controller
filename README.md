# Intel&reg; Euclid&trade; Robot movement controller sample.

This nodelet registered to a number of goals topics (where goal1 has the highest priority and goalN has the Nth priority), and send command velocity messages to direct the robot to that goal. The goal values are actually the sensor input of where the goal is with respect to the robot. Note - the goal needs to be sent every "frame", 30 FPS in order to reach a gaol. every frame the goal needs to be updated with the new goal values.

[Intel® Euclid™ Community Site](http://www.euclidcommunity.intel.com).

[Intel® Euclid™ Support Forum](http://www.intel.com/content/www/us/en/support/emerging-technologies/intel-euclid-development-kit.html).

## Subscribed Topics

    goal1 (geometry_msgs::PointStamped)
        X,Y,Z goal (priority 1)
	goal2 (geometry_msgs::PointStamped)
        X,Y,Z goal (priority 2)
	
## Published Topics

Point Cloud

    cmd_vel_mux/input/teleop (geometry_msgs::Twist)
		Command velocity message for the robot

## Parameters

    MaxRobotSpeed(double, default: 1) 
         Maximum speed (forward and reverse) in m/s
    GoalZ(double, default: 0.6) 
         The distance away from the robot which we want to keep the constant distance from
    GoalX(double, default: 0) 
         The distance in the horizontal axis which we want to keep the constant distance from
    ScaleZ(double, default: 1) 
         The scaling factor for translational robot speed
	ScaleX(double, default: 5) 
         The scaling factor for rotational robot speed
    DamperDivisor(double, default: 30) 
         Linear Speed Damping factor. higher means slower change in linear speed.
	GoalInterval(float, default: 0.1)
		minimum threshold in which the robot will stop moving if reached that distance from the goal
	Enabled(bool, default: true)
		Enable flag for the algorithm
		
## Contributing to the Project

The Intel&reg; Euclid&trade; Robot movement controller sample is developed and distributed under
a BSD-3 license as noted in [License file](LICENSE).

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

## Configuration:

| Version        | Best Known           |
|:-------------- |:---------------------|
| OS             | Ubuntu 16.04 LTS     |
| ROS            | Kinetic              |

