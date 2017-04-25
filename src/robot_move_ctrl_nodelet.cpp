/******************************************************************************
Copyright (c) 2016, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include <thread>
#include <chrono>

#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"

#include "robot_move_ctrl_nodelet.h"

//Nodelet dependencies
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(	realsense_robot_control::RobotMovementControlNodelet, nodelet::Nodelet)

namespace realsense_robot_control
{
	

	//******************************
	// Public Methods
	//******************************

	RobotMovementControlNodelet::~RobotMovementControlNodelet()
	{
		ROS_INFO_STREAM("Done - RobotMovementControlNodelet");
	}

	void RobotMovementControlNodelet::onInit()
	{
		ROS_INFO_STREAM("Starting RobotMovementControlNodelet");

		fillConfigMap();

		ros::NodeHandle& nh = getNodeHandle();

		mLastGoalsReceivedTimestamps.push_back(0);
		mLastGoalsReceivedTimestamps.push_back(0);

		//Setting default values. when the X = goalX and Z=goalZ. this will cause the robot to be in place as the error will be zero.
		mGoal.push_back(mGoalX);
		mGoal.push_back(0);
		mGoal.push_back(mGoalZ);

		//subscribe to depth_follower/goal and person_follower/goal"
		mSubGoal1 = nh.subscribe("goal1", 1, &RobotMovementControlNodelet::goal1Callback, this);
		mSubGoal2 = nh.subscribe("goal2", 1, &RobotMovementControlNodelet::goal2Callback, this);

		//advertise
		mCmdPub = nh.advertise<geometry_msgs::Twist> ("cmd_vel_mux/input/teleop", 1);
  
		// Initialize dynamic reconfigure
		mReconfigureServer.reset(new dynamic_reconfigure::Server<robot_movement_control::movement_controlConfig>(getPrivateNodeHandle()));
		mReconfigureServer->setCallback(boost::bind(&RobotMovementControlNodelet::ConfigureCallback, this, _1, _2));

		mDeviceThread = boost::shared_ptr< boost::thread >(new boost::thread(boost::bind(&RobotMovementControlNodelet::mainLoop, this)));

	}

	//===================================
	//	Member Functions
	//===================================

	void RobotMovementControlNodelet::mainLoop()
	{

		ros::Rate r(30);
		//X,Y,Z of the centroid
		while (ros::ok())
		{
			float x = mGoal[0];
			float y = mGoal[1];
			float z = mGoal[2];

			if (mEnabled)
			{
				geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());

				float errorX = x - mGoalX;
				float errorZ = z - mGoalZ;
				//Linear velocity:
				//------------------
				// Simple gradual
				mLastCmdX = (mDamperDivisor*mLastCmdX + errorZ * mScaleZ)/(mDamperDivisor+1) ;

				//Stabilize
				if (fabs(mLastCmdX) < mGoalInterval)
				{
					mLastCmdX = 0;
				}

				//do not pass the limits
				if (fabs(mLastCmdX) > mMaxRobotSpeed)
				{
					if (mLastCmdX > 0)
						mLastCmdX = mMaxRobotSpeed;
					else
						mLastCmdX = -mMaxRobotSpeed;
				}

				cmd->linear.x = mLastCmdX;

				//Angular velocity:
				//------------------

				mLastCmdZ = (mDamperDivisor*mLastCmdZ + errorX * mScaleX) / (mDamperDivisor + 1);

				//Stabilize
				if (fabs(mLastCmdZ) < mGoalInterval)
				{
					mLastCmdZ = 0;
				}

				cmd->angular.z = -mLastCmdZ;

				mCmdPub.publish(cmd);

				mGoal[0] = mGoalX;
				mGoal[1] = 0;
				mGoal[2] = mGoalZ;

			}

			ros::spinOnce();
			r.sleep();

		}
	}

	void RobotMovementControlNodelet::fillConfigMap()
	{
		std::vector<std::string> args = getMyArgv();
		while (args.size() > 1)
		{
			mConfig[args[0]] = args[1];
			args.erase(args.begin());
			args.erase(args.begin());
		}


		mEnabled = true;

		mGoalZ=0.6;
		mScaleZ=1.0;
		mScaleX=5.0;
		mMaxRobotSpeed = 1.0;

		mGoalX = 0;
		mGoalInterval = 0.1;
		mDamperDivisor = 30;

		char* key = (char*)"GoalInterval";
		if (mConfig.find(key) != mConfig.end())
		{
			ROS_INFO("Setting %s to %s", key, mConfig.at(key).c_str());
			mGoalInterval = atof(mConfig.at(key).c_str());
		}

		key = (char*)"DamperDivisor";
		if (mConfig.find(key) != mConfig.end())
		{
			ROS_INFO("Setting %s to %s", key, mConfig.at(key).c_str());
			mDamperDivisor = atof(mConfig.at(key).c_str());
		}

		key = (char*)"GoalZ";
		if (mConfig.find(key) != mConfig.end())
		{
			ROS_INFO("Setting %s to %s", key, mConfig.at(key).c_str());
			mGoalZ = atof(mConfig.at(key).c_str());
		}

		key = (char*)"ZScale";
		if (mConfig.find(key) != mConfig.end())
		{
			ROS_INFO("Setting %s to %s", key, mConfig.at(key).c_str());
			mScaleZ = atof(mConfig.at(key).c_str());
		}

		key = (char*)"XScale";
		if (mConfig.find(key) != mConfig.end())
		{
			ROS_INFO("Setting %s to %s", key, mConfig.at(key).c_str());
			mScaleX = atof(mConfig.at(key).c_str());
		}

		key = (char*)"GoalX";
		if (mConfig.find(key) != mConfig.end())
		{
			ROS_INFO("Setting %s to %s", key, mConfig.at(key).c_str());
			mGoalX = atof(mConfig.at(key).c_str());
		}


		key = (char*)"MaxRobotSpeed";
		if (mConfig.find(key) != mConfig.end())
		{
			ROS_INFO("Setting %s to %s", key, mConfig.at(key).c_str());
			mMaxRobotSpeed = atof(mConfig.at(key).c_str());
		}

		//more defaults
		mLastCmdX = 0;
		mLastCmdZ = 0;

	}

	void RobotMovementControlNodelet::process(float x, float y, float z)
	{
		mGoal[0] = x;
		mGoal[1] = y;
		mGoal[2] = z;
	}


	//***********************************
	// Callback functions
	//***********************************

	bool RobotMovementControlNodelet::shouldProcessGoal(int indx)
	{

		for (int i = 0; i < mLastGoalsReceivedTimestamps.size() && i < indx; i++)
		{
			if (mLastGoalsReceivedTimestamps[indx] < mLastGoalsReceivedTimestamps[i] + INTERVAL)
			{
				return false;
			}
		}

		return true;
	}

	void RobotMovementControlNodelet::ConfigureCallback(robot_movement_control::movement_controlConfig &config, uint32_t level)
	{

		mEnabled = config.mEnabled;

		mGoalInterval = config.GoalInterval;
		mDamperDivisor = config.DamperDivisor;
		mGoalZ = config.GoalZ;
		mScaleZ = config.ZScale;
		mScaleX = config.XScale;
		mGoalX = config.GoalX;
		mMaxRobotSpeed = config.MaxRobotSpeed;

	}


	void RobotMovementControlNodelet::goal1Callback(const geometry_msgs::PointStamped& goal)
	{
		ROS_INFO_STREAM("goal = "<<goal.point.z);
		mLastGoalsReceivedTimestamps[0] = msecTime();
		process(goal.point.x, goal.point.y, goal.point.z);
	}

	void RobotMovementControlNodelet::goal2Callback(const geometry_msgs::PointStamped& goal)
	{
		mLastGoalsReceivedTimestamps[1] = msecTime();
		if (shouldProcessGoal(1))
		{
			process(goal.point.x, goal.point.y, goal.point.z);
		}
	}



	//***********************************
	// Utils functions
	//***********************************

	bool RobotMovementControlNodelet::to_bool(std::string str)
	{
		std::transform(str.begin(), str.end(), str.begin(), ::tolower);
		std::istringstream is(str);

		bool b;

		is >> std::boolalpha >> b;

		return b;
	}

	long RobotMovementControlNodelet::msecTime()
	{
		struct timeval time;
		gettimeofday(&time, NULL);
		return time.tv_sec * 1000 + time.tv_usec / 1000;
	}

}

