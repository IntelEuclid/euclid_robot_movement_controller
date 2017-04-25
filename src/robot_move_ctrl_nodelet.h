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

#pragma once
# ifndef ROBOT_MOVEMENT_CONTROL_NODELET
# define ROBOT_MOVEMENT_CONTROL_NODELET

///////////////////////////////////////////////
/// Dependencies
///////////////////////////////////////////////

#include <nodelet/nodelet.h>
#include <std_msgs/Float32.h>

#include <dynamic_reconfigure/server.h>
#include <boost/thread.hpp>

#include "robot_movement_control/movement_controlConfig.h"

#include <vector>

namespace realsense_robot_control
{
	///////////////////////////////////////////////
	///	RobotMovementControlNodelet -
	///		This nodelet registered to a number of goals topics (where goal1 has the highest priority and goalN has the Nth priority), and send command velocity messages 
	///			to direct the robot to that goal.
	///			The goal values are actually the sensor input of where the goal is with respect to the robot.
	/// 		
	///			Note - the goal needs to be sent every "frame", 30 FPS in order to reach a gaol. every frame the goal needs to be updated
	///			with the new goal values. 
	///////////////////////////////////////////////
	class RobotMovementControlNodelet : public nodelet::Nodelet
	{
	public :
		const int INTERVAL = 300;

		//===================================
		//	Interface
		//===================================
		virtual void onInit();

		~RobotMovementControlNodelet();
 
	private:
		//===================================
		//	Member Functions
		//===================================
		
		/**
		* populates the parameters from the launch parameters
		*/
		void fillConfigMap();

		/**
		* process the goal and send command velocity messages
		*/
		void process(float x, float y, float z);


		/**
		* Publishes a goal at the given 3D point for Rviz
		*/
		void publishGoal(double x, double y, double z);

		/**
		* Main function that publishes the goals 
		*/
		virtual void mainLoop();

		/**
		* Returns True/False if the goal given by its index should be proccessed   
		*/	
		bool shouldProcessGoal(int goalIndex);

		//***********************************
		// Callback functions
		//***********************************

		void goal1Callback(const geometry_msgs::PointStamped& goal);
		void goal2Callback(const geometry_msgs::PointStamped& goal);

		/**
		* Dynamic Reconfigure Callback
		*/
		void ConfigureCallback(robot_movement_control::movement_controlConfig &config, uint32_t level);

		//***********************************
		// Utils functions
		//***********************************

		/**
		* simple str->bool
		*/
		bool to_bool(std::string str);

		long msecTime();

		

		//===================================
		//	Member Variables
		//===================================
	
		ros::Subscriber mSubGoal2; /**< subscriber to the goal topic */
		ros::Subscriber mSubGoal1; /**< subscriber to the goal topic */

		ros::Publisher mCmdPub; /**< publisher of the velocity command (moving the robot)  */

		std::map<std::string,std::string> mConfig; /**< holds all the configuration received from the command line */

		double mLastCmdX; /**< stores the last linear velocity  */
		double mLastCmdZ; /**< stores the last angular velocity  */
		double mMaxRobotSpeed; /**< Maximum speed (forward and reverse) in m/s */
		
		double mGoalZ; /**< The distance away from the robot which we want to keep the constant distance from */
		double mScaleZ; /**< The scaling factor for translational robot speed */
		double mScaleX; /**< The scaling factor for rotational robot speed */
		double mGoalX; /**< The distance in the horizontal axis which we want to keep the constant distance from */

		bool   mEnabled; /**< Enable/disable flag */

		double mDamperDivisor;  /**< Linear Speed Dumping factor. higher means slower change in linear speed. */

		float mGoalInterval; /**< minimum threshold in which the robot will stop moving if reached that distance from the goal */


		std::string mFrame; /**< Frame ID for the Marker message */

		std::vector<long> mLastGoalsReceivedTimestamps; /**< a vector of timestamps of the received callbacks  */

		boost::shared_ptr<boost::thread> mDeviceThread; /**< thread to hold the loop function  */

		std::vector<float> mGoal;/**< a vector to hold the latest accepted goal  */

		boost::shared_ptr<dynamic_reconfigure::Server<robot_movement_control::movement_controlConfig> > mReconfigureServer; /**< Reconfigure server */

	};
}


#endif // ROBOT_MOVEMENT_CONTROL_NODELET

