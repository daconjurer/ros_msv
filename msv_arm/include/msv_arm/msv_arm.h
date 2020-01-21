/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Robótica de la Mixteca
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Universidad Tecnológica de la Mixteca nor
 *     the names of its contributors may be used to endorse or promote
 *     products derived from this software without specific prior
 *     written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
 
/////////////////////////////////////////////////////////////////////////////////////////
/// @file ROS header for the arm class of the MSV-01 rescue robot.
/// @author Victor Esteban Sandoval-Luna
///
/////////////////////////////////////////////////////////////////////////////////////////

#ifndef MSV_DXL_H_
#define MSV_DXL_H_

// Serial communication definitions
#define OCMD_START								0x10
#define OCMD_SETUP								0x20
#define OCMD_INIT									0x30
#define OCMD_READY								0x31

#define DXL_PORT_READY						0x70
#define DXL_BAUDRATE_READY				0x71

#define DXL_PORT_ERROR						0x80
#define DXL_BAUDRATE_ERROR				0x81

// Dynamixel PacketHandler error definitions
#define PH_SUCCESS								0x90  // Tx/Rx packet communication success
#define PH_PORT_BUSY							0x91  // Port is busy (in use)
#define PH_TX_FAIL								0x92  // Failed transmit instruction packet
#define PH_RX_FAIL								0x93  // Failed get status packet
#define PH_TX_ERROR								0x94  // Incorrect instruction packet
#define PH_RX_WAITING							0x95  // Now recieving status packet
#define PH_RX_TIMEOUT							0x96  // There is no status packet
#define PH_RX_CORRUPT							0x97  // Incorrect status packet
#define PH_NOT_AVAILABLE					0x98  // Port not available

#define DXL_SUCCESS								0xA0
#define DXL_VOLTAGE								0xA1
#define DXL_ANGLE									0xA2
#define DXL_OVERHEAT							0xA3
#define DXL_RANGE									0xA4
#define DXL_CHECKSUM							0xA5
#define DXL_OVERLOAD							0xA6
#define DXL_INSTRUCTION						0xA7

#include <ros/ros.h>
#include <herkulex_sdk.h>
#include <unistd.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

using namespace herkulex;

class MsvArm: public ServoHerkulex
{
	private:
		ros::NodeHandle n_arm;
		ros::Publisher pub_angles;
		ros::Publisher pub_effector;
		
		// For the robotic arm control
		ros::Subscriber sub_mode;
		ros::Subscriber sub_joy;
		
		int on;
		int y, x, z;
		int up, down;
		int power;
		std::vector<float> angles = std::vector<float> (1);
		
		std_msgs::Float32MultiArray arm_angles;
		
		void joyCallback (const sensor_msgs::Joy::ConstPtr& joy);
		void modeCallback (const std_msgs::String::ConstPtr& mode);
		
		int constrainPower (const int& pwr);
		
	public:
		// Constructor
		MsvArm (char* port, const int& baudrate, const int& verb, const int& speed);
		
		void close ();
		
};// End of class MsvArm

#endif
