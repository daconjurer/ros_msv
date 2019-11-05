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
/// @file ROS header for the main class of the MSV-01 rescue robot.
/// @author Victor Esteban Sandoval-Luna
///
/// Based on the "rescue" ROS metapackage from José Armando Sánchez-Rojas.
/// Based on the Modbus protocol "lightmodbus" library (RTU) from Jacek Wieczorek,
/// please see https://github.com/Jacajack/liblightmodbus.
/////////////////////////////////////////////////////////////////////////////////////////

#ifndef MSV_MAIN_H_
#define MSV_MAIN_H_

#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <inttypes.h>

class MsvMain
{
	private:
		ros::NodeHandle n_main;
		// MSV-01 robot main publishers
		ros::Publisher pub_state;
		ros::Publisher pub_mode;
		ros::Publisher pub_power;
		ros::Publisher pub_vel;
		// Joy node subscriber
		ros::Subscriber sub_joy;
		
		// Traction power
		int power;
		// Buttons memories
		int l, a, f, s;
		int sel;
		// Twist message publish flag
		int pub_flag;
		// Buttons indexes
		int linear, angular, up, down;
		// Robot modes
		int mode, turn_mode;
		
		// Joy node callback
		void joyCallback (const sensor_msgs::Joy::ConstPtr& joy);
		
	public:
		MsvMain ();
		virtual ~MsvMain () {}
		
};// End of class MsvMain

#endif
