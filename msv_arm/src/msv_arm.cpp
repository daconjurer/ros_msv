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
/// @file ROS msv_dxl class of the MSV-01 rescue robot for ROBOTIS OpenCM 9.04
/// instruction through serial port. This code assumes the OpenCM 9,04 board runs the
/// DynamixelSDK 3.5.4 or superior for the AX-12A servos instruction.
///
/// DYNAMIXEL SDK running on the OpenCM 9.04 C board.
/// Please see https://github.com/ROBOTIS-GIT/DynamixelSDK
///
/// For further information on the OpenCM 9,04 controller bard,
/// please see http://emanual.robotis.com/docs/en/parts/controller/opencm904/
///
/// @author Victor Esteban Sandoval-Luna
///
/////////////////////////////////////////////////////////////////////////////////////////

#include <msv_arm/msv_arm.h>

using namespace herkulex;

MsvArm::MsvArm (char* const port, const int& baudrate, const int& verb, const int& speed): ServoHerkulex(verb) 
{
	ROS_INFO("SETTING ARM UP...");
	
	// Servos position topic
	pub_angles = n_arm.advertise<std_msgs::Float32>("msv/arm_angles", 1);
	// End-effector unitary speed direction along each axis
	pub_effector_vel = n_arm.advertise<geometry_msgs::Twist>("msv/effector_uvel", 1000);
	
	// Servos controller topic
	sub_joy = n_arm.subscribe("joy", 10, &MsvArm::joyCallback, this);
	
	// End-effector cartesian space positioning
	x = 4;
	y = 5;
	// up and down equal to +z and -z, respectively
	up = 4;
	down = 6;
	
	power = constrainPower(speed);
	
}

// Controller topic callback function
void MsvArm::joyCallback (const sensor_msgs::Joy::ConstPtr& joy)
{
	if (on == 1) {
		geometry_msgs::Twist twist;
		
		int dx = -1*joy->axes[x];
		int dy = joy->axes[y];
		
		if (joy->buttons[up] == 1) {
			ROS_INFO("MOVING ARM (+z)");
			twist.linear.z = joy->buttons[up];
		}
		else if (joy->buttons[down] == 1) {
			ROS_INFO("MOVING ARM (-z)");
			twist.linear.z = -1*joy->buttons[down];
		}
		
		if (dx == 1) {
			// Move servo until the button is released
			ROS_INFO("MOVING ARM (+x)");
		}
		else if (dx == -1) {
			// Move servo until the button is released
			ROS_INFO("MOVING ARM (-x)");
		}
		
		if (dy == 1) {
			// Move servo until the button is released
			ROS_INFO("MOVING ARM (+y)");
		}
		else if (dy == -1) {
			// Move servo until the button is released
			ROS_INFO("MOVING ARM (-y)");
		}
		
		twist.linear.x = power*x;
		twist.linear.y = power*y;
		
		pub_effector_vel.publish(twist);
	}
}

// MSV-01 robot controller mode callback function
void MsvArm::modeCallback (const std_msgs::String::ConstPtr& mode)
{
	if (mode->data == "arm") {
		on = 1;
		ROS_INFO("ARM ENABLED");
	}
	
	if (mode->data == "robot_z") {
		ROS_INFO("ARM DISABLED");
		on = 0;
	}
}

int MsvArm::constrainPower (const int& pwr) {
	if (pwr > 1 || pwr < 0) {return 1;}
	return pwr;
}

void MsvArm::close () 
{
	// Close DXL port
}
