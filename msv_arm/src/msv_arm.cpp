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
/// @file ROS msv_arm class of the MSV-01 rescue robot for ROBOTIS OpenCM 9.04
/// instruction through serial port. This code assumes the OpenCM 9,04 board runs the
/// DynamixelSDK 3.5.4 or superior for the AX-12A servos instruction.
///
/// DYNAMIXEL SDK running on the OpenCM 9.04 C board.
/// Please see https://github.com/ROBOTIS-GIT/DynamixelSDK
///
/// For further information on the OpenCM 9,04 controller bard,
/// please see http://emanual.robotis.com/docs/en/parts/controller/opencm904/
///
/// The use of the Twist message might be useful in case the inverse kinematics functions
/// are implemented. Then the linear/angular components of the Twist message could be used
/// to determining the input position for the inverse kinematics calculation. Note this
/// would require using a variable for storing the accumulated value every time the robot
/// moves, as well as setting maximum and minimum values to prevent wrong calculations.
///
/// In case only forward kinematics is to be tested/used, the HerkuleXcommands should be
/// used right after the "MOVING" labels printed by ROS_STREAM. The Twist command is then
/// not necessary.
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
	pub_angles = n_arm.advertise<std_msgs::Float32MultiArray>("msv/arm_angles", 10);
	// End-effector speed direction along each axis
	pub_effector = n_arm.advertise<geometry_msgs::Twist>("msv/effector", 1000);
	
	// Servos controller topic
	sub_joy = n_arm.subscribe("joy", 10, &MsvArm::joyCallback, this);
	
	// End-effector cartesian space positioning
	x = 4;
	y = 5;
	// up and down equal to +z and -z, respectively
	up = 4;
	down = 6;
	
	power = constrainPower(speed);
	
	// Multiarray set up (for publishing the read angles)
	arm_angles.layout.dim.push_back(std_msgs::MultiArrayDimension());
	// Number of servos
	arm_angles.layout.dim[0].size = 1;
	arm_angles.layout.dim[0].stride = 1;
	arm_angles.layout.dim[0].label = "arm_angles";
	
	// SEtup HerkuleX SDK functions (e.g. echo testing, turning LEDs ON, enabling torque, etc).
	// setLED(2,5);
	// torqueOn(5);
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
			// moveAngle0201(90,5,1,11.2);
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
		
		pub_effector.publish(twist);
		
		// Feedback routine
		// angles[0] = getAngle0201(5);
		// arm_angles.data.push_back(angles[0]);
		// pub_angles.publish(arm_angles);
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
	// Close HerkuleX port
	closeHerkuleX();
}
