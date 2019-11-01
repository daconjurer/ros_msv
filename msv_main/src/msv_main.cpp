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
/// @file ROS main class of the MSV-01 rescue robot for general control using the ROS
/// Joy messages. The node publishes the teleoperation commands, as well as the control
/// commands for the robotic arm and an orientation system for the RGBD sensor.
/// @author Victor Esteban Sandoval-Luna
///
/// Based on the "rescue" ROS metapackage from José Armando Sánchez-Rojas.
/// Based on the Modbus protocol "lightmodbus" library (RTU) from Jacek Wieczorek,
/// please see https://github.com/Jacajack/liblightmodbus.
/////////////////////////////////////////////////////////////////////////////////////////

/* PENDINGS */
// Manage verbosity
// Publish moving state

#include <msv_main/msv_main.h>

MsvMain::MsvMain (const int& verb) 
{
	ROS_INFO("SETTING ROBOT UP...");
	
	// Joy controller topic
	sub_joy = n_main.subscribe("joy", 1, &MsvMain::joyCallback, this);
	
	// Robot's state topic (published when the slave confirms wether the robot is moving or not)
	pub_state = n_main.advertise<std_msgs::String>("msv/state", 1);
	// Mode topic (published every time a new mode is selected [SELECT button])
	pub_mode = n_main.advertise<std_msgs::String>("msv/mode", 1);
	// Power topic (published every time the max power changes [START button])
	pub_power = n_main.advertise<std_msgs::UInt8>("msv/power", 1);
	// Twist topic (debug purposes, published every time the cmd_vel of the robot changes)
	pub_vel = n_main.advertise<geometry_msgs::Twist>("msv/cmd_vel", 10);
	// Modbus traction system coils topic (published every time a new Joy command arrives)
	pub_coils = n_main.advertise<std_msgs::UInt8MultiArray>("msv/ts_coils", 10);
	// Modbus traction system holding regs topic (published every time a new Joy command arrives)
	pub_regs = n_main.advertise<std_msgs::UInt8MultiArray>("msv/ts_regs", 10);
	
	// liblightmodbus Master struct init
	mec = modbusMasterInit(&master);
	if (mec != MODBUS_OK)
		ROS_INFO("MODBUS MASTER COULD NOT BE INITIALIZED");
	else ROS_INFO("MODBUS MASTER INITIALIZED");
	
	// Verbosity and Max traction power attributes
	verbosity = verb;
	power = 90;
	
	sel = 0;
	l = 0;
	a = 0;
	f = 0;
	s = 0;
	
	send = 0;
	
	ROS_INFO("MAX POWER: 100%%. MIN POWER: 35%%.");
	// Joystick attributes
	linear = 1;
	angular = 2;
	up = 5;
	down = 7;
	
	mode = 0; // Robot mode
	turn_mode = 0;  // Turn around z axis
	
	// Multiarrays set up
	bpt_preset_hregs.layout.dim.push_back(std_msgs::MultiArrayDimension());
	bpt_preset_hregs.layout.dim[0].size = 17;
	bpt_preset_hregs.layout.dim[0].stride = 1;
	bpt_preset_hregs.layout.dim[0].label = "ts_preset_hregs";
	
	bpt_forced_coils.layout.dim.push_back(std_msgs::MultiArrayDimension());
	bpt_forced_coils.layout.dim[0].size = 10;
	bpt_forced_coils.layout.dim[0].stride = 1;
	bpt_forced_coils.layout.dim[0].label = "ts_forced_coils";
	
	ROS_INFO("VERBOSITY: %d", verbosity);
	
	ROS_INFO("SET UP DONE");
}

// Joy callback method
void MsvMain::joyCallback (const sensor_msgs::Joy::ConstPtr& joy) 
{
	int select, start;
	int send = 0;
	std_msgs::UInt8 p;
	
	select = joy->buttons[8];
	start = joy->buttons[9];
	
	// Not intended to change any external value
	if (select == 1) {
		std_msgs::String out;
		
		switch (sel) {
			case 0:
				// From Robot mode to Arm mode
				mode = 1;
				sel++;
				out.data = "arm";
				pub_mode.publish(out);
				ROS_INFO("ARM CONTROLLER MODE");
				break;
			case 1:
				// From Arm mode to Robot mode (with power control)
				mode = 0;
				sel++;
				turn_mode = 0;
				out.data = "robot_z";
				pub_mode.publish(out);
				ROS_INFO("CONTROLLING POWER");
				break;
			case 2:
				// Robot mode (power control disabled, turn around z axis)
				sel++;
				turn_mode = 0;
				ROS_INFO("ROBOT CONTROLLER MODE (Turn around z axis)");
				break;
			case 3:
				// Robot mode (power control disabled, turn around z axis)
				sel++;
				turn_mode = 1;
				out.data = "robot_s";
				pub_mode.publish(out);
				ROS_INFO("ROBOT CONTROLLER MODE (Smooth turn)");
				break;
			case 4:
				// From Robot mode to RGBD mode
				sel = 0;
				mode = 1;
				turn_mode = 0;
				out.data = "rgbd";
				pub_mode.publish(out);
				ROS_INFO("RGBD SYSTEM CONTROLLER MODE");
				break;
			default:
				break;
		}
		s = select;
		return;
	}
	
	if (start == 1) {
		switch (sel) {
			case 2:
				if (power == 254) {		// 255 (100 %)
					power = 89;		// 90
					ROS_INFO("POWER: 34%%");
				} else {
					int tpwr;
					power += 55;
					tpwr = int(power*100.0/254.0);		// 255
					ROS_INFO("POWER: %d%%", tpwr);
				}
				p.data = power;
				pub_power.publish(p);
				break;
			default:
				break;
		}
		return;
	}
	
	// Robot Controller mode
	if (mode == 0 && s != 1) {
		geometry_msgs::Twist twist;
		int angular_z  = -power*joy->axes[angular];
		int linear_x = power*joy->axes[linear];
		
		hregs[2] = 0x0000;
		hregs[3] = 0x0000;
		
		/* Forward/reverse */
		if (linear_x > 0) {
			send = 1;
			// SV
			hregs[0] = (uint16_t)linear_x;
			hregs[1] = (uint16_t)linear_x;
			
			// F/R
			coils[0] = 0x11;    // 0001 0001
			ROS_INFO("Moving: forward");
		}
		else if (linear_x < 0) {
			send = 1;
			// SV
			hregs[0] = (uint16_t)(-1*linear_x);
			hregs[1] = (uint16_t)(-1*linear_x);
			
			// F/R
			coils[0] = 0x12;    // 0001 0010
			ROS_INFO("Moving: reverse");
		}
		else if (linear_x == 0 && l != 0) {
			send = 1;
			// SV
			hregs[0] = 0x0000;
			hregs[1] = 0x0000;
			
			// Stop
			coils[0] = 0x00;    // 0000 0000 (could be 0x03 as well)
			ROS_INFO("STOP");
		}
		
		/* Turn routines */
		// Turn around z axis (mode 0)
		else if (angular_z > 0 && turn_mode == 0) {
			send = 1;
			// SV
			hregs[0] = (uint16_t)angular_z;
			hregs[1] = (uint16_t)angular_z;
			
			// F/R
			coils[0] = 0x13;    // 0001 0011
			ROS_INFO("Z axis turning: right");
		}
		else if (angular_z < 0 && turn_mode == 0) {
			send = 1;
			// SV
			hregs[0] = (uint16_t)(-1*angular_z);
			hregs[1] = (uint16_t)(-1*angular_z);
			
			// F/R
			coils[0] = 0x10;    // 0001 0000
			ROS_INFO("Z axis turning: left");
		}
		else if ((angular_z == 0 && turn_mode == 0) && a != 0) {
			send = 1;
			coils[0] = 0x00;
			// SV
			hregs[0] = 0x0000;
			hregs[1] = 0x0000;
			
			// Stop
			coils[0] = 0x00;    // 0000 0000 (could be 0x03 as well)
			ROS_INFO("STOP");
		}
		// Smooth turn (mode 1)
		else if (angular_z > 0 && turn_mode == 1) {
			send = 1;
			// SV
			hregs[0] = (uint16_t)angular_z;
			hregs[1] = (uint16_t)(angular_z/2);
			
			// F/R
			coils[0] = 0x11;    // 0001 0001
			ROS_INFO("Smooth turning: right");
		}
		else if (angular_z < 0 && turn_mode == 1) {
			send = 1;
			// SV
			hregs[1] = (uint16_t)(-1*angular_z);
			hregs[0] = (uint16_t)(-angular_z/2);
			
			// F/R
			coils[0] = 0x12;    // 0001 0010
			ROS_INFO("Smooth turning: left");
		}
		else if ((angular_z == 0 && turn_mode == 1) && a != 0) {
			send = 1;
			// SV
			hregs[0] = 0x0000;
			hregs[1] = 0x0000;
			
			// Stop
			coils[0] = 0x00;    // 0000 0000 (could be 0x03 as well)
			ROS_INFO("STOP");
		}
		
		if (joy->buttons[down] == 1) {
			f = joy->buttons[down];
			
			send = 1;
			// Flippers DOWN
			hregs[2] = 0x00FE;
			hregs[3] = 0x00FE;
			
			// Direction
			coils[0] = 0x14;    // 0001 0100
			ROS_INFO("Moving flippers: down");
			
			// Debug
			twist.linear.y = -f;
		}
		else if (joy->buttons[up] == 1) {
			f = joy->buttons[up];
			
			send = 1;
			// Flippers UP
			hregs[2] = 0x00FE;
			hregs[3] = 0x00FE;
			
			// Direction
			coils[0] = 0x18;    // 0001 1000
			ROS_INFO("Moving flippers: up");
			
			// Debug
			twist.linear.y = f;
		}
		else if ((joy->buttons[up] == 0 && joy->buttons[down] == 0) && f != 0) {
			f = 0;
			send = 1;
			// Flippers OFF
			hregs[2] = 0x0000;
			hregs[3] = 0x0000;
			
			// Stop
			coils[0] = 0x00;    // 0000 0000 (could be 0x0C as well)
			ROS_INFO("Flippers stopped");
		}
		
		l = linear_x;
		a = angular_z;
		
		if (send == 1) {
			// Debug
			twist.angular.z = angular_z;
			twist.linear.x = linear_x;
			pub_vel.publish(twist);
			
			if (verbosity == 0) {
				// Holding registers frame
				modbusBuildRequest16(&master,0,0,4,hregs.data());
				bpt_preset_hregs.data.clear();
				for (int i = 0; i < master.request.length; i++) {
					bpt_preset_hregs.data.push_back(master.request.frame[i]);
				}
				pub_regs.publish(bpt_preset_hregs);
				
				// Coils frame
				modbusBuildRequest15(&master,0,0,5,coils.data());
				bpt_forced_coils.data.clear();
				for (int i = 0; i < master.request.length; i++) {
					bpt_forced_coils.data.push_back(master.request.frame[i]);
				}
				pub_coils.publish(bpt_forced_coils);
				
			} else {
				// Holding registers frame
				modbusBuildRequest16(&master,0,0,4,hregs.data());
				bpt_preset_hregs.data.clear();
				for (int i = 0; i < master.request.length; i++) {
					bpt_preset_hregs.data.push_back(master.request.frame[i]);
				}
				pub_regs.publish(bpt_preset_hregs);
				//printQuery();
				printRegs();
				
				// Coils frame
				modbusBuildRequest15(&master,0,0,5,coils.data());
				bpt_forced_coils.data.clear();
				for (int i = 0; i < master.request.length; i++) {
					bpt_forced_coils.data.push_back(master.request.frame[i]);
				}
				pub_coils.publish(bpt_forced_coils);
				//printQuery();
				printCoils();
			}
		}
	}
	s = select;
}

void MsvMain::printQuery () 
{
	for (int i = 0; i < master.request.length; i++) {
		printf("%X ", master.request.frame[i]);
	}
	printf("\n");
}

void MsvMain::printRegs () 
{
	for (int i = 0; i < hregs.size(); i++) {
		printf("%X ", hregs[i]);
	}
	printf("\n");
}

void MsvMain::printCoils () 
{
	for (int i = 0; i < coils.size(); i++) {
		printf("%X ", coils[i]);
	}
	printf("\n");
}

