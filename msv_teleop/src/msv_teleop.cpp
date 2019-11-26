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
/// @file ROS msv_teleop class of the MSV-01 rescue robot for remote control using
/// commands from the ROS msv_main node. The commands are sent using the Modbus RTU
/// protocol through serial port (connected to the USART of the BPT board).
///
/// @author Victor Esteban Sandoval-Luna
///
/// Based on the "rescue" ROS metapackage from José Armando Sánchez-Rojas.
/////////////////////////////////////////////////////////////////////////////////////////

/* TODO */
// Add sensors debug mode when the serial port is not available (using the sendPacketBPT method)

#include <msv_teleop/msv_teleop.h>

MsvTeleop::MsvTeleop (char* const port, const int& baudrate, const int& verb) : bpt(port,baudrate)
{
	ROS_INFO("SETTING TELEOP NODE UP...");
	
	// cmd_vel commands subscriber
	sub_vel = n_teleop.subscribe("msv/cmd_vel", 1, &MsvTeleop::twistCallback, this);
	// turn mode subscriber
	sub_mode = n_teleop.subscribe("msv/mode", 1, &MsvTeleop::modeCallback, this);
	
	// Actuators' data topic (published when the slave sends the actuators data)
	pub_actuators = n_teleop.advertise<msv_msgs::Actuators>("msv/actuators", 1);
	// Modbus traction system read coils topic (published every time the slave sends the coils data)
	pub_rcoils = n_teleop.advertise<std_msgs::UInt8MultiArray>("msv/ts_read_coils",1);
	// Modbus traction system read input registers topic (published every time the slave sends the input registers data)
	pub_iregs = n_teleop.advertise<std_msgs::UInt8MultiArray>("msv/ts_read_iregs",1);
	// Modbus traction system forced coils topic (published every time a new Joy command arrives)
	pub_fcoils = n_teleop.advertise<std_msgs::UInt8MultiArray>("msv/ts_forced_coils", 10);
	// Modbus traction system holding regs topic (published every time a new Joy command arrives)
	pub_hregs = n_teleop.advertise<std_msgs::UInt8MultiArray>("msv/ts_preset_hregs", 10);
	
	// Port handler
	if (bpt.openPort()) {
		ROS_INFO("CONTROLLER BPT SERIAL PORT OPEN. INITIALIZING BPT NOW...");
	} else {
		ROS_ERROR("Port not available. The node will not start");
		while (ros::ok());
		
		// Controller debug mode (see TODO)
		//ROS_INFO("CONTROLLER DEBUG MODE");
	}
	
	// liblightmodbus Master struct init
	mec = modbusMasterInit(&master);
	if (mec != MODBUS_OK) {
		ROS_ERROR("MODBUS MASTER COULD NOT BE INITIALIZED");
		while (ros::ok());
	} else { ROS_INFO("MODBUS MASTER INITIALIZED");}
	
	// Multiarrays set up
	bpt_read_iregs.layout.dim.push_back(std_msgs::MultiArrayDimension());
	bpt_read_iregs.layout.dim[0].size = 8;
	bpt_read_iregs.layout.dim[0].stride = 1;
	bpt_read_iregs.layout.dim[0].label = "ts_read_iregs";
	
	bpt_read_coils.layout.dim.push_back(std_msgs::MultiArrayDimension());
	bpt_read_coils.layout.dim[0].size = 8;
	bpt_read_coils.layout.dim[0].stride = 1;
	bpt_read_coils.layout.dim[0].label = "ts_read_coils";
	
	bpt_preset_hregs.layout.dim.push_back(std_msgs::MultiArrayDimension());
	bpt_preset_hregs.layout.dim[0].size = 17;
	bpt_preset_hregs.layout.dim[0].stride = 1;
	bpt_preset_hregs.layout.dim[0].label = "ts_preset_hregs";
	
	bpt_forced_coils.layout.dim.push_back(std_msgs::MultiArrayDimension());
	bpt_forced_coils.layout.dim[0].size = 10;
	bpt_forced_coils.layout.dim[0].stride = 1;
	bpt_forced_coils.layout.dim[0].label = "ts_forced_coils";
	
	// Verbosity attribute
	verbosity = verb;
	// Turn mode attribute (z axis turn by default)
	turn_mode = 0;
	
	l = 0;
	a = 0;
	f = 0;
	
	// Actuators data array and Actuators message set up
	for (int i = 0; i < 6; i++) {
		actuators[i] = 0.0;
	}
	actuators_msg.header.frame_id = "msv_actuators";
	actuators_msg.values.reserve(6);
	
	ROS_INFO("MSV_TELEOP READY");
}

void MsvTeleop::modeCallback (const std_msgs::String::ConstPtr& mode) 
{
	if (mode->data == "robot_z") {
		turn_mode = 0;
	}
	if (mode->data == "robot_s") {
		turn_mode = 1;
	}
}

void MsvTeleop::twistCallback (const geometry_msgs::Twist& twist)
{
	float linear_x = twist.linear.x;
	float angular_z = twist.angular.z;
	float linear_y = twist.linear.y;
	
	hregs[2] = 0x0000;
	hregs[3] = 0x0000;
	
	// Decode the twist values and convert into 8-bits data (for PWM)
	
	// Forward/reverse
	if (linear_x > 0) {
		// SV
		hregs[0] = (uint16_t)linear_x;
		hregs[1] = (uint16_t)linear_x;
		
		// F/R
		fcoils[0] = 0x11;    // 0001 0001
	}
	else if (linear_x < 0) {
		// SV
		hregs[0] = (uint16_t)(-1*linear_x);
		hregs[1] = (uint16_t)(-1*linear_x);
		
		// F/R
		fcoils[0] = 0x12;    // 0001 0010
	}
	else if (linear_x == 0 && l != 0) {
		// SV
		hregs[0] = 0x0000;
		hregs[1] = 0x0000;
		
		// Stop
		fcoils[0] = 0x00;    // 0000 0000 (could be 0x03 as well)
	}
	
	// Turn routines
	// Turn around z axis (mode 0)
	else if (angular_z > 0 && turn_mode == 0) {
		// SV
		hregs[0] = (uint16_t)angular_z;
		hregs[1] = (uint16_t)angular_z;
		
		// F/R
		fcoils[0] = 0x13;    // 0001 0011
	}
	else if (angular_z < 0 && turn_mode == 0) {
		// SV
		hregs[0] = (uint16_t)(-1*angular_z);
		hregs[1] = (uint16_t)(-1*angular_z);
		
		// F/R
		fcoils[0] = 0x10;    // 0001 0000
	}
	else if ((angular_z == 0 && turn_mode == 0) && a != 0) {
		// SV
		hregs[0] = 0x0000;
		hregs[1] = 0x0000;
		
	// Stop
		fcoils[0] = 0x00;    // 0000 0000 (could be 0x03 as well)
	}
	// Smooth turn (mode 1)
	else if (angular_z > 0 && turn_mode == 1) {
		// SV
		hregs[0] = (uint16_t)angular_z;
		hregs[1] = (uint16_t)(angular_z/2);
		
		// F/R
		fcoils[0] = 0x11;    // 0001 0001
	}
	else if (angular_z < 0 && turn_mode == 1) {
		// SV
		hregs[1] = (uint16_t)(-1*angular_z);
		hregs[0] = (uint16_t)(-angular_z/2);
		
		// F/R
		fcoils[0] = 0x12;    // 0001 0010
	}
	else if ((angular_z == 0 && turn_mode == 1) && a != 0) {
		// SV
		hregs[0] = 0x0000;
		hregs[1] = 0x0000;
		
		// Stop
		fcoils[0] = 0x00;    // 0000 0000 (could be 0x03 as well)
	}
	
	if (linear_y < 0) {
		f = linear_y;
		// Flippers DOWN
		hregs[2] = 0x00FE;
		hregs[3] = 0x00FE;
		
		// Direction
		fcoils[0] = 0x14;    // 0001 0100
	}
	else if (linear_y > 0) {
		f = linear_y;
		// Flippers UP
		hregs[2] = 0x00FE;
		hregs[3] = 0x00FE;
		
		// Direction
		fcoils[0] = 0x18;    // 0001 1000
	}
	else if (linear_y == 0 && f != 0) {
		f = 0;
		// Flippers OFF
		hregs[2] = 0x0000;
		hregs[3] = 0x0000;
		
		// Stop
		fcoils[0] = 0x00;    // 0000 0000 (could be 0x0C as well)
	}
	
	l = linear_x;
	a = angular_z;
	
	/* Build up, send and publish frames */
	
	// Coils forcing frame
	mec = modbusBuildRequest15(&master,2,0,5,fcoils.data());
	sendreceivePacketBPT(8,verbosity);
	
	bpt_forced_coils.data.clear();
	for (int i = 0; i < master.request.length; i++) {
		bpt_forced_coils.data.push_back(master.request.frame[i]);
	}
	pub_fcoils.publish(bpt_forced_coils);

	// Holding registers presetting frame
	mec = modbusBuildRequest16(&master,2,0,4,hregs.data());
	sendreceivePacketBPT(8,verbosity);
	
	bpt_preset_hregs.data.clear();
	for (int i = 0; i < master.request.length; i++) {
		bpt_preset_hregs.data.push_back(master.request.frame[i]);
	}
	pub_hregs.publish(bpt_preset_hregs);
}

void MsvTeleop::readCoils ()
{
	// Coils reading frame
	mec = modbusBuildRequest01(&master,2,5,2);		// addresses 6-7
		//sendPacketBPT(verbosity);
	sendreceivePacketBPT(6,verbosity);
	
	bpt_read_coils.data.clear();
	for (int i = 0; i < master.request.length; i++) {
		bpt_read_coils.data.push_back(master.request.frame[i]);
	}
	pub_rcoils.publish(bpt_read_coils);
	
	// Coils data decoding and publishing
	for (int i = 0; i < master.data.count; i++) {
		rcoils[i] = modbusMaskRead(master.data.coils,master.data.length,i);
		//printf("%d\n",rcoils[i]);
	}
	
	for (int i = 0; i < 2; i++) {
		alarms[i] = (rcoils[i] == 0x80) ? 1 : 0;
	}
	
	actuators_msg.alarms = alarms;
	return;
}

void MsvTeleop::readInputRegisters ()
{
	// Input registers reading frame
	mec = modbusBuildRequest03(&master,2,49,4);		// adresses 50 - 53
	
	//sendPacketBPT(verbosity);
	sendreceivePacketBPT(13,verbosity);
	bpt_read_iregs.data.clear();
	for (int i = 0; i < master.request.length; i++) {
		bpt_read_iregs.data.push_back(master.request.frame[i]);
	}
	pub_iregs.publish(bpt_read_iregs);
	
	// Input registers data decoding and publishing
	for (int i = 0; i < master.data.count; i++) {
		iregs[i] = master.data.regs[i];
		actuators[i] = iregs[i];
	}
	
	actuators_msg.values = actuators;
	return;
}

void MsvTeleop::sense ()
{
	readCoils();
	readInputRegisters();
	
	actuators_msg.header.stamp = ros::Time::now();
	pub_actuators.publish(actuators_msg);
}

int MsvTeleop::sendreceivePacketBPT (const int& ack_length, const int& verbose)
{
	uint8_t rl = master.request.length + 1;
	ack_modbus.resize(ack_length);
	int n, k;
	
	uint8_t* buf = master.request.frame;
	*(buf+rl-1) = 0xFF;
	
	// Send query
	bpt.clearPort();
	k = bpt.writePort(buf, rl);
	usleep ((rl + ack_length) * 10);
	
	// Receive response
	for (int i = 0; i < ack_length; i++) {
		n = bpt.readPort(&ack_modbus[i]);
	}
	
	if (k != rl) {
		return -1;
	}
	
	if (verbose) {
		for (int j = 0; j < ack_length; j++) {
			printf("%X ", ack_modbus[j]);
		}
		printf("\n");
	}
	
	master.response.length = ack_length;
	master.response.frame = ack_modbus.data();
	
	// Parse response using lightmodbus
	mec = modbusParseResponse(&master);
	if (mec != MODBUS_OK) {
		ROS_ERROR("MODBUS RESPONSE IS NOT CORRECT (MEC %d)",mec);
		return -2;
	}
	
	return n;
}

int MsvTeleop::sendPacketBPT (const int& verbose)
{
	int k = 0;
	uint8_t rl = master.request.length + 1;
	
	uint8_t* buf = master.request.frame;
	*(buf+rl-1) = 0xFF;
	
	bpt.clearPort();
	k = bpt.writePort(buf,rl);
	usleep (rl * 10);
	
	if (k != rl) {
		return -1;
	}
	
	if (verbose) {
		for (int j = 0; j < rl-1; j++) {
			printf("%X ", master.request.frame[j]);
		}
		printf("\n");
	}
	
	return k;
}

void MsvTeleop::printQuery ()
{
	for (int i = 0; i < master.request.length; i++) {
		printf("%X ", master.request.frame[i]);
	}
	printf("\n");
}

void MsvTeleop::close ()
{
	bpt.closePort();
}

