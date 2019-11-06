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
/// @file ROS msv_electric class of the MSV-01 rescue robot for electric sensing
/// (current and voltage) through serial port. Based on the Modbus protocol
/// "lightmodbus" library (RTU) for microcontoller instruction. For further details
/// about the electric variables sensed, please refer to the technical paper as well
/// as the Memory table in the metapackage folder.
///
/// @author Victor Esteban Sandoval-Luna
///
/// Based on the Modbus protocol "lightmodbus" library (RTU) from Jacek Wieczorek,
/// please see https://github.com/Jacajack/liblightmodbus.
/////////////////////////////////////////////////////////////////////////////////////////

#include <msv_electric/msv_electric.h>

/* PENDING: */
// Read the keyboard to turn LEDs on and off

MsvElectric::MsvElectric (char* const port, const int& baudrate, const int& verb) : bps(port,baudrate) 
{
	ROS_INFO("SETTING ELECTRIC NODE UP...");
	
	// Joy controller topic (for LEDs control through functions buttons)
	sub_joy = n_electric.subscribe("joy", 1, &MsvElectric::joyCallback, this);
	
	// Modbus sensing system coils topic (published all the time)
	pub_coils = n_electric.advertise<std_msgs::UInt8MultiArray>("msv/ss_coils", 10);
	// Modbus sensing system holding regs topic (published all the time)
	pub_regs = n_electric.advertise<std_msgs::UInt8MultiArray>("msv/ss_regs", 10);
	// MSV_01 sensors data topic (published all the time)
	pub_sensors = n_electric.advertise<msv_msgs::Electric>("msv/sensors", 10);
	
	// Port handler
	if (bps.openPort()) {
		ROS_INFO("CONTROLLER BPS SERIAL PORT OPEN. INITIALIZING BPS NOW...");
	} else ROS_INFO("SENSORS DEBUG MODE");
	
	// liblightmodbus Master struct init
	mec = modbusMasterInit(&master);
	if (mec != MODBUS_OK)
		ROS_INFO("MODBUS MASTER COULD NOT BE INITIALIZED");
	else ROS_INFO("MODBUS MASTER INITIALIZED");
	
	// Verbosity attribute
	verbosity = verb;
	
	// All the LEDs off
	coils[0] = 0x00;
	
	// Multiarrays set up
	bps_read_iregs.layout.dim.push_back(std_msgs::MultiArrayDimension());
	bps_read_iregs.layout.dim[0].size = 8;
	bps_read_iregs.layout.dim[0].stride = 1;
	bps_read_iregs.layout.dim[0].label = "ss_read_iregs";
	
	bps_forced_coils.layout.dim.push_back(std_msgs::MultiArrayDimension());
	bps_forced_coils.layout.dim[0].size = 10;
	bps_forced_coils.layout.dim[0].stride = 1;
	bps_forced_coils.layout.dim[0].label = "ss_forced_coils";
	
	// Sensors data array and Electric message set up
	for (int i = 0; i < 11; i++) {
		sensors[i] = 0.0;
	}
	sensors_msg.header.frame_id = "msv_sensors";
	sensors_msg.values.reserve(11);
}

// Joy callback method
void MsvElectric::joyCallback (const sensor_msgs::Joy::ConstPtr& joy) 
{
	int pub_flag = 0;
	int triangle, circle, cross, square;
	uint8_t start = coils[0];
	
	triangle = joy->buttons[TRIANGLE_BUTTON];
	circle = joy->buttons[CIRCLE_BUTTON];
	cross = joy->buttons[CROSS_BUTTON];
	square = joy->buttons[SQUARE_BUTTON];
	
	if (triangle) {
		pub_flag = 1;
		if (start & 0x01) {
			start &= ~(0x01);
			ROS_INFO("LED 1 OFF");
		} else {
			start |= 0x01;
			ROS_INFO("LED 1 ON");
		}
	}
	
	if (circle) {
		pub_flag = 1;
		if (start & 0x02) {
			start &= ~(0x02);
			ROS_INFO("LED 2 OFF");
		} else {
			start |= 0x02;
			ROS_INFO("LED 2 ON");
		}
	}
	
	if (cross) {
		pub_flag = 1;
		if (start & 0x04) {
			start &= ~(0x04);
			ROS_INFO("LED 3 OFF");
		} else {
			start |= 0x04;
			ROS_INFO("LED 3 ON");
		}
	}
	
	if (square) {
		pub_flag = 1;
		if (start & 0x08) {
			start &= ~(0x08);
			ROS_INFO("LED 4 OFF");
		} else {
			start |= 0x08;
			ROS_INFO("LED 4 ON");
		}
	}
	
	/*
	// Debug
	if (pub_flag) {
		ROS_INFO("%X", start);
	}
	*/
	
	coils[0] = start;
}

void MsvElectric::senseMSV ()
{
	if (verbosity == 1) {
		/* Coils request frame building */
		modbusBuildRequest15(&master,2,7,4,coils.data());
		//ROS_INFO("BPS COILS QUERY:");
		//printQuery();
		
		// Send request
		//sendPacketBPS(1);
		sendreceivePacketBPS(1,8);
		
		// Frame publishing
		bps_forced_coils.data.clear();
		for (int i = 0; i < master.request.length; i++) {
			bps_forced_coils.data.push_back(master.request.frame[i]);
		}
		pub_coils.publish(bps_forced_coils);
		
		/* Input registers request frame building */
		modbusBuildRequest03(&master,2,0,11);
		
		// Send request
		//sendPacketBPS(1);
		sendreceivePacketBPS(1,27);
	} else {
		/* Coils frame request frame building */
		modbusBuildRequest15(&master,2,7,4,coils.data());
		
		// Send request
		//sendPacketBPS(0);
		sendreceivePacketBPS(0,8);
		
		// Frame publishing
		bps_forced_coils.data.clear();
		for (int i = 0; i < master.request.length; i++) {
			bps_forced_coils.data.push_back(master.request.frame[i]);
		}
		pub_coils.publish(bps_forced_coils);
		
		/* Input registers request frame building */
		modbusBuildRequest03(&master,2,0,11);
		
		// Send request
		//sendPacketBPS(0);
		sendreceivePacketBPS(0,27);
	}
	
	// Frames publishing
	bps_read_iregs.data.clear();
	for (int i = 0; i < master.request.length; i++) {
		bps_read_iregs.data.push_back(master.request.frame[i]);
	}
	pub_regs.publish(bps_read_iregs);
	
	// Sensors data decoding and publishing
	for (int i = 0; i < master.data.count; i++) {
		sensors[i] = master.data.regs[i];
	}
	
	sensors_msg.header.stamp = ros::Time::now();
	sensors_msg.values = sensors;
	pub_sensors.publish(sensors_msg);
}

int MsvElectric::sendreceivePacketBPS (const int& verbose, const int& ack_length) 
{
	uint8_t rl = master.request.length + 1;
	ack_modbus.resize(ack_length);
	int n, k, i;
	
	uint8_t* buf = master.request.frame;
	*(buf+rl-1) = 0xFF;
	
	// Send query
	bps.clearPort();
	k = bps.writePort(buf, rl);
	usleep ((rl + ack_length) * 12);
	
	// Receive response
	for (i = 0; i < ack_length; i++) {
		n = bps.readPort(&ack_modbus[i]);
	}
	
	if (k != rl) {
		return -1;
	}
	
	master.response.length = ack_length;
	master.response.frame = ack_modbus.data();
	
	// Parse response using lightmodbus
	mec = modbusParseResponse(&master);
	if (mec != MODBUS_OK) {
		ROS_ERROR("MODBUS RESPONSE IS NOT CORRECT");
		return -2;
	}
	
	if (verbose) {
		for (int j = 0; j < ack_length; j++) {
			printf("%X ", ack_modbus[j]);
		}
		printf("\n");
	}
	
	return n;
}

int MsvElectric::sendPacketBPS (const int& verbose) 
{
	int k = 0;
	uint8_t rl = master.request.length + 1;
	
	uint8_t* buf = master.request.frame;
	*(buf+rl-1) = 0xFF;
	
	bps.clearPort();
	k = bps.writePort(buf,rl);
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

void MsvElectric::printQuery () 
{
	for (int i = 0; i < master.request.length; i++) {
		printf("%X ", master.request.frame[i]);
	}
	printf("\n");
}

void MsvElectric::close () 
{
	bps.closePort();
}

