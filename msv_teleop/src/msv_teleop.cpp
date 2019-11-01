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

/* PENDINGS */
// Publish actuators
// Manage verbosity

#include <msv_teleop/msv_teleop.h>

MsvTeleop::MsvTeleop (const int& verb)
{
	ROS_INFO("SETTING TELEOP NODE UP...");
	
	// Main publisher topics
	sub_coils = n_teleop.subscribe("msv/ts_coils", 1, &MsvTeleop::coilsCallback, this);
	sub_regs = n_teleop.subscribe("msv/ts_regs", 1, &MsvTeleop::regsCallback, this);
	
	// Port handler
	if (bpt.openPort()) {
		ROS_INFO("CONTROLLER BPT SERIAL PORT OPEN. INITIALIZING BPT NOW...");
	}
	else ROS_INFO("CONTROLLER DEBUG MODE");
	
	// liblightmodbus Master struct init
	mec = modbusMasterInit(&master);
	if (mec != MODBUS_OK)
		ROS_INFO("MODBUS MASTER COULD NOT BE INITIALIZED");
	else ROS_INFO("MODBUS MASTER INITIALIZED");
	
	// Verbosity attribute
	verbosity = verb;
	
	// Actuators data array and Actuators message set up
	for (int i = 0; i < 6; i++) {
		actuators[i] = 0.0;
	}
	actuators_msg.header.frame_id = "msv_actuators";
	actuators_msg.values.reserve(6);
}

void MsvTeleop::coilsCallback (const std_msgs::UInt8MultiArray::ConstPtr& buffer) 
{
	int i = 0;
	// Decode the data of the Multiarray struct
	for(std::vector<uint8_t>::const_iterator it = buffer->data.begin(); it != buffer->data.end(); ++it)
	{
		coils_query[i] = *it;
		i++;
	}
	// Coils forcing frame
	//sendPacketBPT(1,coils_query);
	sendreceivePacketBPT(1,10,coils_query);
	
	// Coils reading frame
	modbusBuildRequest01(&master,0,5,2);		// addresses 6-7
	
	// Coils data decoding and publishing
	uint8_t temp = 0;
	for (int i = 0; i < master.data.count; i++) {
		temp = modbusMaskRead(master.data.coils,master.data.length,i);
	}
}

void MsvTeleop::regsCallback (const std_msgs::UInt8MultiArray::ConstPtr& buffer) 
{
	int i = 0;
	// Decode the data of the Multiarray struct
	for(std::vector<uint8_t>::const_iterator it = buffer->data.begin(); it != buffer->data.end(); ++it)
	{
		iregs_query[i] = *it;
		i++;
	}
	
	// Holding registers writting frame
	//sendPacketBPT(1,regs_query);
	sendreceivePacketBPT(1,17,iregs_query);
	
	// Input registers reading frame
	modbusBuildRequest03(&master,0,49,4);		// adresses 50 - 53
	sendreceivePacketBPT(1,8);
	
	// Input registers data decoding and publishing
	for (int i = 0; i < master.data.count; i++) {
		actuators[i] = master.data.regs[i];
	}
}

int MsvTeleop::sendreceivePacketBPT (const int& verbose, const int& ack_length, std::vector<uint8_t>& buffer) 
{
	int n, k;
	uint8_t rl = buffer.size() + 1;
	
	uint8_t* buf = buffer.data();
	*(buf+rl-1) = 0xFF;
	
	bpt.clearPort();
	k = bpt.writePort(buf,rl);
	usleep ((rl + ack_length) * 10);
	
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
	
	return n;
}

int MsvTeleop::sendreceivePacketBPT (const int& verbose, const int& ack_length) 
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

int MsvTeleop::sendPacketBPT (const int& verbose, std::vector<uint8_t>& buffer) 
{
	int k = 0;
	uint8_t rl = buffer.size() + 1;
	
	uint8_t* buf = buffer.data();
	*(buf+rl-1) = 0xFF;
	
	bpt.clearPort();
	k = bpt.writePort(buf,rl);
	usleep (rl * 10);
	
	if (verbose) {
		for (int j = 0; j < rl; j++) {
			printf("%X ", buffer[j]);
		}
		printf("\n");
	}
	
	return k;
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
	
	if (verbose) {
		for (int j = 0; j < rl; j++) {
			printf("%X ", master.request.frame[j]);
		}
		printf("\n");
	}
	
	return k;
}

