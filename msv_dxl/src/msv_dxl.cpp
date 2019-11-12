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

#include <msv_dxl/msv_dxl.h>

#define DXL_PITCH             12 // Dynamixel ID: 12
#define DXL_YAW               14 // Dynamixel ID: 14
#define DXL_ROLL              10 // Dynamixel ID: 10
#define DXL_ARM               11 // Dynamixel ID: 11

#define NUM_SERVOS            4  // Number of servos (must be at least 3, ideally)
# if !defined(NUM_SERVOS)
	#define NUM_SERVOS          3
#endif
#define RGBD_SERVOS           3  // Number of servos for RGBD system

#define OCMD_INIT_LENGTH      4

#ifdef NUM_SERVOS
	#define OCMD_DATA_LENGTH    (2*NUM_SERVOS + 2)	// OCMD_DATA_LENGTH = (no. of servos*2)+2
	#define OCMD_SETUP_LENGTH   OCMD_DATA_LENGTH
#endif

MsvDxl::MsvDxl (char* const port, const int& baudrate, const int& verb) : ocmd(port,baudrate) 
{
	ROS_INFO("SETTING DYNAMIXELS UP...");
	
	// DXL port handler
	if (ocmd.openPort()) {
		ROS_INFO("CONTROLLER OCMD SERIAL PORT OPEN");
		ROS_INFO("INITIALIZING OCMD NOW...");
		if (initOCMD() == -1) {
			ROS_ERROR ("OCMD INIT ROUTINE FAILED");
		}
	}
	else ROS_INFO("OCMD DEBUG MODE");
	
	// Servos position topic
	pub_angles = n_dxl.advertise<std_msgs::Float32MultiArray>("msv/rgbd_angles", 1);
	// Robotic arm base angle
	pub_base = n_dxl.advertise<std_msgs::Float32>("msv/arm_base", 1000);
	// Servos controller topic
	sub_joy = n_dxl.subscribe("joy", 10, &MsvDxl::joyCallback, this);
	// MSV-01 robot mode topic
	sub_mode = n_dxl.subscribe("msv/mode", 1, &MsvDxl::modeCallback, this);
	
	// Set up
	ocmd_angles.layout.dim.push_back(std_msgs::MultiArrayDimension());
	ocmd_angles.layout.dim[0].size = RGBD_SERVOS;
	ocmd_angles.layout.dim[0].stride = 1;
	ocmd_angles.layout.dim[0].label = "rgbd_angles";
	
	raw_angles.resize(OCMD_DATA_LENGTH);
	raw_data.resize(OCMD_DATA_LENGTH);
	read_angles.resize(NUM_SERVOS);
	
	// Init
	raw_angles[0] = 0xFF; raw_angles[OCMD_DATA_LENGTH-1] = 0xFF;
	arm_base.data = 0.0;
	
	// RGBD orientation system disabled
	on = 0;
	send = 0;
	
	// For servos control
	x = 4;
	y = 5;
	// up and down equal to +z and -z, respectively
	up = 4;
	down = 6;
}

// Controller topic callback function
void MsvDxl::joyCallback (const sensor_msgs::Joy::ConstPtr& joy)
{
	send = 0;
	// X axis can be controlled from both ARM and RGBD modes
	if (on != 0) {
		int dx = -1*joy->axes[x];
		
		if (dx == 1) {
			// Move servo until the button is released
			if (on == 1) {
				ROS_INFO("MOVING RGBD SENSOR (+x)");
			} else ROS_INFO("MOVING ARM BASE (+x)");
			
			send = 1;
		}
		else if (dx == -1) {
			// Move servo until the button is released
			if (on == 1) {
				ROS_INFO("MOVING RGBD SENSOR (-x)");
			} else ROS_INFO("MOVING ARM BASE (-x)");
			send = 1;
		}
	}
	
	if (on == 1) {
		int dy = joy->axes[y];
		
		if (joy->buttons[up] == 1) {
			// Move servo
			ROS_INFO("MOVING RGBD SENSOR (+z)");
			send = 1;
		}
		else if (joy->buttons[down] == 1) {
			// Move servo
			ROS_INFO("MOVING RGBD SENSOR (-z)");
			send = 1;
		}
		
		if (dy == 1) {
			// Move servo
			ROS_INFO("MOVING RGBD SENSOR (+y)");
			send = 1;
		}
		else if (dy == -1) {
			// Move servo
			ROS_INFO("MOVING RGBD SENSOR (-y)");
			send = 1;
		}
	}
	
	if (send == 1) {
		int sent = 0;
		if (verbosity == 1) {
			//sendPacketOCMD(1);
			sent = sendreceivePacketOCMD(1,OCMD_DATA_LENGTH);
		} else {
			//sendPacketOCMD(0);
			sent = sendreceivePacketOCMD(0,OCMD_DATA_LENGTH);
		}
		
		if (sent == -1) {//OCMD_DATA_LENGTH) {
			int j = 1;
			uint16_t temp = 0;
			for (int i = 0; i < NUM_SERVOS; i++) { 
				temp = (uint16_t)(((raw_data[j]) << 8) | (0x00FF & ((uint16_t)raw_data[j+1])));
				read_angles[i] = (float)(temp + 0.0);
				j += 2;
			}
			
			// Publish the data from ACK frame (angles)
			ocmd_angles.data.clear();
			for (int i = 0; i < RGBD_SERVOS; i++) {
				ocmd_angles.data.push_back(read_angles[i+1]);
			}
			arm_base.data = read_angles[0];
			
			pub_angles.publish(ocmd_angles);
			pub_base.publish(arm_base);
		}
	}
}

// MSV-01 robot controller mode callback function
void MsvDxl::modeCallback (const std_msgs::String::ConstPtr& mode) 
{
	// Communication disabled by default
	on = 0;
	
	if (mode->data == "rgbd") {
		on = 1;
		ROS_INFO("RGBD SYSTEM ENABLED");
	}
	if (mode->data == "arm") {
		ROS_INFO("RGBD SYSTEM DISABLED");
		on = 2;
	}
}

int MsvDxl::initOCMD () 
{
	int k, n, ack_counter = 0;
	uint8_t init[1] = {OCMD_START};
	uint8_t setup[1] = {OCMD_SETUP};
	uint8_t init_buffer[OCMD_INIT_LENGTH] = {0};
	uint8_t setup_buffer[OCMD_SETUP_LENGTH] = {0};
	uint8_t error_code = 0;
	
	// Send the INIT query
	ocmd.clearPort();
	k = ocmd.writePort(init,1);
	usleep ((1 + OCMD_INIT_LENGTH) * 10);
	
	// Receive ACK
	for (int i = 0; i < OCMD_INIT_LENGTH; i++) { 
		n = ocmd.readPort(&init_buffer[i]);
		if (n != 1) {
			break;
		} else ack_counter++;
	}
	
	if (k != 1) {
		ROS_ERROR("Failed to send the INIT query to OCMD.");
		return -1;
	}
	
	// Decode ACK
	if (ack_counter != OCMD_INIT_LENGTH) {
		ROS_ERROR("OCMD Init ACK is corrupted.");
		return -1;
	}
	
	if (init_buffer[0] != OCMD_INIT) {
		ROS_ERROR("OCMD is not connected.");
		return -1;
	} else ROS_INFO("OCMD is connected.");
	
	if (init_buffer[1] != DXL_PORT_READY) {
		ROS_ERROR("Failed to set serial port.");
		return -1;
	} ROS_INFO("Serial port succesfully set.");
	
	if (init_buffer[2] != DXL_BAUDRATE_READY) {
		ROS_ERROR("DXL Baudrate setting failed.");
		return -1;
	} else ROS_INFO("DXL Baudrate set succesfully.");
	
	if (init_buffer[3] != OCMD_READY) {
		ROS_ERROR("OCMD could not start.");
		return -1;
	} else ROS_INFO("OCMD is now ready.");
	
	// Send the SETUP query
	ack_counter = 0;
	ocmd.clearPort();
	k = ocmd.writePort(setup,1);
	usleep ((1 + OCMD_SETUP_LENGTH) * 10);
	
	// Read servos set up ACK
	for (int i = 0; i < OCMD_SETUP_LENGTH; i++) { 
		n = ocmd.readPort(&setup_buffer[i]);
		if (n != 1) {
			break;
		} else ack_counter++;
	}
	
	if (k != 1) {
		ROS_ERROR("Failed to send the SETUP query to OCMD.");
		return -1;
	}
	
	// Decode ACK
	if (ack_counter != OCMD_SETUP_LENGTH) {
		ROS_ERROR("OCMD Setup ACK is corrupted (length: %d).",ack_counter);
		return -1;
	}
	
	if (setup_buffer[0] != 0xFF) {
		ROS_ERROR("OCMD Setup ACK is corrupted (start of query: %X).", setup_buffer[0]);
		return -1;
	}
	
	if (setup_buffer[OCMD_SETUP_LENGTH-1] != 0xFF) {
		ROS_ERROR("OCMD Setup ACK is corrupted (end of query: %X).", setup_buffer[OCMD_SETUP_LENGTH-1]);
		return -1;
	}
	
	for (int i = 1; i < OCMD_SETUP_LENGTH-1; i++) {
		error_code = setup_buffer[i];
		if (i%2) {
			if (error_code != PH_RX_TIMEOUT) {//PH_SUCCESS) {
				ROS_INFO_STREAM(errorPacketHandler(error_code));
				return -1;
			}
		} else {
			// Even (0)
			if (error_code != DXL_SUCCESS) {
				ROS_INFO_STREAM(errorDxl(error_code));
				return -1;
			}
		}
	}
	
	ROS_INFO("Dynamixel servos now ready.");
	return 0;
}

int MsvDxl::initAngles ()
{
	//raw_angles
}

int MsvDxl::sendreceivePacketOCMD (const int& verbose, const int& ack_length) 
{
	int n, k, ack_count1 = 0, ack_count2 = 0, ack_count3 = 0;
	uint8_t write_ack_buffer[OCMD_DATA_LENGTH] = {0};
	uint8_t read_ack_buffer[OCMD_DATA_LENGTH] = {0};
	uint8_t error_code = 0;
	
	// Send the query to OCMD
	ocmd.clearPort();
	k = ocmd.writePort(raw_angles.data(),OCMD_DATA_LENGTH);
	usleep ((OCMD_DATA_LENGTH + ack_length) * 10);
	
	// Receive ACK
	for (int i = 0; i < ack_length; i++) { 
		n = ocmd.readPort(&write_ack_buffer[i]);
		if (n != 1) {
			break;
		} else ack_count1++;
	}
	
	for (int j = 0; j < ack_length; j++) { 
		n = ocmd.readPort(&read_ack_buffer[j]);
		if (n != 1) {
			break;
		} else ack_count2++;
	}
	
	for (int k = 0; k < ack_length; k++) { 
		n = ocmd.readPort(&raw_data[k]);
		if (n != 1) {
			break;
		} else ack_count3++;
	}
	
	if (k != OCMD_DATA_LENGTH) {
		ROS_ERROR("Failed to send the query to OCMD.");
		return -1;
	}
	
	/* Check for errors in the ACKs and data */
	
	// Check lengths
	if (ack_count1 != OCMD_DATA_LENGTH) {
		ROS_ERROR("OCMD Write ACK is corrupted (length: %d).", ack_count1);
		return -1;
	}
	
	if (ack_count2 != OCMD_DATA_LENGTH) {
		ROS_ERROR("OCMD Read ACK is corrupted (length: %d).", ack_count2);
		return -1;
	}
	
	if (ack_count3 != OCMD_DATA_LENGTH) {
		ROS_ERROR("OCMD data is corrupted (length: %d).", ack_count3);
		return -1;
	}
	
	// Check starts and ends of queries
	if (write_ack_buffer[0] != 0xFF) {
		ROS_ERROR("OCMD Write ACK is corrupted (start of query: %X).", write_ack_buffer[0]);
		return -1;
	}
	
	if (write_ack_buffer[OCMD_DATA_LENGTH-1] != 0xFF) {
		ROS_ERROR("OCMD Write ACK is corrupted (end of query: %X).", write_ack_buffer[OCMD_SETUP_LENGTH-1]);
		return -1;
	}
	
	if (read_ack_buffer[0] != 0xFF) {
		ROS_ERROR("OCMD Read ACK is corrupted (start of query: %X).", read_ack_buffer[0]);
		return -1;
	}
	
	if (read_ack_buffer[OCMD_DATA_LENGTH-1] != 0xFF) {
		ROS_ERROR("OCMD Read ACK is corrupted (end of query: %X).", read_ack_buffer[OCMD_SETUP_LENGTH-1]);
		return -1;
	}
	
	if (raw_data[0] != 0xFF) {
		ROS_ERROR("OCMD data is corrupted (start of query: %X).", raw_data[0]);
		return -1;
	}
	
	if (raw_data[OCMD_DATA_LENGTH-1] != 0xFF) {
		ROS_ERROR("OCMD data is corrupted (end of query: %X).", raw_data[OCMD_SETUP_LENGTH-1]);
		return -1;
	}
	
	// Check for errors in the Write ACK
	for (int i = 1; i < OCMD_DATA_LENGTH-1; i++) {
		error_code = read_ack_buffer[i];
		if (i%2) {
			// Odd
			if (error_code != PH_SUCCESS) {
				ROS_INFO_STREAM(errorPacketHandler(error_code));
				return -1;
			}
		} else {
			// Even
			if (error_code != DXL_SUCCESS) {
				ROS_INFO_STREAM(errorDxl(error_code));
				return -1;
			}
		}
	}
	
	// Check for errors in the Read ACK
	for (int i = 1; i < OCMD_DATA_LENGTH-1; i++) {
		error_code = write_ack_buffer[i];
		if (i%2) {
			// Odd
			if (error_code != PH_SUCCESS) {
				ROS_INFO_STREAM(errorPacketHandler(error_code));
				return -1;
			}
		} else {
			// Even
			if (error_code != DXL_SUCCESS) {
				ROS_INFO_STREAM(errorDxl(error_code));
				return -1;
			}
		}
	}
	
	if (verbose) {
		for (int j = 0; j < ack_length; j++) {
			printf("%X ", raw_data[j]);
		}
		printf("\n");
	}
	
	return ack_count3;
}

int MsvDxl::sendPacketOCMD (const int& verbose) 
{
	int k = 0;
	
	// Send the query to OCMD
	ocmd.clearPort();
	k = ocmd.writePort(raw_angles.data(),OCMD_DATA_LENGTH);
	usleep (OCMD_DATA_LENGTH * 10);
	
	if (verbose) {
		for (int j = 0; j < OCMD_DATA_LENGTH; j++) {
			printf("%X ", raw_angles[j]);
		}
		printf("\n");
		}
	
	return k;
}

std::string MsvDxl::errorPacketHandler(const int& code) 
{
	std::string error("Communication Error: ");
	
	switch (code) {
		case PH_PORT_BUSY:
			error += "Port is busy (in use).";
			break;
		case PH_TX_FAIL:
			error += "Failed transmit instruction packet.";
			break;
		case PH_RX_FAIL:
			error += "Failed get status packet.";
			break;
		case PH_TX_ERROR:
			error += "Incorrect instruction packet.";
			break;
		case PH_RX_WAITING:
			error += "Now recieving status packet.";
			break;
		case PH_RX_TIMEOUT:
			error += "There is no status packet.";
			break;
		case PH_RX_CORRUPT:
			error += "Incorrect status packet.";
			break;
		case PH_NOT_AVAILABLE:
			error += "Port not available.";
			break;
		default:
			error = error + "Unknown error (" + std::to_string(code) + ").";
			break;
	}
	
	return error;
}

std::string MsvDxl::errorDxl (const int& code) 
{
	std::string error("DXL Error: ");
	
	switch (code) {
		case DXL_VOLTAGE:
			error += "Supplied voltage is out of the range.";
			break;
		case DXL_ANGLE:
			error += "Goal position is written out of the range.";
			break;
		case DXL_OVERHEAT:
			error += "Temperature is out of the range.";
			break;
		case DXL_RANGE:
			error += "Command (setting value) is out of the range for use.";
			break;
		case DXL_CHECKSUM:
			error += "Instruction packet checksum is incorrect.";
			break;
		case DXL_OVERLOAD:
			error += "Current load cannot be controlled by the set torque.";
			break;
		case DXL_INSTRUCTION:
			error += "Undefined instruction or delivering the action command without the reg_write command.";
			break;
		default:
			error = error + "Unknown error (" + std::to_string(code) + ").";
			break;
	}
	
	return error;
}

void MsvDxl::close () 
{
	ocmd.closePort();
}

