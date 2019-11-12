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
/// @file ROS header for the dxl class of the MSV-01 rescue robot.
/// @author Victor Esteban Sandoval-Luna
///
/////////////////////////////////////////////////////////////////////////////////////////

#ifndef MSV_DXL_H_
#define MSV_DXL_H_

// Serial communication definitions
#define OCMD_START                0x10
#define OCMD_SETUP                0x20
#define OCMD_INIT                 0x30
#define OCMD_READY                0x31

#define DXL_PORT_READY            0x70
#define DXL_BAUDRATE_READY        0x71

#define DXL_PORT_ERROR            0x80
#define DXL_BAUDRATE_ERROR        0x81

// Dynamixel PacketHandler error definitions
#define PH_SUCCESS                0x90  // Tx/Rx packet communication success
#define PH_PORT_BUSY              0x91  // Port is busy (in use)
#define PH_TX_FAIL                0x92  // Failed transmit instruction packet
#define PH_RX_FAIL                0x93  // Failed get status packet
#define PH_TX_ERROR               0x94  // Incorrect instruction packet
#define PH_RX_WAITING             0x95  // Now recieving status packet
#define PH_RX_TIMEOUT             0x96  // There is no status packet
#define PH_RX_CORRUPT             0x97  // Incorrect status packet
#define PH_NOT_AVAILABLE          0x98  // Port not available

#define DXL_SUCCESS               0xA0
#define DXL_VOLTAGE               0xA1
#define DXL_ANGLE                 0xA2
#define DXL_OVERHEAT              0xA3
#define DXL_RANGE                 0xA4
#define DXL_CHECKSUM              0xA5
#define DXL_OVERLOAD              0xA6
#define DXL_INSTRUCTION           0xA7

#include <ros/ros.h>
#include <msv_main/port_handler.h>
#include <boost/asio.hpp>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <string.h>

class MsvDxl
{
	private:
		// Serial port
		msv::PortHandler ocmd;
		
		ros::NodeHandle n_dxl;
		ros::Publisher pub_angles;
		ros::Publisher pub_base;
		
		// For the remote control
		ros::Subscriber sub_mode;
		ros::Subscriber sub_joy;
		
		// Vector to store angles to be written (byte vales)
		std::vector<uint8_t> raw_angles = std::vector<uint8_t> (1);
		// Vector to store read raw data
		std::vector<uint8_t> raw_data = std::vector<uint8_t> (1);
		// Vector to store angles data
		std::vector<float> read_angles = std::vector<float> (1);
		
		// Messages to be published
		std_msgs::Float32MultiArray ocmd_angles;
		std_msgs::Float32 arm_base;
		
		int on;
		int x, y, up, down;
		int send;
		
		int verbosity;
		
		// Physical limits of moving range (MIN, MAX, MIN, MAX, ..)
		std::vector<uint16_t> angles_limits = std::vector<uint16_t> (1);
		
		void joyCallback (const sensor_msgs::Joy::ConstPtr& joy);
		void modeCallback (const std_msgs::String::ConstPtr& mode);
		
		int initOCMD ();
		int initAngles ();
		
		// Communication methods for the sensors port
		int sendreceivePacketOCMD (const int& verbose, const int& ack_length);
		// For debug purposes
		int sendPacketOCMD (const int& verbose);
		
		std::string errorPacketHandler(const int& code);
		std::string errorDxl(const int& code);
		
	public:
		MsvDxl (char* port, const int& baudrate, const int& verb);
		virtual ~MsvDxl () {}
		
		// Default AX-12 A range limits
		const int MIN_GOAL_POSITION = 0x0000; // 0 bits, 0 degrees
		const int MAX_GOAL_POSITION = 0x03FF; // 1023 bits, 300 degrees
		
		void close ();
};// End of class MsvDxl

#endif
