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
/// @file ROS header for the teleop class of the MSV-01 rescue robot.
/// @author Victor Esteban Sandoval-Luna
///
/// Based on the Modbus protocol "lightmodbus" library (RTU) from Jacek Wieczorek,
/// please see https://github.com/Jacajack/liblightmodbus.
///
/// Based on the "rescue" ROS metapackage from José Armando Sánchez-Rojas.
/////////////////////////////////////////////////////////////////////////////////////////

#ifndef MSV_TELEOP_H_
#define MSV_TELEOP_H_

#include <ros/ros.h>
#include <msv_main/port_handler.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <time.h>
#include <inttypes.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt8MultiArray.h>
#include <msv_msgs/Actuators.h>

#include <lightmodbus/lightmodbus.h>
#include <lightmodbus/master.h>
#include <lightmodbus/slave.h>

class MsvTeleop
{
	private:
		// lightmodbus Master configuration struct
		ModbusMaster master;
		// For Master Exit code
		uint8_t mec;
		
		// Modbus coils and actuators data to be read
		std::vector<uint8_t> coils = std::vector<uint8_t> (1);
		std::vector<float> actuators = std::vector<float> (6);
		
		// Arrays to be published
		std_msgs::UInt8MultiArray bpt_read_iregs;
		std_msgs::UInt8MultiArray bpt_read_coils;
		
		// Messages to be published
		std_msgs::UInt8MultiArray bps_iregs;
		std_msgs::UInt8MultiArray bps_coils;
		msv_msgs::Actuators actuators_msg;
		
		// Serial port handler
		msv::PortHandler bpt;
		
		// Interface actuators nodes
		ros::NodeHandle n_teleop;
		ros::Subscriber sub_coils;
		ros::Subscriber sub_regs;
		ros::Publisher pub_actuators;
		
		// Arrays for decodifying
		std::vector<uint8_t> iregs_query = std::vector<uint8_t> (17);
		std::vector<uint8_t> coils_query = std::vector<uint8_t> (10);
		
		int verbosity;
		
		std::vector<uint8_t> ack_modbus = std::vector<uint8_t> (1);
		
		void coilsCallback (const std_msgs::UInt8MultiArray::ConstPtr& buffer);
		void regsCallback (const std_msgs::UInt8MultiArray::ConstPtr& buffer);
		
		// Communication methods for the controller port
		int sendreceivePacketBPT (const int& verbose, const int& ack_length, std::vector<uint8_t>& buffer);
		int sendreceivePacketBPT (const int& verbose, const int& ack_length);
		
		// For debug purposes
		int sendPacketBPT (const int& verbose, std::vector<uint8_t>& buffer);
		int sendPacketBPT (const int& verbose);
		
	public:
		MsvTeleop (const int& verb);
		virtual ~MsvTeleop () {}
		
};// End of class MsvTeleop

#endif

