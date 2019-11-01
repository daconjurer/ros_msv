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
/// @file ROS header for the electric class of the MSV-01 rescue robot.
/// @author Victor Esteban Sandoval-Luna
///
/// Based on the Modbus protocol "lightmodbus" library (RTU) from Jacek Wieczorek,
/// please see https://github.com/Jacajack/liblightmodbus.
/////////////////////////////////////////////////////////////////////////////////////////

#ifndef MSV_ELECTRIC_H_
#define MSV_ELECTRIC_H_

#include <ros/ros.h>
#include <msv_main/port_handler.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <time.h>
#include <inttypes.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt8MultiArray.h>
#include <msv_msgs/Electric.h>

#include <lightmodbus/lightmodbus.h>
#include <lightmodbus/master.h>
#include <lightmodbus/slave.h>

class MsvElectric
{
	private:
		// lightmodbus Master configuration struct
		ModbusMaster master;
		// For Master Exit code
		uint8_t mec;
		
		// Modbus coils and sensors data to be read
		std::vector<uint8_t> coils = std::vector<uint8_t> (1);
		std::vector<float> sensors = std::vector<float> (11);
		
		// Messages to be published
		std_msgs::UInt8MultiArray bps_read_iregs;
		std_msgs::UInt8MultiArray bps_forced_coils;
		msv_msgs::Electric sensors_msg;
		
		// Serial port handler
		msv::PortHandler bps;
		
		// Interface sensors nodes
		ros::NodeHandle n_electric;
		ros::Publisher pub_coils;
		ros::Publisher pub_regs;
		ros::Publisher pub_sensors;
		
		int verbosity;
		
		std::vector<uint8_t> ack_modbus = std::vector<uint8_t> (1);
		
		// Communication methods for the sensors port
		int sendreceivePacketBPS (const int& verbose, const int& ack_length);
		
		// For debug purposes
		int sendPacketBPS (const int& verbose);
		
		// For Modbus/Serial debugging
		void printQuery ();
		
	public:
		MsvElectric (const int& verb);
		virtual ~MsvElectric () {}
		
		// Sensing routine
		void senseMSV ();
		
		// Close port method
	void close ();
	
};// End of class msv_teleop

#endif

