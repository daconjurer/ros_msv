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
/// @file ROS msv_imu node of the MSV-01 rescue robot for IMU MPU6050 data processing.
/// For easier display of the data, the InvenSense Teapot.format is used.The commands
/// This node instructs the ARDI board (Arduino NANO based) on data transmission through
/// serial port.
/// @author Victor Esteban Sandoval-Luna
///
/////////////////////////////////////////////////////////////////////////////////////////

// Serial device port name
#define PORT_NAME               "/dev/ttyUSB0"
#define BAUDRATE                115200

// DMP error definitions
#define INITMEMLOAD_ERROR       0x71
#define DMPCONFUPDATE_ERROR     0x72
#define OVERFLOW_ERROR          0x73

// Serial communication definitions
#define IMU_INIT                0x01
#define IMU_TEST                0x02
#define IMU_TEST_SUCC           0x50
#define IMU_TEST_FAIL           0x51

#define DMP_INIT                0x60
#define DMP_ENABLE              0x61
#define INT_ENABLE              0x62
#define DMP_READY               0x63

#define COM_START               0x0A
#define COM_STOP                0x0B
#define OBC_START               0x10
#define ARDI_READY              0x20
#define SEND_ARDI_DATA          0x30
#define EOQ                     0xFF

#include <ros/ros.h>
#include <msv_main/port_handler.h>
#include <boost/asio.hpp>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

int initIMU (msv::PortHandler& port);
void getIMUData (msv::PortHandler& port, std::vector<uint8_t>& data);

int main (int argc, char **argv)
{
	ros::init(argc, argv, "msv_imu");
	ros::NodeHandle n_imu;
	ros::NodeHandle priv_nh("~");
	
	int baudrate_;
	std::string port_name_;
	
	priv_nh.param("port", port_name_, std::string(PORT_NAME));
	priv_nh.param("baudrate", baudrate_, BAUDRATE);
	
	// Resume flag
	bool rf = false;
	
	// Arrays for data processing
	std::vector<uint8_t> data = std::vector<uint8_t> (28);
	std_msgs::UInt8MultiArray imu_data;
	
	// Multiarray set up
	imu_data.layout.dim.push_back(std_msgs::MultiArrayDimension());
	imu_data.layout.dim[0].size = 28;
	imu_data.layout.dim[0].stride = 1;
	imu_data.layout.dim[0].label = "imu_raw";
	
	ros::Publisher pub_imu = n_imu.advertise<std_msgs::UInt8MultiArray>("msv/imu_raw", 10);
	
	imu_data.data.clear();
	for (int i = 0; i < 28; i++) {
		imu_data.data.push_back(0);
	}
	
	try {
		// Serial port handler
		msv::PortHandler ph((char*)(port_name_.c_str()),baudrate_);
		//msv::PortHandler ph;
		
		ros::Rate loop_rate(10);
		
		ROS_INFO("SETTING IMU NODE UP...");
		
		// Port handler
		if (ph.openPortArduino()) {
			ROS_INFO("SERIAL PORT OPEN");
			ROS_INFO("WAITING FOR ARDI TO BOOT...");
			/* wait for the Arduino to reboot */
 			usleep(3500000);
			if (initIMU(ph) == -1) {
				return -1;
			}
		}
		else {
			ROS_ERROR("Port not available. The node will not start");
			while (ros::ok());
		}
		
		ROS_INFO("READING IMU");
		while (ros::ok()) {
			ros::spinOnce();
			loop_rate.sleep();
			
			getIMUData(ph,data);
				
			imu_data.data.clear();
			for (int i = 0; i < 28; i++) {
				imu_data.data.push_back(data[i]);
			}
			pub_imu.publish(imu_data);
		} ph.closePort();
	} catch (boost::system::system_error ex) {
		ROS_ERROR("Error setting up the msv_imu node. Error: %s", ex.what());
		return -1;
	}
}

int initIMU (msv::PortHandler& port) 
{
	int k, n, c;
	uint8_t init[1] = {0x10};
	uint8_t rb[4] = {0};
	
	// Send the INIT query
	port.clearPort();
	k = port.writePort(init,1);
	usleep ((1 + 4) * 10);
	
	if (k != 1) {
		ROS_ERROR("Failed to send the INIT query to ARDI.");
		return -1;
	}
	
	// Receive ACK
	n = port.readPort(rb,4);
	
	// Decode ACK
	if (n != 4) {
		ROS_ERROR("ARDI Init ACK is corrupted.");
		return -1;
	}
	
	if (rb[0] != ARDI_READY) {
		ROS_ERROR("ARDI is not connected.");
		return -1;
	} else ROS_INFO("ARDI is connected.");
	
	if (rb[1] != IMU_INIT) {
		ROS_ERROR("Failed to initialize IMU.");
		return -1;
	} ROS_INFO("IMU initialized.");
	
	if (rb[2] != IMU_TEST || rb[3] != IMU_TEST_SUCC) {
		ROS_ERROR("I2C connection test failed.");
		return -1;
	} else ROS_INFO("I2C connection tested succesfully.");
	
	init[0] = COM_START;// data[1] = COM_START; data[2] = EOQ;
	
	// Send the communication start query
	port.clearPort();
	k = port.writePort(init,1);
	usleep ((1 + 4) * 10);
	
	if (k != 1) {
		ROS_ERROR("Failed to send the COM_START query to ARDI.");
		return -1;
	}
	
	// Receive ACK
	n = port.readPort(rb,4);
	
	// Decode ACK
	if (n != 4) {
		ROS_ERROR("ARDI COM ACK is corrupted.");
		return -1;
	}
	
	if (rb[0] != DMP_INIT) {
		ROS_ERROR("Failed to initialize the DMP.");
		switch (rb[0]) {
			case INITMEMLOAD_ERROR:
				ROS_ERROR("Initial memory load failed.");
				return -1;
			case DMPCONFUPDATE_ERROR:
				ROS_ERROR("DMP configuration updates failed.");
				return -1;
			default:
				ROS_ERROR("Unknown IMU error.");
				return -1;
		}
	} else {
		ROS_INFO("DMP initialized.");
		
		if (rb[1] != DMP_ENABLE) {
			ROS_ERROR("DMP could not start.");
			return -1;
		} else ROS_INFO("DMP started.");
		
		if (rb[2] != INT_ENABLE) {
			ROS_ERROR("Arduino interrupt detection could not be enabled.");
			return -1;
		} else ROS_INFO("Arduino interrupt detection enabled.");
		
		if (rb[3] != DMP_READY) {
			ROS_ERROR("DMP will not work.");
			return -1;
		} else ROS_INFO("DMP is now ready.");
	}
	
	return 0;
}

void getIMUData (msv::PortHandler& port, std::vector<uint8_t>& data) 
{
	int k, n;
	uint8_t com[1] = {SEND_ARDI_DATA};
	
	// Send the INIT query
	port.clearPort();
	k = port.writePort(com,1);
	usleep ((1 + 28) * 10);
	
	if (k != 1) {
		ROS_ERROR("Failed to send the COM query to ARDI.");
		return;
	}
	
	// Receive ACK
	n = port.readPort(data.data(),28);
	
	// Check if ACK is corrupted
	if (n != 28) {
		ROS_ERROR("ARDI packet is corrupted.");
		return;
	}
}

