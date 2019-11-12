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
/// @file ROS msv_teleop node of the MSV-01 rescue robot for remote control using
/// commands from the ROS msv_main node. The commands are sent using the Modbus RTU
/// protocol through serial port (connected to the USART of the BPT board).
///
/// @author Victor Esteban Sandoval-Luna
///
/// Based on the "rescue" ROS metapackage from José Armando Sánchez-Rojas.
/////////////////////////////////////////////////////////////////////////////////////////

#define PORT_NAME               "/dev/ttyUSB0"
#define BAUDRATE                115200
#define VERBOSITY               0

#include <msv_teleop/msv_teleop.h>
#include <boost/asio.hpp>

int main (int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "msv_teleop");
	ros::NodeHandle priv_nh("~");
	
	int baudrate_;
	std::string port_name_;
	int verbosity_;
	
	priv_nh.param("port", port_name_, std::string(PORT_NAME));
	priv_nh.param("baudrate", baudrate_, BAUDRATE);
	priv_nh.param("verbosity", verbosity_, VERBOSITY);
	
	try {
		//Create an object of class MsvTeleop that will do the job
		MsvTeleop *teleop;
		MsvTeleop msv01_teleop((char*)(port_name_.c_str()),baudrate_,verbosity_);
		teleop = &msv01_teleop;
		
		ros::Rate loop_rate(10);
		
		while (ros::ok()) {
			ros::spinOnce();
			teleop->sense();
			loop_rate.sleep();
		}
		teleop->close();
		return 0;
	} catch (boost::system::system_error ex) {
		ROS_ERROR("Error instantiating msv_teleop object. Error: %s", ex.what());
		return -1;
	}
}
