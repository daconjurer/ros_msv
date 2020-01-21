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
/// @file ROS msv_arm package main application
///
/// @author Victor Esteban Sandoval-Luna
///
/////////////////////////////////////////////////////////////////////////////////////////

#define PORT_NAME								"/dev/ttyUSB0"
#define BAUDRATE								115200
#define VERBOSITY								0
#define SPEED										0.5

#include <msv_arm/msv_arm.h>
#include <boost/asio.hpp>

int main (int argc, char **argv)
{
	//Initiate ROS
	ros::init(argc, argv, "msv_arm");
	ros::NodeHandle priv_nh("~");
	
	int baudrate_;
	std::string port_name_;
	int verbosity_;
	float speed_;
	
	priv_nh.param("port", port_name_, std::string(PORT_NAME));
	priv_nh.param("baudrate", baudrate_, BAUDRATE);
	priv_nh.param("verbosity", verbosity_, VERBOSITY);
	priv_nh.param("speed", speed_, (float)SPEED);
	
	try {
		//Create an object of class MsvArm that will do the job
		MsvArm *arm;
		MsvArm msv01_arm((char*)(port_name_.c_str()),baudrate_,verbosity_,speed_);
		arm = &msv01_arm;
		
		ros::Rate loop_rate(5);
		
		while (ros::ok()) {
			ros::spinOnce();
			loop_rate.sleep();
		}
		arm->close();
		return 0;
	} catch (boost::system::system_error ex) {
		ROS_ERROR("Error instantiating msv_dxl object. Error: %s", ex.what());
		return -1;
	}
	return 0;
}

