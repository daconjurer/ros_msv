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
/// @file ROS msv_main node of the MSV-01 rescue robot for general control using the ROS
/// Joy messages. The node publishes the teleoperation commands, as well as the control
/// commands for the robotic arm and an orientation system for the RGBD sensor.
/// @author Victor Esteban Sandoval-Luna
///
/// Based on the "rescue" ROS metapackage from José Armando Sánchez-Rojas.
/// Based on the Modbus protocol "lightmodbus" library (RTU) from Jacek Wieczorek,
/// please see https://github.com/Jacajack/liblightmodbus.
/////////////////////////////////////////////////////////////////////////////////////////

/*
* PENDINGS
* arm
* rgbd
*/

#include <msv_main/msv_main.h>
#include <boost/asio.hpp>

int main (int argc, char **argv)
{
  //Initialize ROS
  ros::init(argc, argv, "msv_main");
  ros::NodeHandle priv_nh("~");
  
  int _verbosity;
  
  if (priv_nh.getParam("verbosity", _verbosity)) {
    ROS_INFO("Param _verbosity: %d", _verbosity);
  }
  else ROS_ERROR("Failed to get param: _verbosity");
  
  boost::asio::io_service io;
  
  try {
    //Create an object of class MsvMain that will do the job
    MsvMain *robot;
    MsvMain msv01(_verbosity);
    robot = &msv01;
    
    ros::Rate loop_rate(10);

    while (ros::ok()) {
      // Node callbacks handling
      ros::spinOnce();
      loop_rate.sleep();
    }
    
    return 0;
  } catch (boost::system::system_error ex) {
    ROS_ERROR("Error instantiating msv_main object. Error: %s", ex.what());
    return -1;
  }
}

