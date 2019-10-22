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

MsvDxl::MsvDxl () 
{
  ROS_INFO("SETTING DYNAMIXELS UP...");

  // DXL port handler
  dxl_port.setPortName("/dev/ttyACM0");
  if (dxl_port.openPort()) {
    ROS_INFO("CONTROLLER OCMD SERIAL PORT OPEN. INITIALIZING OCMD NOW...");
  }
  else ROS_INFO("SENSORS DEBUG MODE");

  ocmd_angles.layout.dim.push_back(std_msgs::MultiArrayDimension());
  ocmd_angles.layout.dim[0].size = 3;
  ocmd_angles.layout.dim[0].stride = 1;
  ocmd_angles.layout.dim[0].label = "rgbd_angles";

  // Servos position topic
  pub_angles = n_dxl.advertise<std_msgs::Float32MultiArray>("msv/rgbd_angles", 1);
  // Robotic arm base angle
  pub_base = n_dxl.advertise<std_msgs::Float32>("msv/arm_base", 1000);

  // Servos controller topic
  sub_joy = n_dxl.subscribe("joy", 10, &MsvDxl::joyCallback, this);
  // MSV-01 robot mode topic
  sub_mode = n_dxl.subscribe("msv/mode", 1, &MsvDxl::modeCallback, this);

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
  if (on == 1) {

    int dx = -1*joy->axes[x];
    int dy = joy->axes[y];

    if (joy->buttons[up] == 1) {
      // Move servo until the button is released
      ROS_INFO("MOVING RGBD SENSOR (+z)");
      send = 1;
    }
    else if (joy->buttons[down] == 1) {
      // Move servo until the button is released
      ROS_INFO("MOVING RGBD SENSOR (-z)");
      send = 1;
    }

    if (dx == 1) {
      // Move servo until the button is released
      ROS_INFO("MOVING RGBD SENSOR (+x)");
      send = 1;
    }
    else if (dx == -1) {
      // Move servo until the button is released
      ROS_INFO("MOVING RGBD SENSOR (-x)");
      send = 1;
    }

    if (dy == 1) {
      // Move servo until the button is released
      ROS_INFO("MOVING RGBD SENSOR (+y)");
      send = 1;
    }
    else if (dy == -1) {
      // Move servo until the button is released
      ROS_INFO("MOVING RGBD SENSOR (-y)");
      send = 1;
    }
    
    if (send == 1) {
      // Frame publishing
      ocmd_angles.data.clear();
      for (int i = 0; i < 3; i++) {
        ocmd_angles.data.push_back(i+10.3);
      }
      pub_angles.publish(ocmd_angles);
    }
  }
}

// MSV-01 robot controller mode callback function
void MsvDxl::modeCallback (const std_msgs::String::ConstPtr& mode)
{
  if (mode->data == "rgbd") {
    on = 1;
    ROS_INFO("RGBD SYSTEM ENABLED");
  }
  if (mode->data == "arm") {
    ROS_INFO("RGBD SYSTEM DISABLED");
    on = 0;
  }
}

