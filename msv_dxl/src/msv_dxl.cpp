/*******************************************************************************
* Copyright 2019 Robótica de la Mixteca
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/////////////////////////////////////////////////////////////////////////////////////////
/// @file ROS Node of the MSV-01 rescue robot RGBD orientation system using the ROBOTIS
/// Dynamixel Servos powered by the Dynamixel SDK.
/// @author Victor Esteban Sandoval-Luna
/////////////////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include "dxl_handler.h"
#include <unistd.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

using namespace msv;

class msv_dxl
{
  private:
    // Serial port
    DxlHandler dxl;

    ros::NodeHandle n_dxl;
    ros::Publisher pub_angles;
    ros::Publisher pub_base;

    // For the RGBD orientation system control
    ros::Subscriber sub_mode;
    ros::Subscriber sub_joy;

    int on;
    int x, y, up, down;

    void joyCallback (const sensor_msgs::Joy::ConstPtr& joy);
    void modeCallback (const std_msgs::String::ConstPtr& mode);

  public:
    msv_dxl (int verb ) : dxl("/dev/ttyUSB2") {
      ROS_INFO("SETTING DYNAMIXELS UP...");

      // DXL handler
      dxl.setBaudRate(460800);
      if (dxl.openPort())
        ROS_INFO("DXL SERIAL PORT OPEN");

      // Servos position topic
      pub_angles = n_dxl.advertise<std_msgs::Float32>("msv/RGBD_angles", 1);
      // Robotic arm base angle
      pub_base = n_dxl.advertise<std_msgs::UInt16>("msv/arm_base", 1000);

      // Servos controller topic
      sub_joy = n_dxl.subscribe("joy", 10, &msv_dxl::joyCallback, this);
      // MSV-01 robot mode topic
      sub_mode = n_dxl.subscribe("msv/mode", 1, &msv_dxl::modeCallback, this);

      // RGBD orientation system disabled
      on = 0;

      // For servos control
      x = 4;
      y = 5;
      // up and down equal to +z and -z, respectively
      up = 4;
      down = 6;

    }
};// End of class msv_dxl

// Controller topic callback function
void msv_dxl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if (on == 1) {
    geometry_msgs::Twist twist;

    int dx = -1*joy->axes[x];
    int dy = joy->axes[y];

    if (joy->buttons[up] == 1) {
      // Move servo until the button is released
      ROS_INFO("MOVING RGBD SENSOR (+z)");
      twist.linear.z = joy->buttons[up];
    }
    else if (joy->buttons[down] == 1) {
      // Move servo until the button is released
      ROS_INFO("MOVING RGBD SENSOR (-z)");
      twist.linear.z = joy->buttons[down];
    }

    if (dx == 1) {
      // Move servo until the button is released
      ROS_INFO("MOVING RGBD SENSOR (+x)");
    }
    else if (dx == -1) {
      // Move servo until the button is released
      ROS_INFO("MOVING RGBD SENSOR (-x)");
    }

    if (dy == 1) {
      // Move servo until the button is released
      ROS_INFO("MOVING RGBD SENSOR (+y)");
    }
    else if (dy == -1) {
      // Move servo until the button is released
      ROS_INFO("MOVING RGBD SENSOR (-y)");
    }

    twist.linear.x = x;
    twist.linear.y = y;

    //.publish(twist);
  }
}

// MSV-01 robot controller mode callback function
void msv_dxl::modeCallback(const std_msgs::String::ConstPtr& mode)
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

int main(int argc, char **argv)
{

  //Initiate ROS
  ros::init(argc, argv, "msv_dxl");

  //Create an object of class msv_dxl that will do the job
  msv_dxl *rgbd;
  msv_dxl d(1);
  rgbd = &d;

  ros::spin();

  return 0;
}
