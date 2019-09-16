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
/// @file ROS Node of the MSV-01 rescue robot robotic arm using the Hovis HerkuleX Servos
/// and the ROBOTIS Dynamixel Servos (virtually) powered by the "herkulex_sdk" SDK and
/// the Dynamixel SDK, respectively.
/// @author Victor Esteban Sandoval-Luna
/////////////////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include "herkulex_sdk.h"
#include <unistd.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

using namespace herkulex;

class msv_arm: public ServoHerkulex
{
  private:
    ros::NodeHandle n_arm;
    ros::Publisher pub_angles;
    ros::Publisher pub_effector_vel;

    // Base joint virtual dynamixel
    ros::Subscriber sub_dxl;

    // For the robotic arm control
    ros::Subscriber sub_mode;
    ros::Subscriber sub_joy;

    int on;
    int y, x, z;
    int up, down;
    int power;

    void joyCallback (const sensor_msgs::Joy::ConstPtr& joy);
    void dxlCallback (const std_msgs::UInt16::ConstPtr& dxl);
    void modeCallback (const std_msgs::String::ConstPtr& mode);

    int constrainPower (int gpwr);

  public:
    msv_arm (int verb ): ServoHerkulex(verb) {
      ROS_INFO("SETTING ARM UP...");

      // Servos position topic
      pub_angles = n_arm.advertise<std_msgs::Float32>("msv/arm_angles", 1);
      // End-effector unitary speed direction along each axis
      pub_effector_vel = n_arm.advertise<geometry_msgs::Twist>("msv/effector_uvel", 10);

      // Servos controller topic
      sub_joy = n_arm.subscribe("joy", 10, &msv_arm::joyCallback, this);
      // Virtual dynamixel object topic (base joint)
      sub_dxl = n_arm.subscribe("msv/arm_base", 1, &msv_arm::dxlCallback, this);
      // MSV-01 robot mode topic
      sub_mode = n_arm.subscribe("msv/mode", 1, &msv_arm::modeCallback, this);

      // Arm disabled
      on = 0;

      // End-effector cartesian space positioning
      x = 4;
      y = 5;
      // up and down equal to +z and -z, respectively
      up = 4;
      down = 6;

      power = 1;

    }

    msv_arm (int verb, int pwr): ServoHerkulex(verb) {
      ROS_INFO("SETTING ARM UP...");

      // Servos position topic
      pub_angles = n_arm.advertise<std_msgs::Float32>("msv/arm_angles", 1);
      // End-effector unitary speed direction along each axis
      pub_effector_vel = n_arm.advertise<geometry_msgs::Twist>("msv/effector_uvel", 1000);

      // Servos controller topic
      sub_joy = n_arm.subscribe("joy", 10, &msv_arm::joyCallback, this);
      // Virtual dynamixel object topic (base joint)
      sub_dxl = n_arm.subscribe("msv/arm_base", 1, &msv_arm::dxlCallback, this);

      // End-effector cartesian space positioning
      x = 4;
      y = 5;
      // up and down equal to +z and -z, respectively
      up = 4;
      down = 6;

      power = constrainPower(pwr);

    }
};// End of class msv_arm

// Controller topic callback function
void msv_arm::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if (on == 1) {
    geometry_msgs::Twist twist;

    int dx = -1*joy->axes[x];
    int dy = joy->axes[y];

    if (joy->buttons[up] == 1) {
      ROS_INFO("MOVING ARM (+z)");
      twist.linear.z = joy->buttons[up];
    }
    else if (joy->buttons[down] == 1) {
      ROS_INFO("MOVING ARM (-z)");
      twist.linear.z = -1*joy->buttons[down];
    }

    if (dx == 1) {
      // Move servo until the button is released
      ROS_INFO("MOVING ARM (+x)");
    }
    else if (dx == -1) {
      // Move servo until the button is released
      ROS_INFO("MOVING ARM (-x)");
    }

    if (dy == 1) {
      // Move servo until the button is released
      ROS_INFO("MOVING ARM (+y)");
    }
    else if (dy == -1) {
      // Move servo until the button is released
      ROS_INFO("MOVING ARM (-y)");
    }

    twist.linear.x = power*x;
    twist.linear.y = power*y;

    pub_effector_vel.publish(twist);
  }
}

// Controller topic callback function
void msv_arm::dxlCallback(const std_msgs::UInt16::ConstPtr& dxl)
{
  ROS_INFO("GOT IT BRO [%f]", dxl->data);
  std_msgs::String output;
  output.data = "ok";

  pub_angles.publish(output);
}

// MSV-01 robot controller mode callback function
void msv_arm::modeCallback(const std_msgs::String::ConstPtr& mode)
{
  if (mode->data == "arm") {
    on = 1;
    ROS_INFO("ARM ENABLED");
  }
  if (mode->data == "robot_z") {
    ROS_INFO("ARM DISABLED");
    on = 0;
  }
}

int msv_arm::constrainPower (int gpwr) {
  if (gpwr > 1 || gpwr < 0)
    return 1;

  return gpwr;
}

int main(int argc, char **argv)
{

  //Initiate ROS
  ros::init(argc, argv, "msv_arm");

  //Create an object of class msv_arm that will do the job
  msv_arm *arm;
  msv_arm hklx(1);
  arm = &hklx;

  //arm->setPortLabel(port_label);
  //bool ctrl;

  //ctrl = arm->setLED(2,5);

  //ctrl = arm->torqueOn(5);

  //std::cout << "move 5 angle" << '\n';
  //bool c = arm->moveAngle0201(90,5,1,11.2);
  //std::cout << c << std::endl;

  ros::spin();

  return 0;
}
