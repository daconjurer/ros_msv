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
/// @file ROS main node of the MSV-01 rescue robot performing a teleoperated motion
/// controller using the ROS Joy messages as well as sensors reading tasks through
/// keyboard. Based on the Modbus protocol "lightmodbus" library (RTU) for
/// microcontoller instruction.
/// @author Victor Esteban Sandoval-Luna
///
/// Based on the "rescue" ROS metapackage from José Armando Sánchez-Rojas.
/////////////////////////////////////////////////////////////////////////////////////////

#include "ros/ros.h"
#include "port_handler.h"
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <time.h>
#include <inttypes.h>

#include <lightmodbus/lightmodbus.h>
#include <lightmodbus/master.h>
#include <lightmodbus/slave.h>

using namespace msv;

class msv_main
{
  private:
    // lightmodbus Master configuration struct
	  ModbusMaster master;

    // Modbus registers and coils
    std::vector<uint16_t> mhregs = std::vector<uint16_t> (4);
    std::vector<uint16_t> miregs = std::vector<uint16_t> (6);
    std::vector<uint16_t> siregs = std::vector<uint16_t> (12);
    std::vector<uint8_t> mcoils = std::vector<uint8_t> (1);   // 5 coils
    std::vector<uint8_t> lcoils = std::vector<uint8_t> (1);   // 4 coils

    // Modbus slave response frame
    std::vector<uint8_t> ack_modbus = std::vector<uint8_t> (1);

    // For Master Error checking
    //ModbusError merr;
	  // For Master Exit code
    uint8_t mec;

    // Serial ports
    PortHandler bpt, bps;
    // Send flag (valid button pressed)
    int send;
    // ACK check
    int ack;

    // MSV-01 robot main nodes
    ros::NodeHandle n_main;
    ros::Publisher pub_state;
    ros::Publisher pub_mode;
    ros::Publisher pub_power;
    ros::Publisher pub_vel;

    // Joy node subscriber
    ros::Subscriber sub_joy;

    // Interface actuators nodes
    ros::Publisher pub_actuators;
    ros::Publisher pub_leds;

    // Interface sensors nodes
    ros::Publisher pub_co2;
    ros::Publisher pub_sensors;

    // Verbosity and traction power attributes
    int verbosity;
    int power;

    // Buttons memories
    int l, a, f, s;
    int sel;

    void joyCallback (const sensor_msgs::Joy::ConstPtr& joy);
    
    void delay (int ms);

    int setVerb (int verb);

    // Communication methods for the controller and sensors ports, respectively.
    int sendreceivePacketBPT (int verbose, int ack_length);
    int sendreceivePacketBPS (int verbose, int ack_length);

    // For debug purposes
    int sendPacketBPT (int verbose);
    int sendPacketBPS (int verbose);

    void printACK ();

    // For locomotion control
    int linear, angular;
    int up, down;

    // For joystick use (0 for robot, 1 for arm)
    int mode;
    int turn_mode;  // For turn mode (0 for turning around z axis, 0 for smooth)

  public:
    // Sensing routine (continuous)
    void sensingBPS ();

    //msv_main (int verb) : bpt("/dev/ttyUSB1"), bps("/dev/ttyACM0") {
    msv_main (int verb) : bpt("/dev/ttyUSB1"), bps("/dev/ttyUSB2") {
      ROS_INFO("SETTING ROBOT UP...");

      // Port handler
      bpt.setBaudRate(460800);
      if (bpt.openPort())
        ROS_INFO("CONTROLLER SERIAL PORT OPEN");
      else ROS_INFO("CONTROLLER DEBUG MODE");

      bps.setBaudRate(460800);
      if (bps.openPort())
        ROS_INFO("SENSORS SERIAL PORT OPEN");
      else ROS_INFO("SENSORS DEBUG MODE");

      // liblightmodbus Master struct init
      mec = modbusMasterInit(&master);
      if (mec != MODBUS_OK)
        ROS_INFO("MODBUS MASTER COULD NOT BE INITIALIZED");
      else ROS_INFO("MODBUS MASTER INITIALIZED");

      // Joy controller topic
      sub_joy = n_main.subscribe("joy", 1, &msv_main::joyCallback, this);

      // Robot's state topic (published when the slave confirms wether the robot is moving or not)
      pub_state = n_main.advertise<std_msgs::String>("msv/state", 1);
      // Mode topic (published every time a new mode is selected [SELECT button])
      pub_mode = n_main.advertise<std_msgs::String>("msv/mode", 1);
      // Power topic (published every time the max power changes [START button])
      pub_power = n_main.advertise<std_msgs::UInt8>("msv/power", 1);
      // Twist topic (debug purposes, published every time the cmd_vel of the robot changes)
      pub_vel = n_main.advertise<geometry_msgs::Twist>("msv/cmd_vel", 10);

      // Actuators topic (published when the slave reports/confirms the state of the actuators)
      pub_actuators = n_main.advertise<std_msgs::UInt8>("msv/actuators", 1);

      // LEDs topic (published every time the slave reports/confirms the state of the LEDs)
      pub_leds = n_main.advertise<std_msgs::UInt8>("msv/leds", 1);

      // Power sensors topic (published every time the slave reports the state of the sensors)
      pub_sensors = n_main.advertise<std_msgs::UInt8>("msv/sensors", 1);

      // CO2 sensor topic (published every time the slave reports the state of the sensor)
      pub_co2 = n_main.advertise<std_msgs::UInt8>("msv/co2", 1);

      // Verbosity and Max traction power attributes
      verbosity = setVerb(verb);
      power = 90;

      sel = 0;
      l = 0;
      a = 0;
      f = 0;
      s = 0;

      send = 0;
      ack = 0;

      ROS_INFO("MAX POWER: 100%%. MIN POWER: 35%%.");
      // Joystick attributes
      linear = 1;
      angular = 2;
      up = 5;
      down = 7;

      mode = 0; // Robot mode
      turn_mode = 0;  // Turn around z axis
      
      // BPT and BPS init routines
      initBPT();
    	initPBS();

      ROS_INFO("SET UP DONE");
    }

};// End of class msv_main

void msv_main::joyCallback (const sensor_msgs::Joy::ConstPtr& joy) 
{
  int select, start;
  int send = 0;
  std_msgs::UInt8 p;

  select = joy->buttons[8];
  start = joy->buttons[9];

  // Not intended to change any external value
  if (select == 1) {
    std_msgs::String out;

    switch (sel) {
      case 0:
        // From Robot mode to Arm mode
        mode = 1;
        sel++;
        out.data = "arm";
        pub_mode.publish(out);
        ROS_INFO("ARM CONTROLLER MODE");
        break;
      case 1:
        // From Arm mode to Robot mode (with power control)
        mode = 0;
        sel++;
        turn_mode = 0;
        out.data = "robot_z";
        pub_mode.publish(out);
        ROS_INFO("CONTROLLING POWER");
        break;
      case 2:
        // Robot mode (power control disabled, turn around z axis)
        sel++;
        turn_mode = 0;
        ROS_INFO("ROBOT CONTROLLER MODE (Turn around z axis)");
        break;
      case 3:
        // Robot mode (power control disabled, turn around z axis)
        sel++;
        turn_mode = 1;
        out.data = "robot_s";
        pub_mode.publish(out);
        ROS_INFO("ROBOT CONTROLLER MODE (Smooth turn)");
        break;
      case 4:
        // From Robot mode to RGBD mode
        sel = 0;
        mode = 1;
        turn_mode = 0;
        out.data = "rgbd";
        pub_mode.publish(out);
        ROS_INFO("RGBD SYSTEM CONTROLLER MODE");
        break;
      default:
        break;
    }
    s = select;
    return;
  }

  if (start == 1) {
    switch (sel) {
      case 2:
        if (power == 255) {
          power = 90;
          p.data = 255;
          pub_power.publish(p);
          ROS_INFO("POWER: 34%%");
        } else {
          int tpwr;
          power += 55;
          tpwr = int(power*100.0/255.0);
          p.data = power;
          pub_power.publish(p);
          ROS_INFO("POWER: %d%%", tpwr);
        }
        break;
      default:
        break;
    }
    return;
  }

  // Robot Controller mode
  if (mode == 0 && s != 1) {
    geometry_msgs::Twist twist;
    int angular_z  = -power*joy->axes[angular];
    int linear_x = power*joy->axes[linear];

    mhregs[2] = 0x0000;
    mhregs[3] = 0x0000;

    // Forward/reverse
    if (linear_x > 0) {
      send = 1;
      // SV
      mhregs[0] = (uint16_t)linear_x;
      mhregs[1] = (uint16_t)linear_x;

      // F/R
      mcoils[0] = 0x11;    // 0001 0001
      ROS_INFO("Moving: forward");
    }
    else if (linear_x < 0) {
      send = 1;
      // SV
      mhregs[0] = (uint16_t)(-1*linear_x);
      mhregs[1] = (uint16_t)(-1*linear_x);

      // F/R
      mcoils[0] = 0x12;    // 0001 0010
      ROS_INFO("Moving: reverse");
    }
    else if (linear_x == 0 && l != 0) {
      send = 1;
      // SV
      mhregs[0] = 0x0000;
      mhregs[1] = 0x0000;

      // Stop
      mcoils[0] = 0x00;    // 0000 0000 (could be 0x03 as well)
      ROS_INFO("STOP");
    }

    // Turn routines
    else {
      // Turn around z axis (mode 0)
      if (angular_z > 0 && turn_mode == 0) {
        send = 1;
        // SV
        mhregs[0] = (uint16_t)angular_z;
        mhregs[1] = (uint16_t)angular_z;

        // F/R
        mcoils[0] = 0x13;    // 0001 0011
        ROS_INFO("Z axis turning: right");
      }
      else if (angular_z < 0 && turn_mode == 0) {
        send = 1;
        // SV
        mhregs[0] = (uint16_t)(-1*angular_z);
        mhregs[1] = (uint16_t)(-1*angular_z);

        // F/R
        mcoils[0] = 0x10;    // 0001 0000

        ROS_INFO("Z axis turning: left");
      }
      else if ((angular_z == 0 && turn_mode == 0) && a != 0) {
        send = 1;
        mcoils[0] = 0x00;
        // SV
        mhregs[0] = 0x0000;
        mhregs[1] = 0x0000;

        // Stop
        mcoils[0] = 0x00;    // 0000 0000 (could be 0x03 as well)
        ROS_INFO("STOP");
      }
      // Smooth turn (mode 1)
      else if (angular_z > 0 && turn_mode == 1) {
        send = 1;
        // SV
        mhregs[0] = (uint16_t)angular_z;
        mhregs[1] = (uint16_t)(angular_z/2);

        // F/R
        mcoils[0] = 0x11;    // 0001 0001
        ROS_INFO("Smooth turning: right");
      }
      else if (angular_z < 0 && turn_mode == 1) {
        send = 1;
        // SV
        mhregs[0] = (uint16_t)(-1*angular_z);
        mhregs[1] = (uint16_t)((-1/2)*angular_z);

        // F/R
        mcoils[0] = 0x12;    // 0001 0010
        ROS_INFO("Smooth turning: left");
      }
      else if ((angular_z == 0 && turn_mode == 1) && a != 0) {
        send = 1;
        // SV
        mhregs[0] = 0x0000;
        mhregs[1] = 0x0000;

        // Stop
        mcoils[0] = 0x00;    // 0000 0000 (could be 0x03 as well)
        ROS_INFO("STOP");
      }
    }

    if (joy->buttons[down] == 1) {
      f = joy->buttons[down];

      send = 1;
      // Flippers DOWN
      mhregs[2] = 0x00FF;
      mhregs[3] = 0x00FF;

      // Direction
      mcoils[0] = 0x14;    // 0001 0100
      ROS_INFO("Moving flippers: down");

      // Debug
      twist.linear.y = -f;
    }
    else if (joy->buttons[up] == 1) {
      f = joy->buttons[up];

      send = 1;
      // Flippers UP
      mhregs[2] = 0x00FF;
      mhregs[3] = 0x00FF;

      // Direction
      mcoils[0] = 0x18;    // 0001 1000
      ROS_INFO("Moving flippers: up");

      // Debug
      twist.linear.y = f;
    }
    else if ((joy->buttons[up] == 0 && joy->buttons[down] == 0) && f != 0) {
      f = 0;
      send = 1;
      // Flippers OFF
      mhregs[2] = 0x0000;
      mhregs[3] = 0x0000;

      // Stop
      mcoils[0] = 0x00;    // 0000 0000 (could be 0x0C as well)
      ROS_INFO("Flippers stopped");
    }

    l = linear_x;
    a = angular_z;

    // Serial communication routine
    if (send == 1) {
      // Debug
      twist.angular.z = angular_z;
      twist.linear.x = linear_x;
      pub_vel.publish(twist);

      if (verbosity == 0) {
        // Holding registers frame
        modbusBuildRequest16(&master,0,0,4,mhregs.data());
        sendPacketBPT(0);

        //ack = sendreceivePacketBPT(0,8);

        //if (ack != 8){}
          //ROS_INFO("Corrupt Modbus response (BPT).");

        // Coils frame
        modbusBuildRequest15(&master,0,0,5,mcoils.data());
        sendPacketBPT(0);

        //ack = sendreceivePacketBPT(0,8);

        //if (ack != 8){}
          //ROS_INFO("Corrupt Modbus response (BPT).");

      } else {
        // Holding registers frame
        modbusBuildRequest16(&master,0,0,4,mhregs.data());
        sendPacketBPT(1);

        //ack = sendreceivePacketBPT(1,8);

        //if (ack != 8){}
          //ROS_INFO("Corrupt Modbus response (BPT).");
        //else printACK();
        
        delay(7);

        // Coils frame
        modbusBuildRequest15(&master,0,0,5,mcoils.data());
        sendPacketBPT(1);

        //ack = sendreceivePacketBPT(1,8);

        //if (ack != 8){}
          //ROS_INFO("Corrupt Modbus response (BPT).");
        //else printACK();
      }
    }
  }
  s = select;
}

void msv_main::sensingBPS () 
{
  if (verbosity == 1) {
    ROS_INFO("MSV_MAIN SENSING NOW.");

    // Sensors input registers frame
    modbusBuildRequest04(&master,1,0,12);
    sendPacketBPS(1);

    // Actuators input registers frame
    modbusBuildRequest04(&master,1,12,6);
    sendPacketBPS(1);

    // Servos input registers frame
    modbusBuildRequest04(&master,1,99,3);
    sendPacketBPS(1);

    // Holding registers frame
    modbusBuildRequest03(&master,1,0,5);
    sendPacketBPS(1);

    // Motors & LEDs coils frame
    modbusBuildRequest01(&master,1,0,9);
    sendPacketBPS(1);

    // Servos coils frame
    modbusBuildRequest01(&master,1,99,3);
    sendPacketBPS(1);

  } else {
    // Sensors input registers frame
    modbusBuildRequest04(&master,1,0,12);
    sendPacketBPS(0);

    // Actuators input registers frame
    modbusBuildRequest04(&master,1,12,6);
    sendPacketBPS(0);

    // Servos input registers frame
    modbusBuildRequest04(&master,1,99,3);
    sendPacketBPS(0);

    // Holding registers frame
    modbusBuildRequest03(&master,1,0,5);
    sendPacketBPS(0);

    // Motors & LEDs coils frame
    modbusBuildRequest01(&master,1,0,9);
    sendPacketBPS(0);

    // Servos coils frame
    modbusBuildRequest01(&master,1,99,3);
    sendPacketBPS(0);
  }
}

void msv_main::delay (int ms) 
{
    clock_t start_time = clock(); 
    while (clock() < start_time + ms);
}

int msv_main::setVerb (int verb) 
{
  if (verb != 0 && verb != 1) {
    ROS_INFO("VERBOSITY: OFF");
    return 0;
  }

  ROS_INFO("VERBOSITY: ON");
  return verb;
}

int msv_main::sendreceivePacketBPT (int verbose, int ack_length) 
{
  uint8_t rl = master.request.length;
  ack_modbus.resize(ack_length);
  int k;

  bpt.clearPort();
  k = bpt.writePort(master.request.frame, rl);
  usleep ((rl + ack_length) * 10);

  int n = bpt.readPort(ack_modbus.data(), ack_length);

  if (k != rl) {
    return -1;
  }

  if (verbose) {
    for (int j = 0; j < rl; j++) {
      printf("%X ", master.request.frame[j]);
    }
    printf("\n");
  }

  return n;
}

int msv_main::sendreceivePacketBPS (int verbose, int ack_length) 
{
  uint8_t rl = master.request.length;
  ack_modbus.resize(ack_length);
  int k;

  bps.clearPort();
  k = bps.writePort(master.request.frame, rl);
  usleep ((rl + ack_length) * 10);

  int n = bps.readPort(ack_modbus.data(), ack_length);

  if (k != rl) {
    return -1;
  }

  if (verbose) {
    for (int j = 0; j < rl; j++) {
      printf("%X ", master.request.frame[j]);
    }
    printf("\n");
    std::cout << "Slave response:";
    for (int j = 0; j < ack_length; j++) {
      printf("%X ", ack_modbus[j]);
    }
    printf("\n");
  }

  return n;
}

int msv_main::sendPacketBPT (int verbose) 
{
  uint8_t rl = master.request.length;
  int k;

  bpt.clearPort();
  k = bpt.writePort(master.request.frame, rl);
  usleep (rl * 10);

  if (verbose) {
    for (int j = 0; j < rl; j++) {
      printf("%X ", master.request.frame[j]);
    }
    printf("\n");
  }

  return k;
}

int msv_main::sendPacketBPS (int verbose) 
{
  uint8_t rl = master.request.length;
  int k;

  bps.clearPort();
  k = bps.writePort(master.request.frame, rl);
  usleep (rl * 10);

  if (verbose) {
    for (int j = 0; j < rl; j++) {
      printf("%X ",master.request.frame[j]);
    }
    printf("\n");
  }

  return k;
}

void msv_main::printACK () {
  std::cout << "Slave response: ";
  for (int j = 0; j < ack_modbus.size(); j++) {
    printf("%X ", ack_modbus[j]);
  }
  printf("\n");
}

int main (int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "msv_main");

  //Create an object of class msv_main that will do the job
  msv_main *robot;
  msv_main msv01(1);
  robot = &msv01;
  ROS_INFO("ROBOT MSV-01 IS READY");

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    // Publishing routine
    // robot->sensingBPS();

    // Node callbacks handling
    ros::spinOnce();
    loop_rate.sleep();
  }

  // ros::spin();

  return 0;
}
