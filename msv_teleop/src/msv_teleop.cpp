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
/// @file ROS msv_teleop class of the MSV-01 rescue robot for remote control using
/// commands from the ROS msv_main node. The commands are sent using the Modbus RTU
/// protocol through serial port (connected to the USART of the BPT board).
///
/// @author Victor Esteban Sandoval-Luna
///
/// Based on the "rescue" ROS metapackage from José Armando Sánchez-Rojas.
/////////////////////////////////////////////////////////////////////////////////////////

/* PENDINGS */
// Publish actuators
// Manage verbosity

#include <msv_teleop/msv_teleop.h>

MsvTeleop::MsvTeleop (int verb)
{
  ROS_INFO("SETTING TELEOP NODE UP...");
  
  // Main publisher topics
  sub_coils = n_teleop.subscribe("msv/ts_coils", 1, &MsvTeleop::coilsCallback, this);
  sub_regs = n_teleop.subscribe("msv/ts_regs", 1, &MsvTeleop::regsCallback, this);
  
  // Port handler
    if (bpt.openPort()) {
      ROS_INFO("CONTROLLER BPT SERIAL PORT OPEN. INITIALIZING BPT NOW...");
    }
    else ROS_INFO("CONTROLLER DEBUG MODE");
}

void MsvTeleop::coilsCallback (const std_msgs::UInt8MultiArray::ConstPtr& buffer) 
{
  int i = 0;
  // Decode the data of the Multiarray struct
  for(std::vector<uint8_t>::const_iterator it = buffer->data.begin(); it != buffer->data.end(); ++it)
  {
    coils_query[i] = *it;
    i++;
  }
  //sendPacketBPT(1,coils_query);
  sendreceivePacketBPT(1,10,coils_query);
}

void MsvTeleop::regsCallback (const std_msgs::UInt8MultiArray::ConstPtr& buffer) 
{
  int i = 0;
  // Decode the data of the Multiarray struct
  for(std::vector<uint8_t>::const_iterator it = buffer->data.begin(); it != buffer->data.end(); ++it)
  {
    regs_query[i] = *it;
    i++;
  }
  //sendPacketBPT(1,regs_query);
  sendreceivePacketBPT(1,17,regs_query);
}

void MsvTeleop::delay (int ms) 
{
  clock_t start_time = clock(); 
  while (clock() < start_time + ms);
}

int MsvTeleop::sendreceivePacketBPT (int verbose, int ack_length, std::vector<uint8_t> buffer) 
{
  uint8_t rl = buffer.size();
  ack_modbus.resize(ack_length);
  int n, k, i;

  bpt.clearPort();
  k = bpt.writePort(buffer.data(), rl);
  usleep ((rl + ack_length) * 12);
  
  for (i = 0; i < ack_length; i++) {
    n = bpt.readPort(&ack_modbus[i]);
  }
  
  if (k != rl) {
    return -1;
  }

  if (verbose) {
    for (int j = 0; j < ack_length; j++) {
      printf("%X ", ack_modbus[j]);
    }
    printf("\n");
  }

  return n;
}

int MsvTeleop::sendPacketBPT (int verbose, std::vector<uint8_t> buffer) 
{
  uint8_t rl = buffer.size();
  int k;

  bpt.clearPort();
  k = bpt.writePort(buffer.data(), rl);
  usleep (rl * 10);

  if (verbose) {
    for (int j = 0; j < rl; j++) {
      printf("%X ", buffer[j]);
    }
    printf("\n");
  }

  return k;
}

