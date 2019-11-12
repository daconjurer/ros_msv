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
/// @file Arduino file for the MSV_01 "OCMD" board. This board is based on the OpenCM
/// 9.04 and allow the on-board computer to control the ROBOTIS Dynamixel AX-12A servo
/// motors of the MSV_01 robot through UART. After setting up the serial port for
/// communication with the OBC, move and read commands are recevied and then sent to the
/// servo motors using the ROBOTIS DynamixelSDK 3.5.4 or superior (running on the board).
///
/// @author Victor Esteban Sandoval-Luna
///
/// DYNAMIXEL SDK running on the OpenCM 9.04 C board.
/// Please see https://github.com/ROBOTIS-GIT/DynamixelSDK
///
/// For further information on the OpenCM 9,04 controller bard,
/// please see http://emanual.robotis.com/docs/en/parts/controller/opencm904/
///
/////////////////////////////////////////////////////////////////////////////////////////

#include <DynamixelSDK.h>

#define LED_PIN                         14

// Serial communication definitions
#define OCMD_START                      0x10
#define OCMD_SETUP                      0x20
#define OCMD_INIT                       0x30
#define OCMD_READY                      0x31

#define OCMD_DATA_LENGTH                10    // OCMD_WRITE_LENGTH = (no. of servos*2)+2
#define OCMD_SETUP_LENGTH               10
#define OCMD_INIT_LENGTH                4

#define DXL_PORT_READY                  0x70
#define DXL_BAUDRATE_READY              0x71

#define DXL_PORT_ERROR                  0x80
#define DXL_BAUDRATE_ERROR              0x81

// Dynamixel PacketHandler error definitions
#define PH_SUCCESS                      0x90  // Tx/Rx packet communication success
#define PH_PORT_BUSY                    0x91  // Port is busy (in use)
#define PH_TX_FAIL                      0x92  // Failed transmit instruction packet
#define PH_RX_FAIL                      0x93  // Failed get status packet
#define PH_TX_ERROR                     0x94  // Incorrect instruction packet
#define PH_RX_WAITING                   0x95  // Now recieving status packet
#define PH_RX_TIMEOUT                   0x96  // There is no status packet
#define PH_RX_CORRUPT                   0x97  // Incorrect status packet
#define PH_NOT_AVAILABLE                0x98  // Port not available

#define DXL_SUCCESS                     0xA0
#define DXL_VOLTAGE                     0xA1
#define DXL_ANGLE                       0xA2
#define DXL_OVERHEAT                    0xA3
#define DXL_RANGE                       0xA4
#define DXL_CHECKSUM                    0xA5
#define DXL_OVERLOAD                    0xA6
#define DXL_INSTRUCTION                 0xA7

// Protocol 1.0 Error bit
#define ERRBIT_VOLTAGE                  1     // Supplied voltage is out of the range (operating volatage set in the control table)
#define ERRBIT_ANGLE                    2     // Goal position is written out of the range (from CW angle limit to CCW angle limit)
#define ERRBIT_OVERHEAT                 4     // Temperature is out of the range (operating temperature set in the control table)
#define ERRBIT_RANGE                    8     // Command (setting value) is out of the range for use.
#define ERRBIT_CHECKSUM                 16    // Instruction packet checksum is incorrect.
#define ERRBIT_OVERLOAD                 32    // The current load cannot be controlled by the set torque.
#define ERRBIT_INSTRUCTION              64    // Undefined instruction or delivering the action command without the reg_write command.

// Dynamixel AX-12A Control table
#define ADDR_TORQUE_ENABLE              24
#define ADDR_GOAL_POSITION              30
#define ADDR_CURRENT_POSITION           36
#define ADDR_MOVING_SPEED               32

// Dynamixel SDK Protocol version
#define PROTOCOL_VERSION                1.0

// Default settings
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyACM0"

#define DXL_ARM                         11              // Dynamixel ID: 11 
#define DXL_PITCH                       12              // Dynamixel ID: 12
#define DXL_YAW                         14              // Dynamixel ID: 14
#define DXL_ROLL                        7               // Dynamixel ID: 10

#define MOVING_SPEED                    0x10
#define TORQUE_ENABLE                   1               // Value for enabling the torque
#define TORQUE_DISABLE                  0               // Value for disabling the torque

/* THESE ARE THE PHYSICAL LIMITS DEPENDING ON THE SERVOS MOUNTING OR CONFIGURATIONS. MAKE SURE THEY ARE CORRECT BEFORE LAUNCHING THE APPLICATION */
#define ARM_MINIMUM_POSITION_VALUE      341             // ARM Dynamixel will rotate between this value
#define ARM_MAXIMUM_POSITION_VALUE      681             // and this value.
#define PITCH_MINIMUM_POSITION_VALUE    300             // PITCH Dynamixel will rotate between this value
#define PITCH_MAXIMUM_POSITION_VALUE    450             // and this value.
#define YAW_MINIMUM_POSITION_VALUE      312             // YAW Dynamixel will rotate between this value
#define YAW_MAXIMUM_POSITION_VALUE      712             // and this value.
#define ROLL_MINIMUM_POSITION_VALUE     0               // ROLL Dynamixel will rotate between this value
#define ROLL_MAXIMUM_POSITION_VALUE     1023            // and this value.

#define DXL_MOVING_STATUS_THRESHOLD     20              // Dynamixel moving status threshold

#define NUM_SERVOS                      4

// Debug functions
void sendTxRxResult (const int& dxl_comm_result) 
{
  uint8_t result = 0;
  
  switch (1 - dxl_comm_result) {
    case 1:
      result = PH_SUCCESS;
      break;
    case 1001:
      result = PH_PORT_BUSY;
      break;
    case 1002:
      result = PH_TX_FAIL;
      break;
    case 1003:
      result = PH_RX_FAIL;
      break;
    case 2001:
      result = PH_TX_ERROR;
      break;
    case 3001:
      result = PH_RX_WAITING;
      break;
    case 3002:
      result = PH_RX_TIMEOUT;
      break;
    case 3003:
      result = PH_RX_CORRUPT;
      break;
    case 9001:
      result = PH_NOT_AVAILABLE;
      break;
    default:
      break;
  }
  Serial.write(result);
}

void sendRxPacketError (const uint8_t& error) 
{
  uint8_t dxl_error = DXL_SUCCESS;
  
  if (error & ERRBIT_VOLTAGE)
    dxl_error = DXL_VOLTAGE;

  if (error & ERRBIT_ANGLE)
    dxl_error = DXL_ANGLE;

  if (error & ERRBIT_OVERHEAT)
    dxl_error = DXL_OVERHEAT;

  if (error & ERRBIT_RANGE)
    dxl_error = DXL_RANGE;

  if (error & ERRBIT_CHECKSUM)
    dxl_error = DXL_CHECKSUM;

  if (error & ERRBIT_OVERLOAD)
    dxl_error = DXL_OVERLOAD;

  if (error & ERRBIT_INSTRUCTION)
    dxl_error = DXL_INSTRUCTION;
  
  Serial.write(dxl_error);
}

// Error checking functions
void storeTxRxResult (const int& dxl_comm_result, const int& ack_index, uint8_t* ack) 
{
  uint8_t result = 0;
  uint8_t i = 0; 
  
  switch (1 - dxl_comm_result) {
    case 1:
      result = PH_SUCCESS;
      break;
    case 1001:
      result = PH_PORT_BUSY;
      break;
    case 1002:
      result = PH_TX_FAIL;
      break;
    case 1003:
      result = PH_RX_FAIL;
      break;
    case 2001:
      result = PH_TX_ERROR;
      break;
    case 3001:
      result = PH_RX_WAITING;
      break;
    case 3002:
      result = PH_RX_TIMEOUT;
      break;
    case 3003:
      result = PH_RX_CORRUPT;
      break;
    case 9001:
      result = PH_NOT_AVAILABLE;
      break;
    default:
      break;
  }

  i = (2*ack_index)+1;
  ack[i] = result;
}

void storeRxPacketError (const uint8_t& error, const int& ack_index, uint8_t* ack) 
{
  uint8_t dxl_error = DXL_SUCCESS;
  uint8_t i = 0;
  
  if (error & ERRBIT_VOLTAGE)
    dxl_error = DXL_VOLTAGE;

  if (error & ERRBIT_ANGLE)
    dxl_error = DXL_ANGLE;

  if (error & ERRBIT_OVERHEAT)
    dxl_error = DXL_OVERHEAT;

  if (error & ERRBIT_RANGE)
    dxl_error = DXL_RANGE;

  if (error & ERRBIT_CHECKSUM)
    dxl_error = DXL_CHECKSUM;

  if (error & ERRBIT_OVERLOAD)
    dxl_error = DXL_OVERLOAD;

  if (error & ERRBIT_INSTRUCTION)
    dxl_error = DXL_INSTRUCTION;

  i = (2*ack_index)+2;
  ack[i] = dxl_error;
}

void enableTorque (dynamixel::PortHandler* port_handler, dynamixel::PacketHandler* packet_handler, const int& dxl_id, const int& ack_index, uint8_t* ack) 
{
  int dxl_comm_result = COMM_TX_FAIL;   // Dynamixel communication result
  uint8_t dxl_error = 0;                // Dynamixel error
  
  dxl_comm_result = packet_handler->write1ByteTxRx(port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  storeTxRxResult(dxl_comm_result, ack_index, ack);
  storeRxPacketError(dxl_error, ack_index, ack);
}

void disableTorque (dynamixel::PortHandler* port_handler, dynamixel::PacketHandler* packet_handler, const int& dxl_id, const int& ack_index, uint8_t* ack) 
{
  int dxl_comm_result = COMM_TX_FAIL;   // Dynamixel communication result
  uint8_t dxl_error = 0;                // Dynamixel error
  
  dxl_comm_result = packet_handler->write1ByteTxRx(port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  storeTxRxResult(dxl_comm_result, ack_index, ack);
  storeRxPacketError(dxl_error, ack_index, ack);
}

void moveAngle (dynamixel::PortHandler* port_handler, dynamixel::PacketHandler* packet_handler, const int& dxl_id, const int& ack_index, uint8_t* ack, const int& dxl_goal)
{
  int dxl_comm_result = COMM_TX_FAIL;   // Dynamixel communication result
  uint8_t dxl_error = 0;                // Dynamixel error
  
  dxl_comm_result = packet_handler->write2ByteTxRx(port_handler, dxl_id, ADDR_GOAL_POSITION, dxl_goal, &dxl_error);
  storeTxRxResult(dxl_comm_result, ack_index, ack);
  storeRxPacketError(dxl_error, ack_index, ack);
}

void readAngle (dynamixel::PortHandler* port_handler, dynamixel::PacketHandler* packet_handler, const int& dxl_id, const int& ack_index, uint8_t* ack, uint16_t& pos)
{
  int dxl_comm_result = COMM_TX_FAIL;   // Dynamixel communication result
  uint8_t dxl_error = 0;                // Dynamixel error
  
  dxl_comm_result = packet_handler->read2ByteTxRx(port_handler, dxl_id, ADDR_CURRENT_POSITION, &pos, &dxl_error);
  storeTxRxResult(dxl_comm_result, ack_index, ack);
  storeRxPacketError(dxl_error, ack_index, ack);
}

void setMovingSpeed (dynamixel::PortHandler *port_handler, dynamixel::PacketHandler *packet_handler, const int& dxl_id, const int& ack_index, uint8_t* ack, const int& moving_speed)
{
  int dxl_comm_result = COMM_TX_FAIL;   // Dynamixel communication result
  uint8_t dxl_error = 0;                // Dynamixel error
  
  dxl_comm_result = packet_handler->write2ByteTxRx(port_handler, dxl_id, ADDR_MOVING_SPEED, moving_speed, &dxl_error);
  storeTxRxResult(dxl_comm_result, ack_index, ack);
  storeRxPacketError(dxl_error, ack_index, ack);
}

void sendACK (uint8_t* ack)
{
  // Send ACK with the communication results
  for (int i = 0; i < OCMD_DATA_LENGTH; i++) {
    // debug
    //ack[i] = 0;
    Serial.write(ack[i]);
  }
  // debug
  //ack[0] = 0xFF; ack[OCMD_DATA_LENGTH-1] = 0xFF;
  //Serial.write(ack,OCMD_DATA_LENGTH);
}

void setup ()
{
  uint8_t init_byte = 0, data_byte = 0;
  uint8_t init_ack[OCMD_INIT_LENGTH] = {0};
  uint8_t setup_ack[OCMD_SETUP_LENGTH] = {0};
  uint8_t data_ack[OCMD_DATA_LENGTH] = {0};
  uint8_t data_buffer[OCMD_DATA_LENGTH] = {0};
  uint8_t angles_ack[OCMD_DATA_LENGTH] = {0};

  int pos_counter = 1;
  
  // Serial communication initialization
  Serial.begin(115200);
  while(!Serial);

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  uint16_t dxl_pos[NUM_SERVOS] = {0};                             // Present positions (arm,yaw,pitch,roll)
  int32_t dxl_goals[NUM_SERVOS] = {0};                            // Goal positions (arm,yaw,pitch,roll)
  int dxl_ids[NUM_SERVOS] = {DXL_ARM,DXL_YAW,DXL_PITCH,DXL_ROLL}; // Servos IDs (arm,yaw,pitch,roll)

  // Init ACK packets
  setup_ack[0] = 0xFF; setup_ack[OCMD_SETUP_LENGTH-1] = 0xFF;
  data_ack[0] = 0xFF; data_ack[OCMD_DATA_LENGTH-1] = 0xFF;
  angles_ack[0] = 0xFF; angles_ack[OCMD_DATA_LENGTH-1] = 0xFF;

  // Wait for the on-board computer Setup instruction (OCMD_START)
  do {
    while (!Serial.available());
    digitalWrite(LED_PIN,LOW);
    init_byte = Serial.read();
    // OCMD_START condition received
    if (init_byte == OCMD_START) {
      // Init board for the Node instructions
      init_ack[0] = OCMD_INIT;

      // Open port
      if (portHandler->openPort()) {
        init_ack[1] = DXL_PORT_READY;
      } else {
        init_ack[1] = DXL_PORT_ERROR;
        break;
      }

      // Set port baudrate
      if (portHandler->setBaudRate(BAUDRATE)) {
        init_ack[2] = DXL_BAUDRATE_READY;
      } else {
        init_ack[2] = DXL_BAUDRATE_ERROR;
        break;
      }
      
      // Enable Dynamixels Torques
      for (int i = 0; i < NUM_SERVOS; i++) {
        enableTorque(portHandler, packetHandler, dxl_ids[i], i, setup_ack);
      }

      // Board ready for the Node instructions
      init_ack[3] = OCMD_READY;
    }
    digitalWrite(LED_PIN,HIGH);
  } while (init_byte != OCMD_START);

  // Send the Init ACK packet
  Serial.write(init_ack, OCMD_INIT_LENGTH);

  // Wait for the on-board computer Setup instruction (OBC_START)
  do {
    while (!Serial.available());
    digitalWrite(LED_PIN,LOW);
    init_byte = Serial.read();
    // OCMD_SETUP condition received
    if (init_byte == OCMD_SETUP) {
       // Send the Setup ACK packet
      Serial.write(setup_ack, OCMD_SETUP_LENGTH);
    }
    digitalWrite(LED_PIN,HIGH);
  } while (init_byte != OCMD_SETUP);

  while (1) {
    // read Serial port for new move commands
    uint8_t obc_i = 0;
  
    while (obc_i < OCMD_DATA_LENGTH) {
      if (Serial.available()) {
        digitalWrite(LED_PIN,LOW);
        data_byte = Serial.read();

        // Decode the data buffer and update the goals
        if (obc_i == OCMD_DATA_LENGTH-1) {
          // Process data
          for (int i = 0; i < NUM_SERVOS; i++) {
            dxl_goals[i] = (((uint16_t)data_buffer[1]) << 8) | (0x00FF & ((uint16_t)data_buffer[2]));
          }

          // Write Dynamixels positions
          for (int i = 0; i < NUM_SERVOS; i++) {
            moveAngle(portHandler, packetHandler, dxl_ids[i], i, data_ack, dxl_goals[i]);
          }

          // Send ACK with the communication results
          sendACK(data_ack);

          // Read Dynamixels positions
          for (int i = 0; i < NUM_SERVOS; i++) {
            readAngle(portHandler, packetHandler, dxl_ids[i], i, data_ack, dxl_pos[i]);
          }

          // Send ACK with the communciation results
          sendACK(data_ack);

          // Build up the angles ACK from dxl_pos data 
          for (int i = 0; i < NUM_SERVOS; i++) {
            angles_ack[pos_counter] = (unsigned char)(dxl_pos[i] >> 8);
            angles_ack[pos_counter + 1] = (unsigned char)(dxl_pos[i] & 0x00FF);
            pos_counter+=2;
          }
          pos_counter = 1;
          
          // Send the ACK with the current positions
          sendACK(angles_ack);
          
          obc_i = 0;
          digitalWrite(LED_PIN,HIGH);
        } else {
          // Store incoming data
          data_buffer[obc_i] = data_byte;
          obc_i++;
        }
      }
    }
  }
}

void loop ()
{
 // Nothing to be done here
}

