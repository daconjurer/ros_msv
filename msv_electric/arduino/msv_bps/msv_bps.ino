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
/// @file Arduino file for the MSV_01 "BPT" board. This board is based on the Blue Pill
/// and it is intended to retrieve all the electric sensors data to the on-board computer
/// (OBC) of the MSV_01 robot through UART to be displayed in a ROS topic. After setting 
/// up the serial port for communication with the OBC, data is collected directly using
/// isolated current sensors and from the MSPS board (which collects non-isolated current
/// and voltage sensors data. The board waits for the MSPS board to set up and initialize
/// and then starts the serial communciation process as a "master". Once all the sensing 
/// information is available, upon previous request, it will be sent to the OBC using the
/// Modbus protocol and the internal memory table (with the sensors values) will be
/// updated.
///
/// @author Victor Esteban Sandoval-Luna
///
/// The Modbus communication running on the master (OBC) is implemented using the
/// liblighmodbus Modbus RTU library from Jacek Wieczorek.
/// Please see https://github.com/jrowberg/i2cdevlib
///
/// The Modbus responses are built non-automatically at this stage, more sensors to
/// be added. Full compatibility with the liblightmodbus library on the BPS board soon
/// to become.
///
/////////////////////////////////////////////////////////////////////////////////////////

/* TODO */

#define LED_PIN PC13

// These defs should match the ones in the msv_electric_node.cpp file
#define NUM_FORCED_COILS                  4
#define NUM_READ_IREGS                    11
#define FORCE_COILS_REQUEST_LENGTH        10
#define READ_IREGS_REQUEST_LENGTH         8
#define FORCE_COILS_RESPONSE_LENGTH       8
#ifdef NUM_READ_IREGS
  #define READ_IREGS_RESPONSE_LENGTH      (2*NUM_READ_IREGS + 5)  // READ_IREGS_RESPONSE_LENGTH = (NUM_READ_IREGS*2)+5
#endif

#if defined(NUM_FORCED_COILS) & defined(NUM_READ_IREGS)
  #define NUM_ADDRESSES                   (NUM_FORCED_COILS + NUM_READ_IREGS)
#endif

// These defs should match the ones in the msv_msps sketch
#define START_MSPS            0x01
#define START_COM             0x02
#define STOP_COM              0x03
#define SEND_MSPS_DATA        0x04
#define MSPS_READY            0x11
#define COM_STARTED           0x12
#define COM_STOPPED           0x13

#define MSPS_DATA_LENGTH      8

// LEDs default PWM values
#define LED1_PWM              100
#define LED2_PWM              100
#define LED3_PWM              100
#define LED4_PWM              100

/** MEMORY TABLE **/
// Refer to the README file for further details
static uint8_t coils[NUM_FORCED_COILS] = {0};		// LEDs
static uint16_t iregs[NUM_READ_IREGS] = {0};		// Electric sensors + CO2 (9 + 2) 
static uint8_t addresses[NUM_ADDRESSES] = {0};	// coils + regs addresses

// Modbus buffers
static uint8_t request[10] = {0};		/// For request reading
static uint8_t iregs_response[READ_IREGS_RESPONSE_LENGTH] = {0};	// For input regs response command
static uint8_t coils_response[FORCE_COILS_RESPONSE_LENGTH] = {0};	// For coils response command

// MSPS buffer
static uint8_t data[8] = {0};

// Pins to be used for the ADC reading
static uint16_t a0 = 0;
static uint16_t a1 = 0;
static uint16_t a2 = 0;
static uint16_t a3 = 0;
static uint16_t a4 = 0;
static uint16_t a5 = 0;
static uint16_t a6 = 0;
static uint16_t a7 = 0;
static uint16_t a8 = 0;

// Channels buffers for filtering
static uint8_t ch0 = 0;
static uint8_t ch1 = 0;
static uint8_t ch2 = 0;
static uint8_t ch3 = 0;
static uint8_t ch4 = 0;
static uint8_t ch5 = 0;
static uint8_t ch6 = 0;
static uint8_t ch7 = 0;
static uint8_t ch8 = 0;

/* Sensing & control signals (Analog & PWM) */

// PWM output pins
static int LIDAR = PA8;
static int LED1 = PB6;
static int LED2 = PB7;
static int LED3 = PB8;
static int LED4 = PB9;

// Analog input pins
static int V_BAT = PA0;
static int I_OBC = PA1;
static int I_ES = PA2;
static int I_DXL = PA3;
static int CO2 = PA5;
static int I_ARM = PA6;
static int I_DCM = PA7;
static int I_BL2 = PB0;
static int I_BL1 = PB1;

// Serial communication variables
static uint8_t i_obc = 0;					// Incoming bytes counter (communication between BPS and OBC)
static uint8_t obc_buffer = 0;		// Byte buffer for Serial2 (communication between BPS and OBC)
static uint8_t msps_buffer = 0;		// Byte buffer for Serial (communication between BPS and MSPS)

// This function generates the CRC for the Modbus responses
uint16_t ModbusCRC (uint8_t buf, uint16_t crc)
{
	// XOR byte to LSB of CRC
	crc ^= (uint16_t)buf;
	
	// Each bit loop
	for (int i = 8; i != 0; i--) {    
		if ((crc & 0x0001) != 0) {
			crc >>= 1;
			crc ^= 0xA001;
		} else crc >>= 1;
	}
	return crc; 
}

// Called within the setup () Arduino method
void setUp ()
{
	// MSPS communication interface
	Serial.begin(9600);
	while(!Serial);
	
	// OBC communication interface
	Serial2.begin(115200);
	while(!Serial2);
	
	pinMode(LIDAR, OUTPUT);
	pinMode(LED1, OUTPUT);
	pinMode(LED2, OUTPUT);
	pinMode(LED3, OUTPUT);
	pinMode(LED4, OUTPUT);
	
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, HIGH);
	
	// Call the initMSPS and standby methods
	initMSPS();
	standby();
	
	// Lidar PWM ouput (to get ~300 rpm)
	analogWrite(LIDAR, 153);
}

// Initilializes the MSPS board performing an init routine
void initMSPS ()
{
	uint8_t start_buffer = 0, com_buffer = 0;
	// Send START_MSPS command
	Serial.write(START_MSPS);
	
	do {
		while (!Serial.available());
		start_buffer = Serial.read();
		
		// MSPS_READY condition received
		if (start_buffer == MSPS_READY) {
			// Send START_COM command
			Serial.write(START_COM);
			digitalWrite(LED_PIN,LOW);
			
			do {
				while (!Serial.available());
				com_buffer = Serial.read();
				
				// START_COM ACK received
				if (com_buffer == COM_STARTED) {
					break;
				}
			} while (com_buffer != COM_STARTED);
			
			digitalWrite(LED_PIN,HIGH);
			break;
		}
	} while (start_buffer != MSPS_READY);
}

// Initializes the outputs with convenient values
void standby ()
{
	// 255 is used as a 0 for the PWM outputs that will be isolated using optocouplers
	analogWrite(LIDAR, 0);
	analogWrite(LED1, 255);
	analogWrite(LED2, 255);
	analogWrite(LED3, 255);
	analogWrite(LED4, 255);
	
	// Call the initMB method
	initMB();
}

void initMB ()
{
	// Header fill up (input registers response)
	iregs_response[0] = 0x01;
	iregs_response[1] = 0x03;
	iregs_response[2] = 0x16;
	
	// Whole fill up (forced coils response)
	coils_response[0] = 0x01;
	coils_response[1] = 0x0F;
	coils_response[2] = 0x00;
	coils_response[3] = 0x05;
	coils_response[4] = 0x00;
	coils_response[5] = 0x04;
	
	// Calculate CRC (forced coils response)
	uint16_t crc = 0xFFFF;
	for (int j = 0; j < 6; j++) {
		crc = ModbusCRC(coils_response[j],crc);
	}
	
	// Add the CRC
	coils_response[6] = (uint8_t)(crc & 0x00FF);
	coils_response[7] = (uint8_t)(crc >> 8);
}

void sense ()
{
	// Clean up regs array (needed for filtering)
	for (int j = 0; j < 11; j++) {
		iregs[j] = 0;
	}
	
	// Read the analog values at power stage (MSPS board)
	getPowerSensing();
	
	/* Sensing loop */
	for (int j = 0; j < 5; j++) {
		// Read the analog values:
		a0 = analogRead(I_ARM);
		a1 = analogRead(I_DCM);
		a2 = analogRead(I_BL2);
		a3 = analogRead(I_BL1);
		a4 = analogRead(I_OBC);
		a5 = analogRead(I_ES);
		a6 = analogRead(I_DXL);
		a7 = analogRead(V_BAT);
		a8 = analogRead(CO2);
		
		iregs[0] += a0; iregs[1] += a1; iregs[2] += a2;
		iregs[3] += a3; iregs[4] += a4; iregs[5] += a5;
		iregs[6] += a6; iregs[8] += a7; iregs[10] += a8;
	}
	
	// Update the regs array (AKA Memory table)
	iregs[0] /= 5; iregs[1] /= 5; iregs[2] /= 5;
	iregs[3] /= 5; iregs[4] /= 5; iregs[5] /= 5;
	iregs[6] /= 5; iregs[8] /= 5; iregs[10] /= 5;
	// End of mean filter
}

void getPowerSensing ()
{
	// Data counter
	uint8_t power_i = 0;
	
	// Send data request to MSPS
	Serial.write(SEND_MSPS_DATA);
	
	// Read the data
	do {
		while (!Serial.available());
		msps_buffer = Serial.read();
		digitalWrite(LED_PIN,LOW);
		data[power_i] = msps_buffer;
		power_i++;
	} while (power_i < MSPS_DATA_LENGTH);
	
	// Fill up the message with the sensors data
	iregs[7] = (((uint16_t)data[1]) << 8) | (0x00FF & ((uint16_t)data[2]));
	iregs[9] = (((uint16_t)data[5]) << 8) | (0x00FF & ((uint16_t)data[6]));
}

void mapOutputs (unsigned int len)
{
	// Only the forcing coils request modifies the outputs
	if (len != FORCE_COILS_REQUEST_LENGTH) {
		return;
	}
	
	// Init the elements in case no LED will be turned on
	for (int i = 0;i < 4; i++) {
		coils[i]= 0;
	}
	
	// Store each coil value
	uint8_t led_coils = request[7];
	uint8_t led1 = led_coils & 0x01;
	uint8_t led2 = led_coils & 0x02;
	uint8_t led3 = led_coils & 0x04;
	uint8_t led4 = led_coils & 0x08;
	
	// Map coils values
	analogWrite(LED1, (led1) ? 0 : 255-LED1_PWM);
	analogWrite(LED2, (led2) ? 0 : 255-LED2_PWM);
	analogWrite(LED3, (led3) ? 0 : 255-LED3_PWM);
	analogWrite(LED4, (led4) ? 0 : 255-LED4_PWM);
	
	// Update the coils array if needed (AKA Memory table)
	if (led1) {coils[0]= 1;}
	if (led2) {coils[1]= 1;}
	if (led3) {coils[2]= 1;}
	if (led4) {coils[3]= 1;}
	
	return;
}

void sendACK (uint8_t len)
{
	uint16_t crc = 0xFFFF;
	
	if (len == FORCE_COILS_REQUEST_LENGTH) 
	{
		// Send the response
		for (int j = 0; j < FORCE_COILS_RESPONSE_LENGTH; j++) {
			Serial2.write(coils_response[j]); 
		}
		return;
	}
	
	if (len == READ_IREGS_REQUEST_LENGTH) 
	{
		// Fill up the input registers response array data
		iregs_response[3] = (unsigned char)(iregs[0] >> 8);
		iregs_response[4] = (unsigned char)(iregs[0] & 0x00FF);
		iregs_response[5] = (unsigned char)(iregs[1] >> 8);
		iregs_response[6] = (unsigned char)(iregs[1] & 0x00FF);
		iregs_response[7] = (unsigned char)(iregs[2] >> 8);
		iregs_response[8] = (unsigned char)(iregs[2] & 0x00FF);
		iregs_response[9] = (unsigned char)(iregs[3] >> 8);
		iregs_response[10] = (unsigned char)(iregs[3] & 0x00FF);
		iregs_response[11] = (unsigned char)(iregs[4] >> 8);
		iregs_response[12] = (unsigned char)(iregs[4] & 0x00FF);
		iregs_response[13] = (unsigned char)(iregs[5] >> 8);
		iregs_response[14] = (unsigned char)(iregs[5] & 0x00FF);
		iregs_response[15] = (unsigned char)(iregs[6] >> 8);
		iregs_response[16] = (unsigned char)(iregs[6] & 0x00FF);
		iregs_response[17] = (unsigned char)(iregs[7] >> 8);
		iregs_response[18] = (unsigned char)(iregs[7] & 0x00FF);
		iregs_response[19] = (unsigned char)(iregs[8] >> 8);
		iregs_response[20] = (unsigned char)(iregs[8] & 0x00FF);
		iregs_response[21] = (unsigned char)(iregs[9] >> 8);
		iregs_response[22] = (unsigned char)(iregs[9] & 0x00FF);
		iregs_response[23] = (unsigned char)(iregs[10] >> 8);
		iregs_response[24] = (unsigned char)(iregs[10] & 0x00FF);
		
		// Calculate CRC (read input registers)
		for (int j = 0; j < READ_IREGS_RESPONSE_LENGTH-2; j++) {
			crc = ModbusCRC(iregs_response[j],crc);
		}
		
		// Add the CRC (read input registers)
		iregs_response[25] = (uint8_t)(crc & 0x00FF);
		iregs_response[26] = (uint8_t)(crc >> 8);
		
		// Send the response (read input registers)
		for (int j = 0; j < READ_IREGS_RESPONSE_LENGTH; j++) {
			Serial2.write(iregs_response[j]); 
		}
		return;
	}
	
	// Wrong request length case
	Serial2.write(0xFF);
	return;
}

void setup () {
	// Call the setUp method
	setUp();
}

void loop ()
{
	if (Serial2.available())
	{
		digitalWrite(LED_PIN,LOW);
		obc_buffer = Serial2.read();
		
		// End of request (from OBC)
		if (obc_buffer == 0xFF) {
			// Run the sensing routine
			sense();
			mapOutputs(i_obc);
			sendACK(i_obc);
			i_obc = 0;
			digitalWrite(LED_PIN,HIGH);
		} else {
			request[i_obc] = obc_buffer;
			i_obc++;
		}
	}
}

