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
/// and it is intended to control the tracks system of the MSV_01 from a node running on
/// the on-board computer through UART. The data is recieved using the Modbus protocol
/// and then decodified for moving the actuators, updating the current state of the
/// actuators in the internal memory table and finally sending a response back to the
/// OBC with the updated states of the actuators.
///
/// @author Victor Esteban Sandoval-Luna
///
/// The Modbus communication running on the master (OBC) is implemented using the
/// liblighmodbus Modbus RTU library from Jacek Wieczorek.
/// Please see https://github.com/jrowberg/i2cdevlib
///
/// The Modbus responses are built non-automatically at this stage, more actuators to
/// be added. Full compatibility with the liblightmodbus library on the BPT board soon
/// to become.
///
/////////////////////////////////////////////////////////////////////////////////////////

/* TODO */

#define LED_PIN PC13

// These defs should match the ones in the msv_teleop_node.cpp file
#define NUM_FORCED_COILS                  5
#define NUM_READ_COILS                    2
#if defined(NUM_READ_COILS) & defined(NUM_FORCED_COILS)
	#define NUM_COILS                       (NUM_FORCED_COILS + NUM_READ_COILS)
#endif
#define NUM_PRESET_HREGS                  4
#define NUM_READ_IREGS                    4
#if defined(NUM_PRESET_HREGS) & defined(NUM_READ_IREGS)
	#define NUM_REGS                       (NUM_PRESET_HREGS + NUM_READ_IREGS)
#endif

#define READ_COILS_REQUEST_LENGTH         8
#define FORCE_COILS_REQUEST_LENGTH        10
#ifdef NUM_PRESET_HREGS
	#define PRESET_HREGS_REQUEST_LENGTH     (2*NUM_PRESET_HREGS + 9)
#endif
#define READ_IREGS_REQUEST_LENGTH         8

#define FORCE_COILS_RESPONSE_LENGTH       8
#define READ_COILS_RESPONSE_LENGTH        6
#define PRESET_HREGS_RESPONSE_LENGTH      8
#ifdef NUM_READ_IREGS
	#define READ_IREGS_RESPONSE_LENGTH      (2*NUM_READ_IREGS + 5)
#endif

#if defined(NUM_COILS) & defined(NUM_REGS)
	#define NUM_ADDRESSES                   (NUM_COILS + NUM_REGS)
#endif

/** MEMORY TABLE **/
// Refer to the README file for further details
static uint8_t coils[NUM_COILS] = {0};          // Drivers logic inputs + alarms
static uint16_t hregs[NUM_PRESET_HREGS] = {0};  // Drivers PWM values (speed)
static uint16_t iregs[NUM_READ_IREGS] = {0};    // Drivers speed signals (rpms)
static uint8_t addresses[15] = {0};             // coils + regs addresses

// Modbus buffers
static uint8_t request[10] = {0};
static uint8_t hregs_response[PRESET_HREGS_RESPONSE_LENGTH] = {0};       // For holding regs presetting response command
static uint8_t force_coils_response[FORCE_COILS_RESPONSE_LENGTH] = {0};  // For coils forcing response command
static uint8_t iregs_response[READ_IREGS_RESPONSE_LENGTH] = {0};         // For input regs reading response command
static uint8_t read_coils_response[READ_COILS_RESPONSE_LENGTH] = {0};    // For coils reading response command

/* Driver control signals (Logic & PWM) */

// PWM output pins
static int SV1 = PA8;
static int SV2 = PB7;
static int PA = PB1;
static int PB = PA6;

// Logic output pins
static int FR1 = PB14;
static int FR2 = PB15;
static int A_1 = PB10;
static int A_2 = PB11;
static int B_1 = PA5;
static int B_2 = PA4;

// PWM input pins
static int AA = PA15;
static int BA = PB3;
static int AB = PB4;
static int BB = PB6;
static int SPEED1 = PB12;
static int SPEED2 = PA11;

// Logic input pins
static int ALM1 = PB13;
static int ALM2 = PA12;

// Serial communication variables
static uint8_t i_obc = 0;       // Incoming bytes counter (communication between BPT and OBC)
static uint8_t obc_buffer = 0;  // Byte buffer for Serial (communication between BPT and OBC)

// Interruption handling variables (PWM reading pin states)
volatile int input_state = 0;                   // Current state of the pin
volatile uint16_t before_start_time[6] = {0};   // Previous time (rising edge instant)
volatile uint16_t start_time[6] = {0};          // Current time (next rising edge instant)
volatile uint16_t period[6] = {0};              // Period of signal

// Generates the CRC for the Modbus responses
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
	// OBC communication interface
	Serial.begin(115200);
	while(!Serial);
	
	pinMode(SV1, OUTPUT);
	pinMode(SV2, OUTPUT);
	pinMode(PA, OUTPUT);
	pinMode(PB, OUTPUT);
	
	pinMode(FR1, OUTPUT);
	pinMode(FR2, OUTPUT);
	pinMode(A_1, OUTPUT);
	pinMode(A_2, OUTPUT);
	pinMode(B_1, OUTPUT);
	pinMode(B_2, OUTPUT);
	
	// Attach functions to external interrupts of PWM reading pins
	pinMode(AA, INPUT);
	attachInterrupt(AA, getAAPeriod, CHANGE);
	pinMode(BA, INPUT);
	attachInterrupt(BA, getBAPeriod, CHANGE);
	pinMode(AB, INPUT);
	attachInterrupt(AB, getABPeriod, CHANGE);
	pinMode(BB, INPUT);
	attachInterrupt(BB, getBBPeriod, CHANGE);
	pinMode(SPEED1, INPUT);
	attachInterrupt(SPEED1, getSpeed1Period, CHANGE);
	pinMode(SPEED2, INPUT);
	attachInterrupt(SPEED2, getSpeed2Period, CHANGE);
	
	pinMode(ALM1, INPUT);
	pinMode(ALM2, INPUT);
	
	// Configure LED for output
	pinMode(LED_PIN, OUTPUT);
	
	// Call the standby method
	standby();
}

// Initializes the outputs with convenient values
void standby ()
{
	digitalWrite(LED_PIN, HIGH);
	
	// Drivers PWM ouput (both zero, but 255 used for optocoupler-isolated drivers)
	analogWrite(SV1, 255);
	analogWrite(SV2, 255);
	analogWrite(PA, 0);
	analogWrite(PB, 0);
	
	// Call the initMB method
	initMB();
}

// Initializes the response arrays with the info required in the Modbus protocol
void initMB ()
{
	// Header fill up(input registers response)
	iregs_response[0] = 0x02;
	iregs_response[1] = 0x03;
	iregs_response[2] = 0x08;
	
	// Whole fill up (holding registers response)
	hregs_response[0] = 0x02;
	hregs_response[1] = 0x10;
	hregs_response[2] = 0x00;
	hregs_response[3] = 0x00;
	hregs_response[4] = 0x00;
	hregs_response[5] = 0x04;
	
	// Whole fill up (forced coils response)
	force_coils_response[0] = 0x02;
	force_coils_response[1] = 0x0F;
	force_coils_response[2] = 0x00;
	force_coils_response[3] = 0x00;
	force_coils_response[4] = 0x00;
	force_coils_response[5] = 0x05;
	
	// Header fill up (read coils response)
	read_coils_response[0] = 0x02;
	read_coils_response[1] = 0x01;
	read_coils_response[2] = 0x01;
	
	// Calculate CRC (forced coils response)
	uint16_t crc = 0xFFFF;
	for (int j = 0; j < 6; j++) {
		crc = ModbusCRC(force_coils_response[j],crc);
	}
	
	// Add the CRC (forced coils response)
	force_coils_response[6] = (uint8_t)(crc & 0x00FF);
	force_coils_response[7] = (uint8_t)(crc >> 8);
	
	// Calculate CRC (holding registers response)
	crc = 0xFFFF;
	for (int j = 0; j < 6; j++) {
		crc = ModbusCRC(hregs_response[j],crc);
	}
	
	// Add the CRC (holding registers response)
	hregs_response[6] = (uint8_t)(crc & 0x00FF);
	hregs_response[7] = (uint8_t)(crc >> 8);
}

// Decodes the info from the Modbus message and maps it to the output pins
void mapOutputs (unsigned int len)
{
	// Map coils values if a coils forcing request arrived
	if (len == FORCE_COILS_REQUEST_LENGTH) {
		
		// Init the coils in case no actuator will be turned on
		for (int i = 0;i < 5; i++) {
			coils[i]= 0;
		}
		
		int c = request[7];
		// Forward or smooth turning
		if (c == 0x11) {
			digitalWrite(FR1, LOW);
			digitalWrite(FR2, HIGH);
		}
		// Reverse
		if (c == 0x12) {
			digitalWrite(FR1, HIGH);
			digitalWrite(FR2, LOW);
		}
		// Right
		if (c == 0x13) {
			digitalWrite(FR1, HIGH);
			digitalWrite(FR2, HIGH);
		}
		// Left
		if (c == 0x10) {
			digitalWrite(FR1, LOW);
			digitalWrite(FR2, LOW);
		}
		
		// Flippers up
		if (c == 0x18) {
			digitalWrite(A_1, LOW);
			digitalWrite(A_2, HIGH);
			digitalWrite(B_1, HIGH);
			digitalWrite(B_2, LOW);
		}
		
		// Flippers down
		if (c == 0x14) {
			digitalWrite(A_1, HIGH);
			digitalWrite(A_2, LOW);
			digitalWrite(B_1, LOW);
			digitalWrite(B_2, HIGH);
		}
		
		// Store each coil value
		uint8_t traction_coils = request[7];
		uint8_t motor1_dir = traction_coils & 0x01;
		uint8_t motor2_dir = traction_coils & 0x02;
		uint8_t flipperl_dir = traction_coils & 0x04;
		uint8_t flipperr_dir = traction_coils & 0x08;
		uint8_t moving = traction_coils & 0x10;
		
		// Updates the coils array (AKA Memory table)
		if (motor1_dir) {coils[0]= 1;}
		if (motor2_dir) {coils[1]= 1;}
		if (flipperl_dir) {coils[2]= 1;}
		if (flipperr_dir) {coils[3]= 1;}
		if (moving) {coils[4]= 1;}
		
		return;
	}
	
	// Map holding registers values if a holding registers presetting request arrived
	if (len == PRESET_HREGS_REQUEST_LENGTH) {
		int blc1_pwm = (request[8] > 255) ? 255 : 255-request[8];
		int blc2_pwm = (request[10] > 255) ? 255 : 255-request[10];
		int fl_pwm = (request[12] > 255) ? 255 : 255-request[12];
		int fr_pwm = (request[14] > 255) ? 255 : 255-request[14];
		
		analogWrite(SV1, blc1_pwm);
		analogWrite(SV2, blc2_pwm);
		analogWrite(PA, fl_pwm);
		analogWrite(PB, fr_pwm);
		
		// Updates the regs array (AKA Memory table)
		hregs[0] = blc1_pwm;
		hregs[1] = blc2_pwm;
		hregs[2] = fl_pwm;
		hregs[3] = fr_pwm;
		
		return;
	}
}

// Sends the Modbus response for each Modbus request
void sendACK (uint8_t len)
{
	// 
	uint16_t crc = 0xFFFF;
	
	if (len == FORCE_COILS_REQUEST_LENGTH) {
		// Send the response (forced coils registers)
		for (int j = 0; j < FORCE_COILS_RESPONSE_LENGTH; j++) {
			Serial.write(force_coils_response[j]);
		}
		return;
	}
	
	if (len == PRESET_HREGS_REQUEST_LENGTH) {
		// Send the response (holding registers)
		for (int j = 0; j < PRESET_HREGS_RESPONSE_LENGTH; j++) {
			Serial.write(hregs_response[j]);
		}
		return;
	}
	
	if (len == READ_COILS_REQUEST_LENGTH && request[1] == 0x01) {
		// Fill up the read coils response array
		uint8_t read_coils = 0x60;
		read_coils_response[3] = read_coils; // Mask pending so only the necessary coils are sent
		
		// Calculate CRC
		for (int j = 0; j < READ_COILS_RESPONSE_LENGTH-2; j++) {
			crc = ModbusCRC(read_coils_response[j],crc);
		}
		
		// Add the CRC (read coils)
		read_coils_response[READ_COILS_RESPONSE_LENGTH-2] = (uint8_t)(crc & 0x00FF);
		read_coils_response[READ_COILS_RESPONSE_LENGTH-1] = (uint8_t)(crc >> 8);
		
		// Send the response (read coils)
		for (int j = 0; j < READ_COILS_RESPONSE_LENGTH; j++) {
			Serial.write(read_coils_response[j]); 
		}
		return;
	}
	
	if (len == READ_IREGS_REQUEST_LENGTH && request[1] == 0x03) {
		// Fill up the input registers response array
		
		// Main actuators rpms calculation.
		// See the BLD-300 controller datsheet for details (4-pole motors)
		iregs[0] = (uint16_t)(5000000/period[4]);
		iregs[1] = (uint16_t)(5000000/period[5]);
		iregs[2] = (uint16_t)(1/period[0]);
		iregs[3] = (uint16_t)(1/period[2]);
		
		iregs_response[3] = (unsigned char)(iregs[0] >> 8);
		iregs_response[4] = (unsigned char)(iregs[0] & 0x00FF);
		iregs_response[5] = (unsigned char)(iregs[1] >> 8);
		iregs_response[6] = (unsigned char)(iregs[1] & 0x00FF);
		iregs_response[7] = (unsigned char)(iregs[2] >> 8);
		iregs_response[8] = (unsigned char)(iregs[2] & 0x00FF);
		iregs_response[9] = (unsigned char)(iregs[3] >> 8);
		iregs_response[10] = (unsigned char)(iregs[3] & 0x00FF);
		
		// Calculate CRC (input registers)
		for (int j = 0; j < READ_IREGS_RESPONSE_LENGTH-2; j++) {
			crc = ModbusCRC(iregs_response[j],crc);
		}
		
		// Add the CRC (input registers)
		iregs_response[READ_IREGS_RESPONSE_LENGTH-2] = (uint8_t)(crc & 0x00FF);
		iregs_response[READ_IREGS_RESPONSE_LENGTH-1] = (uint8_t)(crc >> 8);
		
		// Send the response (input registers)
		for (int j = 0; j < READ_IREGS_RESPONSE_LENGTH; j++) {
			Serial.write(iregs_response[j]);
		}
		return;
	}
	
	// Wrong request length case
	Serial.write(0xFF);
	return;
}

void setup ()
{
	// Call the setUp method
	setUp();
}

// Reads the Modbus commands from the OBC
void loop ()
{
	if (Serial.available())
	{
		digitalWrite(LED_PIN,LOW);
		obc_buffer = Serial.read();
		
		// End of request (from OBC)
		if (obc_buffer == 0xFF) {
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

/* External interruptions service routines (ISR) activation functions */

// Gets the period of signal AA
void getAAPeriod ()
{
	// Read the input pin:
	input_state = digitalRead(AA);
	
	if (input_state == HIGH) {
		// If the current state is HIGH, period completed
		period[0] = start_time[0] - before_start_time[0];
		before_start_time[0] = start_time[0];
	}
	else {
		start_time[0] = micros();
	}
}

// Gets the period of signal BA
void getBAPeriod ()
{
	// Read the input pin:
	input_state = digitalRead(BA);
	
	if (input_state == HIGH) {
		// If the current state is HIGH, period completed
		period[1] = start_time[1] - before_start_time[1];
		before_start_time[1] = start_time[1];
	}
	else {
		start_time[1] = micros();
	}
}

// Gets the period of signal AB
void getABPeriod ()
{
	// Read the input pin:
	input_state = digitalRead(AB);
	
	if (input_state == HIGH) {
		// If the current state is HIGH, period completed
		period[2] = start_time[2] - before_start_time[2];
		before_start_time[2] = start_time[2];
	}
	else {
		start_time[2] = micros();
	}
}

// Gets the period of signal BB
void getBBPeriod ()
{
	// Read the input pin:
	input_state = digitalRead(BB);
	
	if (input_state == HIGH) {
		// If the current state is HIGH, period completed
		period[3] = start_time[3] - before_start_time[3];
		before_start_time[3] = start_time[3];
	}
	else {
		start_time[3] = micros();
	}
}

// Gets the period of signal Speed1
void getSpeed1Period ()
{
	// Read the input pin:
	input_state = digitalRead(SPEED1);
	
	if (input_state == HIGH) {
		// If the current state is HIGH, period completed
		period[4] = start_time[4] - before_start_time[4];
		before_start_time[4] = start_time[4];
	}
	else {
		start_time[4] = micros();
	}
}

// Gets the period of signal Speed2
void getSpeed2Period ()
{
	// Read the input pin:
	input_state = digitalRead(SPEED2);
	
	if (input_state == HIGH) {
		// If the current state is HIGH, period completed
		period[5] = start_time[5] - before_start_time[5];
		before_start_time[5] = start_time[5];
	}
	else {
		start_time[5] = micros();
	}
}

