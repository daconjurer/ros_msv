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
/// @file Arduino file for the MSV_01 "MSPS" board. This board is placed in the power
/// stage of the electronic interface (please refer to the technical document) and allows
/// the low current and voltage sensing within this stage. This board acts as a slave of
/// the BPS board (in the signal/control stage) and reports the sensors data through an
/// isolated serial interface. Sensors data is collected using non-isolated current and
/// voltage sensors attached to the ADC channels (0,3 and 4).
///
/// @author Victor Esteban Sandoval-Luna
///
/////////////////////////////////////////////////////////////////////////////////////////

// These defs should match the ones in the msv_bpt skecth
#define START_MSPS          0x01		// Start up instruction
#define START_COM           0x02		// Communication start instruction
#define STOP_COM            0x03		// Communication stop instruction
#define SEND_MSPS_DATA      0x04		// Message sending instruction
#define MSPS_READY          0x11		// MSPS ready ACK
#define COM_STARTED         0x12		// COM enabled ACK
#define COM_STOPPED         0x13		// COM disabled ACK

/* TODO */

// Pins to be used for the ADC reading
static unsigned int a0 = 0;
static unsigned int a3 = 0;
static unsigned int a4 = 0;

// Channels buffers for filtering
static unsigned short ch0 = 0;
static unsigned short ch3 = 0;
static unsigned short ch4 = 0;

// Serial communication variables
static unsigned char data[8] = {0};		// Data buffer (message to be sent to BPS)
static unsigned char b = 0;						// Byte buffer
static unsigned int start = 0;				// Routine enabling flag

void setup () {
	// Buffers for starting up the MSPS board
	unsigned char c = 0, d = 0;
	
	// Initialize serial communications at 9600 bps (USB cable compatible)
	Serial.begin(9600);
	
	pinMode(14,OUTPUT);
	digitalWrite(14,LOW);
	
	// Head and foot init of data buffer
	data[0] = 0xFF;
	data[7] = 0xFF;
	
	while (!Serial);
	// Then UART Ready
	
	do {
		while (!Serial.available());
		c = Serial.read();
		digitalWrite(14,HIGH);
		// START_MSPS condition received
		if (c == START_MSPS) {
			// Send READY ACK command
			Serial.write(MSPS_READY);
			digitalWrite(14,LOW);
			
			do {
				while (!Serial.available());
				d = Serial.read();
				digitalWrite(14,HIGH);
				
				// START_COM condition received
				if (d == START_COM) {
					// Send START_COM ACK
					Serial.write(COM_STARTED);
					digitalWrite(14,LOW);
					start = 1;
					break;
				}
			} while (d != START_COM);
			break;
		}
	} while (c != START_MSPS);
	digitalWrite(14,HIGH);
}

void loop () {
	// COM enabled condition checked
	if (start == 1) {
		/* Sensing loop */
		for (unsigned char j = 0; j < 5; j++) {
			// Read the analog values:
			a0 = analogRead(A0);
			a3 = analogRead(A3);
			a4 = analogRead(A4);
			
			ch0 += a0;
			ch3 += a3;
			ch4 += a4;
		}
		
		ch0 /= 5;
		ch3 /= 5;
		ch4 /= 5;
		// End of mean filter
		
		// Fill up the message with the sensors data
		data[1] = (unsigned char)(ch0 >> 8);
		data[2] = (unsigned char)(ch0 & 0x00FF);
		data[3] = (unsigned char)(ch3 >> 8);
		data[4] = (unsigned char)(ch3 & 0x00FF);
		data[5] = (unsigned char)(ch4 >> 8);
		data[6] = (unsigned char)(ch4 & 0x00FF);
		
		ch0 = 0;
		ch3 = 0;
		ch4 = 0;
		
		// Check whether a STOP_COM condition arrived
		if (Serial.available() > 0) {
			b = Serial.read();
			// STOP_COM condition received
			if (b == STOP_COM) {
				start = 0;
			} else if (b == SEND_MSPS_DATA) {
				digitalWrite(14,HIGH);
				for (unsigned char i = 0; i < 8; i++) {
					Serial.write(data[i]);
				}
				digitalWrite(14,LOW);
			}
		}
		
	} else {
		// Check whether a START_COM condition arrived
		while (!Serial.available()) {
			b = Serial.read();
			// START_COM condition received
			if (b == START_COM) {
				// Send START_COM ACK
				Serial.write(COM_STARTED);
				start = 1;
			}
		}
	}
}

