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
/// @file Arduino file for the MSV_01 "ARDI" board. This board is based on the Arduino
/// NANO and it is intended to send the IMU MPU6050 data to the on-board computer
/// through UART. The sketch is based on the generic MPU6050 sketch and the data
/// is sent following the Invensense teapot packet structure.
/// @author Victor Esteban Sandoval-Luna
///
/// The I2C communication is implemented using the I2Cdev library from Jeff Rowberg.
/// Please see https://github.com/jrowberg/i2cdevlib
///
/// The MPU6050 data is processed using the MPU6050_6Axis_MotionApps20 library,
/// part of the I2Cdev library.
///
/////////////////////////////////////////////////////////////////////////////////////////

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required by I2Cdev.h (I2C/TWI communication for Arduino)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

#define LED_PIN                 13
#define INTERRUPT_PIN           2

// DMP error definitions
#define INITMEMLOAD_ERROR       0x71
#define DMPCONFUPDATE_ERROR     0x72
#define OVERFLOW_ERROR          0x73

// Serial communication definitions
#define IMU_INIT                0x01
#define IMU_TEST                0x02
#define IMU_TEST_SUCC           0x50
#define IMU_TEST_FAIL           0x51

#define DMP_INIT                0x60
#define DMP_ENABLE              0x61
#define INT_ENABLE              0x62
#define DMP_READY               0x63

#define COM_START               0x0A
#define COM_STOP                0x0B
#define OBC_START               0x10
#define ARDI_READY              0x20
#define SEND_ARDI_DATA          0x30

static MPU6050 mpu;
static bool blinkState = false;
static uint8_t com_flag = 0;
static uint8_t obc_command = 0;

// MPU control/status vars
static bool dmpReady = false;   // set true if DMP init was successful
static uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
static uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
static uint16_t fifoCount;      // count of all bytes currently in FIFO
static uint8_t fifoBuffer[64];  // FIFO storage buffer

// Packet structure for InvenSense teapot demo
static uint8_t teapotPacket[28] = {'$',0x03,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x00,0x00,'\r','\n'};

// Indicates whether the MPU interrupt pin has gone high or not
static volatile bool mpuInterrupt = false;

// Interrupt service routine (ISR) activation function
static void dmpDataReady ()
{
	mpuInterrupt = true;
}

void setup ()
{
	// Join I2C bus (Not performed by I2Cdev in automatic)
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		Wire.begin();
		TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
		Fastwire::setup(400, true);
	#endif
	
	// Initialize serial communication
	Serial.begin(115200);
	while (!Serial);
	
	// configure LED for output
	pinMode(LED_PIN, OUTPUT);
	
	static uint8_t obc_ack[4] = {0};
	static uint8_t c = 0;
	static uint8_t f = 0;
	static uint8_t i_obc = 0;
	static uint8_t init_query[3] = {0};
	
	// Wait for the on-board computer Setup instruction (OBC_START)
	while (f == 0) {
		if (Serial.available()) {
			c = Serial.read();
			digitalWrite(14,LOW);
			
			if (c == OBC_START) {
				// Once finished
				digitalWrite(14,LOW);
				// Build up ACK
				obc_ack[0] = ARDI_READY;  
				// Initialize the MPU6050 and inform the on-board computer
				obc_ack[1] = IMU_INIT;
				mpu.initialize();
				pinMode(INTERRUPT_PIN, INPUT);
				
				// Test I2C connection
				obc_ack[2] = IMU_TEST;
				obc_ack[3] = mpu.testConnection() ? IMU_TEST_SUCC : IMU_TEST_FAIL;
				
				for (int i = 0; i < 4; i++) {
					Serial.write(obc_ack[i]);
				}
				c = 0;
				f = 1;
			}
		}
	}
	
	f = 0;
	// Wait for the on-board computer communication start instruction (COM_START)
	while (f == 0) {
		if (Serial.available()) {
			c = Serial.read();
			digitalWrite(14,LOW);
			
			if (c == COM_START) {
				// Initialize the DMP (Digital Motion Processor)
				uint8_t dmpStatus = mpu.dmpInitialize();
				obc_ack[0] = DMP_INIT;
				
				// Load gyro offsets (may vary)
				mpu.setXGyroOffset(220);
				mpu.setYGyroOffset(76);
				mpu.setZGyroOffset(-85);
				mpu.setZAccelOffset(1788);
				
				// Make sure the initializing worked (returns 0 if so)
				if (dmpStatus == 0) {
					// turn on the DMP, now that it's ready
					mpu.setDMPEnabled(true);
					obc_ack[1] = DMP_ENABLE;
					
					// Enable Arduino interrupt detection
					attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
					obc_ack[2] = INT_ENABLE;
					mpuIntStatus = mpu.getIntStatus();
					
					// Enable the DMP Ready flag so the loop() function knows the DMP is ready
					dmpReady = true;
					obc_ack[3] = DMP_READY;
					
					// get expected DMP packet size for later comparison
					packetSize = mpu.dmpGetFIFOPacketSize();
				} else {
					// ERROR!
					// 1 = initial memory load failed
					// 2 = DMP configuration updates failed
					// (if it's going to break, usually the code will be 1)
					if (dmpStatus == 1) {
						obc_ack[0] = INITMEMLOAD_ERROR;
					}
					if (dmpStatus == 2) {
						obc_ack[0] = DMPCONFUPDATE_ERROR;
					}
					
					obc_ack[1] = 0x00; obc_ack[2] = 0x00; obc_ack[3] = 0x00;
				}
				
				for (int j = 0; j < 4; j++) {
					Serial.write(obc_ack[j]);
				}
				
				// Enable communication and leave init loops
				com_flag = 1;
				f = 1;
			}
		}
	}
}

void loop ()
{
	// Leave if programming failed
	if (!dmpReady) {
		return;
	}
	
	// Wait for interrupt or extra packet(s) available
	while (!mpuInterrupt && fifoCount < packetSize);
	
	// Reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();
	
	// Get current FIFO count
	fifoCount = mpu.getFIFOCount();
	
	// Check for overflow (it should never happen)
	if (((mpuIntStatus & 0x10) || fifoCount == 1024) && com_flag == 1) {
		// Reset for a clean new process
		mpu.resetFIFO();
		Serial.write(OVERFLOW_ERROR);
		
		// Otherwise, check for DMP data ready interrupt (this should happen frequently)
	} else if (mpuIntStatus & 0x02 && com_flag == 1) {
		// Wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) {
			fifoCount = mpu.getFIFOCount();
		}
		
		// Read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);
		
		// Track FIFO count in case there is more than one packet available
		// (this allows immediate reading without waiting for an interrupt)
		fifoCount -= packetSize;
		
		/* OUTPUTS */
		
		// Quaternion values in InvenSense Teapot demo format:
		teapotPacket[2] = fifoBuffer[0];
		teapotPacket[3] = fifoBuffer[1];
		teapotPacket[4] = fifoBuffer[4];
		teapotPacket[5] = fifoBuffer[5];
		teapotPacket[6] = fifoBuffer[8];
		teapotPacket[7] = fifoBuffer[9];
		teapotPacket[8] = fifoBuffer[12];
		teapotPacket[9] = fifoBuffer[13];
		// Gyro values
		teapotPacket[10] = fifoBuffer[16];
		teapotPacket[11] = fifoBuffer[17];
		teapotPacket[12] = fifoBuffer[20];
		teapotPacket[13] = fifoBuffer[21];
		teapotPacket[14] = fifoBuffer[24];
		teapotPacket[15] = fifoBuffer[25];
		// Accelerometer values
		teapotPacket[16] = fifoBuffer[28];
		teapotPacket[17] = fifoBuffer[29];
		teapotPacket[18] = fifoBuffer[32];
		teapotPacket[19] = fifoBuffer[33];
		teapotPacket[20] = fifoBuffer[36];
		teapotPacket[21] = fifoBuffer[37];
		// Temperature
		int16_t temperature = mpu.getTemperature();
		teapotPacket[22] = temperature >> 8;
		teapotPacket[23] = temperature & 0xFF;
		
		if (Serial.available()) {
			obc_command = Serial.read();
			
			if (obc_command == SEND_ARDI_DATA) {
				// Finish
				Serial.write(teapotPacket,28);
			}
		}
		
		teapotPacket[25]++; // packetCount, loops at 0xFF on purpose
		
		// Blink LED to indicate activity
		blinkState = !blinkState;
		digitalWrite(LED_PIN, blinkState);
	}
}

