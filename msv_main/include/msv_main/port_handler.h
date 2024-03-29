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

////////////////////////////////////////////////////////
/// @file Serial port handler through USB-TTL converter
/// @author Victor Esteban Sandoval-Luna
////////////////////////////////////////////////////////

#ifndef MSV_PORTHANDLER_H_
#define MSV_PORTHANDLER_H_

#include <iostream>
#include <cerrno>
#include <fcntl.h>
#include <string.h>
#include <cstring>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

namespace msv
{

class PortHandler
{
	private:
		// ATTRIBUTES
		char*   port_name_;
		int     socket_fd_;
		int     baudrate_;
		
		// METHODS
		
		// Set the attributes of the serial interface
		int setInterfaceAttribs (const int& fd, const int& baudrate, const int& parity);
		int setInterfaceAttribsArduino (const int& fd, const int& baudrate, const int& parity);
		
		// Default Baudrate
		const int DEFAULT_BAUDRATE = 115200;
		
	public:
		// Constructors
		PortHandler ();
		PortHandler (char* portname, const int& baudrate);
		
		virtual ~PortHandler() { }
		
		// Opens the serial port
		bool openPort ();
		bool openPortArduino ();
		
		// Closes the serial port
		void closePort ();
		
		// Clears up the port
		void clearPort ();
		
		// Sets the name of the port
		void setPortName (char* port_name);
		
		// Gets the name of the port
		char* getPortName ();
		
		// Sets the baudrate of the port
		int setBaudRate (const int& baudrate);
		
		// Gets the number of bits available for reading from the port buffer
		int getBytesAvailable ();
		
		// Reads buffer from port
		int readPort (uint8_t* packet, const int& length);
		
		// Reads byte from port
		int readPort (uint8_t* byte);
		
		// Writes buffer to port
		int writePort (uint8_t* packet, const int& length);
		
		// Writes byte to port
		int writePort (uint8_t* byte);
};

}

#endif
