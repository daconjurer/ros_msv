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

/* Author: Victor Esteban Sandoval-Luna */

#include <msv_main/port_handler.h>

using namespace msv;

// Constructor
PortHandler::PortHandler () 
{
	port_name_ = (char*)"/dev/ttyUSB0";
	socket_fd_ = -1;
	setBaudRate(115200);
}

PortHandler::PortHandler (char* portname, int baudrate) 
{
	port_name_= portname;
	socket_fd_ = -1;
	setBaudRate(baudrate);
}

bool PortHandler::openPort () 
{
	socket_fd_ = open (port_name_, O_RDWR | O_NOCTTY | O_SYNC);
	
	if (socket_fd_ < 0) {
		std::cout << "Error opening " << port_name_ << ": " << std::strerror(errno) <<  std::endl;
		return false;
	}
	
	setInterfaceAttribs(socket_fd_, baudrate_, 0);
	tcflush(socket_fd_, TCIFLUSH);
	
	return true;
}

bool PortHandler::openPortArduino () 
{
	socket_fd_ = open(port_name_, O_RDWR | O_NOCTTY);
	
	if (socket_fd_ < 0) {
		std::cout << "Error opening " << port_name_ << ": "<< std::strerror(errno) <<  std::endl;
		return false;
	}
	
	setInterfaceAttribsArduino(socket_fd_, baudrate_, 0);
	tcflush(socket_fd_, TCIFLUSH);
	
	return true;
}

void PortHandler::closePort () 
{
	if(socket_fd_ != -1) {close(socket_fd_);}
	socket_fd_ = -1;
}

void PortHandler::clearPort () 
{
	tcflush(socket_fd_, TCIFLUSH);
}

void PortHandler::setPortName (const char* portname) 
{
	port_name_ = (char*)portname;
}

char* PortHandler::getPortName () 
{
	return port_name_;
}

int PortHandler::setBaudRate (const int baudrate) 
{
	// Considering both Hovis HerkuleX servos limits and TTL hardware limits (up to 500000 bps)
	int br = -1;
	
	switch (baudrate) {
		case 9600:
			baudrate_ = B9600;
			br = 0;
		case 19200:
			baudrate_ = B19200;
			br = 0;
		case 38400:
			baudrate_ = B38400;
			br = 0;
		case 57600:
			baudrate_ = B57600;
			br = 0;
		case 115200:
			baudrate_ = B115200;
			br = 0;
		case 230400:
			baudrate_ = B230400;
			br = 0;
		case 460800:
			baudrate_ = B460800;
			br = 0;
		default:
			baudrate_ = B115200;
			br = 0;
	}
	
	if (br == -1) {
		std::cout << "Error setting baudrate: Invalid baudrate." << std::endl;
		return br;
	}
	
	return baudrate;
}

int PortHandler::getBytesAvailable () 
{
	int bytes_available;
	ioctl(socket_fd_, FIONREAD, &bytes_available);
	return bytes_available;
}

int PortHandler::readPort (uint8_t *packet, int length) 
{
	return read(socket_fd_, packet, length);
}

int PortHandler::readPort (uint8_t *byte) 
{
	return read(socket_fd_, byte, 1);
}

int PortHandler::writePort (uint8_t *packet, int length) 
{
	return write(socket_fd_, packet, length);
}

int PortHandler::writePort (uint8_t* byte) 
{
	return write(socket_fd_, byte, 1);
}

int PortHandler::setInterfaceAttribs (int fd, int baudrate, int parity) 
{
	struct termios tty;
	
	if (tcgetattr(fd, &tty) < 0) {
		std::cout << "Error getting term attributes of " << port_name_ << ": " << std::strerror(errno) <<  std::endl;;
		return -1;
	}
	
	cfsetispeed(&tty, baudrate);
	cfsetospeed(&tty, baudrate);
	
	// cflag
	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;		// 8-bit chars
	tty.c_cflag |= (CLOCAL | CREAD);	// ignore modem controls, enable reading
	tty.c_cflag &= ~(PARENB | PARODD);	// shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;
	
	// iflag
	tty.c_iflag &= ~IGNBRK;		// disable break processing
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);		// shut off xon/xoff ctrl
	
	// lflag
	tty.c_lflag = 0;	// no signaling chars, no echo,non-canonical processing
	
	// oflag
	tty.c_oflag = 0;	// no remapping, no delays
	
	// cc
	tty.c_cc[VMIN]  = 0;	// read doesn't block
	tty.c_cc[VTIME] = 5;	// 0.5 seconds read timeout
	
	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		std::cout << "Error " << std::strerror(errno) << " from tcsetattr.\n";
	return -1;
	}
	
	if (!isatty(fd)) {
		std::cout << "Error " << std::strerror(errno) << " from isatty.\n";
		return -1;
	}
	
	if (tcgetattr(fd, &tty) != 0) {
		std::cout << "Error " << std::strerror(errno) << " from tcgetattr.\n";
		return -1;
	}
	
	return 0;
}

int PortHandler::setInterfaceAttribsArduino (int fd, int baudrate, int parity) 
{
	struct termios tty;
	
	if (tcgetattr(fd, &tty) < 0) {
		std::cout << "Error getting term attributes of " << port_name_ << ": " << std::strerror(errno) <<  std::endl;;
		return -1;
	}
	
	cfsetispeed(&tty, baudrate);
	cfsetospeed(&tty, baudrate);
	
	// cflag
	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;
	tty.c_cflag &= ~CRTSCTS;	// no flow control
	tty.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
	
	// iflag
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
	
	// lflag
	tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
	
	//oflag
	tty.c_oflag &= ~OPOST; // make raw
	
	// cc
	tty.c_cc[VMIN]  = 1;
	tty.c_cc[VTIME] = 0;
	
	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		std::cout << "Error " << std::strerror(errno) << " from tcsetattr.\n";
		return -1;
	}
	
	if (!isatty(fd)) {
		std::cout << "Error " << std::strerror(errno) << " from isatty.\n";
		return -1;
	}
	
	if (tcgetattr(fd, &tty) != 0) {
		std::cout << "Error " << std::strerror(errno) << " from tcgetattr.\n";
		return -1;
	}
	
	return 0;
}

