/*******************************************************************************
* Copyright 2018 Robótica de la Mixteca
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

/* Author: Victor Esteban Sandoval-Luna */

#include <msv_main/port_handler.h>

using namespace msv;

// Constructor
PortHandler::PortHandler () {
  port_name = (char*)"/dev/ttyUSB0";
  socket_fd = -1;
  setBaudRate(115200);
}

bool PortHandler::openPort () {
  socket_fd = open (port_name, O_RDWR | O_NOCTTY | O_SYNC);

  if (socket_fd < 0) {
    std::cout << "Error opening " << port_name << ": "<< std::strerror(errno) <<  std::endl;
    return false;
  }

  setInterfaceAttribs(socket_fd, baudrate_, 0);
  tcflush(socket_fd, TCIFLUSH);

  return true;
}

void PortHandler::closePort () {
  if(socket_fd != -1)
    close(socket_fd);
  socket_fd = -1;
}

void PortHandler::clearPort () {
  tcflush(socket_fd, TCIFLUSH);
}

void PortHandler::setPortName (const char* portname) {
  port_name = (char*)portname;
}

char* PortHandler::getPortName () {
  return port_name;
}

int PortHandler::setBaudRate (const int baudrate) {
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

int PortHandler::getBaudRate ()
{
  return remapBaudRate(baudrate_);
}

int PortHandler::getBytesAvailable () {
  int bytes_available;
  ioctl(socket_fd, FIONREAD, &bytes_available);
  return bytes_available;
}

int PortHandler::readPort (uint8_t *packet, int length) {
  return read(socket_fd, packet, length);
}

int PortHandler::readPort (uint8_t *byte) {
  return read(socket_fd, byte, 1);
}

int PortHandler::writePort (uint8_t *packet, int length) {
  return write(socket_fd, packet, length);
}

int PortHandler::writePort (uint8_t *byte) {
  return write(socket_fd, byte, 1);
}

int PortHandler::setInterfaceAttribs (int fd, int baudrate, int parity) {
  struct termios tty;

  memset(&tty,0,sizeof(tty));
  tty.c_iflag = 0;
  tty.c_oflag = 0;
  tty.c_cflag = CS8|CREAD|CLOCAL;           // 8n1, see termios.h for more information
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 5;

  cfsetospeed(&tty,baudrate);            // 115200 baud
  cfsetispeed(&tty,baudrate);            // 115200 baud

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    std::cout << "Error " << std::strerror(errno) << " from tcsetattr.\n";
    return -1;
  }

  if(!isatty(fd)) {
    std::cout << "Error " << std::strerror(errno) << " from isatty.\n";
    return -1;
  }

  if (tcgetattr(fd, &tty) != 0) {
    std::cout << "Error " << std::strerror(errno) << " from tcgetattr.\n";
    return -1;
  }

  return 0;
}

int PortHandler::remapBaudRate (const int baudrate) {
  switch (baudrate) {
    case B9600:
      return 9600;
    case B19200:
      return 19200;
    case B38400:
      return 38400;
    case B57600:
      return 57600;
    case B115200:
      return 115200;
    case B230400:
      return 230400;
    case B460800:
      return 460800;
    default:
      return -1;
  }
}
