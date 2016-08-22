/**
 * @file   serial_comm.cpp
 * @author Chittaranjan S Srinivas
 *
 * @brief  This file defines the SerialComm class.
 *
 * Copyright (C) 2016  Chittaranjan Srinivas Swaminathan
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 *
 */

#include <wts_driver/serial_comm.hpp>


namespace wts_driver {

SerialComm::SerialComm(std::string port, unsigned int baud_rate) :
        io_service_(),
        serial_(io_service_,port) {
  serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
}

bool SerialComm::writeBytes (const std::vector <uint8_t>& bytesToWrite) {

  boost::system::error_code err;

  //std::cout << "\nAttempting to send " << sizeof(bytesToWrite) << " bytes...";
  boost::asio::write(serial_, boost::asio::buffer(bytesToWrite), err);

  //std::cout << "\nGot error: " << err.message();
  // 0 is success. Return false if it is not zero.
  if(err != 0) return false;
  else return true;
}

bool SerialComm::writeConstBufferSequence (const std::vector <boost::asio::const_buffer>& buffersToWrite) {

  boost::system::error_code err;

  //std::cout << "\nAttempting to send a packet.";
  boost::asio::write(serial_, buffersToWrite, err);

  //std::cout << "\nGot error: " << err.message();
  // 0 is success. Return false if it is not zero.
  if(err != 0) return false;
  else return true;
}

bool SerialComm::readMutableBufferSequence (std::vector <boost::asio::mutable_buffer>& buffersRead) {
  boost::system::error_code err;

  //std::cout << "\nAttempting to read a packet.";
  boost::asio::read(serial_, buffersRead, err);

  //std::cout << "\nGot error: " << err.message();
  // 0 is success. Return false if it is not zero.
  if(err != 0) return false;
  else return true;
}

bool SerialComm::readBytes (std::vector <uint8_t>& bytesRead) {

  boost::system::error_code err;

  //std::cout << "\nAttempting to read " << bytesRead.size() << " bytes...";
  std::size_t s = boost::asio::read(serial_, boost::asio::buffer(bytesRead), err);

  //std::cout << "\nRead " << s << " bytes. Got error: " << err.message();
  // 0 is success. Return false if it is not zero.
  if(err != 0) return false;
  else return true;

}

SerialComm::~SerialComm() {
  //std::cout << "\nClosing the serial port.\n";
  serial_.close();
}

} // namespace wts.




