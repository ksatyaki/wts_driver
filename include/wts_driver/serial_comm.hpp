/**
 * @file   serial_comm.hpp
 * @author Chittaranjan S Srinivas
 *
 * @brief  This file declares the SerialComm class.
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

#ifndef SERIAL_COMM_HPP_
#define SERIAL_COMM_HPP_

#include <boost/asio.hpp>

namespace wts_driver {

class SerialComm {
public:
  /**
   * Constructor.
   * \param port Device name, example "/dev/ttyACM0" or "COM4"
   * \param baud_rate Communication speed, example 9600 or 115200
   * \throws boost::system::system_error if cannot open the
   * serial device
   */
  SerialComm (std::string port, unsigned int baud_rate);

  /**
   * Write an array of bytes to the serial port - blocking call.
   * \param bytesToWrite The vector of bytes to write to the serial port.
   * \returns false on failure and true on success.
   */
  bool writeBytes (const std::vector <uint8_t>& bytesToWrite);

  /**
   * Write any fundamental data-type to the serial port. Blocking call.
   * \param dataToWrite The object to write to the serial port.
   * \returns false on failure and true on success.
   */
  template <typename T>
  bool writeToSerialPort (const T& dataToWrite);

  /**
    * Read any fundamental data-type from the serial port. Blocking call.
    * \param dataToWrite The object to read from the serial port.
    * \returns false on failure and true on success.
    */
   template <typename T>
   bool readFromSerialPort (T& dataRead);

  /**
   * Write a ConstBufferSequence to the serial port.
   */
  bool writeConstBufferSequence (const std::vector <boost::asio::const_buffer>& buffersToWrite);

  /**
   * Read a MutableBufferSequence from the serial port.
   */
  bool readMutableBufferSequence (std::vector <boost::asio::mutable_buffer>& buffersRead);

  /**
   * Read a vector of bytes from the serial port - blocking call.
   * \param bytesRead The vector of bytes that were read from the serial port.
   * \returns false on failure and true on success.
   */
  bool readBytes (std::vector <uint8_t>& bytesRead);

  ~SerialComm ();

  /**
   * Return the serial port handle as a reference.
   */
  inline boost::asio::serial_port& serial() { return serial_; }

  /**
   * Return the serial port handle as a reference.
   */
  inline boost::asio::io_service& io_service() { return io_service_; }

private:

  boost::asio::io_service io_service_;
  boost::asio::serial_port serial_;

};

template <typename T>
bool SerialComm::writeToSerialPort (const T& dataToWrite) {

  boost::system::error_code err;

  //std::cout << "\nAttempting to send " << sizeof(dataToWrite) <<  " bytes...";
  boost::asio::write(serial_, boost::asio::buffer(&dataToWrite, sizeof(dataToWrite)), err);

  //std::cout << "\nGot error: " << err.message();
  // 0 is success. Return false if it is not zero.
  if(err != 0) return false;
  else return true;

}

template <typename T>
bool SerialComm::readFromSerialPort (T& dataRead) {

  boost::system::error_code err;

  //std::cout << "\nAttempting to read " << sizeof(dataRead) <<  " bytes...";
  boost::asio::read(serial_, boost::asio::buffer(&dataRead, sizeof(dataRead)), err);

  //std::cout << "\nGot error: " << err.message();
  // 0 is success. Return false if it is not zero.
  if(err != 0) return false;
  else return true;

}


}







#endif /* SERIAL_COMM_HPP_ */
