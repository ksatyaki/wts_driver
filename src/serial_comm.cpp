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


namespace wts {

  SerialComm::SerialComm(std::string port, unsigned int baud_rate) :
    io(),
    serial(io,port) {
    serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
  }

}

  /**
   * Write a string to the serial device.
   * \param s string to write
   * \throws boost::system::system_error on failure
   */

  void writeCommand(uint8_t command)
  {

    boost::system::error_code err;

    std::cout << "\nAttempting to send a byte...";

    serial.write_some(boost::asio::buffer(&command,sizeof(uint8_t)),err);

    std::cout << "\nGot error: " << err.message();
    std::flush(std::cout);
  }

  /**
   * Blocks until a line is received from the serial device.
   * Eventual '\n' or '\r\n' characters at the end of the string are removed.
   * \return a string containing the received line
   * \throws boost::system::system_error on failure
   */
  boost::array<unsigned char,10> readByte()
  {
    //Reading data char by char, code is optimized for simplicity, not speed
    using namespace boost;
    boost::array<unsigned char,10> c;
    serial.read_some(asio::buffer(&c,sizeof(c)));
    return c;
  }

  ~SimpleSerial() {
    serial.close();
  }

private:
  boost::asio::io_service io;
  boost::asio::serial_port serial;
};

int main (int argn, char* args[]) {

  SimpleSerial serial("/dev/ttyACM0", 115200);

  uint8_t command;

  //while(true) {
    std::cout << "\nEnter a command: ";
    std::cin >> command;

    //if(command == 0) break;

    try {
      serial.writeCommand(0xAA);

      serial.writeCommand(0xAA);
      serial.writeCommand(0xAA);

      serial.writeCommand(0x50);

      serial.writeCommand(0x00);
      serial.writeCommand(0x00);

      serial.writeCommand(0xdb);
      serial.writeCommand(0x87);

      //serial.writeCommand(command);


      boost::array<unsigned char, 10>  reply;
      reply.fill(00);
      reply = serial.readByte();
      std::cout << "\n Received reply: ";
      for(int i = 0; i < 10; i++) {
        printf("%x, ", reply[i]);
      }


    } catch(boost::system::system_error& e)
    {
      std::cout<<"\n\nError: "<<e.what()<<std::endl;
      return 1;
    }
 // }

  std::cout << "\nBye";
  return 0;
}



