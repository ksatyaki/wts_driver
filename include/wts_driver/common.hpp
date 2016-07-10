/**
 * @file   common.hpp
 * @author Chittaranjan S Srinivas
 *
 * @brief  This file declares various constants and classes used in
 * the driver.
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

#ifndef WTS_DRIVER_COMMON_HPP_
#define WTS_DRIVER_COMMON_HPP_

#include <stdint.h>
#include <string>
#include <vector>
#include <stdio.h>

namespace wts_driver {

class wts_error {

public:

  std::string message();

  enum error_type {

    E_SUCCESS = 0,            //!< No error
    E_NOT_AVAILABLE,          //!< Device, service or data is not available
    E_NO_SENSOR,              //!< No sensor connected
    E_NOT_INITIALIZED,        //!< The device is not initialized
    E_ALREADY_RUNNING,        //!< Service is already running
    E_FEATURE_NOT_SUPPORTED,  //!< The asked feature is not supported
    E_INCONSISTENT_DATA,      //!< One or more dependent parameters mismatch
    E_TIMEOUT,                //!< Timeout error
    E_READ_ERROR,             //!< Error while reading from a device
    E_WRITE_ERROR,            //!< Error while writing to a device
    E_INSUFFICIENT_RESOURCES, //!< No memory available
    E_CHECKSUM_ERROR,         //!< Checksum error
    E_NO_PARAM_EXPECTED,      //!< No parameters expected
    E_NOT_ENOUGH_PARAMS,      //!< Not enough parameters
    E_CMD_UNKNOWN,            //!< Unknown command
    E_CMD_FORMAT_ERROR,       //!< Command format error
    E_ACCESS_DENIED,          //!< Access denied
    E_ALREADY_OPEN,           //!< The interface is already open
    E_CMD_FAILED,             //!< Command failed
    E_CMD_ABORTED,            //!< Command aborted
    E_INVALID_HANDLE,         //!< invalid handle
    E_NOT_FOUND,              //!< device not found
    E_NOT_OPEN,               //!< device not open
    E_IO_ERROR,               //!< I/O error
    E_INVALID_PARAMETER,      //!< invalid parameter
    E_INDEX_OUT_OF_BOUNDS,    //!< index out of bounds
    E_CMD_PENDING,            //!< Command execution needs more time
    E_OVERRUN,                //!< Data overrun
    E_RANGE_ERROR,            //!< Range error
    E_AXIS_BLOCKED,           //!< Axis is blocked
    E_FILE_EXISTS,             //!< File already exists
    E_OTHER
  } error_type_;

  wts_error();

  wts_error(error_type err);

  bool operator == (error_type err);

};

class wts_command {

public:

  enum command_type {

    // Periodic frame data.
    FRAME_DATA = 0x00,

    // Data Acquisition
    READ_SINGLE_FRAME = 0x20,
    START_PERIODIC_FRAME_ACQ = 0x21,
    STOP_PERIODIC_FRAME_ACQ = 0x22,
    TARE_SENSOR_MATRIX = 0x23,

    // Matrix Management
    GET_MATRIX_INFO = 0x30,
    SET_ACQ_MASK_WINDOW = 0x31,
    SET_ADVANCED_ACQ_MASK = 0x32,
    GET_ACQ_MASK = 0x33,
    SET_THRESHOLD = 0x34,
    GET_THRESHOLD = 0x35,
    SET_FRONT_END_GAIN = 0x36,
    GET_FRONT_END_GAIN = 0x37,
    GET_SENSOR_TYPE = 0x38,

    // System configuration and state.
    READ_DEVICE_TEMPERATURE = 0x46,
    GET_SYSTEM_INFO = 0x50,
    SET_DEVICE_TAG = 0x51,
    GET_DEVICE_TAG = 0x52,

    // Connection Management
    LOOP = 0x06

  };

};

struct SystemInfo {

  /**
   * The type of sensor. Could be unknown or WTS Sensor Module.
   */
  std::string type;

  /**
   * The firmware version.
   */
  std::string firmware_version;

  /**
   * The hardware revision number.
   */
  std::string hw_rev;

  /**
   * Serial number of the device we are talking to.
   */
  int serial_number;

  /**
   * Device tag.
   */
  std::string device_tag;

  /**
   * Print out this struct. Only for debugging.
   */
  void display();

  /**
   * Empty constructor.
   */
  SystemInfo();

  /**
   * Create a SystemInfo struct from the data packets received via serial port.
   */
  SystemInfo(const std::vector <uint8_t>& data_packet);

  /**
   * Create a SystemInfo struct from the different parts.
   */
  SystemInfo(const std::string& type_, const std::string& firmware_version_, const std::string& hw_rev_, const int serial_number);

};

struct MatrixInfo {

  int resolution_x, resolution_y;

  float cell_width, cell_height;

  int full_scale_output;

  /**
   * Print out this struct. Only for debugging.
   */
  void display();

};

class ReceivedUnexpectedCommandIDException : std::exception {
  virtual inline const char* what() const throw() {
    return "Received unexpected command ID as response from the microcontroller.";
  }
};

}




#endif /* WTS_DRIVER_COMMON_HPP_ */
