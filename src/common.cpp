/**
 * @file   common.cpp
 * @author Chittaranjan S Srinivas
 *
 * @brief  This file defines various constants and classes used in
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

#include <wts_driver/common.hpp>

namespace wts_driver {

wts_error::wts_error() {
  error_type_ = E_SUCCESS;
}

wts_error::wts_error(error_type err) :
          error_type_(err) {}

bool wts_error::operator == (error_type err) {
  return this->error_type_ == err;
}

std::string wts_error::message () {

  switch(this->error_type_) {
  case E_SUCCESS:
    return "Success";
  case E_NOT_AVAILABLE:
    return "Device, service or data is not available";
  case E_NO_SENSOR:
    return "No sensor connected";
  case E_NOT_INITIALIZED:
    return "The device is not initialized";
  case E_ALREADY_RUNNING:
    return "Service is already running";
  case E_FEATURE_NOT_SUPPORTED:
    return "The asked feature is not supported";
  case E_INCONSISTENT_DATA:
    return "One or more dependent parameters mismatch";
  case E_TIMEOUT:
    return "Timeout error";
  case E_READ_ERROR:
    return "Error while reading from a device";
  case E_WRITE_ERROR:
    return "Error while writing to a device";
  case E_INSUFFICIENT_RESOURCES:
    return "No memory available";
  case E_CHECKSUM_ERROR:
    return "Checksum error";
  case E_NO_PARAM_EXPECTED:
    return "No parameters expected";
  case E_NOT_ENOUGH_PARAMS:
    return "Not enough parameters";
  case E_CMD_UNKNOWN:
    return "Unknown command";
  case E_CMD_FORMAT_ERROR:
    return "Command format error";
  case E_ACCESS_DENIED:
    return "Access denied";
  case E_ALREADY_OPEN:
    return "The interface is already open";
  case E_CMD_FAILED:
    return "Command failed";
  case E_CMD_ABORTED:
    return "Command aborted";
  case E_INVALID_HANDLE:
    return "invalid handle";
  case E_NOT_FOUND:
    return "device not found";
  case E_NOT_OPEN:
    return "device not open";
  case E_IO_ERROR:
    return "I/O error";
  case E_INVALID_PARAMETER:
    return "invalid parameter";
  case E_INDEX_OUT_OF_BOUNDS:
    return "index out of bounds";
  case E_CMD_PENDING:
    return "Command execution needs more time";
  case E_OVERRUN:
    return "Data overrun";
  case E_RANGE_ERROR:
    return "Range error";
  case E_AXIS_BLOCKED:
    return "Axis is blocked";
  case E_FILE_EXISTS:
    return "File exists";
  case E_OTHER:
    return "Some other type of error occured. Maybe periodic frame acquisition is running?";
  default:
    return "Unknown error";
  }

}

SystemInfo::SystemInfo(){}

SystemInfo::SystemInfo(const std::vector <uint8_t>& data_packet) {

  type = data_packet[0] == 0 ? "Unknown" : "WTS Tactile Sensor Module" ;

  char hw_rev_c_str[4];
  sprintf(hw_rev_c_str, "%hhu", data_packet[1]);

  hw_rev = std::string(hw_rev_c_str);

  serial_number = (data_packet[4]) | (data_packet[5] << 8) | (data_packet[6] << 16) | (data_packet[7] << 24);

  uint8_t major_version = (data_packet[3] >> 4);
  uint8_t minor_version1 = (data_packet[3] & 0x0F);
  uint8_t minor_version2 = (data_packet[2] >> 4);
  uint8_t version_type = (data_packet[2] & 0x0F);

  char version[100];
  sprintf(version, "%hhu.%hhu.%hhu.%hhu", major_version, minor_version1, minor_version2, version_type);

  firmware_version = std::string(version);

}

SystemInfo::SystemInfo(const std::string& type_, const std::string& firmware_version_, const std::string& hw_rev_, const int serial_number_) :
    type(type_),
    firmware_version(firmware_version_),
    hw_rev(hw_rev_),
    serial_number(serial_number_) {

}

void SystemInfo::display() {
  ROS_INFO("Type: %s", type.c_str());
  ROS_INFO("Firmware Version: %s", firmware_version.c_str());
  ROS_INFO("Hardware revision: %s", hw_rev.c_str());
  ROS_INFO("Serial Number: %d", serial_number);
}

void MatrixInfo::display() {
  ROS_INFO("Resolution X: %d", resolution_x);
  ROS_INFO("Resolution Y: %d", resolution_y);
  ROS_INFO("Cell Width: %f m", cell_width);
  ROS_INFO("Cell Height %f m", cell_height);
  ROS_INFO("Full Scale Output: %d m", full_scale_output);
}



}
