/**
 * @file   wts_driver.hpp
 * @author Chittaranjan S Srinivas
 *
 * @brief  This file declares the WTSDriver class.
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

#ifndef WTS_DRIVER_WTS_DRIVER_HPP_
#define WTS_DRIVER_WTS_DRIVER_HPP_

#include <boost/function.hpp>

#include <wts_driver/serial_comm.hpp>
#include <wts_driver/common.hpp>

#include <wts_driver/Frame.h>

namespace wts_driver {

class WTSDriver {
public:

  /**
   * Constructor.
   * \param serial_comm Reference to a SerialComm object.
   */
  WTSDriver(SerialComm& serial_comm);

  /**
   * Destructor.
   */
  virtual ~WTSDriver();

private:

  /**
   * A reference to a SerialComm object.
   */
  SerialComm& serial_comm_;

  /**
   * Matrix information.
   */
  int resolution_x, resolution_y;

  float cell_width, cell_height;

  int full_scale_output;

  SystemInfo system_info;


  /**
   * The CRC table used to calculate the checksum.
   */
  static const uint16_t crc_table[256];

public:
  // -------------------------- //
  // Data Acquisition Functions //
  // -------------------------- //

  /**
   * Read a single frame.
   */
  wts_error::error_type readSingleFrame(Frame& frame, bool compression = false);

  /**
   * Start periodic acquisition of frames.
   */
  wts_error::error_type startPeriodicFrameAcquisition(bool compression = false, uint16_t delay_ms = 0);

  /**
   * Stop any ongoing acquisition.
   */
  wts_error::error_type stopPeriodicFrameAcquisition();

  /**
   * Tare sensor matrix.
   */
  wts_error::error_type tareSensorMatrix(bool tare = true);

  /**
   * Untare sensor matrix.
   */
  inline wts_error::error_type untareSensorMatrix() { return tareSensorMatrix(false); }


  // ----------------- //
  // Matrix management //
  // ----------------- //

  /**
   * Get matrix information.
   * Saves the information in the class' members.
   */
  wts_error::error_type getMatrixInformation();

  /**
   * Get sensor type.
   */
  wts_error::error_type getSensorType(std::string& sensor_type);

  /**
   * Get device temperature.
   */
  wts_error::error_type readDeviceTemperature(int& temperature);

  /**
   * Get the system information.
   */
  wts_error::error_type getSystemInformation();

  /**
   * Set device tag.
   */
  wts_error::error_type setDeviceTag(const std::string& tag);

  /**
   * Get device tag.
   */
  wts_error::error_type getDeviceTag(std::string& device_tag);

  /**
   * Test Communication interface. Uses the LOOP command to test the interface.
   */
  bool testCommunicationInterface();

  /**
   * Display matrix information.
   */
  void displayMatrixInformation();

  /**
   * Display system information.
   */
  void displaySystemInformation();

private:

  //TODO: Must be private. For testing only.

  /**
   * Calculate/update the crc16 value.
   * \param data vector of bytes.
   * 'param crc Value calculated over another array or start value of the crc16 calculation.
   */
  static uint16_t calculateCRC(const std::vector<uint8_t>& data, uint16_t crc_prev = 0xFFFF);

  /**
   * Append the preamble, command id and size to the vector.
   */
  void appendPreambleCommandSize(const wts_command::command_type cmd_type, const uint16_t size, std::vector <uint8_t>& command_message);

  /**
   * Read Acknowledge Command.
   */
  wts_error::error_type readAcknowledgement(const wts_command::command_type cmd_type, std::vector <uint8_t>& returned_parameters);

};

} /* namespace wts */

#endif /* WTS_DRIVER_WTS_DRIVER_HPP_ */
