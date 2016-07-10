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

#include <ros/ros.h>
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

  /**
   * Initialize ros publisher.
   */
  void initROSPublisher(ros::NodeHandle& nh);

private:

  /**
   * ROS publisher for frame information.
   */
  ros::Publisher frames_pub_;

  /**
   * A reference to a SerialComm object.
   */
  SerialComm& serial_comm_;

  /**
   * Matrix information.
   */
  MatrixInfo matrix_info;

  /**
   * System information.
   */
  SystemInfo system_info;

  /**
   * Is periodic frame acquisition on?
   */
  bool periodic_frame_acq_is_running;

  /**
   * A static array for async_read of preamble, command id and size.
   */
  boost::array <uint8_t, 6> in_preamble_cmd_size;

  /**
   * A dynamic array to hold the frame data.
   */
  std::vector <uint8_t> in_frame_data;

  /**
   * The CRC table used to calculate the checksum.
   */
  static const uint16_t crc_table[256];

public:

  inline bool isPeriodicFrameAcqRunning() { return periodic_frame_acq_is_running; }
  // -------------------------- //
  // Data Acquisition Functions //
  // -------------------------- //

  /**
   * Read a single frame.
   */
  wts_error readSingleFrame(Frame& frame, const bool compression = false);

  /**
   * Start periodic acquisition of frames.
   */
  wts_error startPeriodicFrameAcquisition(const bool compression = false, const uint16_t delay_ms = 0);

  /**
   * Stop any ongoing acquisition.
   */
  wts_error stopPeriodicFrameAcquisition();

  /**
   * Tare sensor matrix.
   */
  wts_error tareSensorMatrix(const bool tare = true);

  /**
   * Untare sensor matrix.
   */
  inline wts_error untareSensorMatrix() { return tareSensorMatrix(false); }


  // ----------------- //
  // Matrix management //
  // ----------------- //

  /**
   * Get matrix information.
   * Saves the information in the class' members.
   */
  wts_error getMatrixInformation();

  /**
   * Get sensor type.
   */
  wts_error getSensorType(std::string& sensor_type);

  /**
   * Get device temperature.
   */
  wts_error readDeviceTemperature(int& temperature);

  /**
   * Get the system information.
   */
  wts_error getSystemInformation();

  /**
   * Set device tag.
   */
  wts_error setDeviceTag(const std::string& tag);

  /**
   * Get device tag.
   */
  wts_error getDeviceTag();

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

  /**
   * Serial port callback when periodic frame acquisition is enabled.
   */
  void preambleCommandSizeCallback(const boost::system::error_code& error);

  void frameMessageCallback(const boost::system::error_code& error);

  void otherMessageCallback(const boost::system::error_code& error);

};

} /* namespace wts */

#endif /* WTS_DRIVER_WTS_DRIVER_HPP_ */
