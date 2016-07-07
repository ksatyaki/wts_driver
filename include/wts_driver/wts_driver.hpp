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

#include <wts_driver/serial_comm.hpp>

namespace wts {

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
   * The CRC table used to calculate the checksum.
   */
  static const uint16_t crc_table[256];

public:

  //TODO: Must be private. For testing only.

  /**
   * Calculate/update the crc16 value.
   * \param data vector of bytes.
   * 'param crc Value calculated over another array or start value of the crc16 calculation.
   */
  static uint16_t calculateCRC(const std::vector<uint8_t>& data, uint16_t crc_prev = 0xFFFF);

};

} /* namespace wts */

#endif /* WTS_DRIVER_WTS_DRIVER_HPP_ */
