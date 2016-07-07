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

namespace wts {

class wts_error {

  std::string message();

  enum error_type
  {
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
    E_FILE_EXISTS             //!< File already exists
  } error_type_;

};

}




#endif /* WTS_DRIVER_COMMON_HPP_ */
