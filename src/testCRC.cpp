#include <wts_driver/wts_driver.hpp>

int main (int argn, char* args[]) {

  wts_driver::SerialComm sc(args[1], 115200);

  wts_driver::WTSDriver wts_driver (sc);

//  wts_driver::wts_error error = wts_driver.stopPeriodicFrameAcquisition();
//
//  error = wts_driver.startPeriodicFrameAcquisition();
//  if(error == wts_driver::wts_error::E_SUCCESS) {
//    if(wts_driver.isPeriodicFrameAcqRunning())
//      std::cout << "\nPeriodic Frame Acq has started successfully.";
//    else
//      std::cout << "\nPeriodic Frame Acq has started successfully. But we haven't seen it.";
//  }
//  else {
//    std::cout << "\nSucks." << error.message();
//  }
//
//  error = wts_driver.stopPeriodicFrameAcquisition();

  std::string result;
  wts_driver.getSensorType(result);
  std::cout << "The sensor type is: " << result;

  int temperature;
  wts_driver.readDeviceTemperature(temperature);
  std::cout << "The device temperature is: " << temperature << "°C";

  wts_driver.getSystemInformation();
  wts_driver.getDeviceTag();
  wts_driver.displaySystemInformation();

  wts_driver.getMatrixInformation();
  wts_driver.displayMatrixInformation();

  wts_driver::Frame frame;
  wts_driver.readSingleFrame(frame);
  std::cout << frame;

  return 0;
}
