#include <wts_driver/wts_driver.hpp>

int main (int argn, char* args[]) {

  wts_driver::SerialComm sc(args[1], 115200);

  wts_driver::WTSDriver wts_driver (sc);

  std::vector <uint8_t> packets;

  packets.push_back(0xAA);
  packets.push_back(0xAA);
  packets.push_back(0xAA);

  packets.push_back(0x38);
  packets.push_back(0x00);
  packets.push_back(0x00);

  std::string result;
  wts_driver.getSensorType(result);
  std::cout << "The sensor type is: " << result;

  wts_driver.getDeviceTag(result);
  std::cout << "The device tag is: " << result;

  int temperature;
  wts_driver.readDeviceTemperature(temperature);
  std::cout << "The device temperature is: " << temperature << "°C";

  wts_driver.getSystemInformation();
  wts_driver.displaySystemInformation();

  wts_driver.getMatrixInformation();
  wts_driver.displayMatrixInformation();

  return 0;
}
