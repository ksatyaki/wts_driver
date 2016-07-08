#include <wts_driver/wts_driver.hpp>

int main (int argn, char* args[]) {
  wts_driver::SerialComm sc("/dev/ttyACM0", 115200);

  wts_driver::WTSDriver wts_driver (sc);

  std::vector <uint8_t> packets;

  packets.push_back(0xAA);
  packets.push_back(0xAA);
  packets.push_back(0xAA);

  packets.push_back(0x38);
  packets.push_back(0x00);
  packets.push_back(0x00);

  /*
  uint16_t checksum = wts_driver::WTSDriver::calculateCRC(packets);



  printf("\nChecksum is %x", checksum);

  std::vector <uint8_t> more_packets;

  if(!sc.writeBytes(packets)) {
    printf("\n Screwed.");
  }
  else {
    if(!sc.writeToSerialPort(checksum)) {
      printf("\n checksum failed.");
    }
    else {
      if(!sc.readBytes(24, more_packets)) {
        printf("\nReading screwed up.");
      }
      else {
        std::cout << "\n Received reply: ";
        int i = 0;
        for(i = 0; i < 8; i++) {
          printf("%x, ", more_packets[i]);
        }

        for(i = 8; i < more_packets.size() - 2; i++) {
          printf("%c", more_packets[i]);
        }

      }
    }
  }

  checksum = wts_driver::WTSDriver::calculateCRC(more_packets);
  printf("\nChecksum is %x", checksum);

  */

  std::string result;
  wts_driver.getSensorType(result);

  return 0;
}
