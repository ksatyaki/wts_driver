#include <wts_driver/wts_driver.hpp>

int main (int argn, char* args[]) {
  wts::SerialComm sc("/dev/ttyACM0", 115200);

  wts::WTSDriver wts_driver (sc);

  std::vector <uint8_t> packets;

  //int no;
  //std::cin >> no;

  //packets.resize(no);

  //for(int i = 0; i < no; i++) {
  //  std::cin >> packets[i];
  //}

  packets.push_back(0xAA);
  packets.push_back(0xAA);
  packets.push_back(0xAA);

  packets.push_back(0x50);
  packets.push_back(0x00);
  packets.push_back(0x00);

  uint16_t checksum = wts::WTSDriver::calculateCRC(packets);

  printf("\nChecksum is %x", checksum);

  packets.push_back(checksum & 0xFF);
  packets.push_back(checksum >> 8);

  checksum = wts::WTSDriver::calculateCRC(packets);
  printf("\nChecksum is %x", checksum);

  std::vector <uint8_t> more_packets;

  if(!sc.writeBytes(packets)) {
    printf("\n Screwed.");
  }
  else {
    if(!sc.readBytes(15, more_packets)) {
      printf("\nReading screwed up.");
    }
    else {
      std::cout << "\n Received reply: ";
      for(int i = 0; i < more_packets.size(); i++) {
        printf("%x, ", more_packets[i]);
      }
    }
  }

  return 0;
}
