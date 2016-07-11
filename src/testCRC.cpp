#include <wts_driver/wts_driver.hpp>

int main (int argn, char* args[]) {

  wts_driver::SerialComm sc(args[1], 115200);

  wts_driver::WTSDriver wts_driver (sc);

  sleep(3);

  return 0;
}
