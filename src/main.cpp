#include <wts_driver/wts_driver.hpp>
#include <ros/ros.h>

int main(int argn, char* args[]) {
  ros::init(argn, args, "wts_node");

  ros::NodeHandle nh;

  wts_driver::SerialComm sc(args[1], 115200);

  wts_driver::WTSDriver wts_driver (sc);

  wts_driver.getMatrixInformation();
  wts_driver.getSystemInformation();
  wts_driver.getDeviceTag();
  wts_driver.displaySystemInformation();

  wts_driver.initROSPublisher(nh);
  wts_driver.startPeriodicFrameAcquisition();

  ros::spin();

}
