#include <wts_driver/wts_driver.hpp>
#include <ros/ros.h>

int main(int argn, char* args[]) {
  ros::init(argn, args, "wts_node");

  ros::NodeHandle nh;

  wts_driver::SerialComm sc(args[1], 115200);
  wts_driver::WTSDriver wts_driver (sc);

  float wts_frame_rate;

  if(ros::param::get("/wts_frame_rate", wts_frame_rate)) {
    ROS_INFO("Frame rate: %f", wts_frame_rate);
  }
  else {
    wts_frame_rate = 20.0;
    ROS_WARN("The \"wts_frame_rate\" parameter was not set. Setting frame rate to default value: 20 Hz.");
  }

  wts_driver.initROSPublisher(nh);
  wts_driver.startPeriodicFrameAcquisition(false, 1000.0/wts_frame_rate);

  while(ros::ok()) {
    usleep(100000);
  }

  return 0;

}
