/**
 * @file publisher.cpp
 * @author Aditya Jadhav (amjadhav@umd.edu)
 * @brief Class AnalogSensor definitions
 * @version 0.1
 * @date 2021-10-31
 * 
 * @copyright Copyright (c) 2021
 */

#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()) {
    std_msgs::String msg;

    std::stringstream ss;
    ss << " Hey ! How's it going? " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}
