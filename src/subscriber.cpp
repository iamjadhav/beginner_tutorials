/**
 * @file subscriber.cpp
 * @author Aditya Jadhav (amjadhav@umd.edu)
 * @brief Listener File
 * @version 0.1
 * @date 2021-10-31
 * 
 * @copyright Copyright (c) 2021
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I'm hearing: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  ros::spin();

  return 0;
}
