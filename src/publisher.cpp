/**
* MIT License
*
* Copyright (c) Aditya Jadhav
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
*/

/**
 * @file publisher.cpp
 * @author Aditya Jadhav (amjadhav@umd.edu)
 * @brief ROS Publisher to publish a message to a topic
 * @version 0.1
 * @date 2021-10-31
 *
 * @copyright Copyright (c) 2021
 */

#include <tf/transform_broadcaster.h>
#include <sstream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/AddTwoFloats.h"

// Empty String Initialization
std::string message_1 = "";

/**
 * @brief ROS Service to add two Float Numbers
 *
 * @param req Service Request
 * @param res Service Response
 * @return true
 */
bool floatAddition(beginner_tutorials::AddTwoFloats::Request &req,
                   beginner_tutorials::AddTwoFloats::Response &res) {
  // Warning and Fatal Error if Numbers are Negative
  if ((req.a < 0) && (req.b < 0)) {
      ROS_FATAL_STREAM(" Both Numbers are Negative !!");
  } else if ((req.a < 0) || (req.b < 0)) {
      ROS_WARN_STREAM(" A or B is Negative");
  }
  res.addition = req.a + req.b;  // Addition of the two valid floats

  // Error if the addition is equal to 0
  if (res.addition == 0) {
      ROS_ERROR_STREAM(" The Float Addition is 0 !!");
  }
  ROS_INFO(" Addition of A=%f and B=%f ", req.a, req.b);
  ROS_INFO(" is : [%f]", res.addition);

  // Creating a String of the Service to be added to global string
  std::string text;
  text = " The Float Addition is : " + std::to_string(res.addition);
  message_1 = text;
  return true;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  // TransformBroadcaster object to send transformations
  tf::TransformBroadcaster broad;
  tf::Transform tr;
  tf::Quaternion quater;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::ServiceServer service = n.advertiseService("AddTwoFloats",
                                                  floatAddition);
  int frequency = atoi(argv[1]);  // command-line argument to change publish
                                  // rate
  ros::Rate loop_rate(frequency);

  int count = 0;
  std::string text_1;
  std::string message;

  text_1 = " Hey ! How's it going? ";

  while (ros::ok()) {
    ROS_DEBUG_STREAM("Publish Frequency is : " << frequency);
    std_msgs::String msg;

    // Setting the Rotation, Origin and sending the transform over
    quater.setRPY(0.0, 0.0, 1.0);
    tr.setRotation(quater);
    tr.setOrigin(tf::Vector3(1.0, 2.0, 0.0));
    broad.sendTransform(tf::StampedTransform(tr, ros::Time::now(), "world",
                      "talk"));

    // Final String with Service String and Count
    message = text_1 + std::to_string(count) + "." + message_1;
    msg.data = message;

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}
