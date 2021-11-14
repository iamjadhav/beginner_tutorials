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
 * @file AddTwoFloatsTest.cpp
 * @author Aditya Jadhav (amjadhav@umd.edu)
 * @brief Unit Test to test the Service AddTwoFloatsTest
 * @version 0.1
 * @date 2021-11-13
 *
 * @copyright Copyright (c) 2021
 */

#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/AddTwoFloats.h"

/**
 * @brief These are tests for the existence of service AddTwoFloats
 * and its operation
 */
TEST(AdditionTest, Test_Add_Two_Floats) {
    ros::NodeHandle n;
    auto client = n.serviceClient<beginner_tutorials::
                                AddTwoFloats>("float_addition");

    // Test to validate service existence
    bool exists(client.waitForExistence(ros::Duration(8)));
    EXPECT_FALSE(exists);

    // Service object to get the request and response
    beginner_tutorials::AddTwoFloats service;
    service.request.a = 0.0;
    service.request.b = 0.0;
    client.call(service);

    // Test to validate the service output
    EXPECT_EQ(service.response.addition, service.request.a + service.request.b);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "addition_tester");
    ros::NodeHandle n;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
