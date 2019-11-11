/* Copyright (c) 2019 Ethan Quist
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @file talker.cpp
 *
 * @brief This is a ros code to publish messages
 *
 * @author Ethan Quist
 *
 * @copyright Ethan Quist
 *
 * @params This code needs to take in a hertz integer input to run correctly
 */
#include <tf/transform_broadcaster.h>
#include <ros/console.h>
#include <sstream>
#include <cstdlib>
#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
#include "std_msgs/String.h"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");



  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  /**
   *
   * This code is for Week 11 transforms.
   * Here is the set up code for tf to make the new frame
   */
  tf::TransformBroadcaster br;
  tf::Transform transform;
  ros::Rate rate(10.0);


  // taking the command line argument HERTZ
  ros::Rate loop_rate(10);
  if (argc > 1) {
    int hz = atoi(argv[1]);
    ros::Rate loop_rate(hz);
  } else {
    ros::Rate loop_rate(10);
  }

  /**
   * This section of code will create the client aspect added to my code.
   * It will simulate the client of Add_two_ints and add it to my string
   */
  ros::ServiceClient client = n.serviceClient < beginner_tutorials::AddTwoInts
      > ("add_two_ints");
  beginner_tutorials::AddTwoInts srv;

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    /**
     * This is the second part of the transform code that broadcasts the frame
     */
    transform.setOrigin(tf::Vector3(0.0, 2.0, 0.0));
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;
    std::stringstream ss;

    // using count as the two ints to add in my service
    srv.request.a = count;
    srv.request.b = count;

    /**
     * This section of code calls the Service and has error handling if the service fails
     */
    if (client.call(srv)) {
      ROS_INFO_STREAM("Service Call Worked.");
      int doubleCount = srv.response.sum;
      ss << "Ethan learned ROS " << count << " double count: " << doubleCount;
    } else {
      ROS_ERROR_STREAM("Failed to use service call add_two_ints");
      ss << "Ethan learned ROS " << count;
    }

    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;


    // adding debug logging stream
    ROS_DEBUG_STREAM("count increased " << count);

    /**
     * This is checking the counter to make sure it isn't going to high
     * The fatal error checks to make sure the process wasn't left on too long
     * This is an example of a way to check processes.
     */
    if (count > 1000) {
      ROS_WARN_STREAM("Count is getting high, consider stopping.");
    }

    if (count > 10000) {
      ROS_FATAL_STREAM("FATAL: Count got too high");
    }
  }


  return 0;
}
