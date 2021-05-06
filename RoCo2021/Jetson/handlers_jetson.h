/* DESCRIPTION

This file contains all the handle functions for the different packets received
from RoCo.

Probably have to use boost:bind to be able to pass publisher objects to
handlers.
*/

#include "ros/ros.h"
// add the different types od data needed
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Bool.h"

#include <sstream>
#include <string>

#include "../Src/Build/Build.h"

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <thread>
#include <cstring>

#include "../Src/RoCo.h"

/* MULTI ARRAY
 are made of a
 layout  // specification of data layout
 data   // array of data
 */


// callbacks 

// receive data from ROS and send to RoCo

void fsm_callback(const boost::shared_ptr<std_msgs::UInt32 const> msg, NetworkBus* sender_av, NetworkBus* sender_cs)
{
  FsmPacket packet;
  packet.state = msg->data;
  sender_av->send<FsmPacket>(&packet);
  sender_cs->send<FsmPacket>(&packet);
}


// handlers

// receive data from RoCo and send to ROS

void handle_fsm(uint8_t sender_id, FsmPacket* packet, void* ros_publisher)
{
  std_msgs::UInt32 msg;
  msg.data = packet->state;

  ((ros::Publisher *)ros_publisher)->publish(msg);
}

void handle_potentiometers(uint8_t sender_id, PotentiometersPacket* packet, void* ros_publisher)
{
  std_msgs::Float32MultiArray msg;
  // //Clear array
	msg.data.clear();
  for(int i(0); i < 4; ++i) msg.data.push_back(packet->angles[i]);

  ((ros::Publisher *)ros_publisher)->publish(msg);
}

void handle_barotemp(uint8_t sender_id, Avionics_BaroTempPacket* packet, void* ros_publisher)
{
  std_msgs::Float32MultiArray msg;
  // //Clear array
	msg.data.clear();
  msg.data.push_back(packet->pressure);
  msg.data.push_back(packet->temperature);
  std::cout<<packet->pressure<<"\n"<<packet->temperature<<std::endl;
  //
  ((ros::Publisher *)ros_publisher)->publish(msg);
}

void handle_accelmag(uint8_t sender_id, Avionics_AccelMagPacket* packet, void* ros_publisher)
{
  std_msgs::Float32MultiArray msg;
  //Clear array
	msg.data.clear();

  // float a[] = {packet->acceleration};
  // float ang[] = {packet->angular};
  // float m[] = {packet->magneto};
  for (int i(0); i < 3; ++i) msg.data.push_back(packet->acceleration[i]);
  for (int i(0); i < 3; ++i) msg.data.push_back(packet->angular[i]);
  for (int i(0); i < 3; ++i) msg.data.push_back(packet->magneto[i]);
  //msg.data = {packet->acceleration, packet->angular, packet->magneto};

  ((ros::Publisher *)ros_publisher)->publish(msg);
}