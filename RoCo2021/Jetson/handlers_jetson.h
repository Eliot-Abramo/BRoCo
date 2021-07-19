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
#include "std_msgs/UInt8.h"
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


// handlers

// receive data from RoCo and send to ROS

void handle_barotemp(uint8_t sender_id, Avionics_BaroTempPacket* packet, void* ros_publisher)
{
  std_msgs::Float32MultiArray msg;
  // //Clear array
	msg.data.clear();

  msg.data.push_back(packet->pressure);
  msg.data.push_back(packet->temperature);
  std::cout<<"Pressure: "<<packet->pressure<<std::endl;
  std::cout<<"Temperature: "<<packet->temperature<<std::endl;

  ((ros::Publisher *)ros_publisher)->publish(msg);
}

void handle_accelmag(uint8_t sender_id, Avionics_AccelMagPacket* packet, void* ros_publisher)
{
  std_msgs::Float32MultiArray msg;
  //Clear array
	msg.data.clear();

  for (int i(0); i < 3; ++i) msg.data.push_back(packet->acceleration[i]);
  for (int i(0); i < 3; ++i) msg.data.push_back(packet->angular[i]);
  for (int i(0); i < 3; ++i) msg.data.push_back(packet->magneto[i]);
  std::cout<<"accel 0: "<<packet->acceleration[0]<<std::endl;
  std::cout<<"accel 1: "<<packet->acceleration[1]<<std::endl;
  std::cout<<"accel 2: "<<packet->acceleration[2]<<std::endl;
  ((ros::Publisher *)ros_publisher)->publish(msg);
}

void handle_adc(uint8_t sender_id, Avionics_ADCPacket* packet, void* ros_publisher)
{
  std_msgs::Float32MultiArray msg;
  // //Clear array
	msg.data.clear();

  msg.data.push_back(packet->port);
  msg.data.push_back(packet->voltage);

  std::cout<<"port "<<packet->port<<" : "<< packet->voltage <<std::endl;

  ((ros::Publisher *)ros_publisher)->publish(msg);
}

void handle_mass(uint8_t sender_id, Science_MassPacket* packet, void* ros_publisher)
{
  std_msgs::Float32 msg;
  msg.data = packet->mass;
  std::cout<<"Mass: "<<packet->mass<<std::endl;

  ((ros::Publisher *)ros_publisher)->publish(msg);
}

// callbacks 

// receive data from ROS and send to RoCo

void fsm_callback(const boost::shared_ptr<std_msgs::UInt8MultiArray const> msg,  NetworkBus* sender)
{
  FsmPacket packet;
  packet.task = (msg->data)[0];
  packet.instruction = (msg->data)[1];
  sender->send<FsmPacket>(&packet);
}

void led_callback(const boost::shared_ptr<std_msgs::Bool const> msg,  NetworkBus* sender)
{
  Science_LedPacket packet;
  packet.on = msg->data;
  sender->send<Science_LedPacket>(&packet);
}
