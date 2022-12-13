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

//----------callbacks ----------

// receive data from ROS and send to RoCo

void reset_power_supply_callback(const boost::shared_ptr<std_msgs::Bool const> msg, NetworkBus* sender)
{
  std::cout<<msg->data<<std::endl;
  Reset_PowerSupplyPacket packet;
  packet.reset = msg->data;
  sender->send<Reset_PowerSupplyPacket>(&packet);
  std::cout<<packet.reset<<" sent!"<<std::endl;
}

void switch_avionics_callback(const boost::shared_ptr<std_msgs::Bool const> msg, NetworkBus* sender)
{
  Switch_AvionicsPacket packet;
  packet.on = msg->data;
  sender->send<Switch_AvionicsPacket>(&packet);
}

void switch_raman_callback(const boost::shared_ptr<std_msgs::Bool const> msg, NetworkBus* sender)
{
  Switch_RamanPacket packet;
  packet.on = msg->data;
  sender->send<Switch_RamanPacket>(&packet);
}

void switch_jetson_callback(const boost::shared_ptr<std_msgs::Bool const> msg, NetworkBus* sender)
{
  Switch_JetsonPacket packet;
  packet.on = msg->data;
  sender->send<Switch_JetsonPacket>(&packet);
}

void switch_lidar_callback(const boost::shared_ptr<std_msgs::Bool const> msg, NetworkBus* sender)
{
  Switch_LidarPacket packet;
  packet.on = msg->data;
  sender->send<Switch_LidarPacket>(&packet);
}

void switch_ethernet_callback(const boost::shared_ptr<std_msgs::Bool const> msg, NetworkBus* sender)
{
  Switch_EthernetPacket packet;
  packet.on = msg->data;
  sender->send<Switch_EthernetPacket>(&packet);
}


//----------handlers----------

// receive data from RoCo and send to ROS

void handle_system(uint8_t sender_id, Power_SystemPacket* packet, void* ros_publisher)
{
  std_msgs::Float32MultiArray msg;
  //Clear array
	msg.data.clear();

  msg.data.push_back(packet->battery_charge);
  msg.data.push_back(packet->state);

  ((ros::Publisher *)ros_publisher)->publish(msg);
}

void handle_voltages(uint8_t sender_id, Power_VoltagePacket* packet, void* ros_publisher)
{
  std_msgs::Float32MultiArray msg;
  //Clear array
	msg.data.clear();
  for (int i(0); i < 3; ++i) msg.data.push_back(packet->voltages[i]);

  ((ros::Publisher *)ros_publisher)->publish(msg);
}

void handle_currents(uint8_t sender_id, Power_CurrentPacket* packet, void* ros_publisher)
{
  std_msgs::Float32MultiArray msg;
  //Clear array
	msg.data.clear();
  for (int i(0); i < 3; ++i) msg.data.push_back(packet->currents[i]);

  ((ros::Publisher *)ros_publisher)->publish(msg);
}