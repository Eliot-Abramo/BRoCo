#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Time.h"

#include <sstream>
#include <string>

#include "../Src/Build/Build.h"

#ifdef BUILD_FOR_TESTING


#include <cstdint>
#include <iomanip>
#include <iostream>
#include <thread>
#include <cstring>
#include <boost/bind.hpp>

#include "../Src/RoCo.h"
#include "handlers_jetson.h"

// Interface between Ros and RoCo on jetson
// Packets sent to CS from avionics are converted to ROS here

int main(int argc, char **argv)
{
  // call of init needed before anything else, "" is the name of the node
  ros::init(argc, argv, "interface_jetson");

  // main access point to communications with ROS system
  // NodeHandle initializes this node
  ros::NodeHandle n;

  ros::Rate loop_rate(100);


  //-----define RoCo server/bus-----

  // create server
  NetworkServerIO* server = new NetworkServerIO(PORT_B);

  // connect server
	int32_t result = server->connectServer();
  
  if(result < 0) {
		std::cout << "Network Server IO connection failed with error code " << result << std::endl;
		std::cout << std::strerror(errno) << std::endl;
	} else {
		std::cout << "Connected to network server IO" << std::endl;
	}

  // create server bus
  NetworkBus* server_bus = new NetworkBus(server);

  //-----define ROS topics on which to publish/subscribe-----

  // PUBLISH

  // messages received from avionics directed at CS or nav
  ros::Publisher av_barotemp_pub = n.advertise<std_msgs::Float32MultiArray>("barotemp", 1000);
  ros::Publisher av_accelmag_pub = n.advertise<std_msgs::Float32MultiArray>("accelmag", 1000);
  ros::Publisher av_adc_pub = n.advertise<std_msgs::Float32MultiArray>("adc", 1000);
  ros::Publisher sc_mass_pub = n.advertise<std_msgs::Float32>("mass", 1000);

  // add response packet
  // add fsm packet

  // SUBSCRIBE

  // add fsm packet
  // add request packet
  // receive FSM and send to AV and CS
  ros::Subscriber fsm_sub = n.subscribe<std_msgs::UInt8MultiArray>("state", 1000, boost::bind(fsm_callback, _1, server_bus));
  ros::Subscriber led_sub = n.subscribe<std_msgs::Bool>("led", 1000, boost::bind(led_callback, _1, server_bus));



  //-----define RoCo handlers-----
  server_bus->handle(handle_barotemp, (void*)&av_barotemp_pub);
  server_bus->handle(handle_accelmag, (void*)&av_accelmag_pub);
  server_bus->handle(handle_adc, (void*)&av_adc_pub);
  server_bus->handle(handle_mass, (void*)&sc_mass_pub);

  while (ros::ok())
  {

    // used to handle ros communication events, i.e. callback to function
    ros::spinOnce();

    // enforces the frequency at which this while loop runs
    loop_rate.sleep();

  }

  return 0;
}
#endif
