#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt32.h"
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
// Also serves as a forwarder some of the RoCo packets from avionics

int main(int argc, char **argv)
{
  // call of init needed before anything else, "" is the name of the node
  ros::init(argc, argv, "interface_jetson");

  // main access point to communications with ROS system
  // NodeHandle initializes this node
  ros::NodeHandle n;

  ros::Rate loop_rate(10);


  //-----define RoCo server/bus-----

// create server
  NetworkServerIO* jetson_server_av = new NetworkServerIO(PORT_AV);

  // connect server
	int32_t result = jetson_server_av->connectServer();
  
  if(result < 0) {
		std::cout << "Network Server IO connection failed with error code " << result << std::endl;
		std::cout << std::strerror(errno) << std::endl;
	} else {
		std::cout << "Connected to network server IO" << std::endl;
	}
  
  NetworkBus* jetson_server_bus_av = new NetworkBus(jetson_server_av);

  // create server
  NetworkServerIO* jetson_server_cs = new NetworkServerIO(PORT_CS);

  // connect server
	result = jetson_server_cs->connectServer();
  
  if(result < 0) {
		std::cout << "Network Server IO connection failed with error code " << result << std::endl;
		std::cout << std::strerror(errno) << std::endl;
	} else {
		std::cout << "Connected to network server IO" << std::endl;
	}
  
  NetworkBus* jetson_server_bus_cs = new NetworkBus(jetson_server_cs);

  //-----define ROS topics on which to publish/subscribe-----

  // PUBLISH

  // send FSM to NAV
  ros::Publisher fsm_pub = n.advertise<std_msgs::UInt32>("fsm_av_to_nav", 1000);
  // send potentiometer to NAV
  ros::Publisher potent_pub = n.advertise<std_msgs::Float32MultiArray>("potentiometers", 1000);
  // send barotemp to NAV
  ros::Publisher barotemp_pub = n.advertise<std_msgs::Float32MultiArray>("barotemp_av_to_nav", 1000);
  // send potentiometer to NAV
  ros::Publisher accelmag_pub = n.advertise<std_msgs::Float32MultiArray>("potentiometers", 1000);


  // SUBSCRIBE
  
  // receive FSM and send to AV and CS
  ros::Subscriber fsm_sub= n.subscribe<std_msgs::UInt32>("fsm_nav_to_av", 1000, boost::bind(fsm_callback, _1, jetson_server_bus_av, jetson_server_bus_cs));

  //-----define handlers-----
  jetson_server_bus_av->handle(handle_fsm,  (void*)&fsm_pub);
  jetson_server_bus_av->handle(handle_potentiometers,  (void*)&potent_pub);
  jetson_server_bus_av->handle(handle_barotemp,  (void*)&barotemp_pub);
  jetson_server_bus_av->handle(handle_accelmag,  (void*)&accelmag_pub);

  //-----define forwarders-----
  jetson_server_bus_av->forward<Avionics_BaroTempPacket>(jetson_server_bus_cs);
  jetson_server_bus_av->forward<Avionics_AccelMagPacket>(jetson_server_bus_cs);
  jetson_server_bus_av->forward<Handling_GripperPacket>(jetson_server_bus_cs);
  jetson_server_bus_av->forward<Power_SystemPacket>(jetson_server_bus_cs);
  jetson_server_bus_av->forward<Power_VoltagePacket>(jetson_server_bus_cs);
  jetson_server_bus_av->forward<Power_CurrentPacket>(jetson_server_bus_cs);
  jetson_server_bus_av->forward<Science_MeasurePacket>(jetson_server_bus_cs);



  while (ros::ok())
  {

    // used to handle ros communication events, i.e. callback to function
    ros::spinOnce();

    // enforces the frequency at which this while loop runs (here at 1H z, see above)
    loop_rate.sleep();

  }

  return 0;
}
#endif
