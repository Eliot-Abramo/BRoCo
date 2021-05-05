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
  NetworkServerIO* jetson_server = new NetworkServerIO(PORT_B);

  // connect server
	int32_t result = jetson_server->connectServer();
  
  if(result < 0) {
		std::cout << "Network Server IO connection failed with error code " << result << std::endl;
		std::cout << std::strerror(errno) << std::endl;
	} else {
		std::cout << "Connected to network server IO" << std::endl;
	}
  
  NetworkBus* jetson_server_bus = new NetworkBus(jetson_server);

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
  
  // receive FSM and send to AV
  ros::Subscriber fsm_sub= n.subscribe<std_msgs::UInt32>("fsm_nav_to_av", 1000, boost::bind(fsm_callback, _1, jetson_server_bus));

  //-----define handlers-----
  jetson_server_bus->handle(handle_fsm,  (void*)&fsm_pub);
  jetson_server_bus->handle(handle_potentiometers,  (void*)&potent_pub);
  jetson_server_bus->handle(handle_barotemp,  (void*)&barotemp_pub);
  jetson_server_bus->handle(handle_accelmag,  (void*)&accelmag_pub);

  //-----define forwarders-----
  jetson_server_bus->forward<Avionics_BaroTempPacket>(jetson_server_bus);
  jetson_server_bus->forward<Avionics_AccelMagPacket>(jetson_server_bus);
  jetson_server_bus->forward<Handling_GripperPacket>(jetson_server_bus);
  jetson_server_bus->forward<Power_SystemPacket>(jetson_server_bus);
  jetson_server_bus->forward<Power_VoltagePacket>(jetson_server_bus);
  jetson_server_bus->forward<Power_CurrentPacket>(jetson_server_bus);
  jetson_server_bus->forward<Science_MeasurePacket>(jetson_server_bus);



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
