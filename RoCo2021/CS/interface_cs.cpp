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
#include "handlers_cs.h"

// Interface between Roco and Ros on Control station

int main(int argc, char **argv)
{
  // call of init needed before anything else, "" is the name of the node
  ros::init(argc, argv, "interface_cs");

  // main access point to communications with ROS system
  // NodeHandle initializes this node
  ros::NodeHandle n;

  ros::Rate loop_rate(10);


  //-----define RoCo client/bus-----

  // create server
  NetworkClientIO* client = new NetworkClientIO("127.0.0.1", PORT_A);

  // connect server
	int32_t result = client->connectClient();
  
  // if(result < 0) {
	// 	std::cout << "[POWER SUPPLY] connection failed with error code " << result << std::endl;
	// 	std::cout << std::strerror(errno) << std::endl;
	// } else {
	// 	std::cout << "Connected to [POWER SUPPLY]" << std::endl;
	// }

  // create server bus
  NetworkBus* client_bus = new NetworkBus(client);

  //-----define ROS topics on which to publish/subscribe-----

  // PUBLISH

  // messages received from PS directed at CS
  ros::Publisher ps_system_pub = n.advertise<std_msgs::Float32MultiArray>("system", 1000);
  ros::Publisher ps_voltage_pub = n.advertise<std_msgs::Float32MultiArray>("voltages", 1000);
  ros::Publisher ps_current_pub = n.advertise<std_msgs::Float32MultiArray>("currents", 1000);

  // SUBSCRIBE

  // commands sent to the PS from CS
  ros::Subscriber reset_power_sub= n.subscribe<std_msgs::Bool>("reset_power", 1000, boost::bind(reset_power_supply_callback, _1, client_bus));
  ros::Subscriber switch_avionics_sub = n.subscribe<std_msgs::Bool>("switch_avionics", 1000, boost::bind(switch_avionics_callback, _1, client_bus));
  ros::Subscriber switch_raman_sub = n.subscribe<std_msgs::Bool>("switch_raman", 1000, boost::bind(switch_raman_callback, _1, client_bus));
  ros::Subscriber switch_jetson_sub = n.subscribe<std_msgs::Bool>("switch_jetson", 1000, boost::bind(switch_jetson_callback, _1, client_bus));
  ros::Subscriber switch_lidar_sub = n.subscribe<std_msgs::Bool>("switch_lidar", 1000, boost::bind(switch_lidar_callback, _1, client_bus));
  ros::Subscriber switch_ethernet_sub = n.subscribe<std_msgs::Bool>("switch_ethernet", 1000, boost::bind(switch_ethernet_callback, _1, client_bus));

  // request packet


  //-----define RoCo handlers-----
  client_bus->handle(handle_system,  (void*)&ps_system_pub);
  client_bus->handle(handle_voltages,  (void*)&ps_voltage_pub);
  client_bus->handle(handle_currents,  (void*)&ps_current_pub);


  while (ros::ok())
  {
    // loop to try to (re)connect to server if disconnected
    // if(!(client->is_connected()))
    // {
    //   int result = client->connectClient();
    //   if(result < 0) {
  	//     std::cout << "Power Supply connection failed with error code " << result << std::endl;
  	//     std::cout << std::strerror(errno) << std::endl;
    //   }
    // }

    // used to handle ros communication events, i.e. callback to function
    ros::spinOnce();

    // enforces the frequency at which this while loop runs (here at 1H z, see above)
    loop_rate.sleep();

  }

  return 0;
}
#endif
