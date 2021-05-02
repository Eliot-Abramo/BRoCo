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

  //-----define ROS topics on which to publish-----

  // ros::Publisher av_barotemp_pub = n.advertise<std_msgs::Float32MultiArray>("barotemp", 1000);
  // ros::Publisher av_accelmag_pub = n.advertise<std_msgs::Float32MultiArray>("accelmag", 1000);

  // ros::Publisher ha_gripper_pub = n.advertise<std_msgs::Float32>("gripper", 1000);

  // ros::Publisher po_system_pub = n.advertise<std_msgs::Float32MultiArray>("system", 1000);
  // ros::Publisher po_voltage_pub = n.advertise<std_msgs::Float32MultiArray>("voltages", 1000);
  // ros::Publisher po_current_pub = n.advertise<std_msgs::Float32MultiArray>("currents", 1000);

  // ros::Publisher sc_measure_pub = n.advertise<std_msgs::Float32>("measures", 1000);

  // ros::Publisher ping_pub = n.advertise<std_msgs::Float32>("ping", 1000); // use time msgeg
  // ros::Publisher request_pub = n.advertise<std_msgs::UInt32MultiArray>("request", 1000);
  // ros::Publisher response_pub = n.advertise<std_msgs::UInt32MultiArray>("response", 1000);
  // ros::Publisher progress_pub = n.advertise<std_msgs::UInt32MultiArray>("progress", 1000);
  // ros::Publisher error_pub = n.advertise<std_msgs::UInt8>("error", 1000);



  // 1 Hz refresh rate of the node
  ros::Rate loop_rate(10);


	std::cout << "Starting main test..." << std::endl;


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


  //-----set client bus to handle different packets-----
  // client_2_bus->handle(handle_barotemp,  (void*)&av_barotemp_pub);
  // client_2_bus->handle(handle_accelmag,  (void*)&av_accelmag_pub);

  // client_2_bus->handle(handle_gripper,  (void*)&ha_gripper_pub);

  // client_2_bus->handle(handle_system,  (void*)&po_system_pub);
  // client_2_bus->handle(handle_voltages,  (void*)&po_voltage_pub);
  // client_2_bus->handle(handle_currents,  (void*)&po_current_pub);

  // client_2_bus->handle(handle_measures,  (void*)&sc_measure_pub);


  // //-----define ROS topics on which to subscribe----- Communication to avionics

  // ros::Subscriber reset_power_sub= n.subscribe<std_msgs::Bool>("reset_power", 1000, boost::bind(reset_power_supply_callback, _1, client_2_bus));
  // ros::Subscriber switch_avionics_sub = n.subscribe<std_msgs::Bool>("switch_avionics", 1000, boost::bind(switch_avionics_callback, _1, client_2_bus));
  // ros::Subscriber switch_raman_sub = n.subscribe<std_msgs::Bool>("switch_raman", 1000, boost::bind(switch_raman_callback, _1, client_2_bus));
  // ros::Subscriber switch_jetson_sub = n.subscribe<std_msgs::Bool>("switch_jetson", 1000, boost::bind(switch_jetson_callback, _1, client_2_bus));
  // ros::Subscriber switch_lidar_sub = n.subscribe<std_msgs::Bool>("switch_lidar", 1000, boost::bind(switch_lidar_callback, _1, client_2_bus));
  // ros::Subscriber switch_ethernet_sub = n.subscribe<std_msgs::Bool>("switch_ethernet", 1000, boost::bind(switch_ethernet_callback, _1, client_2_bus));

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
