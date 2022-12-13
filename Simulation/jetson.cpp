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
#include <unistd.h>

#include "../Src/RoCo.h"

// Simulates power supply RoCo server

int main(int argc, char **argv)
{

	std::cout << "Starting power supply simulation..." << std::endl;

  // create server on av side
  NetworkServerIO* jetson_server_av = new NetworkServerIO(PORT_B);

  // connect server
	int32_t result = jetson_server_av->connectServer();
  
  if(result < 0) {
		std::cout << "Network Server IO connection failed with error code " << result << std::endl;
		std::cout << std::strerror(errno) << std::endl;
	} else {
		std::cout << "Connected to network server IO" << std::endl;
	}

  // create server
  NetworkServerIO* jetson_server_cs = new NetworkServerIO(PORT_A);

  // connect server
	result = jetson_server_cs->connectServer();
  
  if(result < 0) {
		std::cout << "Network Server IO connection failed with error code " << result << std::endl;
		std::cout << std::strerror(errno) << std::endl;
	} else {
		std::cout << "Connected to network server IO" << std::endl;
	}
  
  NetworkBus* jetson_server_bus_av = new NetworkBus(jetson_server_av);
  NetworkBus* jetson_server_bus_cs = new NetworkBus(jetson_server_cs);

  // forward data received from avionics to cs
  jetson_server_bus_av->forward<DataPacket>(jetson_server_bus_cs);

  while (1)
  {

    DataPacket packet;
    packet.data = 200;
    jetson_server_bus_av->send<DataPacket>(&packet);
    printf("Datapacket sent!\n");

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
    

  return 0;
}
#endif
