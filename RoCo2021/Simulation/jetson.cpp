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

  // forward data received from avionics to cs
  jetson_server_bus->forward<DataPacket>(jetson_server_bus);

  while (1)
  {

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
    

  return 0;
}
#endif
