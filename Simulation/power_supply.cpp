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

// Simulates power supply RoCo server

#include "../Src/RoCo.h"

void handle_power_supply(uint8_t sender_id, DataPacket* packet, void* jaj){
  printf("(POWER_SUPPLY): datapacket of values %d received from sender %d\n", packet->data, sender_id);
}

int main(int argc, char **argv)
{

	std::cout << "Starting power supply simulation..." << std::endl;

  // create server
  NetworkServerIO* power_supply_server = new NetworkServerIO(PORT_A);

  // connect server
	int32_t result = power_supply_server->connectServer();
  
  if(result < 0) {
		std::cout << "Network Server IO connection failed with error code " << result << std::endl;
		std::cout << std::strerror(errno) << std::endl;
	} else {
		std::cout << "Connected to network server IO" << std::endl;
	}
  
  NetworkBus* power_supply_server_bus = new NetworkBus(power_supply_server);

  int placeholder;

  power_supply_server_bus->handle(handle_power_supply, (void*)&placeholder);

  while (1)
  {

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
    

  return 0;
}
#endif
