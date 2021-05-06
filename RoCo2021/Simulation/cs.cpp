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

// Simulates CS RoCo client

void handle_avionics(uint8_t sender_id,DataPacket* packet, void* jaj){
  printf("(CS): datapacket of values %d received from sender %d\n", packet->data, sender_id);
}

int main(int argc, char **argv)
{

	std::cout << "Starting cs simulation..." << std::endl;

  // create client for 2.4ghz net and corresponding bus
  NetworkClientIO* client_power_supply = new NetworkClientIO("127.0.0.1", PORT_A);
  NetworkBus* client_power_supply_bus = new NetworkBus(client_power_supply);

  // create client for 5ghz net
  NetworkClientIO* client_jetson = new NetworkClientIO("127.0.0.1", PORT_A);
  NetworkBus* client_jetson_bus = new NetworkBus(client_jetson);

  int placeholder;

  client_jetson_bus->handle(handle_avionics, (void*) &placeholder);

  while (1)
  {
    // loop to try to (re)connect to server if disconnected
    if(!(client_power_supply->is_connected()))
    {
      int result = client_power_supply->connectClient();
      if(result < 0) {
  	    std::cout << "Power Supply connection failed with error code " << result << std::endl;
  	    std::cout << std::strerror(errno) << std::endl;
      }
    }

    // loop to try to (re)connect to server if disconnected
    if(!(client_jetson->is_connected()))
    {
      int result = client_jetson->connectClient();
      if(result < 0) {
  	    std::cout << "Jetson connection failed with error code " << result << std::endl;
  	    std::cout << std::strerror(errno) << std::endl;
      }
    }

    // send data to power supply
    if(client_jetson->is_connected()){
      // DataPacket packet;
      // packet.data = 15;
      // client_power_supply_bus->send<DataPacket>(&packet);
      // printf("Datapacket sent!\n");
    }


    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
#endif
