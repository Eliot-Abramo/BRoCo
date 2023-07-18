#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <iostream>
#include <unistd.h>
#include <chrono>

#include "../include/RocoRos/RoCo.h"
#include "../include/RocoRos/UDevDriver.h"
#include "../include/RocoRos/handlers_cs.h"
#include "../include/RocoRos/CanSocketDriver.h"

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/imu.hpp>


#define CANNETWORK 
// #define UARTNETWORK 


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */


// class MinimalPublisher : public rclcpp::Node
// {
//   public:
//     MinimalPublisher()
//     : Node("minimal_publisher"), count_(0)
//     {
//       publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
//       timer_ = this->create_wall_timer(
//       500ms, std::bind(&MinimalPublisher::timer_callback, this));
//     }

//   private:
//     void timer_callback()
//     {
//       auto message = std_msgs::msg::String();
//       message.data = "Hello, world! " + std::to_string(count_++);
//       RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//       publisher_->publish(message);
//     }
//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
//     size_t count_;
// };



int main(int argc, char * argv[])
{
#ifdef CANNETWORK 

  CanSocketDriver* driver = new CanSocketDriver("can0");

  if (driver->isConnected() != 1){
    perror("Driver is not connected");
    while(1);
  }
  uint8_t buffer[64];
  buffer[0] = 0x12;
  buffer[1] = 0x64;
  std::shared_ptr<NetworkBus> bus;
  driver->TxFrameConfig(123, 2, buffer);
  bus = std::make_shared<NetworkBus>(driver);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("my_node");
  auto publisher = node->create_publisher<std_msgs::msg::Int32>("dummy_pub", 10);
  std::function<void(const std_msgs::msg::Int32::SharedPtr)> callback = std::bind(&dummy_callback, std::placeholders::_1, bus);
  auto subscriber = node->create_subscription<std_msgs::msg::Int32>("dummy_sub", 10, callback);

  bus->handle<DummySystem_DummyPacket>(&handle_dummy, (void*)&publisher);

  std::cout << "SPIN" << std::endl;

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
#endif

#ifdef UARTNETWORK 
  UDevDriver* driver = new UDevDriver("/dev/ttyUSB0");

  struct termios tty;

  if(tcgetattr(driver->getFD(), &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;
  tty.c_cflag &= ~HUPCL;

  tty.c_lflag = 0;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag |= 0; 
  tty.c_iflag &= ~(IGNPAR | IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL | INPCK | IMAXBEL);
  tty.c_iflag &= ~IUTF8;

  tty.c_oflag = 0;

  tty.c_cc[VTIME] = 0;
  tty.c_cc[VMIN] = 0;

  cfsetispeed(&tty, B921600);
  cfsetospeed(&tty, B921600);

  if (tcsetattr(driver->getFD(), TCSANOW, &tty) != 0) {
      std::cout << strerror(errno) << std::endl;
  }

  std::shared_ptr<NetworkBus> bus;
  bus = std::make_shared<NetworkBus>(driver);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("my_node");
  auto dummy_publisher = node->create_publisher<std_msgs::msg::Int32>("dummy_pub", 10);
  auto imu_publisher = node->create_publisher<sensor_msgs::msg::Imu>("imu_topic", 10);
  std::function<void(const std_msgs::msg::Int32::SharedPtr)> callback = std::bind(&dummy_callback, std::placeholders::_1, bus);
  auto dummy_subscriber = node->create_subscription<std_msgs::msg::Int32>("dummy_sub", 10, callback);

  bus->handle<DummySystem_DummyPacket>(&handle_dummy, (void*)&dummy_publisher);
  bus->handle<IMU_Packet>(&handle_IMU, (void*)&imu_publisher);
  std::cout << "SPIN" << std::endl;

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
  #endif
  return 0;
}



