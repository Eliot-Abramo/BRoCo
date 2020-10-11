# RoCo
## Abstract
RoCo, the Xplore's rover communication API, allows the Rover's different subsystems to communicate with each other. Every peer in a bus network can broadcast a message on the bus and this message might or might not be handled by the other peers.

Using a high degree of abstraction, this API ensures that a unique software can be shared across the subsystems without the need of porting or installing additional software. Since RoCo will also be used on embedded systems, the software is designed to have a low memory footprint and CPU overhead. Furthermore, static allocation predominates the code's architecture to avoid potential overflows.

In addition, RoCo also includes the communication protocol, that defines which packets will be transmitted and received through specific busses.

When using the RoCo API, please do only use the __RoCo.h__ header file.

## MessageBus API
RoCo's highest degree of abstraction is the MessageBus API, which defines how packets are defined, handled, sent or forwarded.
This MessageBus class only requires the implementations to define standard read and write functions to be functional, as described later.
Since memory allocation is static, it was necessary to define a maximum packet size (256 bytes), as well as a maximum number of unique senders (64). Moreover, there are at most 64 different packet identifiers.

### Packet definition
Every MessageBus method requires an additional type info, which should define the structure of the considered packet. Packets should always be defined in accordance with the following scheme.
```cpp
struct SomePacket {
	// Whatever content you need
} __attribute__((packed));
```
If you fail to define the structure as _packed_, extra padding bytes will be added to the good will of the compiler, which is the most undesirable effect possible since the code is supposed to be completely portable between different system architectures.

Another extra step to define a packet is to add it to the _protocol register_ in _Protocol/ProtocolRegister.h_.
Doing so will allocate memory for the template type of the structure previously defined.

```cpp
#ifdef YOUR_PROTOCOL
REGISTER(SomePacket)
#endif /* YOUR_PROTOCOL */
```

Once those steps are done, you may use the MessageBus API to send this packet to any peer connected to the bus.

### Define
```cpp
template<typename T> bool define(uint8_t identifier)
```
The _define_ method links the given packet structure to a unique identifier. Every identifier consist of two routing bits, which are reserved for now, and six identifier bits that can be freely assigned. It is recommended to define the different packets supported by the bus in the MessageBus' implementation's constructor. Once a packet is defined, it can be freely sent, received or forwarded. Otherwise, calling the later methods will result in failure.
The _define_ method will fail if one of the following conditions is met:
1. The packet identifier is already in use.
2. The packet size is too large (> 256 bytes).
3. The packet type is already defined with another identifier.

### Send
```cpp
template<typename T> bool send(T *message);
```
The _send_ method allows the user to broadcast a defined packet on the bus. Note that thank to the template argument, it is not necessary to specify the nature of the packet being sent: The compiler will infer its nature automatically. The _send_ method will fail if the provided packet has no assigned identifier.

### Receive
```cpp
template<typename T> bool handle(void (*handler)(uint8_t source, T*));
```
The _handle_ method allows the user to receive a defined packet from the bus. This method requires the user to pass a callback function, that will handle the reception of the given packet. Depending on implementation, the source identifier might or might not accurately represent the sender of the packet. Note that thank to the template argument, it is not necessary to specify the nature of the packet being received: The compiler will infer its nature automatically. The _handle_ method will fail if the provided packet has no assigned identifier.

### Forward
```cpp
template<typename T> bool forward(MessageBus* bus);
```
The _forward_ method allows the user to redirect a defined packet from one bus to another. All packets having the same template type as the passed one will be retransmitted to the given bus. The _forward_ method will fail if the provided packet has no assigned identifier.

## Writing an implementation of MessageBus
The _MessageBus_ class defines two virtual methods that need to be implemented by subclasses:

```cpp
virtual uint8_t append(uint8_t* buffer, uint32_t length) = 0; // Must be atomic
virtual void transmit() = 0;
```
As you may guess, the _append_ method is used to send data to the bus controller. Note that in a multithreaded environment, it is required that _append_ gets called in thread-exclusive context (usage of Lock/Mutex is recommended). In addition, _transmit_ is used to flush eventual data that might be buffered in the bus controller.

Moreover, the implementation must ensure that _receive_ is called whenever a buffer is received from the bus. The _receive_ method has the following signature:
```cpp
void receive(uint8_t senderID, uint8_t *pointer, uint32_t length);
```
By implementing the two above methods and calling _receive_ when appropriate, the MessageBus implementation is complete and can be used on every platform that meets the dependency requirements.

## IOBus: A buffered implementation of MessageBus
The IOBus implementation is designed to manage a classic pre-allocated buffer provided by the constructor. Transmission and reception is managed by a dedicated class called _IODriver_. The latter must be inherited and overwrite the following virtual methods:

```cpp
virtual void receive(const std::function<void (uint8_t sender_id, uint8_t* buffer, uint32_t length)> &receiver) = 0;
virtual void transmit(uint8_t* buffer, uint32_t length) = 0;
```

_receive_ provides the _IODriver_ implementation with a callback reception function. This callback function must be called whenever there is incoming data from the bus.

_transmit_ signals the _IODriver_ that the given data buffer must be sent to the bus. As for every I/O driver, it is required for the _transmit_ method to have exclusive context.

## NetworkBus: A TCP/IP implementation of MessageBus
The _NetworkBus_ API inherits the IOBus implementation with a standard buffer size of 256 bytes. The only interest in analysing this implementation is the way the _IODriver_ were developed. Note that _NetworkClientIO_ and _NetworkServerIO_ define slightly different implementations of the _IODriver_. For simplicity, we will assume the low-level implementations of these two _IODriver_ are equivalent.

Using the <sys/socket.h> API, this implementation provides a simple way to communicate between two peers on a network. Note that this implementation is not fully compatible with the LwIP Stack, and thus cannot be used on embedded software.

To establish or interrupt a connection, two methods are available for each _IODriver_. Their usage is transparent.

__NetworkClientIO__
```cpp
int8_t connectClient();
void disconnectClient();
```

__NetworkServerIO__
```cpp
int8_t connectServer();
void disconnectServer();
```

For both implementations, the packet reception is done by reading from the socket through an independent thread. Transmission is simply done by writing to the TCP socket.

## Protocol
The latest protocol (version 20W18) implements the following packet structures. Identifiers are not listed since they are bus-dependant.

#### Ping packet
Allows the user to assess the bus latency.
- Actual timestamp in nanoseconds as _std::chrono::time_point_

#### Connect packet
Signals a new connection on the bus.
- Actual timestamp in nanoseconds as _std::chrono::time_point_

#### Disconnect packet
Signals a disconnection on the bus.
- Actual timestamp in nanoseconds as _std::chrono::time_point_

#### Request packet
Signals the peers that a given resource is requested.
- Request UUID as _uint32_t_
- Action identifier as _uint8_t_
- Target identifier as _uint8_t_
- Request payload as _uint32_t_

#### Acknowledge packet
Acknowledges a request. Undefined if no request was previously performed.
- Request UUID as _uint32_t_
- Primary state of request as _uint8_t_

#### Response packet
Responds to a request. Undefined if no request was previously performed.
- Request UUID as _uint32_t_
- Action identifier as _uint8_t_
- Target identifier as _uint8_t_
- Response payload as _uint32_t_

#### Progress packet
Signals the requester that the resource is being processed and defines an actual state of the request. Undefined if no request was previously performed.
- Request UUID as _uint32_t_
- Progress information as _uint8_t_


#### Data packet
Broadcasts a generic data on the bus.
- Generic data as _uint32_t_

#### Message packet
Broadcasts a generic message on the bus.
- Message data as _uint8_t_[128]

#### Error packet
Signals the peers that the sender has experienced failure and defines an  error identifier representing the situation.
- Error identifier as _uint8_t_

## Building
Since RoCo is designed to be portable software, the same code base can be run on multiple platforms. However, some advanced features, such as Network I/O are not compatible with embedded systems. This is why there are several build configurations that need to be specified when targeting a specific platform.

Before including the _Roco.h_ header file, it is necessary to declare which platform you are targetting:
- BUILD_FOR_CONTROL_STATION
- BUILD_FOR_NAVIGATION
- BUILD_FOR_AVIONICS
- BUILD_FOR_TESTING

It is also of crucial importance that all the peers on the same bus communicate with the same version of RoCo and, in particular, with the same protocol specification.

To enforce this, please do always specify and check the target protocol version in _Build/Build.h_. The latest implementation is version 20W18.

## Examples
### 1. Client to client communication through a server.
_Alice_ and _Bob_ want to assess the quality of the link that connects them. In fact, _Alice_ and _Bob_ are both connected to _Carol_, which routes messages from _Alice_ to _Bob_ and from _Bob_ to _Alice_. _Alice_ tells _Bob_ that she is going to send him her actual time. When _Bob_ receives the message, he will compare his actual time to _Alice_'s actual time and compute by how much it differs. For simplicity, we assume _Alice_, _Bob_ and _Carol_ are one the same machine.

Since _Carol_ is only supposed to send what she receives, using the _forward_ method is appropriate. The implementation of the transmission and reception becomes trivial with the RoCo API:

```cpp
void handle_ping(uint8_t sender_id, PingPacket* packet) {
	std::cout << "Ping C2C: " << (PingPacket().time - packet->time).count() << "ns" << std::endl;
}
```
```cpp
int main() {
  NetworkServerIO* carol_io = new NetworkServerIO(42666);
  NetworkClientIO* alice_io = new NetworkClientIO("127.0.0.1", 42666);
  NetworkClientIO* bob_io = new NetworkClientIO("127.0.0.1", 42666);

  carol_io->connectServer();
  alice_io->connectClient();
  bob_io->connectClient();

  NetworkBus* carol_bus = new NetworkBus(charlie_io); // Alice-Carol-Bob bus
  NetworkBus* alice_bus = new NetworkBus(alice_io); // Alice-Carol bus
  NetworkBus* bob_bus = new NetworkBus(bob_io); // Carol-Bob bus

  carol_bus->forward<PingPacket>(carol_bus); // Rebroadcast on the Alice-Carol-Bob bus
  bob_bus->handle(handle_ping); // Configure the reception

  PingPacket packet;
  alice_bus->send(&packet); // Send to Carol, who will send to Bob
}
```
