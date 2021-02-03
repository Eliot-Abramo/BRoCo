//
// Created by Omar El Malki on 03.12.20.
//


//#include <../Src/MessageBus.h>
//#include <../Src/NetworkClientIO.h>
//#include <../Src/NetworkServerIO.h>
//#include <../Src/NetworkBus.h>
//#include <../Src/MessageBus.cpp>
//#include "gtest/gtest.h"
//
//static int received = 0;
//
//void handle_ping(uint8_t sender_id, PingPacket* packet) {
//    std::cout << "Ping C2C: " << (PingPacket().time - packet->time).count() << "ns" << std::endl;
//    received ++;
//}

//static uint8_t received_name[32];
//
//void handle_connect(uint8_t sender_id, ConnectPacket* packet) {
//    std::cout << "Connect: " << packet->name << std::endl;
//    for(int i = 0; i < 32; i++){
//        received_name[i] = packet->name[i];
//    }
//    received ++;
//}
//
//void handle_disconnect(uint8_t sender_id, DisconnectPacket* packet) {
//    std::cout << "Disconnect " << std::endl;
//    received ++;
//}

//static uint16_t received_uuid;
//static uint8_t received_action_id;
//static uint8_t received_target_id;
//static uint32_t received_payload;
//
//void handle_request(uint8_t sender_id, RequestPacket* packet) {
//    std::cout << "Request: " << "UUID: " << packet->uuid << ", ActionID: " << packet->action_id << ", TargetID: " << packet->target_id << ", Payload: " << packet->payload << std::endl;
//    received_uuid = packet->uuid;
//    received_action_id = packet->action_id;
//    received_target_id = packet->target_id;
//    received_payload = packet->payload;
//    received ++;
//}

//static uint8_t received_state;
//
//void handle_acknowledge(uint8_t sender_id, AcknowledgePacket* packet) {
//    std::cout << "Ack: " << "UUID: " << packet->uuid << ", State: " << packet->result << std::endl;
//    received_uuid = packet->uuid;
//    received_state = packet->result;
//    received ++;
//}

//void handle_response(uint8_t sender_id, ResponsePacket* packet) {
//    std::cout << "Response: " << "UUID: " << packet->uuid << ", ActionID: " << packet->action_id << ", TargetID: " << packet->target_id << ", Payload: " << packet->payload << std::endl;
//    received_uuid = packet->uuid;
//    received_action_id = packet->action_id;
//    received_target_id = packet->target_id;
//    received_payload = packet->payload;
//    received ++;
//}
//
//static uint8_t received_progress;
//
//void handle_progress(uint8_t sender_id, ProgressPacket* packet) {
//    std::cout << "Progress: " << "UUID: " << packet->uuid << ", Info: " << packet->progress << std::endl;
//    received_uuid = packet->uuid;
//    received_progress = packet->progress;
//    received ++;
//}
//
//static uint32_t received_data;
//void handle_data(uint8_t sender_id, DataPacket* packet) {
//    std::cout << "Data: " << packet->data << std::endl;
//    received_data = packet->data;
//    received ++;
//}

//static uint8_t received_message[128];
//
//void handle_message(uint8_t sender_id, MessagePacket* packet) {
//    std::cout << "Message: " << packet->message << std::endl;
//    for (int i = 0; i < 128; ++i) {
//        received_message[i] = packet->message[i];
//    }
//    received ++;
//}

//static uint8_t received_error_id;
//
//void handle_error(uint8_t sender_id, ErrorPacket* packet) {
//    std::cout << "ErrorID: " << packet->error_id << std::endl;
//    received_error_id = packet->error_id;
//    received ++;
//}
//
//TEST(MessageBusTestSuite, PingPacketPortBTest) { // 12/2/2020 -> 737761
//
//    std::cout << "Starting main test..." << std::endl;
//    received = 0;
//
//    NetworkServerIO* server_io = new NetworkServerIO(PORT_B);
//
//
//    int32_t result = server_io->connectServer();
//
//    if(result < 0) {
//        std::cout << "Network Server IO connection failed with error code " << result << std::endl;
//        std::cout << std::strerror(errno) << std::endl;
//    } else {
//        std::cout << "Connected to network server IO" << std::endl;
//    }
//
//    NetworkClientIO* client_io_1 = new NetworkClientIO("127.0.0.1", PORT_B);
//
//
//    result = client_io_1->connectClient();
//
//    if(result < 0) {
//        std::cout << "Network Client IO connection failed with error code " << result << std::endl;
//        std::cout << std::strerror(errno) << std::endl;
//    }
//
//    NetworkClientIO* client_io_2 = new NetworkClientIO("127.0.0.1", PORT_B);
//
//
//    result = client_io_2->connectClient();
//
//    if(result < 0) {
//        std::cout << "Network Client IO connection failed with error code " << result << std::endl;
//        std::cout << std::strerror(errno) << std::endl;
//    }
//
//    PingPacket packet;
//
//
//    NetworkBus* server_bus = new NetworkBus(server_io);
//    NetworkBus* client_1_bus = new NetworkBus(client_io_1);
//    NetworkBus* client_2_bus = new NetworkBus(client_io_2);
//
//
//    server_bus->forward<PingPacket>(server_bus);
//    client_2_bus->handle(handle_ping);
//
//    client_1_bus->send(&packet);
//
//    std::cout << "Test finished" << std::endl;
//
//    std::this_thread::sleep_for(std::chrono::nanoseconds(3000000000));
//
//        EXPECT_TRUE(received == 1);
//}
//
//TEST(MessageBusTestSuite, PingPacketPortATest) { // 12/2/2020 -> 737761
//
//    std::cout << "Starting main test..." << std::endl;
//    received = 0;
//
//    NetworkServerIO* server_io = new NetworkServerIO(PORT_A);
//
//    // server_io->receive(&handle_input);
//
//    int32_t result = server_io->connectServer();
//
//    if(result < 0) {
//        std::cout << "Network Server IO connection failed with error code " << result << std::endl;
//        std::cout << std::strerror(errno) << std::endl;
//    } else {
//        std::cout << "Connected to network server IO" << std::endl;
//    }
//
//    NetworkClientIO* client_io_1 = new NetworkClientIO("127.0.0.1", PORT_A);
//
//
//    result = client_io_1->connectClient();
//
//    if(result < 0) {
//        std::cout << "Network Client IO connection failed with error code " << result << std::endl;
//        std::cout << std::strerror(errno) << std::endl;
//    }
//
//    NetworkClientIO* client_io_2 = new NetworkClientIO("127.0.0.1", PORT_A);
//
//
//    result = client_io_2->connectClient();
//
//    if(result < 0) {
//        std::cout << "Network Client IO connection failed with error code " << result << std::endl;
//        std::cout << std::strerror(errno) << std::endl;
//    }
//
//    PingPacket packet;
//
//
//    NetworkBus* server_bus = new NetworkBus(server_io);
//    NetworkBus* client_1_bus = new NetworkBus(client_io_1);
//    NetworkBus* client_2_bus = new NetworkBus(client_io_2);
//
//
//    server_bus->forward<PingPacket>(server_bus);
//    client_2_bus->handle(handle_ping);
//
//    client_1_bus->send(&packet);
//
//    std::cout << "Test finished" << std::endl;
//
//    std::this_thread::sleep_for(std::chrono::nanoseconds(3000000000));
//
//    EXPECT_TRUE(received == 1);
//}
//
//
//TEST(MessageBusTestSuite, PingPacketRandomPortNumbersTest) { // 12/2/2020 -> 737761
//
//    std::cout << "Starting main test..." << std::endl;
//
//    // it will take approx 3 sec per test
//    int numberOfTests = 10;
//
//    for(int i = 0; i < numberOfTests; i++){
//
//        std::cout << "Starting test..." << std::endl;
//        received = 0;
//
//        srand(time(0));
//
//        int rand_port = rand() % (49151 - 1024) + 1024;
//
//        NetworkServerIO* server_io = new NetworkServerIO(rand_port);
//
//
//        int32_t result = server_io->connectServer();
//
//        if(result < 0) {
//            std::cout << "Network Server IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        } else {
//            std::cout << "Connected to network server IO" << std::endl;
//        }
//
//        NetworkClientIO* client_io_1 = new NetworkClientIO("127.0.0.1", rand_port);
//
//        result = client_io_1->connectClient();
//
//        if(result < 0) {
//            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        }
//
//        NetworkClientIO* client_io_2 = new NetworkClientIO("127.0.0.1", rand_port);
//
//
//        result = client_io_2->connectClient();
//
//        if(result < 0) {
//            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        }
//
//        PingPacket packet;
//
//
//        NetworkBus* server_bus = new NetworkBus(server_io);
//        NetworkBus* client_1_bus = new NetworkBus(client_io_1);
//        NetworkBus* client_2_bus = new NetworkBus(client_io_2);
//
//
//        server_bus->forward<PingPacket>(server_bus);
//        client_2_bus->handle(handle_ping);
//
//        client_1_bus->send(&packet);
//
//        std::cout << "Test finished" << std::endl;
//
//        std::this_thread::sleep_for(std::chrono::nanoseconds(3000000000));
//
//        EXPECT_TRUE(received == 1);
//    }
//}


//TEST(MessageBusTestSuite, ConnectPacketRandomPortNumbersTest) { // 12/2/2020 -> 737761
//
//    std::cout << "Starting main test..." << std::endl;
//
//    // it will take approx 3 sec per test
//    int numberOfTests = 10;
//
//    for (int i = 0; i < numberOfTests; i++) {
//
//        std::cout << "Starting test..." << std::endl;
//        received = 0;
//
//        srand(time(0));
//
//        int rand_port = rand() % (49151 - 1024) + 1024;
//
//        NetworkServerIO *server_io = new NetworkServerIO(rand_port);
//
//
//        int32_t result = server_io->connectServer();
//
//        if (result < 0) {
//            std::cout << "Network Server IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        } else {
//            std::cout << "Connected to network server IO" << std::endl;
//        }
//
//        NetworkClientIO *client_io_1 = new NetworkClientIO("127.0.0.1", rand_port);
//
//        result = client_io_1->connectClient();
//
//        if (result < 0) {
//            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        }
//
//        NetworkClientIO *client_io_2 = new NetworkClientIO("127.0.0.1", rand_port);
//
//
//        result = client_io_2->connectClient();
//
//        if (result < 0) {
//            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        }
//
//        ConnectPacket packet;
//
//        uint8_t sent_name[32];
//
//        for (int i = 0; i < 32; ++i) {
//            sent_name[i] = packet.name[i];
//        }
//
//
//        NetworkBus *server_bus = new NetworkBus(server_io);
//        NetworkBus *client_1_bus = new NetworkBus(client_io_1);
//        NetworkBus *client_2_bus = new NetworkBus(client_io_2);
//
//
//        server_bus->forward<ConnectPacket>(server_bus);
//        client_2_bus->handle(handle_connect);
//
//        client_1_bus->send(&packet);
//
//        std::cout << "Test finished" << std::endl;
//
//        std::this_thread::sleep_for(std::chrono::nanoseconds(3000000000));
//
//        for (int i = 0; i < 32; ++i) {
//            EXPECT_TRUE(sent_name[i] == received_name[i]);
//        }
//        EXPECT_TRUE(received == 1);
//    }
//}

//TEST(MessageBusTestSuite, DisconnectPacketRandomPortNumbersTest) { // 12/2/2020 -> 737761
//
//    std::cout << "Starting main test..." << std::endl;
//
//    // it will take approx 3 sec per test
//    int numberOfTests = 10;
//
//    for (int i = 0; i < numberOfTests; i++) {
//
//        std::cout << "Starting test..." << std::endl;
//        received = 0;
//
//        srand(time(0));
//
//        int rand_port = rand() % (49151 - 1024) + 1024;
//
//        NetworkServerIO *server_io = new NetworkServerIO(rand_port);
//
//
//        int32_t result = server_io->connectServer();
//
//        if (result < 0) {
//            std::cout << "Network Server IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        } else {
//            std::cout << "Connected to network server IO" << std::endl;
//        }
//
//        NetworkClientIO *client_io_1 = new NetworkClientIO("127.0.0.1", rand_port);
//
//        result = client_io_1->connectClient();
//
//        if (result < 0) {
//            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        }
//
//        NetworkClientIO *client_io_2 = new NetworkClientIO("127.0.0.1", rand_port);
//
//
//        result = client_io_2->connectClient();
//
//        if (result < 0) {
//            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        }
//
//        DisconnectPacket packet;
//
//
//        NetworkBus *server_bus = new NetworkBus(server_io);
//        NetworkBus *client_1_bus = new NetworkBus(client_io_1);
//        NetworkBus *client_2_bus = new NetworkBus(client_io_2);
//
//
//        server_bus->forward<DisconnectPacket>(server_bus);
//        client_2_bus->handle(handle_disconnect);
//
//        client_1_bus->send(&packet);
//
//        std::cout << "Test finished" << std::endl;
//
//        std::this_thread::sleep_for(std::chrono::nanoseconds(3000000000));
//
//        EXPECT_TRUE(received == 1);
//    }
//}

//TEST(MessageBusTestSuite, RequestPacketRandomPortNumbersTest) { // 12/2/2020 -> 737761
//
//    std::cout << "Starting main test..." << std::endl;
//
//    // it will take approx 3 sec per test
//    int numberOfTests = 10;
//
//    for (int i = 0; i < numberOfTests; i++) {
//
//        std::cout << "Starting test..." << std::endl;
//        received = 0;
//
//        srand(time(0));
//
//        int rand_port = rand() % (49151 - 1024) + 1024;
//
//        NetworkServerIO *server_io = new NetworkServerIO(rand_port);
//
//
//        int32_t result = server_io->connectServer();
//
//        if (result < 0) {
//            std::cout << "Network Server IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        } else {
//            std::cout << "Connected to network server IO" << std::endl;
//        }
//
//        NetworkClientIO *client_io_1 = new NetworkClientIO("127.0.0.1", rand_port);
//
//        result = client_io_1->connectClient();
//
//        if (result < 0) {
//            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        }
//
//        NetworkClientIO *client_io_2 = new NetworkClientIO("127.0.0.1", rand_port);
//
//
//        result = client_io_2->connectClient();
//
//        if (result < 0) {
//            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        }
//
//        RequestPacket packet;
//
//        uint16_t sent_uuid = packet.uuid;
//        uint8_t sent_action_id = packet.action_id;
//        uint8_t sent_target_id = packet.target_id;
//        uint32_t sent_payload = packet.payload;
//
//        NetworkBus *server_bus = new NetworkBus(server_io);
//        NetworkBus *client_1_bus = new NetworkBus(client_io_1);
//        NetworkBus *client_2_bus = new NetworkBus(client_io_2);
//
//
//        server_bus->forward<RequestPacket>(server_bus);
//        client_2_bus->handle(handle_request);
//
//        client_1_bus->send(&packet);
//
//        std::cout << "Test finished" << std::endl;
//
//        std::this_thread::sleep_for(std::chrono::nanoseconds(3000000000));
//
//        EXPECT_TRUE(received_uuid == sent_uuid);
//        EXPECT_TRUE(received_action_id == sent_action_id);
//        EXPECT_TRUE(received_target_id == sent_target_id);
//        EXPECT_TRUE(received_payload == sent_payload);
//        EXPECT_TRUE(received == 1);
//    }
//}

//TEST(MessageBusTestSuite, AckPacketRandomPortNumbersTest) { // 12/2/2020 -> 737761
//
//    std::cout << "Starting main test..." << std::endl;
//
//    // it will take approx 3 sec per test
//    int numberOfTests = 10;
//
//    for (int i = 0; i < numberOfTests; i++) {
//
//        std::cout << "Starting test..." << std::endl;
//        received = 0;
//
//        srand(time(0));
//
//        int rand_port = rand() % (49151 - 1024) + 1024;
//
//        NetworkServerIO *server_io = new NetworkServerIO(rand_port);
//
//
//        int32_t result = server_io->connectServer();
//
//        if (result < 0) {
//            std::cout << "Network Server IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        } else {
//            std::cout << "Connected to network server IO" << std::endl;
//        }
//
//        NetworkClientIO *client_io_1 = new NetworkClientIO("127.0.0.1", rand_port);
//
//        result = client_io_1->connectClient();
//
//        if (result < 0) {
//            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        }
//
//        NetworkClientIO *client_io_2 = new NetworkClientIO("127.0.0.1", rand_port);
//
//
//        result = client_io_2->connectClient();
//
//        if (result < 0) {
//            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        }
//
//        AcknowledgePacket packet;
//
//        uint16_t sent_uuid = packet.uuid;
//        uint8_t sent_state = packet.result;
//
//        NetworkBus *server_bus = new NetworkBus(server_io);
//        NetworkBus *client_1_bus = new NetworkBus(client_io_1);
//        NetworkBus *client_2_bus = new NetworkBus(client_io_2);
//
//
//        server_bus->forward<AcknowledgePacket>(server_bus);
//        client_2_bus->handle(handle_acknowledge);
//
//        client_1_bus->send(&packet);
//
//        std::cout << "Test finished" << std::endl;
//
//        std::this_thread::sleep_for(std::chrono::nanoseconds(3000000000));
//
//        EXPECT_TRUE(received_uuid == sent_uuid);
//        EXPECT_TRUE(received_state == sent_state);
//        EXPECT_TRUE(received == 1);
//    }
//}
//
//TEST(MessageBusTestSuite, ResponsePacketRandomPortNumbersTest) { // 12/2/2020 -> 737761
//
//    std::cout << "Starting main test..." << std::endl;
//
//    // it will take approx 3 sec per test
//    int numberOfTests = 10;
//
//    for (int i = 0; i < numberOfTests; i++) {
//
//        std::cout << "Starting test..." << std::endl;
//        received = 0;
//
//        srand(time(0));
//
//        int rand_port = rand() % (49151 - 1024) + 1024;
//
//        NetworkServerIO *server_io = new NetworkServerIO(rand_port);
//
//
//        int32_t result = server_io->connectServer();
//
//        if (result < 0) {
//            std::cout << "Network Server IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        } else {
//            std::cout << "Connected to network server IO" << std::endl;
//        }
//
//        NetworkClientIO *client_io_1 = new NetworkClientIO("127.0.0.1", rand_port);
//
//        result = client_io_1->connectClient();
//
//        if (result < 0) {
//            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        }
//
//        NetworkClientIO *client_io_2 = new NetworkClientIO("127.0.0.1", rand_port);
//
//
//        result = client_io_2->connectClient();
//
//        if (result < 0) {
//            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        }
//
//        ResponsePacket packet;
//
//        uint16_t sent_uuid = packet.uuid;
//        uint8_t sent_action_id = packet.action_id;
//        uint8_t sent_target_id = packet.target_id;
//        uint32_t sent_payload = packet.payload;
//
//        NetworkBus *server_bus = new NetworkBus(server_io);
//        NetworkBus *client_1_bus = new NetworkBus(client_io_1);
//        NetworkBus *client_2_bus = new NetworkBus(client_io_2);
//
//
//        server_bus->forward<ResponsePacket>(server_bus);
//        client_2_bus->handle(handle_response);
//
//        client_1_bus->send(&packet);
//
//        std::cout << "Test finished" << std::endl;
//
//        std::this_thread::sleep_for(std::chrono::nanoseconds(3000000000));
//
//        EXPECT_TRUE(received_uuid == sent_uuid);
//        EXPECT_TRUE(received_action_id == sent_action_id);
//        EXPECT_TRUE(received_target_id == sent_target_id);
//        EXPECT_TRUE(received_payload == sent_payload);
//        EXPECT_TRUE(received == 1);
//    }
//}
//
//TEST(MessageBusTestSuite, ProgressPacketRandomPortNumbersTest) { // 12/2/2020 -> 737761
//
//    std::cout << "Starting main test..." << std::endl;
//
//    // it will take approx 3 sec per test
//    int numberOfTests = 10;
//
//    for (int i = 0; i < numberOfTests; i++) {
//
//        std::cout << "Starting test..." << std::endl;
//        received = 0;
//
//        srand(time(0));
//
//        int rand_port = rand() % (49151 - 1024) + 1024;
//
//        NetworkServerIO *server_io = new NetworkServerIO(rand_port);
//
//
//        int32_t result = server_io->connectServer();
//
//        if (result < 0) {
//            std::cout << "Network Server IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        } else {
//            std::cout << "Connected to network server IO" << std::endl;
//        }
//
//        NetworkClientIO *client_io_1 = new NetworkClientIO("127.0.0.1", rand_port);
//
//        result = client_io_1->connectClient();
//
//        if (result < 0) {
//            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        }
//
//        NetworkClientIO *client_io_2 = new NetworkClientIO("127.0.0.1", rand_port);
//
//
//        result = client_io_2->connectClient();
//
//        if (result < 0) {
//            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        }
//
//        ProgressPacket packet;
//
//        uint16_t sent_uuid = packet.uuid;
//        uint8_t sent_progress = packet.progress;
//
//        NetworkBus *server_bus = new NetworkBus(server_io);
//        NetworkBus *client_1_bus = new NetworkBus(client_io_1);
//        NetworkBus *client_2_bus = new NetworkBus(client_io_2);
//
//
//        server_bus->forward<ProgressPacket>(server_bus);
//        client_2_bus->handle(handle_progress);
//
//        client_1_bus->send(&packet);
//
//        std::cout << "Test finished" << std::endl;
//
//        std::this_thread::sleep_for(std::chrono::nanoseconds(3000000000));
//
//        EXPECT_TRUE(received_uuid == sent_uuid);
//        EXPECT_TRUE(received_progress == sent_progress);
//        EXPECT_TRUE(received == 1);
//    }
//}
//
//TEST(MessageBusTestSuite, DataPacketRandomPortNumbersTest) { // 12/2/2020 -> 737761
//
//    std::cout << "Starting main test..." << std::endl;
//
//    // it will take approx 3 sec per test
//    int numberOfTests = 10;
//
//    for (int i = 0; i < numberOfTests; i++) {
//
//        std::cout << "Starting test..." << std::endl;
//        received = 0;
//
//        srand(time(0));
//
//        int rand_port = rand() % (49151 - 1024) + 1024;
//
//        NetworkServerIO *server_io = new NetworkServerIO(rand_port);
//
//
//        int32_t result = server_io->connectServer();
//
//        if (result < 0) {
//            std::cout << "Network Server IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        } else {
//            std::cout << "Connected to network server IO" << std::endl;
//        }
//
//        NetworkClientIO *client_io_1 = new NetworkClientIO("127.0.0.1", rand_port);
//
//        result = client_io_1->connectClient();
//
//        if (result < 0) {
//            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        }
//
//        NetworkClientIO *client_io_2 = new NetworkClientIO("127.0.0.1", rand_port);
//
//
//        result = client_io_2->connectClient();
//
//        if (result < 0) {
//            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        }
//
//        DataPacket packet;
//
//        uint32_t sentData = packet.data;
//
//
//        NetworkBus *server_bus = new NetworkBus(server_io);
//        NetworkBus *client_1_bus = new NetworkBus(client_io_1);
//        NetworkBus *client_2_bus = new NetworkBus(client_io_2);
//
//
//        server_bus->forward<DataPacket>(server_bus);
//        client_2_bus->handle(handle_data);
//
//        client_1_bus->send(&packet);
//
//        std::cout << "Test finished" << std::endl;
//
//        std::this_thread::sleep_for(std::chrono::nanoseconds(3000000000));
//
//        EXPECT_TRUE(sentData == received_data);
//
//        EXPECT_TRUE(received == 1);
//    }
//}
//
//TEST(MessageBusTestSuite, MessagePacketRandomPortNumbersTest) { // 12/2/2020 -> 737761
//
//    std::cout << "Starting main test..." << std::endl;
//
//    // it will take approx 3 sec per test
//    int numberOfTests = 10;
//
//    for (int i = 0; i < numberOfTests; i++) {
//
//        std::cout << "Starting test..." << std::endl;
//        received = 0;
//
//        srand(time(0));
//
//        int rand_port = rand() % (49151 - 1024) + 1024;
//
//        NetworkServerIO *server_io = new NetworkServerIO(rand_port);
//
//
//        int32_t result = server_io->connectServer();
//
//        if (result < 0) {
//            std::cout << "Network Server IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        } else {
//            std::cout << "Connected to network server IO" << std::endl;
//        }
//
//        NetworkClientIO *client_io_1 = new NetworkClientIO("127.0.0.1", rand_port);
//
//        result = client_io_1->connectClient();
//
//        if (result < 0) {
//            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        }
//
//        NetworkClientIO *client_io_2 = new NetworkClientIO("127.0.0.1", rand_port);
//
//
//        result = client_io_2->connectClient();
//
//        if (result < 0) {
//            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        }
//
//        MessagePacket packet;
//
//        uint8_t sent_message[128];
//
//        for (int i = 0; i < 128; ++i) {
//            sent_message[i] = packet.message[i];
//        }
//
//        NetworkBus *server_bus = new NetworkBus(server_io);
//        NetworkBus *client_1_bus = new NetworkBus(client_io_1);
//        NetworkBus *client_2_bus = new NetworkBus(client_io_2);
//
//
//        server_bus->forward<MessagePacket>(server_bus);
//        client_2_bus->handle(handle_message);
//
//        client_1_bus->send(&packet);
//
//        std::cout << "Test finished" << std::endl;
//
//        std::this_thread::sleep_for(std::chrono::nanoseconds(3000000000));
//
//        for (int i = 0; i < 128; ++i) {
//            EXPECT_TRUE(sent_message[i] == received_message[i]);
//        }
//        EXPECT_TRUE(received == 1);
//    }
//}
//
//TEST(MessageBusTestSuite, ErrorPacketRandomPortNumbersTest) { // 12/2/2020 -> 737761
//
//    std::cout << "Starting main test..." << std::endl;
//
//    // it will take approx 3 sec per test
//    int numberOfTests = 10;
//
//    for (int i = 0; i < numberOfTests; i++) {
//
//        std::cout << "Starting test..." << std::endl;
//        received = 0;
//
//        srand(time(0));
//
//        int rand_port = rand() % (49151 - 1024) + 1024;
//
//        NetworkServerIO *server_io = new NetworkServerIO(rand_port);
//
//
//        int32_t result = server_io->connectServer();
//
//        if (result < 0) {
//            std::cout << "Network Server IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        } else {
//            std::cout << "Connected to network server IO" << std::endl;
//        }
//
//        NetworkClientIO *client_io_1 = new NetworkClientIO("127.0.0.1", rand_port);
//
//        result = client_io_1->connectClient();
//
//        if (result < 0) {
//            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        }
//
//        NetworkClientIO *client_io_2 = new NetworkClientIO("127.0.0.1", rand_port);
//
//
//        result = client_io_2->connectClient();
//
//        if (result < 0) {
//            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
//            std::cout << std::strerror(errno) << std::endl;
//        }
//
//        ErrorPacket packet;
//
//        uint8_t sent_error_id = packet.error_id;
//
//        NetworkBus *server_bus = new NetworkBus(server_io);
//        NetworkBus *client_1_bus = new NetworkBus(client_io_1);
//        NetworkBus *client_2_bus = new NetworkBus(client_io_2);
//
//
//        server_bus->forward<ErrorPacket>(server_bus);
//        client_2_bus->handle(handle_error);
//
//        client_1_bus->send(&packet);
//
//        std::cout << "Test finished" << std::endl;
//
//        std::this_thread::sleep_for(std::chrono::nanoseconds(3000000000));
//        EXPECT_TRUE(received_error_id == sent_error_id);
//        EXPECT_TRUE(received == 1);
//    }
//}

