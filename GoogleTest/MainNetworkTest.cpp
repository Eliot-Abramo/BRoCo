//
// Created by Omar El Malki on 03.12.20.
//

#include <../Src/MessageBus.h>
#include <../Src/NetworkClientIO.h>
#include <../Src/NetworkServerIO.h>
#include <../Src/NetworkBus.h>
#include <../Src/Protocol/Protocol.h>
#include "gtest/gtest.h"

// counter for the number of handled packets
static int received = 0;

static uint64_t received_time;
void handle_ping(uint8_t sender_id, PingPacket* packet) {
    std::cout << "Ping C2C: " << (PingPacket().time - packet->time) << "ns" << std::endl;
    received_time = packet->time;
    received ++;
}

static uint16_t received_uuid;
static uint8_t received_action_id;
static uint8_t received_target_id;
static uint32_t received_payload;

void handle_request(uint8_t sender_id, RequestPacket* packet) {
    std::cout << "Request: " << "UUID: " << packet->uuid << ", ActionID: " << packet->action_id << ", TargetID: " << packet->target_id << ", Payload: " << packet->payload << std::endl;
    received_uuid = packet->uuid;
    received_action_id = packet->action_id;
    received_target_id = packet->target_id;
    received_payload = packet->payload;
    received ++;
}

void handle_response(uint8_t sender_id, ResponsePacket* packet) {
    std::cout << "Response: " << "UUID: " << packet->uuid << ", ActionID: " << packet->action_id << ", TargetID: " << packet->target_id << ", Payload: " << packet->payload << std::endl;
    received_uuid = packet->uuid;
    received_action_id = packet->action_id;
    received_target_id = packet->target_id;
    received_payload = packet->payload;
    received ++;
}

static uint32_t received_progress_uuid;
static uint8_t received_progress;

void handle_progress(uint8_t sender_id, ProgressPacket* packet) {
    std::cout << "Progress: " << "UUID: " << packet->uuid << ", Info: " << packet->progress << std::endl;
    received_progress_uuid = packet->uuid;
    received_progress = packet->progress;
    received ++;
}

static uint8_t received_error_id;

void handle_error(uint8_t sender_id, ErrorPacket* packet) {
    std::cout << "ErrorID: " << packet->error_id << std::endl;
    received_error_id = packet->error_id;
    received ++;
}

static float received_pressure;
static float received_temperature;

void handle_avionics_barotemp(uint8_t sender_id, Avionics_BaroTempPacket* packet) {
    std::cout << "Avionics_BaroTempPacket: Pressure: " << packet->pressure << ", Temperature: " << packet->temperature << std::endl;
    received_pressure = packet->pressure;
    received_temperature = packet->temperature;
    received ++;
}

static float received_acceleration[3];
static float received_angular[3];
static float received_magneto[3];

void handle_avionics_accelmag(uint8_t sender_id, Avionics_AccelMagPacket* packet) {
    std::cout << "Avionics_AccelMag: Acceleration: " << packet->acceleration << ", Angular: " << packet->angular << ", Magneto: " << packet->magneto << std::endl;
    for (int i = 0; i < 3; ++i) {
        received_acceleration[i] = packet->acceleration[i];
        received_angular[i] = packet->angular[i];
        received_magneto[i] = packet->magneto[i];
    }
    received ++;
}

static float received_voltage;

void handle_handling_gripper(uint8_t sender_id, Handling_GripperPacket* packet) {
    std::cout << "Handling Gripper Voltage: " << packet->voltage << std::endl;
    received_voltage = packet->voltage;
    received ++;
}

static float received_voltages[4];

void handle_power_voltage(uint8_t sender_id, Power_VoltagePacket* packet) {
    std::cout << "Power Voltages: " << packet->voltages << std::endl;
    for (int i = 0; i < 4; ++i) {
        received_voltages[i] = packet->voltages[i];
    }
    received ++;
}

static float received_currents[4];

void handle_power_current(uint8_t sender_id, Power_CurrentPacket* packet) {
    std::cout << "Power Currents: " << packet->currents << std::endl;
    for (int i = 0; i < 4; ++i) {
        received_currents[i] = packet->currents[i];
    }
    received ++;
}

static float received_battery_charge;
static uint8_t received_state;

void handle_power_system(uint8_t sender_id, Power_SystemPacket* packet) {
    std::cout << "Power: Battery Charge: " << packet->battery_charge << ", State: " << packet->state<< std::endl;
    received_battery_charge = packet->battery_charge;
    received_state = packet->state;
    received ++;
}

static float received_mass;

void handle_science_measure(uint8_t sender_id, Science_MeasurePacket* packet) {
    std::cout << "Measured Mass: " << packet->mass << std::endl;
    received_mass = packet->mass;
    received ++;
}

static uint32_t received_data;

void handle_data(uint8_t sender_id, DataPacket* packet) {
    std::cout << "Data: " << packet->data << std::endl;
    received_data = packet->data;
    received ++;
}

TEST(Protocol21W3TestSuite, PingPacketPortBTest) {

    std::cout << "Starting main test..." << std::endl;
    received = 0;

    NetworkServerIO* server_io = new NetworkServerIO(PORT_B);


    int32_t result = server_io->connectServer();

    if(result < 0) {
        std::cout << "Network Server IO connection failed with error code " << result << std::endl;
        std::cout << std::strerror(errno) << std::endl;
    } else {
        std::cout << "Connected to network server IO" << std::endl;
    }

    NetworkClientIO* client_io_1 = new NetworkClientIO("127.0.0.1", PORT_B);


    result = client_io_1->connectClient();

    if(result < 0) {
        std::cout << "Network Client IO connection failed with error code " << result << std::endl;
        std::cout << std::strerror(errno) << std::endl;
    }

    NetworkClientIO* client_io_2 = new NetworkClientIO("127.0.0.1", PORT_B);


    result = client_io_2->connectClient();

    if(result < 0) {
        std::cout << "Network Client IO connection failed with error code " << result << std::endl;
        std::cout << std::strerror(errno) << std::endl;
    }

    PingPacket packet;

    uint64_t initial_time = packet.time;

    NetworkBus* server_bus = new NetworkBus(server_io);
    NetworkBus* client_1_bus = new NetworkBus(client_io_1);
    NetworkBus* client_2_bus = new NetworkBus(client_io_2);


    server_bus->forward<PingPacket>(server_bus);
    client_2_bus->handle(handle_ping);

    client_1_bus->send(&packet);

    std::cout << "Test finished" << std::endl;

    std::this_thread::sleep_for(std::chrono::nanoseconds(3000000000));

    EXPECT_TRUE(initial_time == received_time);
    EXPECT_TRUE(received == 1);
}

TEST(Protocol21W3TestSuite, PingPacketPortATest) {

    std::cout << "Starting main test..." << std::endl;
    received = 0;

    NetworkServerIO* server_io = new NetworkServerIO(PORT_A);

    // server_io->receive(&handle_input);

    int32_t result = server_io->connectServer();

    if(result < 0) {
        std::cout << "Network Server IO connection failed with error code " << result << std::endl;
        std::cout << std::strerror(errno) << std::endl;
    } else {
        std::cout << "Connected to network server IO" << std::endl;
    }

    NetworkClientIO* client_io_1 = new NetworkClientIO("127.0.0.1", PORT_A);


    result = client_io_1->connectClient();

    if(result < 0) {
        std::cout << "Network Client IO connection failed with error code " << result << std::endl;
        std::cout << std::strerror(errno) << std::endl;
    }

    NetworkClientIO* client_io_2 = new NetworkClientIO("127.0.0.1", PORT_A);


    result = client_io_2->connectClient();

    if(result < 0) {
        std::cout << "Network Client IO connection failed with error code " << result << std::endl;
        std::cout << std::strerror(errno) << std::endl;
    }

    PingPacket packet;

    uint64_t initial_time = packet.time;

    NetworkBus* server_bus = new NetworkBus(server_io);
    NetworkBus* client_1_bus = new NetworkBus(client_io_1);
    NetworkBus* client_2_bus = new NetworkBus(client_io_2);


    server_bus->forward<PingPacket>(server_bus);
    client_2_bus->handle(handle_ping);

    client_1_bus->send(&packet);

    std::cout << "Test finished" << std::endl;

    std::this_thread::sleep_for(std::chrono::nanoseconds(3000000000));

    EXPECT_TRUE(initial_time == received_time);
    EXPECT_TRUE(received == 1);
}


TEST(Protocol21W3TestSuite, PingPacketRandomPortNumbersTest) {

    std::cout << "Starting main test..." << std::endl;

    // it will take approx 3 sec per test
    int numberOfTests = 10;

    for(int i = 0; i < numberOfTests; i++){

        std::cout << "Starting test..." << std::endl;
        received = 0;

        srand(time(0));

        int rand_port = rand() % (49151 - 1024) + 1024;

        NetworkServerIO* server_io = new NetworkServerIO(rand_port);


        int32_t result = server_io->connectServer();

        if(result < 0) {
            std::cout << "Network Server IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        } else {
            std::cout << "Connected to network server IO" << std::endl;
        }

        NetworkClientIO* client_io_1 = new NetworkClientIO("127.0.0.1", rand_port);

        result = client_io_1->connectClient();

        if(result < 0) {
            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        }

        NetworkClientIO* client_io_2 = new NetworkClientIO("127.0.0.1", rand_port);


        result = client_io_2->connectClient();

        if(result < 0) {
            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        }

        PingPacket packet;

        uint64_t initial_time = packet.time;

        NetworkBus* server_bus = new NetworkBus(server_io);
        NetworkBus* client_1_bus = new NetworkBus(client_io_1);
        NetworkBus* client_2_bus = new NetworkBus(client_io_2);


        server_bus->forward<PingPacket>(server_bus);
        client_2_bus->handle(handle_ping);

        client_1_bus->send(&packet);

        std::cout << "Test finished" << std::endl;

        std::this_thread::sleep_for(std::chrono::nanoseconds(3000000000));

        EXPECT_TRUE(initial_time == received_time);
        EXPECT_TRUE(received == 1);
    }
}


TEST(Protocol21W3TestSuite, RequestPacketRandomPortNumbersTest) {

    std::cout << "Starting main test..." << std::endl;

    // it will take approx 3 sec per test
    int numberOfTests = 10;

    for (int i = 0; i < numberOfTests; i++) {

        std::cout << "Starting test..." << std::endl;
        received = 0;

        srand(time(0));

        int rand_port = rand() % (49151 - 1024) + 1024;

        NetworkServerIO *server_io = new NetworkServerIO(rand_port);


        int32_t result = server_io->connectServer();

        if (result < 0) {
            std::cout << "Network Server IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        } else {
            std::cout << "Connected to network server IO" << std::endl;
        }

        NetworkClientIO *client_io_1 = new NetworkClientIO("127.0.0.1", rand_port);

        result = client_io_1->connectClient();

        if (result < 0) {
            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        }

        NetworkClientIO *client_io_2 = new NetworkClientIO("127.0.0.1", rand_port);


        result = client_io_2->connectClient();

        if (result < 0) {
            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        }

        RequestPacket packet;

        uint16_t sent_uuid = packet.uuid;
        uint8_t sent_action_id = packet.action_id;
        uint8_t sent_target_id = packet.target_id;
        uint32_t sent_payload = packet.payload;

        NetworkBus *server_bus = new NetworkBus(server_io);
        NetworkBus *client_1_bus = new NetworkBus(client_io_1);
        NetworkBus *client_2_bus = new NetworkBus(client_io_2);


        server_bus->forward<RequestPacket>(server_bus);
        client_2_bus->handle(handle_request);

        client_1_bus->send(&packet);

        std::cout << "Test finished" << std::endl;

        std::this_thread::sleep_for(std::chrono::nanoseconds(3000000000));

        EXPECT_TRUE(received_uuid == sent_uuid);
        EXPECT_TRUE(received_action_id == sent_action_id);
        EXPECT_TRUE(received_target_id == sent_target_id);
        EXPECT_TRUE(received_payload == sent_payload);
        EXPECT_TRUE(received == 1);
    }
}

TEST(Protocol21W3TestSuite, ResponsePacketRandomPortNumbersTest) {

    std::cout << "Starting main test..." << std::endl;

    // it will take approx 3 sec per test
    int numberOfTests = 10;

    for (int i = 0; i < numberOfTests; i++) {

        std::cout << "Starting test..." << std::endl;
        received = 0;

        srand(time(0));

        int rand_port = rand() % (49151 - 1024) + 1024;

        NetworkServerIO *server_io = new NetworkServerIO(rand_port);


        int32_t result = server_io->connectServer();

        if (result < 0) {
            std::cout << "Network Server IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        } else {
            std::cout << "Connected to network server IO" << std::endl;
        }

        NetworkClientIO *client_io_1 = new NetworkClientIO("127.0.0.1", rand_port);

        result = client_io_1->connectClient();

        if (result < 0) {
            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        }

        NetworkClientIO *client_io_2 = new NetworkClientIO("127.0.0.1", rand_port);


        result = client_io_2->connectClient();

        if (result < 0) {
            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        }

        ResponsePacket packet;

        uint16_t sent_uuid = packet.uuid;
        uint8_t sent_action_id = packet.action_id;
        uint8_t sent_target_id = packet.target_id;
        uint32_t sent_payload = packet.payload;

        NetworkBus *server_bus = new NetworkBus(server_io);
        NetworkBus *client_1_bus = new NetworkBus(client_io_1);
        NetworkBus *client_2_bus = new NetworkBus(client_io_2);


        server_bus->forward<ResponsePacket>(server_bus);
        client_2_bus->handle(handle_response);

        client_1_bus->send(&packet);

        std::cout << "Test finished" << std::endl;

        std::this_thread::sleep_for(std::chrono::nanoseconds(3000000000));

        EXPECT_TRUE(received_uuid == sent_uuid);
        EXPECT_TRUE(received_action_id == sent_action_id);
        EXPECT_TRUE(received_target_id == sent_target_id);
        EXPECT_TRUE(received_payload == sent_payload);
        EXPECT_TRUE(received == 1);
    }
}

TEST(Protocol21W3TestSuite, ProgressPacketRandomPortNumbersTest) {

    std::cout << "Starting main test..." << std::endl;

    // it will take approx 3 sec per test
    int numberOfTests = 10;

    for (int i = 0; i < numberOfTests; i++) {

        std::cout << "Starting test..." << std::endl;
        received = 0;

        srand(time(0));

        int rand_port = rand() % (49151 - 1024) + 1024;

        NetworkServerIO *server_io = new NetworkServerIO(rand_port);


        int32_t result = server_io->connectServer();

        if (result < 0) {
            std::cout << "Network Server IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        } else {
            std::cout << "Connected to network server IO" << std::endl;
        }

        NetworkClientIO *client_io_1 = new NetworkClientIO("127.0.0.1", rand_port);

        result = client_io_1->connectClient();

        if (result < 0) {
            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        }

        NetworkClientIO *client_io_2 = new NetworkClientIO("127.0.0.1", rand_port);


        result = client_io_2->connectClient();

        if (result < 0) {
            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        }

        ProgressPacket packet;

        uint32_t sent_uuid = packet.uuid;
        uint8_t sent_progress = packet.progress;

        NetworkBus *server_bus = new NetworkBus(server_io);
        NetworkBus *client_1_bus = new NetworkBus(client_io_1);
        NetworkBus *client_2_bus = new NetworkBus(client_io_2);


        server_bus->forward<ProgressPacket>(server_bus);
        client_2_bus->handle(handle_progress);

        client_1_bus->send(&packet);

        std::cout << "Test finished" << std::endl;

        std::this_thread::sleep_for(std::chrono::nanoseconds(3000000000));

        EXPECT_TRUE(received_progress_uuid == sent_uuid);
        EXPECT_TRUE(received_progress == sent_progress);
        EXPECT_TRUE(received == 1);
    }
}

TEST(Protocol21W3TestSuite, ErrorPacketRandomPortNumbersTest) {

    std::cout << "Starting main test..." << std::endl;

    // it will take approx 3 sec per test
    int numberOfTests = 10;

    for (int i = 0; i < numberOfTests; i++) {

        std::cout << "Starting test..." << std::endl;
        received = 0;

        srand(time(0));

        int rand_port = rand() % (49151 - 1024) + 1024;

        NetworkServerIO *server_io = new NetworkServerIO(rand_port);


        int32_t result = server_io->connectServer();

        if (result < 0) {
            std::cout << "Network Server IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        } else {
            std::cout << "Connected to network server IO" << std::endl;
        }

        NetworkClientIO *client_io_1 = new NetworkClientIO("127.0.0.1", rand_port);

        result = client_io_1->connectClient();

        if (result < 0) {
            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        }

        NetworkClientIO *client_io_2 = new NetworkClientIO("127.0.0.1", rand_port);


        result = client_io_2->connectClient();

        if (result < 0) {
            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        }

        ErrorPacket packet;

        uint8_t sent_error_id = packet.error_id;

        NetworkBus *server_bus = new NetworkBus(server_io);
        NetworkBus *client_1_bus = new NetworkBus(client_io_1);
        NetworkBus *client_2_bus = new NetworkBus(client_io_2);


        server_bus->forward<ErrorPacket>(server_bus);
        client_2_bus->handle(handle_error);

        client_1_bus->send(&packet);

        std::cout << "Test finished" << std::endl;

        std::this_thread::sleep_for(std::chrono::nanoseconds(3000000000));
        EXPECT_TRUE(received_error_id == sent_error_id);
        EXPECT_TRUE(received == 1);
    }
}


TEST(Protocol21W3TestSuite, Avionics_BaroTempPacketRandomPortNumbersTest) {

    std::cout << "Starting main test..." << std::endl;

    // it will take approx 3 sec per test
    int numberOfTests = 10;

    for (int i = 0; i < numberOfTests; i++) {

        std::cout << "Starting test..." << std::endl;
        received = 0;

        srand(time(0));

        int rand_port = rand() % (49151 - 1024) + 1024;

        NetworkServerIO *server_io = new NetworkServerIO(rand_port);


        int32_t result = server_io->connectServer();

        if (result < 0) {
            std::cout << "Network Server IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        } else {
            std::cout << "Connected to network server IO" << std::endl;
        }

        NetworkClientIO *client_io_1 = new NetworkClientIO("127.0.0.1", rand_port);

        result = client_io_1->connectClient();

        if (result < 0) {
            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        }

        NetworkClientIO *client_io_2 = new NetworkClientIO("127.0.0.1", rand_port);


        result = client_io_2->connectClient();

        if (result < 0) {
            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        }

        Avionics_BaroTempPacket packet;

        float sent_pressure = packet.pressure;
        float sent_temperature = packet.temperature;

        NetworkBus *server_bus = new NetworkBus(server_io);
        NetworkBus *client_1_bus = new NetworkBus(client_io_1);
        NetworkBus *client_2_bus = new NetworkBus(client_io_2);


        server_bus->forward<Avionics_BaroTempPacket>(server_bus);
        client_2_bus->handle(handle_avionics_barotemp);

        client_1_bus->send(&packet);

        std::cout << "Test finished" << std::endl;

        std::this_thread::sleep_for(std::chrono::nanoseconds(3000000000));
        EXPECT_TRUE(received_pressure == sent_pressure);
        EXPECT_TRUE(received_temperature == sent_temperature);
        EXPECT_TRUE(received == 1);
    }
}

TEST(Protocol21W3TestSuite, Avionics_AccelMagPacketRandomPortNumbersTest) {

    std::cout << "Starting main test..." << std::endl;

    // it will take approx 3 sec per test
    int numberOfTests = 10;

    for (int i = 0; i < numberOfTests; i++) {

        std::cout << "Starting test..." << std::endl;
        received = 0;

        srand(time(0));

        int rand_port = rand() % (49151 - 1024) + 1024;

        NetworkServerIO *server_io = new NetworkServerIO(rand_port);


        int32_t result = server_io->connectServer();

        if (result < 0) {
            std::cout << "Network Server IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        } else {
            std::cout << "Connected to network server IO" << std::endl;
        }

        NetworkClientIO *client_io_1 = new NetworkClientIO("127.0.0.1", rand_port);

        result = client_io_1->connectClient();

        if (result < 0) {
            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        }

        NetworkClientIO *client_io_2 = new NetworkClientIO("127.0.0.1", rand_port);


        result = client_io_2->connectClient();

        if (result < 0) {
            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        }

        Avionics_AccelMagPacket packet;

        float sent_acceleration[3];
        float sent_angular[3];
        float sent_magneto[3];

        for (int j = 0; j < 3; ++j) {
            sent_acceleration[j] = packet.acceleration[j];
            sent_angular[j] = packet.angular[j];
            sent_magneto[j] = packet.magneto[j];
        }

        NetworkBus *server_bus = new NetworkBus(server_io);
        NetworkBus *client_1_bus = new NetworkBus(client_io_1);
        NetworkBus *client_2_bus = new NetworkBus(client_io_2);


        server_bus->forward<Avionics_AccelMagPacket>(server_bus);
        client_2_bus->handle(handle_avionics_accelmag);

        client_1_bus->send(&packet);

        std::cout << "Test finished" << std::endl;

        std::this_thread::sleep_for(std::chrono::nanoseconds(3000000000));
        for (int j = 0; j < 3; ++j) {
            EXPECT_TRUE(sent_acceleration[j] == received_acceleration[j]);
            EXPECT_TRUE(sent_angular[j] == received_angular[j]);
            EXPECT_TRUE(sent_magneto[j] == received_magneto[j]);
        }
        EXPECT_TRUE(received == 1);
    }
}

TEST(Protocol21W3TestSuite, Handling_GripperPacketRandomPortNumbersTest) {

    std::cout << "Starting main test..." << std::endl;

    // it will take approx 3 sec per test
    int numberOfTests = 10;

    for (int i = 0; i < numberOfTests; i++) {

        std::cout << "Starting test..." << std::endl;
        received = 0;

        srand(time(0));

        int rand_port = rand() % (49151 - 1024) + 1024;

        NetworkServerIO *server_io = new NetworkServerIO(rand_port);


        int32_t result = server_io->connectServer();

        if (result < 0) {
            std::cout << "Network Server IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        } else {
            std::cout << "Connected to network server IO" << std::endl;
        }

        NetworkClientIO *client_io_1 = new NetworkClientIO("127.0.0.1", rand_port);

        result = client_io_1->connectClient();

        if (result < 0) {
            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        }

        NetworkClientIO *client_io_2 = new NetworkClientIO("127.0.0.1", rand_port);


        result = client_io_2->connectClient();

        if (result < 0) {
            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        }

        Handling_GripperPacket packet;

        float sent_voltage = packet.voltage;

        NetworkBus *server_bus = new NetworkBus(server_io);
        NetworkBus *client_1_bus = new NetworkBus(client_io_1);
        NetworkBus *client_2_bus = new NetworkBus(client_io_2);


        server_bus->forward<Handling_GripperPacket>(server_bus);
        client_2_bus->handle(handle_handling_gripper);

        client_1_bus->send(&packet);

        std::cout << "Test finished" << std::endl;

        std::this_thread::sleep_for(std::chrono::nanoseconds(3000000000));

        EXPECT_TRUE(received_voltage == sent_voltage);
        EXPECT_TRUE(received == 1);
    }
}

TEST(Protocol21W3TestSuite, Power_VoltagePacketRandomPortNumbersTest) {

    std::cout << "Starting main test..." << std::endl;

    // it will take approx 3 sec per test
    int numberOfTests = 10;

    for (int i = 0; i < numberOfTests; i++) {

        std::cout << "Starting test..." << std::endl;
        received = 0;

        srand(time(0));

        int rand_port = rand() % (49151 - 1024) + 1024;

        NetworkServerIO *server_io = new NetworkServerIO(rand_port);


        int32_t result = server_io->connectServer();

        if (result < 0) {
            std::cout << "Network Server IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        } else {
            std::cout << "Connected to network server IO" << std::endl;
        }

        NetworkClientIO *client_io_1 = new NetworkClientIO("127.0.0.1", rand_port);

        result = client_io_1->connectClient();

        if (result < 0) {
            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        }

        NetworkClientIO *client_io_2 = new NetworkClientIO("127.0.0.1", rand_port);


        result = client_io_2->connectClient();

        if (result < 0) {
            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        }

        Power_VoltagePacket packet;

        float sent_voltages[4];

        for (int j = 0; j < 4; ++j) {
            sent_voltages[j] = packet.voltages[j];
        }

        NetworkBus *server_bus = new NetworkBus(server_io);
        NetworkBus *client_1_bus = new NetworkBus(client_io_1);
        NetworkBus *client_2_bus = new NetworkBus(client_io_2);


        server_bus->forward<Power_VoltagePacket>(server_bus);
        client_2_bus->handle(handle_power_voltage);

        client_1_bus->send(&packet);

        std::cout << "Test finished" << std::endl;

        std::this_thread::sleep_for(std::chrono::nanoseconds(3000000000));

        for (int j = 0; j < 4; ++j) {
            EXPECT_TRUE(sent_voltages[j] == received_voltages[j]);
        }
        EXPECT_TRUE(received == 1);
    }
}

TEST(Protocol21W3TestSuite, Power_CurrentPacketRandomPortNumbersTest) {

    std::cout << "Starting main test..." << std::endl;

    // it will take approx 3 sec per test
    int numberOfTests = 10;

    for (int i = 0; i < numberOfTests; i++) {

        std::cout << "Starting test..." << std::endl;
        received = 0;

        srand(time(0));

        int rand_port = rand() % (49151 - 1024) + 1024;

        NetworkServerIO *server_io = new NetworkServerIO(rand_port);


        int32_t result = server_io->connectServer();

        if (result < 0) {
            std::cout << "Network Server IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        } else {
            std::cout << "Connected to network server IO" << std::endl;
        }

        NetworkClientIO *client_io_1 = new NetworkClientIO("127.0.0.1", rand_port);

        result = client_io_1->connectClient();

        if (result < 0) {
            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        }

        NetworkClientIO *client_io_2 = new NetworkClientIO("127.0.0.1", rand_port);


        result = client_io_2->connectClient();

        if (result < 0) {
            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        }

        Power_CurrentPacket packet;

        float sent_currents[4];

        for (int j = 0; j < 4; ++j) {
            sent_currents[j] = packet.currents[j];
        }

        NetworkBus *server_bus = new NetworkBus(server_io);
        NetworkBus *client_1_bus = new NetworkBus(client_io_1);
        NetworkBus *client_2_bus = new NetworkBus(client_io_2);


        server_bus->forward<Power_CurrentPacket>(server_bus);
        client_2_bus->handle(handle_power_current);

        client_1_bus->send(&packet);

        std::cout << "Test finished" << std::endl;

        std::this_thread::sleep_for(std::chrono::nanoseconds(3000000000));

        for (int j = 0; j < 4; ++j) {
            EXPECT_TRUE(sent_currents[j] == received_currents[j]);
        }
        EXPECT_TRUE(received == 1);
    }
}

TEST(Protocol21W3TestSuite, Power_SystemPacketRandomPortNumbersTest) {

    std::cout << "Starting main test..." << std::endl;

    // it will take approx 3 sec per test
    int numberOfTests = 10;

    for (int i = 0; i < numberOfTests; i++) {

        std::cout << "Starting test..." << std::endl;
        received = 0;

        srand(time(0));

        int rand_port = rand() % (49151 - 1024) + 1024;

        NetworkServerIO *server_io = new NetworkServerIO(rand_port);


        int32_t result = server_io->connectServer();

        if (result < 0) {
            std::cout << "Network Server IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        } else {
            std::cout << "Connected to network server IO" << std::endl;
        }

        NetworkClientIO *client_io_1 = new NetworkClientIO("127.0.0.1", rand_port);

        result = client_io_1->connectClient();

        if (result < 0) {
            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        }

        NetworkClientIO *client_io_2 = new NetworkClientIO("127.0.0.1", rand_port);


        result = client_io_2->connectClient();

        if (result < 0) {
            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        }

        Power_SystemPacket packet;

        float sent_battery_charge = packet.battery_charge;
        uint8_t sent_state = packet.state;

        NetworkBus *server_bus = new NetworkBus(server_io);
        NetworkBus *client_1_bus = new NetworkBus(client_io_1);
        NetworkBus *client_2_bus = new NetworkBus(client_io_2);


        server_bus->forward<Power_SystemPacket>(server_bus);
        client_2_bus->handle(handle_power_system);

        client_1_bus->send(&packet);

        std::cout << "Test finished" << std::endl;

        std::this_thread::sleep_for(std::chrono::nanoseconds(3000000000));

        EXPECT_TRUE(received_battery_charge == sent_battery_charge);
        EXPECT_TRUE(received_state == sent_state);
        EXPECT_TRUE(received == 1);
    }
}

TEST(Protocol21W3TestSuite, Science_MeasurePacketRandomPortNumbersTest) {

    std::cout << "Starting main test..." << std::endl;

    // it will take approx 3 sec per test
    int numberOfTests = 10;

    for (int i = 0; i < numberOfTests; i++) {

        std::cout << "Starting test..." << std::endl;
        received = 0;

        srand(time(0));

        int rand_port = rand() % (49151 - 1024) + 1024;

        NetworkServerIO *server_io = new NetworkServerIO(rand_port);


        int32_t result = server_io->connectServer();

        if (result < 0) {
            std::cout << "Network Server IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        } else {
            std::cout << "Connected to network server IO" << std::endl;
        }

        NetworkClientIO *client_io_1 = new NetworkClientIO("127.0.0.1", rand_port);

        result = client_io_1->connectClient();

        if (result < 0) {
            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        }

        NetworkClientIO *client_io_2 = new NetworkClientIO("127.0.0.1", rand_port);


        result = client_io_2->connectClient();

        if (result < 0) {
            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        }

        Science_MeasurePacket packet;

        float sent_mass = packet.mass;

        NetworkBus *server_bus = new NetworkBus(server_io);
        NetworkBus *client_1_bus = new NetworkBus(client_io_1);
        NetworkBus *client_2_bus = new NetworkBus(client_io_2);


        server_bus->forward<Science_MeasurePacket>(server_bus);
        client_2_bus->handle(handle_science_measure);

        client_1_bus->send(&packet);

        std::cout << "Test finished" << std::endl;

        std::this_thread::sleep_for(std::chrono::nanoseconds(3000000000));

        EXPECT_TRUE(received_mass == sent_mass);
        EXPECT_TRUE(received == 1);
    }
}

TEST(Protocol21W3TestSuite, DataPacketRandomPortNumbersTest) {

    std::cout << "Starting main test..." << std::endl;

    // it will take approx 3 sec per test
    int numberOfTests = 10;

    for (int i = 0; i < numberOfTests; i++) {

        std::cout << "Starting test..." << std::endl;
        received = 0;

        srand(time(0));

        int rand_port = rand() % (49151 - 1024) + 1024;

        NetworkServerIO *server_io = new NetworkServerIO(rand_port);


        int32_t result = server_io->connectServer();

        if (result < 0) {
            std::cout << "Network Server IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        } else {
            std::cout << "Connected to network server IO" << std::endl;
        }

        NetworkClientIO *client_io_1 = new NetworkClientIO("127.0.0.1", rand_port);

        result = client_io_1->connectClient();

        if (result < 0) {
            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        }

        NetworkClientIO *client_io_2 = new NetworkClientIO("127.0.0.1", rand_port);


        result = client_io_2->connectClient();

        if (result < 0) {
            std::cout << "Network Client IO connection failed with error code " << result << std::endl;
            std::cout << std::strerror(errno) << std::endl;
        }

        DataPacket packet;

        uint32_t sentData = packet.data;


        NetworkBus *server_bus = new NetworkBus(server_io);
        NetworkBus *client_1_bus = new NetworkBus(client_io_1);
        NetworkBus *client_2_bus = new NetworkBus(client_io_2);


        server_bus->forward<DataPacket>(server_bus);
        client_2_bus->handle(handle_data);

        client_1_bus->send(&packet);

        std::cout << "Test finished" << std::endl;

        std::this_thread::sleep_for(std::chrono::nanoseconds(3000000000));

        EXPECT_TRUE(sentData == received_data);

        EXPECT_TRUE(received == 1);
    }
}



