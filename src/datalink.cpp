#include <functional>
#include <iostream>
#include <chrono>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink/v2.0/common/mavlink_msg_debug_float_array.h>
#include <string.h>
#include <math.h>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "stardos_interfaces/msg/node_heartbeat.hpp"

#include "datalink.hpp"

using namespace mavsdk;
using namespace std::literals::chrono_literals;
using namespace std::placeholders;
using stardos_interfaces::msg::NodeHeartbeat;

Datalink::Datalink(std::string name, uint8_t sysid, uint8_t compid, bool heartbeat, std::string connection_url):
        Node(name),
        name{name},
        sysid{sysid},
        compid{compid},
        heartbeat{heartbeat},
        connection_url{connection_url}
{
	configure(sysid, compid, heartbeat);
	connect();

        std::cout << dc.systems().size() << " systems connected\n";

        RCLCPP_INFO(this->get_logger(), "Subscribing to system feed");

	dc.subscribe_on_new_system(std::bind(&Datalink::check_systems, this));

        RCLCPP_INFO(this->get_logger(), "Creating telemetry publisher");

        publisher = this->create_publisher<NodeHeartbeat>(
                        name + "/telemetry",
                        10);

        subscription = this->create_subscription<NodeHeartbeat>(
                        "/heartbeat",
                        10,
                        std::bind(&Datalink::subscription_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Creating MAVLink Passthrough");

        RCLCPP_INFO(this->get_logger(), "Binding timer callback");

	get_system_timer = this->create_wall_timer(100ms, std::bind(&Datalink::timer_callback, this));
}

void Datalink::configure(uint8_t sysid, uint8_t compid, bool heartbeat) {
	dc.set_configuration(
		Mavsdk::Configuration(sysid, compid, heartbeat)
        );
}

void Datalink::connect() {
        dc.add_any_connection(connection_url);
}

void Datalink::send(float data[]) {
        mavlink_message_t message;

        mavlink_msg_debug_float_array_pack(
                passthrough->get_our_sysid(), // SystemID
                passthrough->get_our_compid(), //My comp ID
                &message, //Message reference
                1, //timeing is 1 sec
                name.c_str(),
                5,
                data 
        );
        
        MavlinkPassthrough::Result result = passthrough->send_message(message);

        if (result != MavlinkPassthrough::Result::Success) {
                std::cout << "command send failed: " << result << "\n";
        }
}

void Datalink::check_systems() {
        auto maybe_drone = dc.systems().back();

        RCLCPP_INFO(this->get_logger(), "Found autopilot");
        dc.subscribe_on_new_system(nullptr);
        drone = maybe_drone;
        passthrough = std::make_shared<MavlinkPassthrough>(drone);
        passthrough->subscribe_message_async(
                        MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY, [this] (mavlink_message_t msg) {
                                this->telemetry_received_callback(msg);
                        });
}

void Datalink::timer_callback() {
        if (dc.systems().size() != 0) {
                check_systems();
                get_system_timer->cancel();
        }
}

void Datalink::subscription_callback(NodeHeartbeat::SharedPtr msg) {
        if (passthrough == nullptr) return;
        float data[2];
        pack_heartbeat_message(msg, data);
        send(data);
}

void Datalink::telemetry_received_callback(mavlink_message_t msg) {
        RCLCPP_INFO(this->get_logger(), "sending packet");
        auto decoded = NodeHeartbeat();

        mavlink_debug_float_array_t * floats = new mavlink_debug_float_array_t();
        mavlink_msg_debug_float_array_decode(&msg, floats);

        unpack_heartbeat_message(&decoded, floats->data);
        this->publisher->publish(decoded);
}

void Datalink::pack_heartbeat_message(NodeHeartbeat::SharedPtr msg, float destination[2]) {
        uint16_t *truedest = (uint16_t*) destination;
        
        truedest[0] = msg->state;
        truedest[1] = msg->errors;
        truedest[2] = msg->requests;
        truedest[3] = msg->failures;
}

void Datalink::unpack_heartbeat_message(NodeHeartbeat *msg, float source[2]) {
        uint16_t *truesource = (uint16_t*) source;
        
        msg->state = truesource[0];
        msg->errors = truesource[1];
        msg->requests = truesource[2];
        msg->failures = truesource[3];
}
