#include <iostream>
#include <chrono>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <string.h>
#include <math.h>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "stardos_interfaces/msg/node_heartbeat.hpp"

#include "datalink.hpp"

using namespace mavsdk;
using namespace std::literals::chrono_literals;

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

        RCLCPP_DEBUG(this->get_logger(), "Subscribing to system feed");

	dc.subscribe_on_new_system(std::bind(&Datalink::check_systems, this));

        RCLCPP_DEBUG(this->get_logger(), "Creating telemetry publisher");

        publisher = this->create_publisher<stardos_interfaces::msg::NodeHeartbeat>(
                        name + "/telemetry",
                        10);

        RCLCPP_DEBUG(this->get_logger(), "Creating MAVLink Passthrough");

        RCLCPP_DEBUG(this->get_logger(), "Binding timer callback");

	this->create_wall_timer(100ms, std::bind(&Datalink::timer_callback, this));
}

void Datalink::configure(uint8_t sysid, uint8_t compid, bool heartbeat) {
	dc.set_configuration(
		Mavsdk::Configuration(sysid, compid, heartbeat)
        );
}

void Datalink::connect() {
	ConnectionResult connection_result = 
		dc.add_any_connection(connection_url);
	RCLCPP_INFO(this->get_logger(), "Connection was a %s\n", connection_result);
}

void Datalink::send() {
        const float data[] = {
                1.8,
                80.0,
                3.33333
        };

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
        
        passthrough->send_message(message);

        std::cout << "sent!\n";
}

void Datalink::check_systems() {
        auto maybe_drone = dc.systems().back();

        RCLCPP_INFO(this->get_logger(), "Found autopilot");
        dc.subscribe_on_new_system(nullptr);
        drone = maybe_drone;
        passthrough = std::make_shared<MavlinkPassthrough>(drone);
}

void Datalink::timer_callback() {
        RCLCPP_INFO(this->get_logger(), "meme");
        if (drone == nullptr) return;

        send();
}
