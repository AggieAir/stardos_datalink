#include <iostream>
#include <chrono>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/log_files/log_files.h>
#include <thread>
#include <string.h>
#include <math.h>
#include "sys/types.h"
#include "sys/sysinfo.h"
#include "sys/statvfs.h"
#include <fstream>
#include <ctime>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "stardos_interfaces/msg/node_heartbeat.hpp"

#include "datalink.hpp"

using namespace mavsdk;

Datalink::Datalink(std::string name, uint8_t sysid, uint8_t compid, bool heartbeat, std::string connection_url):
        Node(name),
        name{name}
{
	configure(sysid, compid, heartbeat);
	connect(connection_url);

	drone = get_system(dc);
	passthrough = std::make_shared<MavlinkPassthrough>(drone);

        publisher = this->create_publisher<stardos_interfaces::msg::NodeHeartbeat>(
                        name + "/telemetry",
                        10);
	  
	start_downlink();
}

Datalink::~Datalink() {
	std::cout << "Destroying Datalink..." << std::endl;
}

void Datalink::start_downlink() {
	downlink_thread = std::thread(&Datalink::downlink_status, this);
}

void Datalink::downlink_status() {
	std::string name = "Copilot";
	do{

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

                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                std::cout << "sent!";
	
	}while (true);
}

void Datalink::configure(uint8_t sysid, uint8_t compid, bool heartbeat) {
	dc.set_configuration(
		Mavsdk::Configuration(sysid, compid, heartbeat)
        );
}

void Datalink::connect(std::string connection_url) {
	mavsdk::ConnectionResult connection_result = 
		dc.add_any_connection(connection_url);
	std::cout << "Connection was a " << connection_result << std::endl;
}

std::shared_ptr<mavsdk::System> Datalink::get_system(Mavsdk& dc) {
	std::cout << "Waiting to discover system..." << std::endl;
	auto prom = std::promise<std::shared_ptr<System>>{};
	auto fut = prom.get_future();

	dc.subscribe_on_new_system([&dc, &prom]()
	{
		auto maybe_drone = dc.systems().back();

		if(maybe_drone->has_autopilot())
		{
			std::cout << "Found Autopilot" << std::endl;
			dc.subscribe_on_new_system(nullptr);
			prom.set_value(maybe_drone);
		}
	});

	if(fut.wait_for(std::chrono::seconds(300)) == std::future_status::timeout)
	{
		std::cout <<"No Autopilot found" << std::endl;
		return{};
	}

	return fut.get();
}
