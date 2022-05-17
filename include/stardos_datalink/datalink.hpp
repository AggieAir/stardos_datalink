#ifndef DATALINK_HPP
#define DATALINK_HPP

#include <functional>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
// #include <thread>

#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "stardos_interfaces/msg/node_heartbeat.hpp"

class Datalink: public rclcpp::Node
{
	public:
	Datalink(std::string name, uint8_t sysid, uint8_t compid, bool heartbeat, std::string connection_url);

	private:
        std::string name;
        uint8_t sysid;
        uint8_t compid;
        bool heartbeat;
        std::string connection_url;
  	mavsdk::Mavsdk dc;
	std::shared_ptr<mavsdk::MavlinkPassthrough> passthrough;
	std::shared_ptr<mavsdk::System> drone;
	std::mutex data_lock;
        rclcpp::Publisher<stardos_interfaces::msg::NodeHeartbeat>::SharedPtr publisher;
	uint64_t uuid;

	void configure(uint8_t sysid, uint8_t compid, bool heartbeat);
	void connect(std::string connection_url);
	void send();
        void check_systems();
        void timer_callback();
};

#endif //DATALINK
