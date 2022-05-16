#ifndef DATALINK_HPP
#define DATALINK_HPP

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <thread>

#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "stardos_interfaces/msg/node_heartbeat.hpp"

class Datalink: public rclcpp::Node
{
	public:
	Datalink(std::string name, uint8_t sysid, uint8_t compid, bool heartbeat, std::string connection_url);
	~Datalink();

	private:
        std::string name;
  	mavsdk::Mavsdk dc;
	std::shared_ptr<mavsdk::MavlinkPassthrough> passthrough;
	std::shared_ptr<mavsdk::System> drone;
	std::mutex data_lock;
	std::thread downlink_thread;
        rclcpp::Publisher<stardos_interfaces::msg::NodeHeartbeat>::SharedPtr publisher;
	uint64_t uuid;

	void async_thread();
	void configure(uint8_t sysid, uint8_t compid, bool heartbeat);
	void connect(std::string connection_url);
	std::shared_ptr<mavsdk::System> get_system(mavsdk::Mavsdk& dc);
	void start_downlink();
	void downlink_status();
};

#endif //DATALINK
