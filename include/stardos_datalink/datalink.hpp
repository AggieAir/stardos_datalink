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
        // ROS node name
        std::string name;
        // MAVLink system ID
        uint8_t sysid;
        // MAVLink component ID
        uint8_t compid;
        // Whether or not should send heartbeats
        bool heartbeat;
        // URL to connect to
        // can be UDP, TCP, or serial
        std::string connection_url;
        // Instance of MAVSDK -- this models the connection
  	mavsdk::Mavsdk dc;
        // This allows us to send our own messages here
	std::shared_ptr<mavsdk::MavlinkPassthrough> passthrough;
        // Reference to other MAVLink system
	std::shared_ptr<mavsdk::System> drone;
        // Publisher to report heartbeat data
        rclcpp::Publisher<stardos_interfaces::msg::NodeHeartbeat>::SharedPtr publisher;

        // Wrapper around Mavsdk::set_configuration
	void configure(uint8_t sysid, uint8_t compid, bool heartbeat);
        // Bind to the connection_url
	void connect();
        // Send a telemetry packet
	void send();
        // Check to see if there is another system; connect if so
        void check_systems();
        // Runs every 100ms
        void timer_callback();
};

#endif //DATALINK
