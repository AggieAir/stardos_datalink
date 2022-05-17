#ifndef DATALINK_HPP
#define DATALINK_HPP

#include <functional>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink/v2.0/mavlink_types.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
// #include <thread>

#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "stardos_interfaces/msg/node_heartbeat.hpp"

using stardos_interfaces::msg::NodeHeartbeat;

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
        rclcpp::TimerBase::SharedPtr get_system_timer;
        // This allows us to send our own messages here
	std::shared_ptr<mavsdk::MavlinkPassthrough> passthrough;
        // Reference to other MAVLink system
	std::shared_ptr<mavsdk::System> drone;
        // Publisher to report heartbeat data
        rclcpp::Publisher<NodeHeartbeat>::SharedPtr publisher;
        rclcpp::Subscription<NodeHeartbeat>::SharedPtr subscription;

        // Wrapper around Mavsdk::set_configuration
	void configure(uint8_t sysid, uint8_t compid, bool heartbeat);
        // Bind to the connection_url
	void connect();
        // Send a telemetry packet
	void send(float data[]);
        // Check to see if there is another system; connect if so
        void check_systems();
        // Runs every 100ms
        void timer_callback();
        // Runs every time we get a heartbeat
        void subsciption_callback(NodeHeartbeat::SharedPtr msg);
        // Process a NodeHeartbeat and turn it into a float array
        void telemetry_received_callback(mavlink_message_t msg);
        void pack_heartbeat_message(NodeHeartbeat::SharedPtr msg, float destination[2]);
        void unpack_heartbeat_message(NodeHeartbeat *msg, float destination[2]);
};

#endif //DATALINK
