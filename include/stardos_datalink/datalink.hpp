#ifndef DATALINK_HPP
#define DATALINK_HPP

#include <functional>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink/v2.0/mavlink_types.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>

#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "stardos_interfaces/msg/node_heartbeat.hpp"
#include "stardos_interfaces/msg/control.hpp"
#include "stardos_interfaces/msg/gps_position.hpp"
#include "stardos_interfaces/msg/attitude.hpp"
#include "stardos_interfaces/msg/system_time.hpp"

#include "floattelem.hpp"

using stardos_interfaces::msg::NodeHeartbeat;
using stardos_interfaces::msg::Control;
using stardos_interfaces::msg::GPSPosition;
using stardos_interfaces::msg::Attitude;
using stardos_interfaces::msg::SystemTime;

class Datalink: public rclcpp::Node
{
public:
	Datalink(
                std::string name,
                uint8_t sysid,
                uint8_t compid,
                bool heartbeat,
                std::string connection_url,
                uint8_t targetsysid,
                uint8_t targetcompid,
                bool autopilot_telemetry
        );

private:
        // ROS node name
        std::string name;
        // Instance of MAVSDK -- this models the connection
  	mavsdk::Mavsdk dc;
        // Reference to autopilot system
	std::shared_ptr<mavsdk::System> autopilot;
        // Reference to MAVLink system to send telemetry to
	std::shared_ptr<mavsdk::System> target;

        // These allow us to pass messages directly to the systems
	std::shared_ptr<mavsdk::MavlinkPassthrough> autopilot_passthrough;
	std::shared_ptr<mavsdk::MavlinkPassthrough> target_passthrough;

        // Runs every second; looks for systems
        rclcpp::TimerBase::SharedPtr get_system_timer;

        // Subscribes to notifications from the STARDOS control node
        rclcpp::Subscription<Control>::SharedPtr control_subscription;

        // Which topics to listen to in order to forward over mavlink
        // * for heartbeats
        std::vector<rclcpp::Subscription<NodeHeartbeat>::SharedPtr> heartbeat_subscriptions;
        // * for control messages
        std::vector<rclcpp::Subscription<Control>::SharedPtr> signal_subscriptions;

        // Which topics to publish onto when we get a mavlink message
        // * for heartbeats
        std::vector<rclcpp::Publisher<NodeHeartbeat>::SharedPtr> heartbeat_publishers;
        // * for control messages
        std::vector<rclcpp::Publisher<Control>::SharedPtr> signal_publishers;

        // Publishers for MAVLink data received from the autopilot
        rclcpp::Publisher<GPSPosition>::SharedPtr gps_publisher;
        rclcpp::Publisher<Attitude>::SharedPtr attitude_publisher;
        rclcpp::Publisher<SystemTime>::SharedPtr systime_publisher;

        floattelem::Message buffered_message;

        // Wrapper around Mavsdk::set_configuration
	void configure();
        // Bind to the connection_url
	void connect();
        // Setup autopilot telemetry
        void setup_autopilot_telemetry(bool activated);
        // Send a telemetry packet
	void send(floattelem::Message msg);
        // Check to see if there is another system; connect if so
        void check_systems();

        /* ********************** *
         * ENTERING CALLBACK LAND *
         * ********************** */

        // Runs every 100ms
        void timer_callback();
        // Runs every time we get a heartbeat
        void heartbeat_callback(int id, NodeHeartbeat::SharedPtr msg);
        // Runs every time we get a control signal
        void signal_callback(int id, Control::SharedPtr msg);
        // What to call when we see a control message
        // This is a STARDOS concept; in this context the control node tells us
        // which topics to publish and subscribe on. It is a JSON string.
        void control_callback(Control::SharedPtr msg);
        // Process a NodeHeartbeat and turn it into a float array
        void array_received_callback(mavlink_message_t msg);
        
        // These all process particular mavlink message types
        // There's not much to see, tbh
        void gps_received_callback(mavlink_message_t msg);
        void attitude_received_callback(mavlink_message_t msg);
        void systime_received_callback(mavlink_message_t msg);

        // Convenience methods -- takes a list of topics and subscribes/creates publishers to all of them
        // Makes it a lot easier to handle "pub" and "sub" lists from the control listener
        template<typename T>
        void fill_subscriber_list(Json::Value& topics, std::vector<typename rclcpp::Subscription<T>::SharedPtr> *dest, std::function<void(int, std::shared_ptr<T>)>);

        template<typename T>
        void fill_publisher_list(Json::Value& topics, std::vector<typename rclcpp::Publisher<T>::SharedPtr> *dest);
};

#endif //DATALINK
