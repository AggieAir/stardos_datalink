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
#include "stardos_interfaces/msg/global_position.hpp"
#include "stardos_interfaces/msg/gps_position.hpp"
#include "stardos_interfaces/msg/attitude.hpp"
#include "stardos_interfaces/msg/system_time.hpp"
#include "stardos_interfaces/msg/system_status.hpp"
#include "stardos_interfaces/msg/star_command_downlink.hpp"
#include "stardos_interfaces/msg/star_command_uplink.hpp"

#include "floattelem.hpp"

using stardos_interfaces::msg::NodeHeartbeat;
using stardos_interfaces::msg::Control;
using stardos_interfaces::msg::GlobalPosition;
using stardos_interfaces::msg::GPSPosition;
using stardos_interfaces::msg::Attitude;
using stardos_interfaces::msg::SystemTime;
using stardos_interfaces::msg::SystemStatus;
using stardos_interfaces::msg::StarCommandDownlink;
using stardos_interfaces::msg::StarCommandUplink;

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
                bool autopilot_telemetry,
                bool starcommand,
                bool publish_system_status
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

        // DEBUG_FLOAT_ARRAY Array ID
        uint16_t array_id;

        // These allow us to pass messages directly to the systems
	std::shared_ptr<mavsdk::MavlinkPassthrough> autopilot_passthrough;
	std::shared_ptr<mavsdk::MavlinkPassthrough> target_passthrough;

        // Runs every second; looks for systems
        rclcpp::TimerBase::SharedPtr get_system_timer;
        // Runs every second; looks for systems
        rclcpp::TimerBase::SharedPtr send_telemetry_timer;

        // Subscribes to notifications from the STARDOS control node
        rclcpp::Subscription<Control>::SharedPtr control_subscription;

        // Which topics to listen to in order to forward over mavlink
        // * for heartbeats
        std::vector<rclcpp::Subscription<NodeHeartbeat>::SharedPtr> heartbeat_subscriptions;
        // * for control messages
        std::vector<rclcpp::Subscription<Control>::SharedPtr> signal_subscriptions;

        // map topics to topic ids
        std::map<std::string, uint8_t> heartbeat_publisher_ids;
        std::map<std::string, uint8_t> heartbeat_subscription_ids;

        // Which topics to publish onto when we get a mavlink message
        // * for heartbeats
        std::vector<rclcpp::Publisher<NodeHeartbeat>::SharedPtr> heartbeat_publishers;
        // * for control messages
        std::vector<rclcpp::Publisher<Control>::SharedPtr> signal_publishers;

        // map topics to topic ids
        std::map<std::string, uint8_t> control_publisher_ids;
        std::map<std::string, uint8_t> control_subscription_ids;

        // System Status Publisher and Subscription
        std::vector<rclcpp::Publisher<SystemStatus>::SharedPtr> system_status_publishers;
        std::vector<rclcpp::Subscription<SystemStatus>::SharedPtr> system_status_subscriptions;

        // map topics to topic ids
        std::map<std::string, uint8_t> system_status_publisher_ids;
        std::map<std::string, uint8_t> system_status_subscription_ids;

        // Publishers for MAVLink data received from the autopilot
        rclcpp::Publisher<GPSPosition>::SharedPtr gps_raw_publisher;
        rclcpp::Publisher<GPSPosition>::SharedPtr gps_position_publisher;
        rclcpp::Publisher<GlobalPosition>::SharedPtr global_position_publisher;
        rclcpp::Publisher<Attitude>::SharedPtr attitude_publisher;
        rclcpp::Publisher<SystemTime>::SharedPtr systime_publisher;

        // Publishers for StarCommand topics (if necessary)
        rclcpp::Publisher<StarCommandDownlink>::SharedPtr starcommand_publisher;
        rclcpp::Subscription<StarCommandUplink>::SharedPtr starcommand_subscription;

        floattelem::Message buffered_message;

        // Wrapper around Mavsdk::set_configuration
	void configure();
        // Bind to the connection_url
	void connect();
        // Setup autopilot telemetry
        void setup_autopilot_telemetry(bool activated);
        // Setup starcommand downlink and uplink
        void setup_starcommand(std::string downlink_topic, std::string uplink_topic);
        // Send a telemetry packet
	void send();
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
        // Runs every time we get a system status message
        void system_status_callback(int id, SystemStatus::SharedPtr msg);
        // What to call when we see a control message
        // This is a STARDOS concept; in this context the control node tells us
        // which topics to publish and subscribe on. It is a JSON string.
        void control_callback(Control::SharedPtr msg);
        // Runs every time we get an uplink message
        void uplink_callback(StarCommandUplink::SharedPtr msg);
        
        // These all process particular mavlink message types
        // There's not much to see, tbh
        void array_received_callback(mavlink_message_t msg);
        void gps_raw_received_callback(mavlink_message_t msg);
        void global_position_received_callback(mavlink_message_t msg);
        void attitude_received_callback(mavlink_message_t msg);
        void systime_received_callback(mavlink_message_t msg);

        // Convenience methods -- takes a list of topics and subscribes/creates publishers to all of them
        // Makes it a lot easier to handle "pub" and "sub" lists from the control listener
        template<typename T>
        void fill_subscriber_list(
                Json::Value& topics,
                std::vector<typename rclcpp::Subscription<T>::SharedPtr> *dest,
                std::map<std::string, uint8_t> *mapping,
                std::function<void(int, std::shared_ptr<T>)>
        );

        template<typename T>
        void fill_publisher_list(
                Json::Value& topics,
                std::vector<typename rclcpp::Publisher<T>::SharedPtr> *dest,
                std::map<std::string, uint8_t> *mapping);
};

#endif //DATALINK
