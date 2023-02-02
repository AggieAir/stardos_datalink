#ifndef DATALINK_HPP
#define DATALINK_HPP

#include <functional>
#include <unordered_set>

#include <istream>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink/v2.0/mavlink_types.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/ftp/ftp.h>

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
#include "stardos_interfaces/srv/topic_list.hpp"

#include "floattelem.hpp"

using namespace stardos_interfaces::msg;
using namespace stardos_interfaces::srv;

class Datalink: public rclcpp::Node {
public:
	Datalink(
                const std::string& name,
                const Json::Value config
        );

private:
        /* ************************ *
         * VARIABLE ZONE            *
         * There are a lot of these *
         * ************************ */

        const Json::Value config;

        // Instance of MAVSDK -- this models the connection
  	mavsdk::Mavsdk dc;
        // Reference to autopilot system
	std::shared_ptr<mavsdk::System> autopilot;
        // Reference to MAVLink system to send telemetry to
	std::shared_ptr<mavsdk::System> target;
	// File Transfer Protocol, for transmitting longer messages.
	std::shared_ptr<mavsdk::Ftp> ftp;

        std::string aircraft;
        std::string payload;

        // DEBUG_FLOAT_ARRAY Array ID
        uint16_t array_id;

        // Where are we
        uint8_t sysid;
        uint8_t compid;

        // Where to send telemetry messages to
        uint8_t targetsysid;
        uint8_t targetcompid;

	uint16_t statustext_id = 0;

        // Where to look for additional configuration files
        std::string extra_config_directory;

        // These allow us to pass messages directly to the systems
	std::shared_ptr<mavsdk::MavlinkPassthrough> autopilot_passthrough;
	std::shared_ptr<mavsdk::MavlinkPassthrough> target_passthrough;

	std::unique_ptr<std::thread> uploading_thread;

        // Runs every second; looks for systems
        rclcpp::TimerBase::SharedPtr get_system_timer;
        // Runs every second; looks for systems
        rclcpp::TimerBase::SharedPtr send_telemetry_timer;

        // Which topics to listen to in order to forward over mavlink
        // * for heartbeats
        std::vector<rclcpp::Subscription<NodeHeartbeat>::SharedPtr> heartbeat_subscriptions;
        // * for control messages
        std::vector<rclcpp::Subscription<Control>::SharedPtr> signal_subscriptions;
        // * for system status messages
        std::map<uint8_t, rclcpp::Subscription<SystemStatus>::SharedPtr> system_status_subscriptions;

	// for configuration setting
	rclcpp::Publisher<Control>::SharedPtr set_config_publisher;
	rclcpp::Subscription<Control>::SharedPtr set_config_subscription;

	// for MAVLink STATUSTEXT messages
	rclcpp::Subscription<Control>::SharedPtr status_text_subscription;

        // map topics to topic ids
        std::map<std::string, uint8_t> heartbeat_subscription_ids;
        std::map<std::string, uint8_t> control_subscription_ids;
        std::map<std::string, uint8_t> system_status_subscription_ids;

        // Which topics to publish onto when we get a mavlink message
        // * for heartbeats
        std::vector<rclcpp::Publisher<NodeHeartbeat>::SharedPtr> heartbeat_publishers;
        // * for control messages
        std::vector<rclcpp::Publisher<Control>::SharedPtr> signal_publishers;
        // * for system status message
        //   (this is a map because we may skip some system id at some point)
        std::map<uint8_t, rclcpp::Publisher<SystemStatus>::SharedPtr> system_status_publishers;

        std::vector<rclcpp::ServiceBase::SharedPtr> services;

        // map topics to topic ids
        std::map<std::string, uint8_t> heartbeat_publisher_ids;
        std::map<std::string, uint8_t> control_publisher_ids;
        std::map<std::string, uint8_t> system_status_publisher_ids;

        // these store the capabilities of the systems we listen to
        std::map<uint8_t, floattelem::SystemCapacity> cached_systems;

        // enum specifying where filesystems can be mounted
        std::map<std::string, uint8_t> mountpoints;
        std::vector<std::string> mountpoint_names;

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
        

        /* **************************** *
         * FUNCTION ZONE                *
         * Non-callback functions first *
         * **************************** */

        void detect_environment();
        // Wrapper around Mavsdk::set_configuration
	void configure();
        // Bind to the connection_url
	void connect();
        // Read configuration (usually passed on stdin)
        void setup_floattelem();
        // Setup basic common topics for copilot and payload heartbeats
        void setup_default_heartbeat_topics();
        // Setup basic common topics for start and end mission
        void setup_default_control_topics();
        // Setup autopilot telemetry
        void setup_autopilot_telemetry();
        // Setup starcommand downlink and uplink
        void setup_starcommand(const std::string& downlink_topic, const std::string& uplink_topic);
        // Load the properties of each system and the status messages they publish
        void load_system_statuses();
        // Load the mountpoint enum
        void load_mountpoints();
        // Send a telemetry packet
	inline void send_buffered_message();
        // Send a telemetry packet
        mavsdk::MavlinkPassthrough::Result send_telemetry(const floattelem::Message &msg);
        // Check to see if there is another system; connect if so
        void check_systems();

        /* ********************** *
         * ENTERING CALLBACK LAND *
         * ********************** */

        // Runs every time we get a heartbeat
        void heartbeat_callback(int id, NodeHeartbeat::SharedPtr msg);
        // Runs every time we get a control signal
        void signal_callback(int id, Control::SharedPtr msg);
        // Runs every time we get a configuration file
        void set_config_callback(Control::SharedPtr msg);
        // Runs every time we get a STATUSTEXT control message
        void status_text_callback(Control::SharedPtr msg);
        // Runs every time we get a system status message
        void system_status_callback(int id, SystemStatus::SharedPtr msg);
        // What to call when we see a control message
        // This is a STARDOS concept; in this context the control node tells us
        // which topics to publish and subscribe on. It is a JSON string.
        void control_callback(Control::SharedPtr msg);
        // Runs every time we get an uplink message
        void uplink_callback(StarCommandUplink::SharedPtr msg);

	void uploading_callback(mavsdk::Ftp::Result, mavsdk::Ftp::ProgressData, std::promise<mavsdk::Ftp::Result>*);

        template<typename T>
        void subscribe_service_callback(
                TopicList::Request::SharedPtr req,
                TopicList::Response::SharedPtr resp,
                std::vector<typename rclcpp::Subscription<T>::SharedPtr> &dest,
                std::map<std::string, uint8_t> &mapping,
                std::function<void(int, std::shared_ptr<T>)>
        );

        template<typename T>
        void publish_service_callback(
                TopicList::Request::SharedPtr req,
                TopicList::Response::SharedPtr resp,
                std::vector<typename rclcpp::Publisher<T>::SharedPtr> &dest,
                std::map<std::string, uint8_t> &mapping
        );

        // These all process particular mavlink message types
        // There's not much to see, tbh
        void array_received_callback(const mavlink_message_t& msg);
        void gps_raw_received_callback(const mavlink_message_t& msg) const;
        void global_position_received_callback(const mavlink_message_t& msg) const;
        void attitude_received_callback(const mavlink_message_t& msg) const;
        void systime_received_callback(const mavlink_message_t& msg) const;

        // Convenience methods -- takes a list of topics and subscribes/creates publishers to all of them
        // Makes it a lot easier to handle "pub" and "sub" lists from the control listener
        template<typename T>
        void fill_subscriber_list(
                const std::vector<std::string> &topics,
                std::vector<typename rclcpp::Subscription<T>::SharedPtr> &dest,
                std::map<std::string, uint8_t> &mapping,
                std::function<void(int, std::shared_ptr<T>)>
        );

        template<typename T>
        void fill_publisher_list(
                const std::vector<std::string> &topics,
                std::vector<typename rclcpp::Publisher<T>::SharedPtr> &dest,
                std::map<std::string, uint8_t> &mapping
        );

        template<typename T>
        static void copy_to_json_array(std::vector<T> &arr, Json::Value &val);

        template<typename T, size_t N>
        static void copy_to_json_array(std::array<T, N> &arr, Json::Value &val);
};

#endif //DATALINK
