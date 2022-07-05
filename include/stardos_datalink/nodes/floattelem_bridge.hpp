#ifndef TELEMETRY_BRIDGE_HPP
#define TELEMETRY_BRIDGE_HPP

#include <functional>

#include <istream>
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

#include "../floattelem.hpp"
#include "./link_node.hpp"

using stardos_interfaces::msg::NodeHeartbeat;
using stardos_interfaces::msg::Control;
using stardos_interfaces::msg::GlobalPosition;
using stardos_interfaces::msg::GPSPosition;
using stardos_interfaces::msg::Attitude;
using stardos_interfaces::msg::SystemTime;
using stardos_interfaces::msg::SystemStatus;
using stardos_interfaces::msg::StarCommandDownlink;
using stardos_interfaces::msg::StarCommandUplink;

class FloatTelemBridge: public LinkNode {
public:
	FloatTelemBridge(
                const std::string& name,
                const Json::Value& config
        );

private:
        /* ************************ *
         * VARIABLE ZONE            *
         * There are a lot of these *
         * ************************ */

        // The index of the array to send in the LOGGING_DATA messages
        uint16_t array_id;

        // Reference to autopilot system
	std::shared_ptr<mavsdk::System> autopilot;
        // These allow us to pass messages directly to the systems
	std::shared_ptr<mavsdk::MavlinkPassthrough> autopilot_passthrough;

        // Runs every second; looks for systems
        rclcpp::TimerBase::SharedPtr send_telemetry_timer;

        // Which topics to listen to in order to forward over mavlink
        // * for heartbeats
        std::vector<rclcpp::Subscription<NodeHeartbeat>::SharedPtr> heartbeat_subscriptions;
        // * for control messages
        std::vector<rclcpp::Subscription<Control>::SharedPtr> signal_subscriptions;
        // * for system status messages
        std::map<uint8_t, rclcpp::Subscription<SystemStatus>::SharedPtr> system_status_subscriptions;

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

        // map topics to topic ids
        std::map<std::string, uint8_t> heartbeat_publisher_ids;
        std::map<std::string, uint8_t> control_publisher_ids;
        std::map<std::string, uint8_t> system_status_publisher_ids;

        // these store the capabilities of the systems we listen to
        std::map<uint8_t, floattelem::SystemCapacity> cached_systems;

        // enum specifying where filesystems can be mounted
        std::map<std::string, uint8_t> mountpoints;
        std::vector<std::string> mountpoint_names;

        floattelem::Message buffered_message;

        /* **************************** *
         * FUNCTION ZONE                *
         * Non-callback functions first *
         * **************************** */

        // Read configuration (usually passed on stdin)
        void setup_floattelem();
        // Send a telemetry packet
	inline void send_buffered_message();
        // Send a telemetry packet
        mavsdk::MavlinkPassthrough::Result send_telemetry(const floattelem::Message &msg);

        /* ********************** *
         * ENTERING CALLBACK LAND *
         * ********************** */

        void target_passthrough_found_callback() override;

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

        // These all process particular mavlink message types
        // There's not much to see, tbh
        void array_received_callback(const mavlink_message_t& msg);
};

#endif //TELEMETRY_BRIDGE_HPP
