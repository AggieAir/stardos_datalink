#ifndef AUTOPILOT_BRIDGE_HPP
#define AUTOPILOT_BRIDGE_HPP

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

class AutopilotBridge: public LinkNode
{
public:
	AutopilotBridge(
                const std::string& name,
                const Json::Value& config
        );

private:
        /* ************************ *
         * VARIABLE ZONE            *
         * ************************ */

        // Publishers for MAVLink data received from the autopilot
        rclcpp::Publisher<GPSPosition>::SharedPtr gps_raw_publisher;
        rclcpp::Publisher<GPSPosition>::SharedPtr gps_position_publisher;
        rclcpp::Publisher<GlobalPosition>::SharedPtr global_position_publisher;
        rclcpp::Publisher<Attitude>::SharedPtr attitude_publisher;
        rclcpp::Publisher<SystemTime>::SharedPtr systime_publisher;

        /* **************************** *
         * FUNCTION ZONE                *
         * Non-callback functions first *
         * **************************** */

        void setup_autopilot_telemetry();

        /* ********************** *
         * ENTERING CALLBACK LAND *
         * ********************** */

        void target_passthrough_found_callback() override;

        // These all process particular mavlink message types
        // There's not much to see, tbh
        void gps_raw_received_callback(const mavlink_message_t& msg) const;
        void global_position_received_callback(const mavlink_message_t& msg) const;
        void attitude_received_callback(const mavlink_message_t& msg) const;
        void systime_received_callback(const mavlink_message_t& msg) const;
};

#endif //AUTOPILOT_BRIDGE_HPP
