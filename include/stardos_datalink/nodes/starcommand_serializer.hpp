#ifndef STARCOMMAND_SERIALIZER_HPP
#define STARCOMMAND_SERIALIZER_HPP

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink/v2.0/mavlink_types.h>

#include "stardos_interfaces/msg/node_heartbeat.hpp"
#include "stardos_interfaces/msg/control.hpp"
#include "stardos_interfaces/msg/system_status.hpp"
#include "stardos_interfaces/msg/star_command_downlink.hpp"
#include "stardos_interfaces/msg/star_command_uplink.hpp"

#include "./floattelem_sender.hpp"
#include "./system_reader_node.hpp"

using stardos_interfaces::msg::NodeHeartbeat;
using stardos_interfaces::msg::Control;
using stardos_interfaces::msg::SystemStatus;
using stardos_interfaces::msg::StarCommandDownlink;
using stardos_interfaces::msg::StarCommandUplink;

class StarCommandSerializer:
        virtual public SystemReaderNode,
        virtual public FloatTelemSenderNode {
public:
	StarCommandSerializer(
                const std::string& name,
                const Json::Value& config
        );

private:
        /* ************************ *
         * VARIABLE ZONE            *
         * ************************ */

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

        // Publishers for StarCommand topics (if necessary)
        rclcpp::Publisher<StarCommandDownlink>::SharedPtr starcommand_publisher;
        rclcpp::Subscription<StarCommandUplink>::SharedPtr starcommand_subscription;

        /* **************************** *
         * FUNCTION ZONE                *
         * Non-callback functions first *
         * **************************** */

        // Setup starcommand downlink and uplink
        void setup_starcommand(const std::string& downlink_topic, const std::string& uplink_topic);

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
        // Runs every time we get an uplink message from starcommand
        void uplink_callback(StarCommandUplink::SharedPtr msg);

        // Callback to call when we add a system
        void add_system(const DatalinkSystem& sys);
};

#endif //STARCOMMAND_SERIALIZER_HPP
