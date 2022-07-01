#ifndef DATALINK_HPP
#define DATALINK_HPP

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

class LinkNode: public rclcpp::Node
{
public:
	LinkNode(
                const std::string& name,
                const Json::Value& config
        );

private:
        /* ************************ *
         * VARIABLE ZONE            *
         * There are a lot of these *
         * ************************ */

        // ROS node name
        std::string name;
        // Configuration JSON
        Json::Value config;
        // Instance of MAVSDK -- this models the connection
  	mavsdk::Mavsdk dc;
        // Reference to MAVLink system to send telemetry to
	std::shared_ptr<mavsdk::System> target;

        // Where are we
        uint8_t sysid;
        uint8_t compid;

        // Where to send telemetry messages to
        uint8_t targetsysid;
        uint8_t targetcompid;

        // Where to look for additional configuration files
        std::string extra_config_directory;

        // These allow us to pass messages directly to the systems
	std::shared_ptr<mavsdk::MavlinkPassthrough> target_passthrough;

        // Runs every second; looks for systems
        rclcpp::TimerBase::SharedPtr get_system_timer;

        /* **************************** *
         * FUNCTION ZONE                *
         * Non-callback functions first *
         * **************************** */

        // Wrapper around Mavsdk::set_configuration
	virtual void configure();
        // Bind to the connection_url
	virtual void connect();
        // Check to see if there is another system; connect if so
        virtual void check_systems();

        virtual void target_passthrough_found_callback() = 0;

        /* ********************** *
         * ENTERING CALLBACK LAND *
         * ********************** */

        // Convenience methods -- takes a list of topics and subscribes/creates publishers to all of them
        // Makes it a lot easier to handle "pub" and "sub" lists from the control listener
        template<typename T>
        void fill_subscriber_list(
                Json::Value& topics,
                std::vector<typename rclcpp::Subscription<T>::SharedPtr> &dest,
                std::map<std::string, uint8_t> &mapping,
                std::function<void(int, std::shared_ptr<T>)>
        );

        template<typename T>
        void fill_publisher_list(
                Json::Value& topics,
                std::vector<typename rclcpp::Publisher<T>::SharedPtr> &dest,
                std::map<std::string, uint8_t> &mapping);
};

#endif //DATALINK
