#include <fstream>
#include <functional>
#include <iostream>
#include <chrono>
#include <string.h>
#include <math.h>
#include <string>
#include <climits>

#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>
#include <utility>

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

#include "link_node.hpp"
#include "floattelem.hpp"

using namespace mavsdk;
using namespace std::literals::chrono_literals;
using namespace std::placeholders;

using rcl_interfaces::msg::ParameterDescriptor;
using stardos_interfaces::msg::NodeHeartbeat;
using stardos_interfaces::msg::Control;
using stardos_interfaces::msg::GlobalPosition;
using stardos_interfaces::msg::GPSPosition;
using stardos_interfaces::msg::Attitude;
using stardos_interfaces::msg::SystemTime;
using stardos_interfaces::msg::SystemStatus;
using stardos_interfaces::msg::StarCommandDownlink;
using stardos_interfaces::msg::StarCommandUplink;

typedef floattelem::Message TelemMessage;
typedef floattelem::Header TelemHeader;

LinkNode::LinkNode(
        const std::string& name,
        const Json::Value& config
) : Node(name),
        name{name},
        config{config}
{
        ParameterDescriptor ro;
        ro.read_only = true;

	configure();
	connect();

        RCLCPP_INFO(this->get_logger(), "Creating telemetry publisher");

        RCLCPP_INFO(this->get_logger(), "Binding timer callback");

        // Yes, the callback to check for the target system is just on a timer.
        // Yes, I know that Mavsdk::subscribe_on_system_added exists.
        // I could not get it to work consistently.
	get_system_timer = this->create_wall_timer(1000ms, std::bind(&LinkNode::check_systems, this));
}

void LinkNode::configure() {
        Json::Value sysidval = config["sysid"];
        Json::Value compidval = config["compid"];
        Json::Value targetsysidval = config["targetsysid"];
        Json::Value targetcompidval = config["targetcompid"];
        Json::Value filesval = config["extra_config_directory"];

        if (
                        !sysidval.isUInt() ||
                        sysidval.asUInt() > UINT8_MAX ||
                        !compidval.isUInt() ||
                        compidval.asUInt() > UINT8_MAX ||
                        !targetsysidval.isUInt() ||
                        targetsysidval.asUInt() > UINT8_MAX ||
                        !targetcompidval.isUInt() ||
                        targetcompidval.asUInt() > UINT8_MAX ||
                        !filesval.isString()
        ) {
                RCLCPP_ERROR(this->get_logger(), "System and Component IDs must be ints within [0, 255]");
                throw std::exception();
        }

        sysid = sysidval.asUInt();
        compid = compidval.asUInt();
        targetsysid = targetsysidval.asUInt();
        targetcompid = targetcompidval.asUInt();
        extra_config_directory = filesval.asString();

        RCLCPP_INFO(
                this->get_logger(),
                "Configuring: %d/%d looking for %d/%d",
                sysidval.asUInt(),
                compidval.asUInt(),
                targetsysidval.asUInt(),
                targetcompidval.asUInt()
        );

	dc.set_configuration(
		Mavsdk::Configuration(
                        (uint8_t) sysidval.asUInt(),
                        (uint8_t) compidval.asUInt(),
                        true
                )
        );
}

void LinkNode::connect() {
        Json::Value urlval = config["connection_url"];
        if (!urlval.isString()) {
                RCLCPP_ERROR(this->get_logger(), "URL must be a string");
                throw std::exception();
        }
        dc.add_any_connection(config["connection_url"].asString());
}

void LinkNode::check_systems() {
        for (auto s : dc.systems()) {
                if (s->get_system_id() == targetsysid && target == nullptr) {
                        // the target for float telemetry
                        RCLCPP_INFO(
                                this->get_logger(),
                                "Found target system (ID=%d)",
                                targetsysid
                        );

                        target = s;
                        target_passthrough = std::make_shared<MavlinkPassthrough>(target);

                        this->target_passthrough_found_callback();
                }
        }
}

template<typename T>
void LinkNode::fill_subscriber_list(
                Json::Value& topics,
                std::vector<typename rclcpp::Subscription<T>::SharedPtr> &dest,
                std::map<std::string, uint8_t> &mapping,
                std::function<void(int, std::shared_ptr<T>)> callback
) {
        int id = 0;
        for (auto v = topics.begin(); v != topics.end(); v++) {
                std::string topic = v->asString();

                RCLCPP_DEBUG(this->get_logger(), "subscribing to %s", topic.c_str());

                dest.push_back(
                        this->create_subscription<T>(
                                topic,
                                10,
                                [this, id, callback] (std::shared_ptr<T> msg) {
                                        callback(id, msg);
                                }
                        )
                );
                
                mapping.insert(std::make_pair(topic, id));

                id++;
        }
}

template<typename T>
void LinkNode::fill_publisher_list(
                Json::Value& topics,
                std::vector<typename rclcpp::Publisher<T>::SharedPtr> &dest,
                std::map<std::string, uint8_t> &mapping
) {
        int id = 0;
        for (auto v = topics.begin(); v != topics.end(); v++) {
                std::string topic = v->asString();

                RCLCPP_DEBUG(this->get_logger(), "subscribing to %s", topic.c_str());

                dest.push_back(
                        this->create_publisher<T>(
                                topic,
                                10
                        )
                );
                
                mapping.insert(std::make_pair(topic, id));
                
                id++;
        }
}
