#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>

#include "rclcpp/rclcpp.hpp"

#include "nodes/mavlinked_node.hpp"
#include "floattelem.hpp"

using namespace mavsdk;
using namespace std::literals::chrono_literals;

MAVLinkedNode::MAVLinkedNode() {
        RCLCPP_INFO(this->get_logger(), "Configuring MAVLink and connecting");
	configure();
	connect();

        RCLCPP_INFO(this->get_logger(), "Beginning to check for systems");
        // Yes, the callback to check for the target system is just on a timer.
        // Yes, I know that Mavsdk::subscribe_on_system_added exists.
        // I could not get it to work consistently.
	get_system_timer = this->create_wall_timer(1000ms, std::bind(&MAVLinkedNode::check_systems, this));
}

void MAVLinkedNode::configure() {
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

void MAVLinkedNode::connect() {
        Json::Value urlval = config["connection_url"];
        if (!urlval.isString()) {
                RCLCPP_ERROR(this->get_logger(), "URL must be a string");
                throw std::exception();
        }
        dc.add_any_connection(config["connection_url"].asString());
}

void MAVLinkedNode::check_systems() {
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

MavlinkPassthrough::Result MAVLinkedNode::send_mavlink(mavlink_message_t& msg) {
        MavlinkPassthrough::Result result = target_passthrough->send_message(msg);

        if (result != MavlinkPassthrough::Result::Success) {
                // this is the best way to turn this into a string
                // i hate it but it's what's gonna have to happen
                std::ostringstream ss;
                ss << result;
                RCLCPP_WARN(this->get_logger(), "command send failed: %d", ss.str().c_str());
        }

        return result;
}
