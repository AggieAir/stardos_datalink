#include <functional>
#include <iostream>
#include <chrono>
#include <string.h>
#include <math.h>
#include <string>

#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>

#include "rclcpp/rclcpp.hpp"
#include "stardos_interfaces/msg/node_heartbeat.hpp"
#include "stardos_interfaces/msg/control.hpp"

#include "datalink.hpp"
#include "floattelem.hpp"

using namespace mavsdk;
using namespace std::literals::chrono_literals;
using namespace std::placeholders;

using stardos_interfaces::msg::NodeHeartbeat;
using stardos_interfaces::msg::Control;

typedef floattelem::Message TelemMessage;
typedef floattelem::Header TelemHeader;

Datalink::Datalink(
        std::string name,
        uint8_t sysid,
        uint8_t compid,
        bool heartbeat,
        std::string connection_url,
        uint8_t targetsysid,
        uint8_t targetcompid
) : Node(name),
        name{name},
        sysid{sysid},
        compid{compid},
        targetsysid{targetsysid},
        targetcompid{targetcompid},
        heartbeat{heartbeat},
        connection_url{connection_url},
        heartbeat_subscriptions{std::vector<rclcpp::Subscription<NodeHeartbeat>::SharedPtr>()},
        heartbeat_publishers{std::vector<rclcpp::Publisher<NodeHeartbeat>::SharedPtr>()}
{
	configure(sysid, compid, heartbeat);
	connect();

        RCLCPP_INFO(this->get_logger(), "Creating telemetry publisher");

        control_subscription = this->create_subscription<Control>(
                        name + "/control",
                        10,
                        std::bind(&Datalink::control_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Binding timer callback");

        // Yes, the callback to check for the target system is just on a timer.
        // Yes, I know that Mavsdk::subscribe_on_system_added exists.
        // I could not get it to work consistently.
	get_system_timer = this->create_wall_timer(1000ms, std::bind(&Datalink::check_systems, this));
}

void Datalink::configure(uint8_t sysid, uint8_t compid, bool heartbeat) {
	dc.set_configuration(
		Mavsdk::Configuration(sysid, compid, heartbeat)
        );
}

void Datalink::connect() {
        dc.add_any_connection(connection_url);
}

void Datalink::send(TelemMessage msg) {
        mavlink_message_t message;

        mavlink_msg_debug_float_array_pack(
                passthrough->get_our_sysid(), // SystemID
                passthrough->get_our_compid(), //My comp ID
                &message, //Message reference
                1, //timeing is 1 sec
                name.c_str(),
                5,
                msg.get_data()
        );
        
        MavlinkPassthrough::Result result = passthrough->send_message(message);

        if (result != MavlinkPassthrough::Result::Success) {
                std::cout << "command send failed: " << result << "\n";
        }
}

void Datalink::check_systems() {
        for (auto s : dc.systems()) {
                if (s->get_system_id() == targetsysid) {
                        RCLCPP_INFO(this->get_logger(), "Found target system (ID=%d)", targetsysid);
                        drone = s;
                        passthrough = std::make_shared<MavlinkPassthrough>(drone);
                        passthrough->subscribe_message_async(
                                        MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY, 
                                        std::bind(&Datalink::telemetry_received_callback, this, _1));

                        get_system_timer->cancel();
                }
        }
}

void Datalink::heartbeat_callback(int id, NodeHeartbeat::SharedPtr msg) {
        if (passthrough == nullptr) {
                RCLCPP_INFO(this->get_logger(), "Tried to send a message before target system was found");
                return;
        }

        send(TelemMessage::pack_heartbeat_message(msg, id));
}

void Datalink::control_callback(Control::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Got control callback");
        Json::Value root;
        Json::Reader reader;

        RCLCPP_INFO(this->get_logger(), "Parsing JSON");
        reader.parse(msg->options, root);

        heartbeat_subscriptions.clear();

        RCLCPP_INFO(this->get_logger(), "Creating publishers");
        auto pub = root["pub"];
        int id = 0;
        for (auto v = pub.begin(); v != pub.end(); v++) {
                std::string topic = v->asString();

                heartbeat_subscriptions.push_back(
                    this->create_subscription<NodeHeartbeat>(
                        topic, 10,
                        [this, id] (NodeHeartbeat::SharedPtr msg) {
                          heartbeat_callback(id, msg);
                        }));
        }

        RCLCPP_INFO(this->get_logger(), "Creating subscribers");
        auto sub = root["sub"];
        id = 0;
        for (auto v = sub.begin(); v != sub.end(); v++) {
                std::string topic = v->asString();
                heartbeat_publishers.push_back(
                        this->create_publisher<NodeHeartbeat>(
                                topic,
                                10));
        }
}

void Datalink::telemetry_received_callback(mavlink_message_t msg) {
        RCLCPP_INFO(this->get_logger(), "received packet");

        mavlink_debug_float_array_t * floats = new mavlink_debug_float_array_t();
        mavlink_msg_debug_float_array_decode(&msg, floats);

        TelemMessage message = TelemMessage(floats->data);
        NodeHeartbeat ros_message = message.unpack_heartbeat_message();
        TelemHeader head = message.get_header();

        if (heartbeat_publishers.size() > head.topic_id) {
                RCLCPP_ERROR(this->get_logger(), "Publisher with ID=%d out of range", head.topic_id);
        } else {
                this->heartbeat_publishers[head.topic_id]->publish(ros_message);
        }
}
