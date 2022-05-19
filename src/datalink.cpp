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

Datalink::Datalink(std::string name, uint8_t sysid, uint8_t compid, bool heartbeat, std::string connection_url):
        Node(name),
        name{name},
        sysid{sysid},
        compid{compid},
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

        RCLCPP_INFO(this->get_logger(), "Creating MAVLink Passthrough");

        RCLCPP_INFO(this->get_logger(), "Binding timer callback");

	get_system_timer = this->create_wall_timer(100ms, std::bind(&Datalink::timer_callback, this));
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

        uint8_t *data8 = (uint8_t*) msg.get_data();

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
        auto maybe_drone = dc.systems().back();

        RCLCPP_INFO(this->get_logger(), "Found autopilot");
        dc.subscribe_on_new_system(nullptr);
        drone = maybe_drone;
        passthrough = std::make_shared<MavlinkPassthrough>(drone);
        passthrough->subscribe_message_async(
                        MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY, [this] (mavlink_message_t msg) {
                                this->telemetry_received_callback(msg);
                        });
}

void Datalink::timer_callback() {
        if (dc.systems().size() != 0) {
                check_systems();
                get_system_timer->cancel();
        }
}

void Datalink::heartbeat_callback(int id, NodeHeartbeat::SharedPtr msg) {
        if (passthrough == nullptr) return;
        send(TelemMessage::pack_heartbeat_message(msg, id));
}

void Datalink::control_callback(Control::SharedPtr msg) {
        Json::Value root;
        Json::Reader reader;
        reader.parse(msg->options, root);
        RCLCPP_INFO(this->get_logger(), "%d", root.size());

        heartbeat_subscriptions.clear();

        auto pub = root["pub"];
        int id = 0;
        for (auto v = pub.begin(); v != pub.end(); v++) {
                std::string topic = v->asString();

                heartbeat_subscriptions.push_back(
                    this->create_subscription<NodeHeartbeat>(
                        topic, 10, [this, id] (NodeHeartbeat::SharedPtr msg) {
                          heartbeat_callback(id, msg);
                        }));
        }

        auto sub = root["sub"];
        id = 0;
        for (auto v = sub.begin(); v != sub.end(); v++) {
                std::string topic = v->asString();
                heartbeat_publishers.push_back(
                        this->create_publisher<NodeHeartbeat>(
                                topic,
                                10));
        }


        // publisher = this->create_publisher<NodeHeartbeat>(
        //                 name + "/telemetry",
        //                 10);

}

void Datalink::telemetry_received_callback(mavlink_message_t msg) {
        RCLCPP_INFO(this->get_logger(), "received packet");

        mavlink_debug_float_array_t * floats = new mavlink_debug_float_array_t();
        mavlink_msg_debug_float_array_decode(&msg, floats);

        TelemMessage message = TelemMessage(floats->data);
        NodeHeartbeat ros_message = message.unpack_heartbeat_message();
        TelemHeader head = message.get_header();

        this->heartbeat_publishers[head.topic_id]->publish(ros_message);
}
