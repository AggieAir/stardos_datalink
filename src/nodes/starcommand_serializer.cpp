#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink/v2.0/mavlink_types.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

#include "rclcpp/rclcpp.hpp"
#include "stardos_interfaces/msg/node_heartbeat.hpp"
#include "stardos_interfaces/msg/control.hpp"
#include "stardos_interfaces/msg/system_status.hpp"

#include "nodes/starcommand_serializer.hpp"
#include "floattelem.hpp"

using namespace mavsdk;
using namespace std::literals::chrono_literals;
using namespace std::placeholders;

typedef floattelem::Message TelemMessage;
typedef floattelem::Header TelemHeader;

StarCommandSerializer::StarCommandSerializer(
        const std::string& name,
        const Json::Value& config
) : BasicDatalinkNode(name, config), MAVLinkedNode(ForwardingOption::ForwardingOn) {
        Json::Value uplinkval = config["starcommand"]["uplink"];
        Json::Value downlinkval = config["starcommand"]["downlink"];

        if (!uplinkval.isString() || !downlinkval.isString()) {
                throw std::runtime_error("Uplink and Downlink topics must be strings");
        }

        setup_starcommand(downlinkval.asString(), uplinkval.asString());
        load_systems(std::bind(&StarCommandSerializer::add_system, this, _1));
}

void StarCommandSerializer::setup_starcommand(
                const std::string& downlink_topic,
                const std::string& uplink_topic
) {
        RCLCPP_INFO(this->get_logger(), "Creating starcommand pub/sub");
        starcommand_publisher = this->create_publisher<StarCommandDownlink>(
                downlink_topic,
                10);

        starcommand_subscription = this->create_subscription<StarCommandUplink>(
                uplink_topic,
                10,
                std::bind(&StarCommandSerializer::uplink_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Creating heartbeat subscribers");
        this->fill_subscriber_list<NodeHeartbeat>(
                config["heartbeat"]["pub"],
                this->heartbeat_subscriptions,
                this->heartbeat_subscription_ids,
                std::bind(&StarCommandSerializer::heartbeat_callback, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Creating control message subscribers");
        this->fill_subscriber_list<Control>(
                config["control"]["pub"],
                this->signal_subscriptions,
                this->control_subscription_ids,
                std::bind(&StarCommandSerializer::signal_callback, this, _1, _2));
}

void StarCommandSerializer::target_passthrough_found_callback() {}

void StarCommandSerializer::heartbeat_callback(int id, NodeHeartbeat::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Serializing StarCommand heartbeat message from topic id %d", id);

        StarCommandDownlink down;

        Json::Value v(Json::objectValue);

        v["state"] = Json::Value(msg->state);
        v["requests"] = Json::Value(msg->requests);
        v["failures"] = Json::Value(msg->failures);
        v["errors"] = Json::Value(msg->errors);

        std::ostringstream json_out;
        json_out << v;

        down.type = "node";
        down.payload = json_out.str();
        down.origin = heartbeat_subscriptions[id]->get_topic_name();

        starcommand_publisher->publish(down);
}

void StarCommandSerializer::signal_callback(int id, Control::SharedPtr ctrl) {
        RCLCPP_INFO(this->get_logger(), "Serializing StarCommand control message from topic id %d", id);

        StarCommandDownlink down;

        Json::Value v(Json::objectValue);

        v["options"] = Json::Value(ctrl->options);

        std::ostringstream json_out;
        json_out << v;

        down.type = "control";
        down.payload = json_out.str();
        down.origin = signal_subscriptions[id]->get_topic_name();

        starcommand_publisher->publish(down);
}

void StarCommandSerializer::system_status_callback(int id, SystemStatus::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Serializing StarCommand control message from topic id %d", id);

        StarCommandDownlink down;

        Json::Value v(Json::objectValue);

        v["cpu_count"] = msg->cpu_count;
        v["cpu_usage"] = Json::arrayValue;
        for (auto cpu = msg->cpu_usage.begin(); cpu != msg->cpu_usage.end(); cpu++) {
                v["cpu_usage"].append(*cpu);
        }
        v["memory"] = Json::arrayValue;
        for (auto mem = msg->memory.begin(); mem != msg->memory.end(); mem++) {
                v["memory"].append(*mem);
        }
        v["swap"] = Json::arrayValue;
        for (auto swap = msg->swap.begin(); swap != msg->swap.end(); swap++) {
                v["swap"].append(*swap);
        }
        v["mounts"] = Json::arrayValue;
        for (auto mount = msg->mounts.begin(); mount != msg->mounts.end(); mount++) {
                v["mounts"].append(*mount);
        }
        v["disks"] = Json::arrayValue;
        for (auto disk = msg->disks.begin(); disk != msg->disks.end(); disk++) {
                v["disks"].append(*disk);
        }
        v["uptime"] = msg->uptime;

        std::ostringstream json_out;
        json_out << v;

        down.type = "control";
        down.payload = json_out.str();
        down.origin = signal_subscriptions[id]->get_topic_name();

        starcommand_publisher->publish(down);
}

void StarCommandSerializer::uplink_callback(StarCommandUplink::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Got data from StarCommand");
        Json::Value root;

        RCLCPP_INFO(this->get_logger(), "Deserializing");
        try {
                std::istringstream(msg->payload) >> root;
        } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "Parse error while reading StarCommand JSON.");
        }

        TelemMessage tmsg;

        if (msg->type == "node") {
                NodeHeartbeat hb;

                hb.state = root["state"].asInt();
                hb.requests = root["requests"].asInt();
                hb.failures = root["failures"].asInt();
                hb.errors = root["errors"].asInt();

                tmsg.push_heartbeat_message(
                        std::shared_ptr<NodeHeartbeat>(&hb),
                        heartbeat_subscription_ids.at(msg->destination)
                );
        } else if (msg->type == "control") {
                tmsg.push_control_message(
                        root["options"].asString(),
                        heartbeat_subscription_ids.at(msg->destination)
                );
        }

        this->send_telemetry(tmsg);
}

void StarCommandSerializer::add_system(const DatalinkSystem& sys) {
        system_status_subscriptions[sys.id] = this->create_subscription<SystemStatus>(
                sys.topic,
                10,
                [this, &sys] (SystemStatus::SharedPtr msg) {
                        this->system_status_callback(sys.id, msg);
                }
        );

        system_status_subscription_ids[sys.topic] = sys.id;
}
