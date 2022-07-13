#include <climits>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink/v2.0/mavlink_types.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

#include "rclcpp/rclcpp.hpp"

#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>

#include "datalink_util.hpp"
#include "nodes/floattelem_bridge.hpp"

using namespace mavsdk;
using namespace std::literals::chrono_literals;
using namespace std::placeholders;

using stardos_interfaces::msg::NodeHeartbeat;
using stardos_interfaces::msg::Control;
using stardos_interfaces::msg::SystemStatus;

typedef floattelem::Message TelemMessage;
typedef floattelem::Header TelemHeader;

FloatTelemBridge::FloatTelemBridge(
        const std::string& name,
        const Json::Value& config
) : BasicDatalinkNode(name, config), MAVLinkedNode(ForwardingOption::ForwardingOn) {
        setup_floattelem();
        load_systems(std::bind(&FloatTelemBridge::add_system, this, _1));
}

void FloatTelemBridge::send_buffered_message() {
        if (send_telemetry(buffered_message) == MavlinkPassthrough::Result::Success) {
                RCLCPP_INFO(this->get_logger(), "Resetting message buffer");
                buffered_message.reset();
        }
}

void FloatTelemBridge::target_passthrough_found_callback() {
        target_passthrough->subscribe_message_async(
                        MAVLINK_MSG_ID_LOGGING_DATA,
                        std::bind(&FloatTelemBridge::array_received_callback, this, _1));

        send_telemetry_timer = this->create_wall_timer(
                        1000ms,
                        std::bind(&FloatTelemBridge::send_buffered_message, this));
}

void FloatTelemBridge::heartbeat_callback(int id, NodeHeartbeat::SharedPtr msg) {
        if (target_passthrough == nullptr) {
                RCLCPP_INFO(this->get_logger(), "Tried to send a message before target system was found");
                return;
        }

        RCLCPP_INFO(this->get_logger(), "Queueing heartbeat message (offset=%d)", buffered_message.get_offset());

        if (!buffered_message.push_heartbeat_message(msg, id)) {
                send_buffered_message();
                buffered_message.reset();
                buffered_message.push_heartbeat_message(msg, id);
        }
}

void FloatTelemBridge::signal_callback(int id, Control::SharedPtr ctrl) {
        if (target_passthrough == nullptr) {
                RCLCPP_INFO(this->get_logger(), "Tried to send a control signal before target system was found");
                return;
        }

        if (ctrl->options.size() > floattelem::MAX_STRING_LENGTH) {
                RCLCPP_ERROR(this->get_logger(), "Option string too long; will be truncated");
        }

        if (!buffered_message.push_control_message(ctrl->options, id)) {
                send_buffered_message();
                buffered_message.reset();
                buffered_message.push_control_message(ctrl->options, id);
        }
}

void FloatTelemBridge::system_status_callback(int id, SystemStatus::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received system status from computer %d", id);
        floattelem::SystemCapacity sc;
        sc.max_memory_mb = msg->memory[1];
        sc.max_swap_mb = msg->swap[1];

        for (auto v = msg->disks.begin(); v != msg->disks.end(); v += 2) {
                sc.disks_size_mb.push_back(v[1]);
        }

        auto inserted = cached_systems.insert(std::make_pair(id, sc)); // pair<pair<key, value>, bool>
        auto entry = inserted.first; // pair<key, value>
        bool success = inserted.second; // bool
        floattelem::SystemCapacity& cap = entry->second;

        TelemMessage tmsg;
        if (success || cap != sc) {
                RCLCPP_INFO(this->get_logger(), "Updating system capabilities");
                RCLCPP_DEBUG(this->get_logger(), "Pushing message");
                tmsg.push_system_capacity_message(&sc, id);
                RCLCPP_DEBUG(this->get_logger(), "Replacing entry in cached_systems");
                cap = sc;
        }

        RCLCPP_DEBUG(this->get_logger(), "Preparing FloatTelem message for SystemStatus");
        floattelem::SlimSystemStatus status;
        RCLCPP_DEBUG(this->get_logger(), "Adding static details");
        status.cpu_usage = msg->cpu_usage;
        status.memory = (uint16_t) ((float) msg->memory[0] / (float) msg->memory[1] * USHRT_MAX);
        status.swap = (uint16_t) ((float) msg->swap[0] / (float) msg->swap[1] * USHRT_MAX);
        status.uptime = msg->uptime;

        RCLCPP_DEBUG(this->get_logger(), "Adding disks");
        for (auto v = msg->disks.begin(); v != msg->disks.end(); v += 2) {
                status.disks.push_back((uint16_t) ((float) *v / (float) *(v+1) * USHRT_MAX));
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Adding mounts");
        for (auto v = msg->mounts.begin(); v != msg->mounts.end(); v++) {
                auto m = mountpoints.find(*v);
                if (m == mountpoints.end()) {
                        RCLCPP_ERROR(this->get_logger(), "%s is not a recognized mountpoint", v->c_str());
                        status.mounts.push_back(255);
                } else {
                        RCLCPP_DEBUG(this->get_logger(), "found mountpoint %s", m->first.c_str());
                        status.mounts.push_back(m->second);
                }
        }

        tmsg.push_system_status_message(&status, id);

        this->send_telemetry(tmsg);
}

void FloatTelemBridge::setup_floattelem() {
        RCLCPP_INFO(this->get_logger(), "Creating heartbeat subscribers");
        this->fill_subscriber_list<NodeHeartbeat>(
                config["heartbeat"]["sub"],
                this->heartbeat_subscriptions,
                this->heartbeat_subscription_ids,
                std::bind(&FloatTelemBridge::heartbeat_callback, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Creating heartbeat publishers");
        this->fill_publisher_list<NodeHeartbeat>(
                config["heartbeat"]["pub"],
                this->heartbeat_publishers,
                this->heartbeat_publisher_ids);

        RCLCPP_INFO(this->get_logger(), "Creating control message subscribers");
        this->fill_subscriber_list<Control>(
                config["control"]["sub"],
                this->signal_subscriptions,
                this->control_subscription_ids,
                std::bind(&FloatTelemBridge::signal_callback, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Creating control message publishers");
        this->fill_publisher_list<Control>(
                config["control"]["pub"],
                this->signal_publishers,
                this->control_publisher_ids
        );
}

void FloatTelemBridge::array_received_callback(const mavlink_message_t& msg) {
        RCLCPP_DEBUG(this->get_logger(), "Got a LOGGING_DATA message");

        mavlink_logging_data_t * inner = new mavlink_logging_data_t();
        mavlink_msg_logging_data_decode(&msg, inner);

        RCLCPP_DEBUG(
                this->get_logger(),
                "The message is from %d/%d and addressed to %d/%d",
                msg.sysid,
                msg.compid,
                inner->target_system,
                inner->target_component
        );

        if (
                        msg.sysid != targetsysid ||
                        msg.compid != targetcompid ||
                        inner->target_system != sysid ||
                        inner->target_component != compid
        ) {
                return;
        }

        TelemMessage message = TelemMessage(inner->data);

        while (message.has_next()) {
                TelemHeader head = message.next_header();
                if (head.msg_type == floattelem::MSG_ID_HEARTBEAT) {
                        RCLCPP_DEBUG(this->get_logger(), "Heartbeat message!");
                        NodeHeartbeat ros_message = message.pop_heartbeat_message();

                        if (head.topic_id >= heartbeat_publishers.size()) {
                                RCLCPP_ERROR(
                                        this->get_logger(),
                                        "Heartbeat publisher with ID=%d out of range",
                                        head.topic_id
                                );
                                continue;
                        }

                        auto pub = this->heartbeat_publishers[head.topic_id];
                        RCLCPP_DEBUG(this->get_logger(), "Publishing on topic %s", pub->get_topic_name());
                        pub->publish(ros_message);
                } else if (head.msg_type == floattelem::MSG_ID_CONTROL) {
                        std::string options = message.pop_control_message();

                        if (head.topic_id >= signal_publishers.size()) {
                                RCLCPP_ERROR(
                                        this->get_logger(),
                                        "Signal publisher with ID=%d out of range",
                                        head.topic_id
                                );
                                continue;
                        }

                        Control c;
                        c.options = options;

                        this->signal_publishers[head.topic_id]->publish(c);
                } else if (head.msg_type == floattelem::MSG_ID_SYSTEM_CAPACITY) {
                        floattelem::SystemCapacity cap = message.pop_system_capacity_message();
                        RCLCPP_INFO(this->get_logger(), "Caching capacities for system %d.", head.topic_id);

                        for (auto v = cap.disks_size_mb.begin(); v != cap.disks_size_mb.end(); v++) {
                                RCLCPP_INFO(this->get_logger(), "Size: %d", *v);
                        }

                        cached_systems[head.topic_id] = cap;
                } else if (head.msg_type == floattelem::MSG_ID_SYSTEM_STATUS) {
                        RCLCPP_INFO(this->get_logger(), "Got status message from system %d.", head.topic_id);
                        floattelem::SlimSystemStatus in = message.pop_system_status_message();

                        auto result = cached_systems.find(head.topic_id);
                        if (result == cached_systems.end()) {
                                RCLCPP_ERROR(this->get_logger(), "System %d not cached!", head.topic_id);
                                continue;
                        }

                        floattelem::SystemCapacity &cap = result->second;

                        RCLCPP_DEBUG(this->get_logger(), "Preparing ROS message");
                        SystemStatus out;
                        RCLCPP_DEBUG(this->get_logger(), "Adding CPU info");
                        out.cpu_count = in.cpu_usage.size();
                        out.cpu_usage = in.cpu_usage;
                        RCLCPP_DEBUG(this->get_logger(), "Adding uptime info");
                        out.uptime = in.uptime;

                        RCLCPP_DEBUG(this->get_logger(), "Creating memory array");
                        out.memory = std::array<uint32_t, 2> {
                                (uint32_t) ((float) in.memory / USHRT_MAX * cap.max_memory_mb),
                                cap.max_memory_mb
                        };

                        RCLCPP_DEBUG(this->get_logger(), "Creating swap array");
                        out.swap = std::array<uint32_t, 2> {
                                (uint32_t) ((float) in.swap / USHRT_MAX * cap.max_swap_mb),
                                cap.max_swap_mb
                        };

                        RCLCPP_DEBUG(this->get_logger(), "Creating mounts vector");
                        for (auto v = in.mounts.begin(); v != in.mounts.end(); v++) {
                                out.mounts.push_back(*v >= mountpoint_names.size() ? "UNKNOWN" : mountpoint_names[*v]);
                        }

                        RCLCPP_DEBUG(this->get_logger(), "Creating disks and disk size vector");
                        for (auto v = std::make_pair(in.disks.begin(), cap.disks_size_mb.begin()); v.first != in.disks.end(); v.first++, v.second++) {
                                out.disks.push_back(
                                        (uint32_t) ((float) *v.first / USHRT_MAX * *v.second)
                                );
                                out.disks.push_back(*v.second);
                        }

                        RCLCPP_DEBUG(this->get_logger(), "Publishing to %s", system_status_publishers[head.topic_id]->get_topic_name());
                        system_status_publishers[head.topic_id]->publish(out);
                } else {
                        RCLCPP_ERROR(
                                this->get_logger(),
                                "Unrecognized message ID: %d",
                                head.msg_type);
                }
        }
}

void FloatTelemBridge::add_system(const DatalinkSystem& sys) {
        if (config["publish_system_status"].asBool()) {
                system_status_publishers.insert(std::make_pair(
                        sys.id,
                        this->create_publisher<SystemStatus>(sys.topic, 10)
                ));
        } else {
                system_status_subscriptions.insert(std::make_pair(sys.id,  this->create_subscription<SystemStatus>(
                        sys.topic,
                        10,
                        [this, &sys] (SystemStatus::SharedPtr msg) {
                                this->system_status_callback(sys.id, msg);
                        }
                )));
        }
}
