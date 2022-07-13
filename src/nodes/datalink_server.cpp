#include <memory>

#include "datalink_util.hpp"
#include "nodes/datalink_server.hpp"

using namespace std::placeholders;

DatalinkServer::DatalinkServer(
        std::string name,
        Json::Value config,
        std::map<DatalinkScope, pid_t> clients
): BasicDatalinkNode(name, config), MAVLinkedNode(mavsdk::ForwardingOption::ForwardingOn) {
        RCLCPP_INFO(this->get_logger(), "Starting to add mavlink clients");
        for (auto c: clients) {
                std::shared_ptr<DatalinkClient> client_struct = std::make_shared<DatalinkClient>();
                this->clients.push_back(client_struct);

                client_struct->pid = c.second;
                client_struct->scope = c.first;
                client_struct->heartbeat_subscription = this->create_subscription<NodeHeartbeat>(
                        get_node_name(c.first) + "/heartbeat",
                        10,
                        [this, client_struct] (NodeHeartbeat::SharedPtr heartbeat) {
                                this->client_heartbeat_callback(client_struct, heartbeat);
                        }
                );
        }
}

void DatalinkServer::target_passthrough_found_callback() {
        setup_message_forwarding(target_passthrough);
}

void DatalinkServer::setup_message_forwarding(std::shared_ptr<mavsdk::MavlinkPassthrough> passthrough) {
        passthrough->intercept_incoming_messages_async(
                std::bind(&DatalinkServer::forward_message, this, _1)
        );
}

bool DatalinkServer::forward_message(mavlink_message_t& msg) {
        if (msg.sysid == targetsysid && own_passthrough) {
                own_passthrough->send_message(msg);
        } else if (msg.sysid == sysid && target_passthrough) {
                target_passthrough->send_message(msg);
        } 

        return true;
}

void DatalinkServer::client_heartbeat_callback(
                std::shared_ptr<DatalinkClient> client,
                NodeHeartbeat::SharedPtr
) {
        if (!client->connected) {
                RCLCPP_INFO(this->get_logger(), "Opening server for component %s", get_node_name(client->scope).c_str());
                int port = config["base_port"].asInt() + client->scope;
                mavsdk::ConnectionResult result = dc.add_any_connection(
                        "udp://:" + std::to_string(port),
                        mavsdk::ForwardingOption::ForwardingOff
                );

                if (result != mavsdk::ConnectionResult::Success) {
                        RCLCPP_ERROR(
                                this->get_logger(),
                                "Couldn't connect to datalink/%s port %d",
                                get_node_name(client->scope).c_str(),
                                port
                        );
                }

                client->connected = true;
        }

        if (!own_passthrough) {
                for (auto s : dc.systems()) {
                        if (s->get_system_id() == sysid) {
                                own_system = s;
                                own_passthrough = std::make_shared<mavsdk::MavlinkPassthrough>(s);
                                // setup_message_forwarding(own_passthrough);
                        }
                }
        }
}
