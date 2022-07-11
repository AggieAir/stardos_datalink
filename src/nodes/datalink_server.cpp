#include <memory>

#include "datalink_util.hpp"
#include "nodes/datalink_server.hpp"

using namespace std::placeholders;

DatalinkServer::DatalinkServer(
        std::string name,
        Json::Value config,
        std::map<DatalinkScope, pid_t> clients
): BasicDatalinkNode(name, config) {
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

void DatalinkServer::target_passthrough_found_callback() {}

void DatalinkServer::client_heartbeat_callback(std::shared_ptr<DatalinkClient> client, NodeHeartbeat::SharedPtr) {
        mavsdk::Mavsdk& conn = client->connection;

        if (client->connected) return;

        RCLCPP_INFO(this->get_logger(), "Opening server for component %s", get_node_name(client->scope).c_str());
        conn.set_configuration(mavsdk::Mavsdk::Configuration(sysid, compid, true));
        mavsdk::ConnectionResult result = conn.add_any_connection("udp://:" + std::to_string(22000 + client->scope));
        if (result != mavsdk::ConnectionResult::Success) {
                RCLCPP_ERROR(
                        this->get_logger(),
                        "Couldn't connect to datalink/%s", get_node_name(client->scope).c_str()
                );
        } else {
                client->connected = true;
        }
}
