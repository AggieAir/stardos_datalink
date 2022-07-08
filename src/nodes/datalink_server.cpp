#include <memory>

#include "datalink_util.hpp"
#include "nodes/datalink_server.hpp"

using namespace std::placeholders;

DatalinkServer::DatalinkServer(
        std::string name,
        Json::Value config,
        std::map<DatalinkScope, pid_t> clients
): BasicDatalinkNode(name, config) {
        for (auto c: clients) {
                std::shared_ptr<mavsdk::Mavsdk> m = std::make_shared<mavsdk::Mavsdk>();
                m->set_configuration(mavsdk::Mavsdk::Configuration(sysid, compid, true));
                m->add_any_connection("udp:/:" + std::to_string(22000 + c.first));
                DatalinkClient client_struct;
                client_struct.pid = c.second;
                client_struct.scope = c.first;
                client_struct.connection = m;
                client_struct.heartbeat_subscription = this->create_subscription<NodeHeartbeat>(
                        get_node_name(c.first),
                        10,
                        std::bind(&DatalinkServer::client_heartbeat_callback, this, _1)
                );
                
                this->clients.push_back(client_struct);
        }
}

void DatalinkServer::target_passthrough_found_callback() {}

void DatalinkServer::client_heartbeat_callback(NodeHeartbeat::SharedPtr) {}
