#include <jsoncpp/json/value.h>
#include <stdint.h>
#include <map>
#include <string>
#include <sys/types.h>

#include "stardos_interfaces/msg/node_heartbeat.hpp"

#include "../datalink_util.hpp"
#include "mavlinked_node.hpp"

using stardos_interfaces::msg::NodeHeartbeat;

class DatalinkServer : public virtual MAVLinkedNode {
public:
        DatalinkServer(
                std::string name,
                Json::Value config,
                std::map<DatalinkScope, pid_t> clients
        );

        struct DatalinkClient {
                pid_t pid;
                DatalinkScope scope;
                std::shared_ptr<mavsdk::Mavsdk> connection;
                rclcpp::Subscription<NodeHeartbeat>::SharedPtr heartbeat_subscription;
        };
protected:
        std::vector<DatalinkClient> clients;

        void target_passthrough_found_callback() override;

        void client_heartbeat_callback(NodeHeartbeat::SharedPtr);
};
