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
                bool connected;
                pid_t pid;
                DatalinkScope scope;
                mavsdk::Mavsdk connection;
                rclcpp::Subscription<NodeHeartbeat>::SharedPtr heartbeat_subscription;
        };
protected:
        std::vector<std::shared_ptr<DatalinkClient>> clients;

        void target_passthrough_found_callback() override;

        void client_heartbeat_callback(std::shared_ptr<DatalinkClient>, NodeHeartbeat::SharedPtr);
};
