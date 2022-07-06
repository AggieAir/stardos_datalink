#include "nodes/basic_datalink_node.hpp"

BasicDatalinkNode::BasicDatalinkNode(
        const std::string& name,
        const Json::Value& config
) : Node(name),
        name{name},
        config{config}
{
        Json::Value ecd = config["extra_config_directory"];
        if (ecd.isString()) {
                extra_config_directory = ecd.asString();
        } else {
                RCLCPP_WARN(this->get_logger(), "extra_config_directory was not given; using ./files");
                extra_config_directory = "./files";
        }
}
