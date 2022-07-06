#include <fstream>

#include "nodes/system_reader_node.hpp"

SystemReaderNode::SystemReaderNode() {
        load_systems();
}

void SystemReaderNode::load_systems() {
        std::ifstream file(extra_config_directory + "/systems.json");
        Json::Value root;

        file >> root;

        for (auto v = root.begin(); v != root.end(); v++) {
                uint8_t     id    = (*v)["id"].asInt();
                std::string name  = (*v)["name"].asString();
                std::string topic = (*v)["topic"].asString();

                add_system(id, name, topic);
        }
}
