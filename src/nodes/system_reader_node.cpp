#include <fstream>

#include "nodes/system_reader_node.hpp"

void SystemReaderNode::load_systems(std::function<void(const DatalinkSystem&)> callback) {
        std::ifstream file(extra_config_directory + "/systems.json");
        Json::Value root;

        file >> root;

        for (auto v = root.begin(); v != root.end(); v++) {
                DatalinkSystem sys;
                sys.id    = (uint8_t) ((*v)["id"].asInt());
                sys.name  = (*v)["name"].asString();
                sys.topic = (*v)["topic"].asString();

                callback(sys);
        }
}
