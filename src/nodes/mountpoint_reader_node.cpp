#include <fstream>

#include "nodes/mountpoint_reader_node.hpp"

MountpointReaderNode::MountpointReaderNode() {
        load_mountpoints();
}

void MountpointReaderNode::load_mountpoints() {
        std::ifstream file(extra_config_directory + "/mountpoints.json");
        Json::Value root;

        file >> root;

        uint8_t i = 0;
        for (auto v = root.begin(); v != root.end(); v++) {
                mountpoints.insert(std::make_pair(v->asString(), i));
                mountpoint_names.push_back(v->asString());
                i++;
        }
}
