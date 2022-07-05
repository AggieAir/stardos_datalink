#include <iostream>
#include <jsoncpp/json/json.h>
#include <memory>
#include <sys/types.h>
#include <unistd.h>

#include "nodes/floattelem_bridge.hpp"
#include "nodes/autopilot_bridge.hpp"
#include "rclcpp/rclcpp.hpp"

#include "datalink.hpp"
#include "datalink_util.hpp"

int main(int argc, char *argv[]) {
        rclcpp::init(argc, argv);

        Json::Value root, config;

        try {
                std::cin >> root;
        } catch (...) {
                std::cerr << "parse error in configuration\n";
                return 1;
        }

        if (!root.isObject()) {
                std::cerr << "expected root to be object, got " << root.type() << "\n";
                return 1;
        }

        if (!(config = root["config"]).isObject()) {
                std::cerr << "expected config to be object, got " << config.type() << "\n";
                return 1;
        }

        std::vector<DatalinkScope> scopes{FLOATTELEM_BRIDGE, AUTOPILOT_BRIDGE};

        for (auto s : scopes) {
                pid_t p = fork();

                if (p == -1) {
                        std::cerr << "Couldn't fork.\n";
                } else if (p == 0) {
                        switch (s) {
                        case FLOATTELEM_BRIDGE:
                                rclcpp::spin(std::make_shared<FloatTelemBridge>("datalink_ftb", config));
                                break;
                        case AUTOPILOT_BRIDGE:
                                rclcpp::spin(std::make_shared<AutopilotBridge>("datalink_apb", config));
                                break;
                        default:
                                break;
                        }
                }
        }
}
