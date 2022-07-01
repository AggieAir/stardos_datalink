#include <iostream>
#include <jsoncpp/json/json.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "datalink.hpp"
#include "datalink_constants.hpp"

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

        rclcpp::spin(std::make_shared<Datalink>(
                "datalink",
                config
        ));
}
