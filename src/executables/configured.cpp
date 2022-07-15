#include <cstdlib>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <memory>
#include <string>
#include <sys/types.h>
#include <unistd.h>
#include <chrono>

#include "nodes/floattelem_bridge.hpp"
#include "nodes/autopilot_bridge.hpp"
#include "nodes/starcommand_serializer.hpp"
#include "nodes/datalink_server.hpp"
#include "nodes/mavlink_camera_capture.hpp"
#include "rclcpp/rclcpp.hpp"

#include "datalink.hpp"
#include "datalink_util.hpp"

std::map<DatalinkScope, pid_t> processes;

void handle_signal(int) {
        std::cout << "Got terminate signal\n";
        for (auto pair : processes) {
                kill(pair.second, SIGTERM);
                std::cout << "Killing " << pair.second << "\n";
        }

        exit(0);
}

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

        std::vector<DatalinkScope> scopes;

        if (config["heartbeat"].isObject() || config["control"].isObject()) {
                scopes.push_back(FLOATTELEM_BRIDGE);
        }

        if (config["autopilot_telemetry"].asBool()) {
                scopes.push_back(AUTOPILOT_BRIDGE);
        }

        if (config["starcommand"].isObject()) {
                scopes.push_back(STARCOMMAND_SERIALIZER);
        }

        if (config["cameras"].isInt()) {
                for (int i  = 0; i < config["cameras"].asInt() && i < 6; i++) {
                        scopes.push_back((DatalinkScope) (CAMERA + i));
                }
        }
        
        config["targetcompid"] = SERVER;

        scopes.push_back(SERVER);

        int port = 22000;
        if (config["base_port"].isInt()) {
                port = config["base_port"].asInt();
        } else {
                std::cerr << "expected a valid base port; using 22000 as default\n";
        }

        for (auto s : scopes) {
                pid_t p = 0;
                if (s == SERVER) {
                        signal(SIGTERM, &handle_signal);
                        signal(SIGINT, &handle_signal);
                } else {
                        p = fork();
                }

                if (p == -1) {
                        std::cerr << "Couldn't fork.\n";
                } else if (p == 0) {
                        port += s;
                        if (s != SERVER) {
                                config["connection_url"] = "udp://127.0.0.1:" + std::to_string(port);
                        }
                        config["compid"] = s;

                        std::string node_name = get_node_name(s);

                        switch (s) {
                        case FLOATTELEM_BRIDGE:
                                rclcpp::spin(std::make_shared<FloatTelemBridge>(node_name, config));
                                break;
                        case AUTOPILOT_BRIDGE:
                                rclcpp::spin(std::make_shared<AutopilotBridge>(node_name, config));
                                break;
                        case STARCOMMAND_SERIALIZER:
                                rclcpp::spin(std::make_shared<StarCommandSerializer>(node_name, config));
                                break;
                        case SERVER:
                                rclcpp::spin(std::make_shared<DatalinkServer>(node_name, config, processes));
                                break;
                        case CAMERA:
                        case CAMERA2:
                        case CAMERA3:
                        case CAMERA4:
                        case CAMERA5:
                        case CAMERA6:
                                rclcpp::spin(std::make_shared<MAVLinkCameraCapture>(node_name, config, s - CAMERA + 1));
                                break;
                        default:
                                break;
                        }
                } else {
                        processes[s] = p;
                }
        }

        while (true) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
}
