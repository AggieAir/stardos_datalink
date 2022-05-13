#include <mavsdk/mavsdk.h>
#include <string>

#include "abstract_datalink.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

const uint8_t SYSIDVEHICLE = 1;
const uint8_t COMPIDCOPILOT = 220;
const bool HEARTBEATCOPILOT = true;

class UDPCopilotDatalink : AbstractDatalinkNode {
public:
  void configure() override {
    mav.set_configuration(mavsdk::Mavsdk::Configuration(
        SYSIDVEHICLE, COMPIDCOPILOT, HEARTBEATCOPILOT));
  }

  mavsdk::ConnectionResult connect() override {
    return mav.add_any_connection(std::string("udp://0.0.0.0:14540"));
  }
};
