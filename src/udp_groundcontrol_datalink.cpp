#include <mavsdk/mavsdk.h>
#include <string>

#include "abstract_datalink.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

const uint8_t SYSIDGCS = 190;
const uint8_t COMPIDGCS = 190;
const bool HEARTBEATGCS = true;

class UDPGroundControlDatalink : AbstractDatalinkNode {
public:
  void configure() override {
    mav.set_configuration(
        mavsdk::Mavsdk::Configuration(SYSIDGCS, COMPIDGCS, HEARTBEATGCS));
  }

  mavsdk::ConnectionResult connect() override {
    return mav.add_any_connection(std::string("udp://0.0.0.0:14540"));
  }
};
