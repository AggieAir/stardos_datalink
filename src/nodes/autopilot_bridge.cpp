#include <fstream>
#include <functional>
#include <iostream>
#include <chrono>
#include <string.h>
#include <math.h>
#include <string>
#include <climits>

#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>
#include <utility>

#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "stardos_interfaces/msg/node_heartbeat.hpp"
#include "stardos_interfaces/msg/control.hpp"
#include "stardos_interfaces/msg/global_position.hpp"
#include "stardos_interfaces/msg/gps_position.hpp"
#include "stardos_interfaces/msg/attitude.hpp"
#include "stardos_interfaces/msg/system_time.hpp"
#include "stardos_interfaces/msg/system_status.hpp"
#include "stardos_interfaces/msg/star_command_downlink.hpp"
#include "stardos_interfaces/msg/star_command_uplink.hpp"

#include "floattelem.hpp"
#include "nodes/autopilot_bridge.hpp"

using namespace mavsdk;
using namespace std::literals::chrono_literals;
using namespace std::placeholders;

using rcl_interfaces::msg::ParameterDescriptor;
using stardos_interfaces::msg::NodeHeartbeat;
using stardos_interfaces::msg::Control;
using stardos_interfaces::msg::GlobalPosition;
using stardos_interfaces::msg::GPSPosition;
using stardos_interfaces::msg::Attitude;
using stardos_interfaces::msg::SystemTime;
using stardos_interfaces::msg::SystemStatus;
using stardos_interfaces::msg::StarCommandDownlink;
using stardos_interfaces::msg::StarCommandUplink;

typedef floattelem::Message TelemMessage;
typedef floattelem::Header TelemHeader;

AutopilotBridge::AutopilotBridge(
        const std::string& name,
        const Json::Value& config
) : BasicDatalinkNode(name, config), MAVLinkedNode(ForwardingOption::ForwardingOn) {
        setup_autopilot_telemetry();
}

void AutopilotBridge::setup_autopilot_telemetry() {
        RCLCPP_INFO(this->get_logger(), "Creating MAVLink bridge publishers");

        gps_raw_publisher = this->create_publisher<GPSPosition>("gps_raw", 10);
        gps_position_publisher = this->create_publisher<GPSPosition>("gps_position", 10);
        global_position_publisher = this->create_publisher<GlobalPosition>("global_position", 10);
        attitude_publisher = this->create_publisher<Attitude>("attitude", 10);
        systime_publisher = this->create_publisher<SystemTime>("system_time", 10);
}

void AutopilotBridge::target_passthrough_found_callback() {
        RCLCPP_INFO(this->get_logger(), "Bridging MAVLink messages");

        target_passthrough->subscribe_message_async(
                        MAVLINK_MSG_ID_GPS_RAW_INT,
                        std::bind(&AutopilotBridge::gps_raw_received_callback, this, _1));

        target_passthrough->subscribe_message_async(
                        MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                        std::bind(&AutopilotBridge::global_position_received_callback, this, _1));

        target_passthrough->subscribe_message_async(
                        MAVLINK_MSG_ID_ATTITUDE,
                        std::bind(&AutopilotBridge::attitude_received_callback, this, _1));

        target_passthrough->subscribe_message_async(
                        MAVLINK_MSG_ID_SYSTEM_TIME,
                        std::bind(&AutopilotBridge::systime_received_callback, this, _1));
}

// here's a fun section full of lots of repetitive code
// this all is responsible for bridging mavlink messages to ROS topics
// the way I do this is by copying all of the fields.
// if we ever switch to using the PX4-ROS2 bridge, this is all getting replaced.

void AutopilotBridge::gps_raw_received_callback(const mavlink_message_t& msg) const {
        mavlink_gps_raw_int_t *gps = new mavlink_gps_raw_int_t();
        mavlink_msg_gps_raw_int_decode(&msg, gps);

        GPSPosition ros_message;

        ros_message.alt = gps->alt;
        ros_message.alt_ellipsoid = gps->alt_ellipsoid;
        ros_message.cog = gps->cog;
        ros_message.eph = gps->eph;
        ros_message.epv = gps->epv;
        ros_message.fix_type = gps->fix_type;
        ros_message.h_acc = gps->h_acc;
        ros_message.hdg_acc = gps->hdg_acc;
        ros_message.lat = gps->lat;
        ros_message.lon = gps->lon;
        ros_message.satellites_visible = gps->satellites_visible;
        ros_message.time_usec = gps->time_usec;
        ros_message.v_acc = gps->v_acc;
        ros_message.vel = gps->vel;
        ros_message.vel_acc = gps->vel_acc;
        ros_message.yaw = gps->yaw;

        gps_raw_publisher->publish(ros_message);
        gps_position_publisher->publish(ros_message);
}

void AutopilotBridge::global_position_received_callback(const mavlink_message_t& msg) const {
        mavlink_global_position_int_t *gps = new mavlink_global_position_int_t();
        mavlink_msg_global_position_int_decode(&msg, gps);

        GlobalPosition ros_message;

        ros_message.time_boot_ms = gps->time_boot_ms;
        ros_message.lat = gps->lat;
        ros_message.lon = gps->lon;
        ros_message.alt = gps->alt;
        ros_message.relative_alt = gps->relative_alt;
        ros_message.vx = gps->vx;
        ros_message.vy = gps->vy;
        ros_message.vz = gps->vz;
        ros_message.hdg = gps->hdg;

        global_position_publisher->publish(ros_message);
}

void AutopilotBridge::attitude_received_callback(const mavlink_message_t& msg) const {
        mavlink_attitude_t *attitude = new mavlink_attitude_t();
        mavlink_msg_attitude_decode(&msg, attitude);

        Attitude ros_message;

        ros_message.time_boot_ms = attitude->time_boot_ms;
        ros_message.roll = attitude->roll;
        ros_message.pitch = attitude->pitch;
        ros_message.yaw = attitude->yaw;
        ros_message.rollspeed = attitude->rollspeed;
        ros_message.pitchspeed = attitude->pitchspeed;
        ros_message.yawspeed = attitude->yawspeed;

        attitude_publisher->publish(ros_message);
}

void AutopilotBridge::systime_received_callback(const mavlink_message_t& msg) const {
        mavlink_system_time_t *systime = new mavlink_system_time_t();
        mavlink_msg_system_time_decode(&msg, systime);

        SystemTime ros_message;

        ros_message.time_boot_ms = systime->time_boot_ms;
        ros_message.time_unix_us = systime->time_unix_usec;

        systime_publisher->publish(ros_message);
}
