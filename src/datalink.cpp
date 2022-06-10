#include <functional>
#include <iostream>
#include <chrono>
#include <string.h>
#include <math.h>
#include <string>

#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>

#include "rclcpp/rclcpp.hpp"
#include "stardos_interfaces/msg/node_heartbeat.hpp"
#include "stardos_interfaces/msg/control.hpp"
#include "stardos_interfaces/msg/gps_position.hpp"
#include "stardos_interfaces/msg/attitude.hpp"
#include "stardos_interfaces/msg/system_time.hpp"

#include "datalink.hpp"
#include "floattelem.hpp"

using namespace mavsdk;
using namespace std::literals::chrono_literals;
using namespace std::placeholders;

using rcl_interfaces::msg::ParameterDescriptor;
using stardos_interfaces::msg::NodeHeartbeat;
using stardos_interfaces::msg::Control;
using stardos_interfaces::msg::GPSPosition;
using stardos_interfaces::msg::Attitude;
using stardos_interfaces::msg::SystemTime;

typedef floattelem::Message TelemMessage;
typedef floattelem::Header TelemHeader;

Datalink::Datalink(
        std::string name,
        uint8_t sysid,
        uint8_t compid,
        bool heartbeat,
        std::string connection_url,
        uint8_t targetsysid,
        uint8_t targetcompid,
        bool autopilot_telemetry
) : Node(name),
        name{name},
        array_id{0},
        heartbeat_subscriptions{std::vector<rclcpp::Subscription<NodeHeartbeat>::SharedPtr>()},
        signal_subscriptions{std::vector<rclcpp::Subscription<Control>::SharedPtr>()},
        heartbeat_publishers{std::vector<rclcpp::Publisher<NodeHeartbeat>::SharedPtr>()},
        signal_publishers{std::vector<rclcpp::Publisher<Control>::SharedPtr>()},
        buffered_message{TelemMessage()}
{
        ParameterDescriptor ro;
        ro.read_only = true;

        // MAVLink stuff, in parameters
        // this node's system id and component id
        this->declare_parameter<int>("sysid", sysid, ro);
        this->declare_parameter<int>("compid", compid, ro);

        // the target node's system id and component id
        this->declare_parameter<int>("targetsysid", targetsysid, ro);
        this->declare_parameter<int>("targetcompid", targetcompid, ro);
        
        // whether to send heartbeats
        this->declare_parameter<bool>("heartbeat", heartbeat, ro);

        // where to forge a connection to
        this->declare_parameter<std::string>("connection_url", connection_url, ro);

        this->declare_parameter<bool>("autopilot_telemetry", autopilot_telemetry, ro);

	configure();
	connect();

        RCLCPP_INFO(this->get_logger(), "Creating telemetry publisher");

        control_subscription = this->create_subscription<Control>(
                        name + "/control",
                        10,
                        std::bind(&Datalink::control_callback, this, _1));

        setup_autopilot_telemetry(this->get_parameter("autopilot_telemetry").as_bool());

        RCLCPP_INFO(this->get_logger(), "Binding timer callback");

        // Yes, the callback to check for the target system is just on a timer.
        // Yes, I know that Mavsdk::subscribe_on_system_added exists.
        // I could not get it to work consistently.
	get_system_timer = this->create_wall_timer(1000ms, std::bind(&Datalink::check_systems, this));
}

void Datalink::configure() {
	dc.set_configuration(
		Mavsdk::Configuration(
                        this->get_parameter("sysid").as_int(),
                        this->get_parameter("compid").as_int(),
                        this->get_parameter("heartbeat").as_bool()
                )
        );
}

void Datalink::connect() {
        dc.add_any_connection(this->get_parameter("connection_url").as_string());
}

void Datalink::setup_autopilot_telemetry(bool activate) {
        if (activate) {
                RCLCPP_INFO(this->get_logger(), "Creating MAVLink bridge publishers");

                gps_publisher = this->create_publisher<GPSPosition>("gps_position", 10);
                attitude_publisher = this->create_publisher<Attitude>("attitude", 10);
                systime_publisher = this->create_publisher<SystemTime>("system_time", 10);
                if (get_system_timer != nullptr && get_system_timer->is_canceled()) {
                        get_system_timer->reset();
                }
        } else {
                gps_publisher = nullptr;
                attitude_publisher = nullptr;
                systime_publisher = nullptr;
        }
}

void Datalink::send() {
        if (buffered_message.is_empty()) {
                return;
        }

        mavlink_message_t message;

        if (array_id == UINT16_MAX) array_id = 0;

        mavlink_msg_logging_data_pack(
                target_passthrough->get_our_sysid(), // SystemID
                target_passthrough->get_our_compid(), //My comp ID
                &message, //Message reference
                this->get_parameter("targetsysid").as_int(),
                this->get_parameter("targetcompid").as_int(),
                array_id++,
                buffered_message.get_offset() * 4,
                0,
                (uint8_t*) buffered_message.get_data()
        );

        MavlinkPassthrough::Result result = target_passthrough->send_message(message);

        if (result != MavlinkPassthrough::Result::Success) {
                std::cout << "command send failed: " << result << "\n";
        } else {
                RCLCPP_INFO(this->get_logger(), "Resetting message buffer");
                buffered_message.reset();
        }
}

void Datalink::check_systems() {
        for (auto s : dc.systems()) {
                if (s->get_system_id() == this->get_parameter("targetsysid").as_int() && target == nullptr) {
                        // the target for float telemetry
                        RCLCPP_INFO(
                                this->get_logger(),
                                "Found target system (ID=%d)",
                                this->get_parameter("targetsysid").as_int()
                        );

                        target = s;
                        target_passthrough = std::make_shared<MavlinkPassthrough>(target);

                        target_passthrough->subscribe_message_async(
                                        MAVLINK_MSG_ID_LOGGING_DATA,
                                        std::bind(&Datalink::array_received_callback, this, _1));

                        send_telemetry_timer = this->create_wall_timer(
                                        1000ms,
                                        std::bind(&Datalink::send, this));
                }

                if (s->get_system_id() == 1 &&
                                autopilot == nullptr &&
                                this->get_parameter("autopilot_telemetry").as_bool()) {
                        // the autopilot
                        RCLCPP_INFO(this->get_logger(), "Found autopilot");

                        if (this->get_parameter("targetsysid").as_int() == 1) {
                                autopilot = target;
                                autopilot_passthrough = target_passthrough;
                        } else {
                                autopilot = s;
                                autopilot_passthrough = std::make_shared<MavlinkPassthrough>(autopilot);
                        }


                        RCLCPP_INFO(this->get_logger(), "Bridging MAVLink messages");

                        autopilot_passthrough->subscribe_message_async(
                                        MAVLINK_MSG_ID_GPS_RAW_INT,
                                        std::bind(&Datalink::gps_received_callback, this, _1));

                        autopilot_passthrough->subscribe_message_async(
                                        MAVLINK_MSG_ID_ATTITUDE,
                                        std::bind(&Datalink::attitude_received_callback, this, _1));

                        autopilot_passthrough->subscribe_message_async(
                                        MAVLINK_MSG_ID_SYSTEM_TIME,
                                        std::bind(&Datalink::systime_received_callback, this, _1));
                }

                if (target != nullptr &&
                                (autopilot != nullptr || !this->get_parameter("autopilot_telemetry").as_bool())) {
                        RCLCPP_INFO(this->get_logger(), "Systems found; ending search.");
                        get_system_timer->cancel();
                }
        }
}

void Datalink::heartbeat_callback(int id, NodeHeartbeat::SharedPtr msg) {
        if (target_passthrough == nullptr) {
                RCLCPP_INFO(this->get_logger(), "Tried to send a message before target system was found");
                return;
        }

        RCLCPP_INFO(this->get_logger(), "Queueing heartbeat message (offset=%d)", buffered_message.get_offset());

        if (!buffered_message.push_heartbeat_message(msg, id)) {
                send();
                buffered_message.reset();
                buffered_message.push_heartbeat_message(msg, id);
        }
}

void Datalink::signal_callback(int id, Control::SharedPtr ctrl) {
        if (target_passthrough == nullptr) {
                RCLCPP_INFO(this->get_logger(), "Tried to send a control signal before target system was found");
                return;
        }

        if (ctrl->options.size() > floattelem::MAX_STRING_LENGTH) {
                RCLCPP_ERROR(this->get_logger(), "Option string too long; will be truncated");
        }

        if (!buffered_message.push_control_message(ctrl->options, id)) {
                send();
                buffered_message.reset();
                buffered_message.push_control_message(ctrl->options, id);
        }
}

void Datalink::control_callback(Control::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Got control callback");
        Json::Value root;
        Json::Reader reader;

        RCLCPP_INFO(this->get_logger(), "Parsing JSON");
        reader.parse(msg->options, root);

        heartbeat_subscriptions.clear();

        RCLCPP_INFO(this->get_logger(), "Creating heartbeat subscribers");
        this->fill_subscriber_list<NodeHeartbeat>(
                root["heartbeat"]["sub"],
                &this->heartbeat_subscriptions,
                std::bind(&Datalink::heartbeat_callback, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Creating heartbeat publishers");
        this->fill_publisher_list<NodeHeartbeat>(root["heartbeat"]["pub"], &this->heartbeat_publishers);

        RCLCPP_INFO(this->get_logger(), "Creating control message subscribers");
        this->fill_subscriber_list<Control>(
                root["control"]["sub"],
                &this->signal_subscriptions,
                std::bind(&Datalink::signal_callback, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Creating control message publishers");
        this->fill_publisher_list<Control>(root["control"]["pub"], &this->signal_publishers);
}

void Datalink::array_received_callback(mavlink_message_t msg) {
        mavlink_logging_data_t * inner = new mavlink_logging_data_t();
        mavlink_msg_logging_data_decode(&msg, inner);

        if (
                        msg.sysid != this->get_parameter("targetsysid").as_int() ||
                        msg.compid != this->get_parameter("targetcompid").as_int() ||
                        inner->target_system != this->get_parameter("sysid").as_int() ||
                        inner->target_component != this->get_parameter("compid").as_int()
        ) {
                return;
        }

        TelemMessage message = TelemMessage(inner->data);

        while (message.has_next()) {
                TelemHeader head = message.next_header();
                if (head.msg_type == floattelem::MSG_ID_HEARTBEAT) {
                        NodeHeartbeat ros_message = message.pop_heartbeat_message();
                        RCLCPP_INFO(this->get_logger(), "Offset now=%d", buffered_message.get_offset());

                        if (head.topic_id >= heartbeat_publishers.size()) {
                                RCLCPP_ERROR(
                                        this->get_logger(),
                                        "Heartbeat publisher with ID=%d out of range",
                                        head.topic_id);
                        } else {
                                this->heartbeat_publishers[head.topic_id]->publish(ros_message);
                        }
                } else if (head.msg_type == floattelem::MSG_ID_CONTROL) {
                        std::string options = message.pop_control_message();

                        if (head.topic_id >= signal_publishers.size()) {
                                RCLCPP_ERROR(
                                        this->get_logger(),
                                        "Signal publisher with ID=%d out of range",
                                        head.topic_id);
                        } else {
                                Control c;
                                c.options = options;
                                this->signal_publishers[head.topic_id]->publish(c);
                        }
                } else {
                        RCLCPP_ERROR(
                                this->get_logger(),
                                "Unrecognized message ID: %d",
                                head.msg_type);
                }
        }
}

// here's a fun section full of lots of repetitive code
// this all is responsible for bridging mavlink messages to ROS topics
// the way I do this is by copying all of the fields.
// if we ever switch to using the PX4-ROS2 bridge, this is all getting replaced.

void Datalink::gps_received_callback(mavlink_message_t msg) {
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

        gps_publisher->publish(ros_message);
}

void Datalink::attitude_received_callback(mavlink_message_t msg) {
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

void Datalink::systime_received_callback(mavlink_message_t msg) {
        mavlink_system_time_t *systime = new mavlink_system_time_t();
        mavlink_msg_system_time_decode(&msg, systime);

        SystemTime ros_message;

        ros_message.time_boot_ms = systime->time_boot_ms;
        ros_message.time_unix_us = systime->time_unix_usec;

        systime_publisher->publish(ros_message);
}

template<typename T>
void Datalink::fill_subscriber_list(Json::Value& topics, std::vector<typename rclcpp::Subscription<T>::SharedPtr> *dest, std::function<void(int, std::shared_ptr<T>)> callback) {
        int id = 0;
        for (auto v = topics.begin(); v != topics.end(); v++) {
                std::string topic = v->asString();

                dest->push_back(
                    this->create_subscription<T>(
                        topic, 10,
                        [this, id, callback] (std::shared_ptr<T> msg) {
                                callback(id, msg);
                        }));
                id++;
        }
}

template<typename T>
void Datalink::fill_publisher_list(Json::Value& topics, std::vector<typename rclcpp::Publisher<T>::SharedPtr> *dest) {
        for (auto v = topics.begin(); v != topics.end(); v++) {
                std::string topic = v->asString();
                dest->push_back(
                        this->create_publisher<T>(
                                topic,
                                10));
        }
}
