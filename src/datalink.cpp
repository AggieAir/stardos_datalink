#include <fstream>
#include <functional>
#include <filesystem>
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
#include "stardos_interfaces/srv/topic_list.hpp"
#include "stardos_interfaces/srv/star_command_topics.hpp"

#include "datalink.hpp"
#include "floattelem.hpp"

using namespace mavsdk;
using namespace std::literals::chrono_literals;
using namespace std::placeholders;
using namespace stardos_interfaces::msg;
using namespace stardos_interfaces::srv;

typedef floattelem::Message TelemMessage;
typedef floattelem::Header TelemHeader;

Datalink::Datalink(
        const std::string& name,
        // uint8_t sysid,
        // uint8_t compid,
        // bool heartbeat,
        // const std::string& connection_url,
        // uint8_t targetsysid,
        // uint8_t targetcompid,
        // bool autopilot_telemetry,
        // bool starcommand,
        // bool publish_system_status,
        const Json::Value config
) : rclcpp::Node(name),
        config{config},
        buffered_message{TelemMessage()}
{
	configure();
	connect();

        if (config["detect_environment"].asBool()) {
		RCLCPP_INFO(this->get_logger(), "Attempting to detect environment...");
                while (payload == "") {
                        detect_environment();
                }
        } else {
                aircraft = "aircraft";
                payload = "payload";
        }

        RCLCPP_INFO(
                this->get_logger(),
                "Running on aircraft '%s' with payload '%s'",
                aircraft.c_str(),
                payload.c_str()
        );

        RCLCPP_INFO(this->get_logger(), "Creating telemetry publisher");

        setup_autopilot_telemetry();

        load_system_statuses();
        load_mountpoints();

        RCLCPP_INFO(this->get_logger(), "Binding timer callback");

        // Yes, the callback to check for the target system is just on a timer.
        // Yes, I know that Mavsdk::subscribe_on_system_added exists.
        // I could not get it to work consistently.
	get_system_timer = this->create_wall_timer(1000ms, std::bind(&Datalink::check_systems, this));

        setup_floattelem();
}

void Datalink::detect_environment() {
        std::string ns(get_namespace());
        int idx = ns.find('/', 1);
        aircraft = ns.substr(1, idx - 1);

        std::vector<std::string> node_names = get_node_names();

        for (std::string n : node_names) {
                int idx1 = n.find('/', 1);
                int idx2 = n.find('/', idx1 + 1);
		int fidx = n.rfind('/');
                std::string computer = n.substr(idx1 + 1, idx2 - idx1 - 1);
		std::string node = n.substr(fidx + 1);

                if (computer != "copilot" && node == "control") {
                        payload = computer;

                        break;
                }
        }
}

void Datalink::configure() {
        Json::Value sysidval = config["sysid"];
        Json::Value compidval = config["compid"];
        Json::Value targetsysidval = config["targetsysid"];
        Json::Value targetcompidval = config["targetcompid"];
        Json::Value filesval = config["extra_config_directory"];

        if (
                        !sysidval.isUInt() ||
                        sysidval.asUInt() > UINT8_MAX ||
                        !compidval.isUInt() ||
                        compidval.asUInt() > UINT8_MAX ||
                        !targetsysidval.isUInt() ||
                        targetsysidval.asUInt() > UINT8_MAX ||
                        !targetcompidval.isUInt() ||
                        targetcompidval.asUInt() > UINT8_MAX ||
                        !filesval.isString()
        ) {
                RCLCPP_ERROR(this->get_logger(), "System and Component IDs must be ints within [0, 255]");
        }

        sysid = sysidval.asUInt();
        compid = compidval.asUInt();
        targetsysid = targetsysidval.asUInt();
        targetcompid = targetcompidval.asUInt();
        extra_config_directory = filesval.asString();

        RCLCPP_INFO(
                this->get_logger(),
                "Configuring: %d/%d looking for %d/%d",
                sysidval.asUInt(),
                compidval.asUInt(),
                targetsysidval.asUInt(),
                targetcompidval.asUInt()
        );

	dc.set_configuration(
		Mavsdk::Configuration(
                        (uint8_t) sysidval.asUInt(),
                        (uint8_t) compidval.asUInt(),
                        true
                )
        );
}

void Datalink::connect() {
        Json::Value urlval = config["connection_url"];
        if (!urlval.isString()) {
                RCLCPP_ERROR(this->get_logger(), "URL must be a string");
                throw std::exception();
        }
        dc.add_any_connection(config["connection_url"].asString());
}

void Datalink::setup_default_heartbeat_topics() {
        std::filesystem::path copilot_path("/"), payload_path("/");
        copilot_path = copilot_path / aircraft / "copilot" / "heartbeat";
        payload_path = payload_path / aircraft / payload / "heartbeat";
        std::vector<std::string> topics{copilot_path.string(), payload_path.string()};

        if (config["publish_default_topics"].asBool()) {
                fill_publisher_list<NodeHeartbeat>(
                        topics,
                        heartbeat_publishers,
                        heartbeat_publisher_ids
                );
        } else {
                fill_subscriber_list<NodeHeartbeat>(
                        topics,
                        heartbeat_subscriptions,
                        heartbeat_subscription_ids,
                        std::bind(&Datalink::heartbeat_callback, this, _1, _2)
                );
        }
}

void Datalink::setup_default_control_topics() {
        std::vector<std::string> topics{"/start_mission", "/end_mission"};

        if (!config["publish_default_topics"].asBool()) {
                fill_publisher_list<Control>(
                        topics,
                        signal_publishers,
                        control_publisher_ids
                );
        } else {
                fill_subscriber_list<Control>(
                        topics,
                        signal_subscriptions,
                        control_subscription_ids,
                        std::bind(&Datalink::signal_callback, this, _1, _2)
                );
        }
}

void Datalink::setup_autopilot_telemetry() {
        if (config["autopilot_telemetry"].asBool()) {
                RCLCPP_INFO(this->get_logger(), "Creating MAVLink bridge publishers");

                gps_raw_publisher = this->create_publisher<GPSPosition>("gps_raw", 10);
                gps_position_publisher = this->create_publisher<GPSPosition>("gps_position", 10);
                global_position_publisher = this->create_publisher<GlobalPosition>("global_position", 10);
                attitude_publisher = this->create_publisher<Attitude>("attitude", 10);
                systime_publisher = this->create_publisher<SystemTime>("system_time", 10);
        }
}

void Datalink::setup_starcommand(const std::string& downlink_topic, const std::string& uplink_topic) {
        starcommand_publisher = this->create_publisher<StarCommandDownlink>(downlink_topic, 10);
        starcommand_subscription = this->create_subscription<StarCommandUplink>(
                uplink_topic,
                10,
                std::bind(&Datalink::uplink_callback, this, _1)
        );
}

void Datalink::load_system_statuses() {
        // std::ifstream file(extra_config_directory + "/systems.json");
        // Json::Value root;

        // file >> root;

        // for (auto v = root.begin(); v != root.end(); v++) {
        //         uint8_t     id    = (*v)["id"].asInt();
        //         std::string name  = (*v)["name"].asString();
        //         std::string topic = (*v)["topic"].asString();

        //         if (config["publish_system_status"].asBool()) {
        //                 system_status_publishers[id] = this->create_publisher<SystemStatus>(topic, 10);
        //         } else if (payloads.find(name) == computers.end()) {
        //                 system_status_subscriptions[id] = this->create_subscription<SystemStatus>(
        //                         topic,
        //                         10,
        //                         [this, id] (SystemStatus::SharedPtr msg) {
        //                                 this->system_status_callback(id, msg);
        //                         }
        //                 );
        //         }
        // }
        
        std::filesystem::path payload_path = std::filesystem::path("/") / aircraft / payload / "status";
        std::filesystem::path copilot_path = std::filesystem::path("/") / aircraft / "copilot" / "status";

        RCLCPP_INFO(
                this->get_logger(),
                "%s on %s and %s for system stataus messages",
                config["publish_system_status"].asBool() ? "publishing" : "listening",
                payload_path.c_str(),
                copilot_path.c_str()
        );

        if (config["publish_system_status"].asBool()) {
                system_status_publishers[0] = this->create_publisher<SystemStatus>(
                        payload_path,
                        10
                );

                system_status_publishers[1] = this->create_publisher<SystemStatus>(
                        copilot_path,
                        10
                );
        } else {
                system_status_subscriptions[0] = this->create_subscription<SystemStatus>(
                        payload_path,
                        10,
                        [this] (SystemStatus::SharedPtr msg) {
                                this->system_status_callback(0, msg);
                        }
                );

                system_status_subscriptions[1] = this->create_subscription<SystemStatus>(
                        copilot_path,
                        10,
                        [this] (SystemStatus::SharedPtr msg) {
                                this->system_status_callback(1, msg);
                        }
                );
        }
}

void Datalink::load_mountpoints() {
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

void Datalink::send_buffered_message() {
        RCLCPP_DEBUG(this->get_logger(), "Trying to send buffered message");
        if (send_telemetry(buffered_message) == MavlinkPassthrough::Result::Success) {
                RCLCPP_INFO(this->get_logger(), "Resetting message buffer");
                buffered_message.reset();
        }
}

MavlinkPassthrough::Result Datalink::send_telemetry(const TelemMessage& msg) {
        RCLCPP_DEBUG(this->get_logger(), "Attempting to send telemetry (size=%hhu)", msg.get_offset());
        if (!target_passthrough || msg.is_empty()) {
                return MavlinkPassthrough::Result::Unknown;
        }

        mavlink_message_t message;

        if (array_id == UINT16_MAX) array_id = 0;

        mavlink_msg_logging_data_pack(
                target_passthrough->get_our_sysid(), // SystemID
                target_passthrough->get_our_compid(), //My comp ID
                &message, //Message reference
                targetsysid,
                targetcompid,
                array_id++,
                msg.get_offset() * 4,
                0,
                msg.get_data()
        );

        MavlinkPassthrough::Result result = target_passthrough->send_message(message);

        if (result != MavlinkPassthrough::Result::Success) {
                std::cout << "command send failed: " << result << "\n";
        }

        return result;
}

void Datalink::check_systems() {
        for (auto s : dc.systems()) {
                if (s->get_system_id() == targetsysid && target == nullptr) {
                        // the target for float telemetry
                        RCLCPP_INFO(
                                this->get_logger(),
                                "Found target system (ID=%d)",
                                targetsysid
                        );

                        target = s;
                        target_passthrough = std::make_shared<MavlinkPassthrough>(target);

                        target_passthrough->subscribe_message_async(
                                        MAVLINK_MSG_ID_LOGGING_DATA,
                                        std::bind(&Datalink::array_received_callback, this, _1));

                        RCLCPP_DEBUG(this->get_logger(), "setting up buffered message sending timer");
                        send_telemetry_timer = this->create_wall_timer(
                                        1000ms,
                                        std::bind(&Datalink::send_buffered_message, this));
                }

                if (s->get_system_id() == 1 &&
                                autopilot == nullptr &&
                                config["autopilot_telemetry"].asBool()
                ) {
                        // the autopilot
                        RCLCPP_INFO(this->get_logger(), "Found autopilot");

                        if (targetsysid == 1) {
                                autopilot = target;
                                autopilot_passthrough = target_passthrough;
                        } else {
                                autopilot = s;
                                autopilot_passthrough = std::make_shared<MavlinkPassthrough>(autopilot);
                        }


                        RCLCPP_INFO(this->get_logger(), "Bridging MAVLink messages");

                        autopilot_passthrough->subscribe_message_async(
                                        MAVLINK_MSG_ID_GPS_RAW_INT,
                                        std::bind(&Datalink::gps_raw_received_callback, this, _1));

                        autopilot_passthrough->subscribe_message_async(
                                        MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                                        std::bind(&Datalink::global_position_received_callback, this, _1));

                        autopilot_passthrough->subscribe_message_async(
                                        MAVLINK_MSG_ID_ATTITUDE,
                                        std::bind(&Datalink::attitude_received_callback, this, _1));

                        autopilot_passthrough->subscribe_message_async(
                                        MAVLINK_MSG_ID_SYSTEM_TIME,
                                        std::bind(&Datalink::systime_received_callback, this, _1));
                }

                if (target != nullptr &&
                                (autopilot != nullptr || !config["autopilot_telemetry"].asBool())) {
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
                send_buffered_message();
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

        TelemMessage tmsg;
        tmsg.push_control_message(ctrl->options, id);

        send_telemetry(tmsg);
}

void Datalink::system_status_callback(int id, SystemStatus::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received system status from computer %d", id);
        floattelem::SystemCapacity sc;
        sc.max_memory_mb = msg->memory[1];
        sc.max_swap_mb = msg->swap[1];

        for (auto v = msg->disks.begin(); v != msg->disks.end(); v += 2) {
                sc.disks_size_mb.push_back(v[1]);
        }

        auto inserted = cached_systems.insert(std::make_pair(id, sc)); // pair<pair<key, value>, bool>
        auto entry = inserted.first; // pair<key, value>
        bool success = inserted.second; // bool
        floattelem::SystemCapacity& cap = entry->second;

        TelemMessage tmsg;
        if (success || cap != sc) {
                RCLCPP_INFO(this->get_logger(), "Updating system capabilities");
                RCLCPP_DEBUG(this->get_logger(), "Pushing message");
                tmsg.push_system_capacity_message(&sc, id);
                RCLCPP_DEBUG(this->get_logger(), "Replacing entry in cached_systems");
                cap = sc;
        }

        RCLCPP_DEBUG(this->get_logger(), "Preparing FloatTelem message for SystemStatus");
        floattelem::SlimSystemStatus status;
        RCLCPP_DEBUG(this->get_logger(), "Adding static details");
        status.cpu_usage = msg->cpu_usage;
        status.memory = (uint16_t) ((float) msg->memory[0] / (float) msg->memory[1] * USHRT_MAX);
        status.swap = (uint16_t) ((float) msg->swap[0] / (float) msg->swap[1] * USHRT_MAX);
        status.uptime = msg->uptime;

        RCLCPP_DEBUG(this->get_logger(), "Adding disks");
        for (auto v = msg->disks.begin(); v != msg->disks.end(); v += 2) {
                status.disks.push_back((uint16_t) ((float) *v / (float) *(v+1) * USHRT_MAX));
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Adding mounts");
        for (auto v = msg->mounts.begin(); v != msg->mounts.end(); v++) {
                auto m = mountpoints.find(*v);
                if (m == mountpoints.end()) {
                        RCLCPP_ERROR(this->get_logger(), "%s is not a recognized mountpoint", v->c_str());
                        status.mounts.push_back(255);
                } else {
                        RCLCPP_DEBUG(this->get_logger(), "found mountpoint %s", m->first.c_str());
                        status.mounts.push_back(m->second);
                }
        }

        tmsg.push_system_status_message(&status, id);

        this->send_telemetry(tmsg);
}

void Datalink::setup_floattelem() {
        RCLCPP_INFO(this->get_logger(), "Setting up FloatTelem configuration services");

        // services.push_back(this->create_service<TopicList>(
        //         "heartbeat_sub",
        //         std::bind(
        //                 &Datalink::subscribe_service_callback<NodeHeartbeat>,
        //                 this, _1, _2,
        //                 this->heartbeat_subscriptions,
        //                 this->heartbeat_subscription_ids,
        //                 std::bind(&Datalink::heartbeat_callback, this, _1, _2)
        //         )
        // ));
        
        setup_default_heartbeat_topics();
        setup_default_control_topics();

        services.push_back(this->create_service<TopicList>(
                "heartbeat_sub",
                [this] (
                        TopicList::Request::SharedPtr req,
                        TopicList::Response::SharedPtr resp
                ) {
                        this->heartbeat_subscriptions.clear();
                        this->heartbeat_subscription_ids.clear();

                        this->setup_default_heartbeat_topics();
                        this->subscribe_service_callback<NodeHeartbeat>(
                                req, resp,
                                this->heartbeat_subscriptions,
                                this->heartbeat_subscription_ids,
                                std::bind(&Datalink::heartbeat_callback, this, _1, _2)
                        );
                }
        ));

        services.push_back(this->create_service<TopicList>(
                "heartbeat_pub",
                [this] (
                        TopicList::Request::SharedPtr req,
                        TopicList::Response::SharedPtr resp
                ) {
                        this->heartbeat_publishers.clear();
                        this->heartbeat_publisher_ids.clear();

                        this->setup_default_heartbeat_topics();
                        this->publish_service_callback<NodeHeartbeat>(
                                req, resp,
                                this->heartbeat_publishers,
                                this->heartbeat_publisher_ids
                        );
                }
        ));

        services.push_back(this->create_service<TopicList>(
                "control_sub",
                [this] (
                        TopicList::Request::SharedPtr req,
                        TopicList::Response::SharedPtr resp
                ) {
                        this->signal_subscriptions.clear();
                        this->control_subscription_ids.clear();

                        this->setup_default_control_topics();
                        this->subscribe_service_callback<Control>(
                                req, resp,
                                this->signal_subscriptions,
                                this->control_subscription_ids,
                                std::bind(&Datalink::signal_callback, this, _1, _2)
                        );
                }
        ));

        services.push_back(this->create_service<TopicList>(
                "control_pub",
                [this] (
                        TopicList::Request::SharedPtr req,
                        TopicList::Response::SharedPtr resp
                ) {
                        this->signal_publishers.clear();
                        this->control_publisher_ids.clear();

                        this->setup_default_control_topics();
                        this->publish_service_callback<Control>(
                                req, resp,
                                this->signal_publishers,
                                this->control_publisher_ids
                        );
                }
        ));

        services.push_back(this->create_service<StarCommandTopics>(
                "starcommand_topics",
                [this] (
                                StarCommandTopics::Request::SharedPtr req,
                                StarCommandTopics::Response::SharedPtr resp
                ) {
                        RCLCPP_INFO(this->get_logger(), "Setting up StarCommand pub/sub");
                        setup_starcommand(
                                req->downlink,
                                req->uplink
                        );

                        resp->complete = true;
                }
        ));
}

void Datalink::uplink_callback(StarCommandUplink::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Got data from StarCommand");
        Json::Value root;
        Json::Reader reader;

        RCLCPP_INFO(this->get_logger(), "Deserializing");
        if (!reader.parse(msg->payload, root)) {
                RCLCPP_ERROR(this->get_logger(), "Invalid JSON");
                return;
        }

        if (msg->type == "node") {
                NodeHeartbeat hb;

                hb.state = root["state"].asInt();
                hb.errors   = root["errors"].asUInt();
                hb.requests = root["requests"].asUInt();
                hb.failures = root["failures"].asUInt();

                for (Json::ArrayIndex i = 0; i < root["data"].size(); i++) {
                        hb.data[i] = root["data"][i].asUInt();
                }

                this->heartbeat_callback(
                        heartbeat_subscription_ids.at(msg->destination),
                        std::shared_ptr<NodeHeartbeat>(&hb)
                );
        } else if (msg->type == "control") {
                Control ctrl;

                ctrl.options = root["options"].asString();

                this->signal_callback(
                        control_subscription_ids.at(msg->destination),
                        std::shared_ptr<Control>(&ctrl)
                );
        }
}

template<typename T>
void Datalink::subscribe_service_callback(
        TopicList::Request::SharedPtr req,
        TopicList::Response::SharedPtr resp,
        std::vector<typename rclcpp::Subscription<T>::SharedPtr> &dest,
        std::map<std::string, uint8_t> &mapping,
        std::function<void(int, std::shared_ptr<T>)> callback
) {
        fill_subscriber_list<T>(
                req->topics,
                dest,
                mapping,
                callback
        );

        resp->final_index = dest.size() - 1;
}

template<typename T>
void Datalink::publish_service_callback(
        TopicList::Request::SharedPtr req,
        TopicList::Response::SharedPtr resp,
        std::vector<typename rclcpp::Publisher<T>::SharedPtr> &dest,
        std::map<std::string, uint8_t> &mapping
) {
        fill_publisher_list<T>(
                req->topics,
                dest,
                mapping
        );

        resp->final_index = dest.size() - 1;
}

void Datalink::array_received_callback(const mavlink_message_t& msg) {
        RCLCPP_DEBUG(this->get_logger(), "Got a LOGGING_DATA message");

        mavlink_logging_data_t * inner = new mavlink_logging_data_t();
        mavlink_msg_logging_data_decode(&msg, inner);

        RCLCPP_DEBUG(
                this->get_logger(),
                "The message is from %d/%d and addressed to %d/%d",
                msg.sysid,
                msg.compid,
                inner->target_system,
                inner->target_component
        );

        if (
                        msg.sysid != targetsysid ||
                        msg.compid != targetcompid ||
                        inner->target_system != sysid ||
                        inner->target_component != compid
        ) {
                return;
        }

        TelemMessage message = TelemMessage(inner->data);

        while (message.has_next()) {
                TelemHeader head = message.next_header();
                if (head.msg_type == floattelem::MSG_ID_HEARTBEAT) {
                        RCLCPP_DEBUG(this->get_logger(), "Heartbeat message!");
                        NodeHeartbeat ros_message = message.pop_heartbeat_message();

                        if (head.topic_id >= heartbeat_publishers.size()) {
                                RCLCPP_ERROR(
                                        this->get_logger(),
                                        "Heartbeat publisher with ID=%d out of range",
                                        head.topic_id
                                );
                                continue;
                        }

                        auto pub = this->heartbeat_publishers[head.topic_id];
                        RCLCPP_DEBUG(this->get_logger(), "Publishing on topic %s", pub->get_topic_name());
                        pub->publish(ros_message);

                        if (!starcommand_publisher) continue;

                        StarCommandDownlink down;

                        Json::Value v(Json::objectValue);

                        v["state"]    = Json::Value(ros_message.state);
                        v["requests"] = Json::Value(ros_message.requests);
                        v["failures"] = Json::Value(ros_message.failures);
                        v["errors"]   = Json::Value(ros_message.errors);
                        copy_to_json_array(ros_message.data, v["data"]);

                        std::ostringstream json_out;
                        json_out << v;

                        down.type = "node";
                        down.payload = json_out.str();
                        down.origin = heartbeat_publishers[head.topic_id]->get_topic_name();

                        starcommand_publisher->publish(down);
                } else if (head.msg_type == floattelem::MSG_ID_CONTROL) {
                        std::string options = message.pop_control_message();

                        if (head.topic_id >= signal_publishers.size()) {
                                RCLCPP_ERROR(
                                        this->get_logger(),
                                        "Signal publisher with ID=%d out of range",
                                        head.topic_id
                                );
                                continue;
                        }

                        Control c;
                        c.options = options;

                        this->signal_publishers[head.topic_id]->publish(c);

                        if (!starcommand_publisher) continue;

                        StarCommandDownlink down;

                        Json::Value v(Json::objectValue);

                        v["options"] = Json::Value(options);

                        std::ostringstream json_out;
                        json_out << Json::StreamWriterBuilder().newStreamWriter();

                        down.type = "node";
                        down.payload = json_out.str();
                        down.origin = signal_publishers[head.topic_id]->get_topic_name();

                        starcommand_publisher->publish(down);
                } else if (head.msg_type == floattelem::MSG_ID_SYSTEM_CAPACITY) {
                        floattelem::SystemCapacity cap = message.pop_system_capacity_message();
                        RCLCPP_INFO(this->get_logger(), "Caching capacities for system %d.", head.topic_id);

                        for (auto v = cap.disks_size_mb.begin(); v != cap.disks_size_mb.end(); v++) {
                                RCLCPP_INFO(this->get_logger(), "Size: %d", *v);
                        }

                        cached_systems[head.topic_id] = cap;
                } else if (head.msg_type == floattelem::MSG_ID_SYSTEM_STATUS) {
                        RCLCPP_INFO(this->get_logger(), "Got status message from system %d.", head.topic_id);
                        floattelem::SlimSystemStatus in = message.pop_system_status_message();

                        auto result = cached_systems.find(head.topic_id);
                        if (result == cached_systems.end()) {
                                RCLCPP_ERROR(this->get_logger(), "System %d not cached!", head.topic_id);

				TelemMessage tmsg;
                                tmsg.push_system_capacity_request_message(head.topic_id);
				this->send_telemetry(tmsg);

                                RCLCPP_DEBUG(this->get_logger(), "Requested system capacity message.");
                                continue;
                        }

                        floattelem::SystemCapacity &cap = result->second;

                        SystemStatus out;
                        out.cpu_count = in.cpu_usage.size();
                        out.cpu_usage = in.cpu_usage;
                        out.uptime = in.uptime;

                        out.memory = std::array<uint32_t, 2> {
                                (uint32_t) ((float) in.memory / USHRT_MAX * cap.max_memory_mb),
                                cap.max_memory_mb
                        };

                        out.swap = std::array<uint32_t, 2> {
                                (uint32_t) ((float) in.swap / USHRT_MAX * cap.max_swap_mb),
                                cap.max_swap_mb
                        };

                        for (auto v = in.mounts.begin(); v != in.mounts.end(); v++) {
                                out.mounts.push_back(*v >= mountpoint_names.size() ? "UNKNOWN" : mountpoint_names[*v]);
                        }

                        for (auto v = std::make_pair(in.disks.begin(), cap.disks_size_mb.begin()); v.first != in.disks.end(); v.first++, v.second++) {
                                out.disks.push_back(
                                        (uint32_t) ((float) *v.first / USHRT_MAX * *v.second)
                                );
                                out.disks.push_back(*v.second);
                        }

                        system_status_publishers[head.topic_id]->publish(out);

                        if (!this->starcommand_publisher) continue;

                        StarCommandDownlink down;

                        Json::Value v(Json::objectValue);

                        copy_to_json_array(out.cpu_usage, v["cpu_usage"]);
                        copy_to_json_array(out.disks, v["disks"]);
                        copy_to_json_array(out.mounts, v["mounts"]);
                        copy_to_json_array(out.memory, v["memory"]);
                        copy_to_json_array(out.swap, v["swap"]);
                        v["uptime"] = out.uptime;
                        v["cpu_count"] = out.cpu_count;

                        std::ostringstream ss;
                        ss << v;
                        down.type = "system";
                        down.payload = ss.str();
                        down.origin = system_status_publishers[head.topic_id]->get_topic_name();

                        starcommand_publisher->publish(down);
                } else if (head.msg_type == floattelem::MSG_ID_SYSTEM_CAPACITY_REQUEST) {
                        uint8_t id = message.pop_system_capacity_request_message();

                        RCLCPP_INFO(
                                this->get_logger(),
                                "Got system status request for system %d",
                                id
                        );

                        auto result = this->cached_systems.find(id);
                        if (result == this->cached_systems.end()) {
                                RCLCPP_ERROR(
                                        this->get_logger(),
                                        "System status request for id %d was for unknown system",
                                        id
                                );
                        } else {
                                TelemMessage tmsg;
                                tmsg.push_system_capacity_message(&result->second, id);
                                this->send_telemetry(tmsg);
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

void Datalink::gps_raw_received_callback(const mavlink_message_t& msg) const {
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

void Datalink::global_position_received_callback(const mavlink_message_t& msg) const {
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

void Datalink::attitude_received_callback(const mavlink_message_t& msg) const {
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

void Datalink::systime_received_callback(const mavlink_message_t& msg) const {
        mavlink_system_time_t *systime = new mavlink_system_time_t();
        mavlink_msg_system_time_decode(&msg, systime);

        SystemTime ros_message;

        ros_message.time_boot_ms = systime->time_boot_ms;
        ros_message.time_unix_us = systime->time_unix_usec;

        systime_publisher->publish(ros_message);
}

template<typename T>
void Datalink::fill_subscriber_list(
                const std::vector<std::string> &topics,
                std::vector<typename rclcpp::Subscription<T>::SharedPtr> &dest,
                std::map<std::string, uint8_t> &mapping,
                std::function<void(int, std::shared_ptr<T>)> callback
) {
        int id = dest.size();

        for (auto topic = topics.begin(); topic != topics.end(); topic++) {
                RCLCPP_DEBUG(this->get_logger(), "subscribing to %s", topic->c_str());

                dest.push_back(
                        this->create_subscription<T>(
                                *topic,
                                10,
                                [this, id, callback] (std::shared_ptr<T> msg) {
                                        callback(id, msg);
                                }
                        )
                );
                
                mapping.insert(std::make_pair(*topic, id));

                id++;
        }
}

template<typename T>
void Datalink::fill_publisher_list(
                const std::vector<std::string> &topics,
                std::vector<typename rclcpp::Publisher<T>::SharedPtr> &dest,
                std::map<std::string, uint8_t> &mapping
) {
        int id = dest.size();

        for (auto topic = topics.begin(); topic != topics.end(); topic++) {
                RCLCPP_DEBUG(this->get_logger(), "subscribing to %s", topic->c_str());

                dest.push_back(
                        this->create_publisher<T>(
                                *topic,
                                10
                        )
                );
                
                mapping.insert(std::make_pair(*topic, id));
                
                id++;
        }
}

template<typename T>
void Datalink::copy_to_json_array(std::vector<T> &arr, Json::Value &val) {
        val = Json::arrayValue;
        for (Json::ArrayIndex i = 0; i < arr.size(); i++) {
                val.append(arr[i]);
        }
}

template<typename T, size_t N>
void Datalink::copy_to_json_array(std::array<T, N> &arr, Json::Value &val) {
        val = Json::arrayValue;
        for (Json::ArrayIndex i = 0; i < arr.size(); i++) {
                val.append(arr[i]);
        }
}
