#include <jsoncpp/json/writer.h>
#include <sys/types.h>
#include <sys/stat.h>
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

#include <rhash.h>

#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "stardos_interfaces/msg/node_heartbeat.hpp"
#include "stardos_interfaces/msg/control.hpp"
#include "stardos_interfaces/msg/global_position.hpp"
#include "stardos_interfaces/msg/gps_position.hpp"
#include "stardos_interfaces/msg/attitude.hpp"
#include "stardos_interfaces/msg/rc_channels.hpp"
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
        dc{mavsdk::Mavsdk{mavsdk::Mavsdk::Configuration{ComponentType::Custom}}},
        buffered_message{TelemMessage()}
{
	mavsdk::log::subscribe([this](mavsdk::log::Level level,
		const std::string& message,
		const std::string& file,
		int line
	) {
		switch (level) {
		case mavsdk::log::Level::Debug:
			RCLCPP_DEBUG(this->get_logger(), "(%s:%d) %s", file.c_str(), line, message.c_str());
			break;
		case mavsdk::log::Level::Info:
			RCLCPP_INFO(this->get_logger(), "(%s:%d) %s", file.c_str(), line, message.c_str());
			break;
		case mavsdk::log::Level::Warn:
			RCLCPP_WARN(this->get_logger(), "(%s:%d) %s", file.c_str(), line, message.c_str());
			break;
		case mavsdk::log::Level::Err:
			RCLCPP_ERROR(this->get_logger(), "(%s:%d) %s", file.c_str(), line, message.c_str());
			break;
		}
		return true;
	});

	Json::StreamWriterBuilder b;
	b.settings_["indentation"] = "";
	writer = std::unique_ptr<Json::StreamWriter>(b.newStreamWriter());

	rhash_library_init();

	if (config["central_config_location"].isString()) {
		central_config_index.emplace(config["central_config_location"].asString());
	}

	{
		std::ifstream cc_fs("/opt/stardos/settings.json");
		cc_fs >> central_config;

		if (central_config_index) {
			central_config_system = central_config[*central_config_index];
		} else if (central_config["ground"].isObject()) {
			central_config_system = central_config["ground"];
		} else if (central_config["copilot"].isObject()) {
			central_config_system = central_config["copilot"];
		} else {
			RCLCPP_ERROR(this->get_logger(), "Cannot find central config location");
			throw std::exception();
		}
	}

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
	setup_starcommand();
	setup_temperatures();
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

	this->server_component = dc.server_component_by_id(this->compid);
	RCLCPP_INFO(this->get_logger(), "Setting up server and component");
	if (this->server_component == nullptr) {
		RCLCPP_ERROR(this->get_logger(), "Could not create server component!");
	}

	this->ftp_server = std::make_shared<mavsdk::FtpServer>(this->server_component);
	this->ftp_server->set_root_dir("/opt/stardos/tmp/ftp");
}

void Datalink::connect() {
        // Json::Value urlval = config["connection_url"];
	Json::Value urlval = central_config_system["mavlink_url"];
        if (!urlval.isString()) {
                RCLCPP_ERROR(this->get_logger(), "URL must be a string");
                throw std::exception();
        }

	RCLCPP_INFO(
		this->get_logger(),
		"Connecting to MAVLink with URL: '%s'",
		urlval.asCString()
	);

        dc.add_any_connection(urlval.asString());

	RCLCPP_DEBUG(this->get_logger(), "setting up buffered message sending timer");
	send_telemetry_timer = this->create_wall_timer(
		1000ms,
		std::bind(&Datalink::send_buffered_message, this)
	);
}

void Datalink::setup_default_heartbeat_topics() {
        std::string copilot_path("/"), payload_path("/");
        copilot_path = copilot_path + aircraft + '/' + "copilot" + '/' + "heartbeat";
        payload_path = payload_path + aircraft + '/' + payload + '/' + "heartbeat";
        std::vector<std::string> topics{copilot_path, payload_path};

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
		
		set_config_publisher = this->create_publisher<Control>("/set_config", 10);

		status_text_subscription = this->create_subscription<Control>(
			"/" + aircraft + "/status_text",
			10,
			std::bind(&Datalink::status_text_callback, this, _1)
		);
        } else {
                fill_subscriber_list<Control>(
                        topics,
                        signal_subscriptions,
                        control_subscription_ids,
                        std::bind(&Datalink::signal_callback, this, _1, _2)
                );

		set_config_subscription = this->create_subscription<Control>(
			"/set_config",
			10,
			std::bind(&Datalink::set_config_callback, this, _1)
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
                rc_channels_publisher = this->create_publisher<RCChannels>("rc_channels", 10);
        }
}

void Datalink::setup_starcommand() { //const std::string& downlink_topic, const std::string& uplink_topic) {
	if (config["starcommand"].asBool() || config["starcommand_local"].asBool()) {
		RCLCPP_INFO(this->get_logger(), "Setting up starcommand ROS bridges");
		starcommand_publisher = this->create_publisher<StarCommandDownlink>(
			"/payload_telemetry_downlink", 10);
		starcommand_subscription = this->create_subscription<StarCommandUplink>(
			"/payload_telemetry_uplink",
			10,
			std::bind(&Datalink::uplink_callback, this, _1)
		);
	}
}

void Datalink::setup_temperatures() {
        std::string temperature_path("/");
        temperature_path = temperature_path + aircraft + '/' + "temperature_probes";

        if (config["listen_for_temperature"].asBool()) {
		RCLCPP_INFO(this->get_logger(), "Creating Temperature Subscribers");
                temperature_subscription = this->create_subscription<TemperatureProbes>(
                        temperature_path,
                        10,
                        std::bind(&Datalink::temperature_callback, this, _1)
                );
        } else {
		RCLCPP_INFO(this->get_logger(), "Creating Temperature Publishers");
                temperature_publisher = this->create_publisher<TemperatureProbes>(temperature_path, 10);
        }

	std::ifstream file(extra_config_directory + "/sensors.json");
        Json::Value root;

        file >> root;

	RCLCPP_INFO(this->get_logger(), "Found %d sensors", root.size());

        for (auto value : root) {
                uint8_t id = value["id"].asUInt();
                std::string name = value["name"].asString();
                std::string label = value["label"].asString();

		TemperatureProbeDetails placeholder = { name, label, id };

                auto details = std::make_shared<TemperatureProbeDetails>(placeholder);

                this->probes_by_id[id] = details;
                this->probes_by_name[name] = details;
        }
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
        
        std::string payload_path = std::string("/") + aircraft + '/' + payload + '/' + "status";
        std::string copilot_path = std::string("/") + aircraft + '/' + "copilot" + '/' + "status";

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
        RCLCPP_INFO(this->get_logger(), "Trying to send buffered message");

	if (target_passthrough == nullptr) {
		RCLCPP_ERROR(this->get_logger(), "Tried to send a message before target system was found.");
		buffered_message.reset();
		return;
	}

        if (send_telemetry(buffered_message) == MavlinkPassthrough::Result::Success) {
                RCLCPP_INFO(this->get_logger(), "Resetting message buffer");
                buffered_message.reset();
        }
}

MavlinkPassthrough::Result Datalink::send_telemetry(const TelemMessage msg) {
	RCLCPP_DEBUG(this->get_logger(), "Attempting to send telemetry (size=%hhu)", msg.get_offset());
	if (!target_passthrough || msg.is_empty()) {
		return MavlinkPassthrough::Result::Unknown;
	}

	MavlinkPassthrough::Result result = target_passthrough->queue_message([this, msg] (MavlinkAddress addr, uint8_t chan) {
		mavlink_message_t message;

		if (array_id == UINT16_MAX) array_id = 0;

		mavlink_msg_logging_data_pack_chan(
			addr.system_id,
			addr.component_id,
			chan,
			&message, //Message reference
			targetsysid,
			targetcompid,
			array_id++,
			msg.get_offset() * 4,
			0,
			msg.get_data()
		);

		return message;
	});

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

                        target_passthrough->subscribe_message(
                                        MAVLINK_MSG_ID_LOGGING_DATA,
                                        std::bind(&Datalink::array_received_callback, this, _1));

			ftp = std::make_shared<mavsdk::Ftp>(*target); 
			ftp->set_target_compid(targetcompid);

			if (-1 == mkdir("/opt/stardos/tmp/ftp", 0644) && errno != EEXIST) {
				RCLCPP_ERROR(this->get_logger(), "error creating ftp directory: %s", strerror(errno));
			}
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

                        autopilot_passthrough->subscribe_message(
                                        MAVLINK_MSG_ID_GPS_RAW_INT,
                                        std::bind(&Datalink::gps_raw_received_callback, this, _1));

                        autopilot_passthrough->subscribe_message(
                                        MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                                        std::bind(&Datalink::global_position_received_callback, this, _1));

                        autopilot_passthrough->subscribe_message(
                                        MAVLINK_MSG_ID_ATTITUDE,
                                        std::bind(&Datalink::attitude_received_callback, this, _1));

                        autopilot_passthrough->subscribe_message(
                                        MAVLINK_MSG_ID_SYSTEM_TIME,
                                        std::bind(&Datalink::systime_received_callback, this, _1));

                        autopilot_passthrough->subscribe_message(
                                        MAVLINK_MSG_ID_RC_CHANNELS,
                                        std::bind(&Datalink::rc_channels_received_callback, this, _1));
                }

                if (target != nullptr &&
                                (autopilot != nullptr || !config["autopilot_telemetry"].asBool())) {
                        RCLCPP_INFO(this->get_logger(), "Systems found; ending search.");
                        get_system_timer->cancel();
                }
        }
}

void Datalink::handle_message_request(uint8_t message_id, uint8_t topic_id) {
        if (message_id == floattelem::MSG_ID_SYSTEM_CAPACITY) {
                RCLCPP_INFO(
                        this->get_logger(),
                        "Got system status request for system %d",
                        topic_id
                );

                auto result = this->cached_systems.find(topic_id);
                if (result == this->cached_systems.end()) {
                        RCLCPP_ERROR(
                                this->get_logger(),
                                "System status request for id %d was for unknown system",
                                topic_id
                        );
                } else {
                        TelemMessage tmsg;
                        tmsg.push_system_capacity_message(&result->second, topic_id);
                        this->send_telemetry(tmsg);
                }
        } else {
                RCLCPP_ERROR(
                        this->get_logger(),
                        "Can't respond to request for message %d",
                        message_id
                );
        }
}

void Datalink::heartbeat_callback(int id, NodeHeartbeat::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Queueing heartbeat message (offset=%d)", buffered_message.get_offset());

        if (!buffered_message.push_heartbeat_message(msg, id)) {
                send_buffered_message();
                buffered_message.reset();
                buffered_message.push_heartbeat_message(msg, id);
        }

	if (!config["starcommand_local"].asBool()) return;

	StarCommandDownlink down;

	Json::Value v(Json::objectValue);

	v["state"]        = Json::Value(msg->state);
	v["requests"]     = Json::Value(msg->requests);
	v["failures"]     = Json::Value(msg->failures);
	v["errors"]       = Json::Value(msg->errors);
	v["performance"]  = Json::Value(msg->performance);
	v["queue_length"] = Json::Value(msg->queue_length);
	copy_to_json_array(msg->data, v["data"]);

	std::ostringstream json_out;
	writer->write(v, &json_out);

	down.type = "node";
	down.payload = json_out.str();
	down.origin = heartbeat_subscriptions[id]->get_topic_name();

	starcommand_publisher->publish(down);
}

void Datalink::signal_callback(int id, Control::SharedPtr ctrl) {
        if (ctrl->options.size() > floattelem::MAX_STRING_LENGTH) {
                RCLCPP_ERROR(
			this->get_logger(),
			"Option string '%s' too long; will be truncated",
			ctrl->options.c_str()
		);
        }

        TelemMessage tmsg;
        tmsg.push_control_message(ctrl->options, id);

        send_telemetry(tmsg);

	if (!config["starcommand_local"].asBool()) return;

	StarCommandDownlink down;

	Json::Value v(Json::objectValue);

	v["options"] = Json::Value(ctrl->options);

	std::ostringstream json_out;
	writer->write(v, &json_out);

	down.type = "control";
	down.payload = json_out.str();
	down.origin = signal_subscriptions[id]->get_topic_name();

	starcommand_publisher->publish(down);
}

void Datalink::set_config_callback(Control::SharedPtr ctrl) {
	RCLCPP_INFO(this->get_logger(), "Setting config over mavlink");

	uploading_thread = std::make_unique<std::thread>([this, ctrl] {
		RCLCPP_INFO(this->get_logger(), "Running on new thread");

		std::promise<mavsdk::Ftp::Result> promise;
		auto future = promise.get_future();

		RCLCPP_INFO(this->get_logger(), "Preparing file upload");

		{
			std::ofstream fs("/opt/stardos/tmp/ftp/buffered_message.json");
			fs << ctrl->options;
			fs.close();
		}

		RCLCPP_INFO(this->get_logger(), "Wrote to the buffered_message file");


		ftp->upload_async(
			"/opt/stardos/tmp/ftp/buffered_message.json",
			"/",
			std::bind(&Datalink::uploading_callback, this, _1, _2, &promise)
		);

		future.wait();

		RCLCPP_INFO(this->get_logger(), "Preparing telemetry message");

		floattelem::Message tmsg;

		uint8_t digest[16];

		rhash_msg(RHASH_MD5, ctrl->options.c_str(), ctrl->options.length(), digest);

		char output[40];

		rhash_print_bytes(output, digest, rhash_get_digest_size(RHASH_MD5), RHPR_HEX | RHPR_UPPERCASE);

		RCLCPP_INFO(this->get_logger(), "Length: %lu; Hash: %s", ctrl->options.length(), output);

		RCLCPP_INFO(this->get_logger(), "Sending telemetry message");

		tmsg.push_set_config_message(digest);
		send_telemetry(tmsg);

		RCLCPP_INFO(this->get_logger(), "Sent telemetry message; invalidating thread");
	});

	uploading_thread->detach();
}

void Datalink::status_text_callback(Control::SharedPtr ctrl) {
	if (target_passthrough == nullptr) {
		RCLCPP_ERROR(this->get_logger(), "Tried to send a message before target system was found.");
		return;
	}

	target_passthrough->queue_message([this, ctrl] (MavlinkAddress addr, uint8_t channel) {
		mavlink_statustext_t statustext;
		statustext.severity = MAV_SEVERITY_INFO;
		statustext.chunk_seq = 0;
		statustext.id = statustext_id;

		if (statustext_id == UINT16_MAX) {
			statustext_id = 0;
		} else {
			statustext_id++;
		}

		strncpy(
			statustext.text,
			ctrl->options.c_str(),
			std::min(50ul, strlen(ctrl->options.c_str()))
		);

		mavlink_message_t msg;

		mavlink_msg_statustext_encode_chan(addr.system_id, addr.component_id, channel, &msg, &statustext);

		return msg;
	});
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

	if (!config["starcommand_local"].asBool()) return;

	StarCommandDownlink down;

	Json::Value v(Json::objectValue);

	copy_to_json_array(msg->cpu_usage, v["cpu_usage"]);
	copy_to_json_array(msg->disks, v["disks"]);
	copy_to_json_array(msg->mounts, v["mounts"]);
	copy_to_json_array(msg->memory, v["memory"]);
	copy_to_json_array(msg->swap, v["swap"]);
	v["uptime"] = msg->uptime;
	v["cpu_count"] = msg->cpu_count;

	std::ostringstream ss;
	writer->write(v, &ss);
	down.type = "system";
	down.payload = ss.str();
	down.origin = system_status_subscriptions[id]->get_topic_name();

	starcommand_publisher->publish(down);
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

void Datalink::temperature_callback(TemperatureProbes::SharedPtr msg) {
        floattelem::SlimTemperatures st;

	RCLCPP_INFO(this->get_logger(), "Got readings for %ld probes", msg->readings.size());

        auto name = msg->ids.begin();
        auto reading = msg->readings.begin();
        for (; name != msg->ids.end(); name++, reading++) {
		if (this->probes_by_name.find(*name) != this->probes_by_name.end()) {
			st.ids.push_back(this->probes_by_name[*name]->id);
		} else {
			RCLCPP_WARN(this->get_logger(), "Unknown probe '%s' ", name->c_str());
			st.ids.push_back(255);
		}

                // we send it over in centi-degrees Celsius.
                st.readings.push_back(round(*reading * 100));
        }

        floattelem::Message tmsg;
        tmsg.push_temperatures_message(&st, 0);

	RCLCPP_INFO(this->get_logger(), "Sending readings over FloatTelem");
	this->send_telemetry(tmsg);
}

void Datalink::uploading_callback(mavsdk::Ftp::Result res, mavsdk::Ftp::ProgressData pd, std::promise<mavsdk::Ftp::Result> *promise) {
	std::ostringstream resultText;
	resultText << res;

	RCLCPP_INFO(this->get_logger(), "FTP result: %s, bytes: %d/%d", resultText.str().c_str(), pd.bytes_transferred, pd.total_bytes);

	if (res != mavsdk::Ftp::Result::Next) {
		promise->set_value(res);
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
                        RCLCPP_INFO(
				this->get_logger(),
				"Received heartbeat message (offset=%hhu, id=%hhu, length=%hhu, topic=%hhu)",
				message.get_offset(),
				head.msg_type,
				head.msg_length,
				head.topic_id
			);

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

                        v["state"]        = Json::Value(ros_message.state);
                        v["requests"]     = Json::Value(ros_message.requests);
                        v["failures"]     = Json::Value(ros_message.failures);
                        v["errors"]       = Json::Value(ros_message.errors);
                        v["performance"]  = Json::Value(ros_message.performance);
                        v["queue_length"] = Json::Value(ros_message.queue_length);
                        copy_to_json_array(ros_message.data, v["data"]);

                        std::ostringstream json_out;
			writer->write(v, &json_out);

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
			writer->write(v, &json_out);

                        down.type = "control";
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
                                tmsg.push_message_request_message(floattelem::MSG_ID_SYSTEM_CAPACITY, head.topic_id);
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
			writer->write(v, &ss);
                        down.type = "system";
                        down.payload = ss.str();
                        down.origin = system_status_publishers[head.topic_id]->get_topic_name();

                        starcommand_publisher->publish(down);
                } else if (head.msg_type == floattelem::MSG_ID_MESSAGE_REQUEST) {
                        uint8_t message_id, argument;

                        std::tie(message_id, argument) = message.pop_message_request_message();

                        if (message_id != floattelem::MSG_ID_SYSTEM_CAPACITY) {
                                RCLCPP_INFO(this->get_logger(), "No handler for message request for id %d (argument=%d)", message_id, argument);
                                continue;
                        }

                        RCLCPP_INFO(
                                this->get_logger(),
                                "Got system capacity request for system %d",
                                argument
                        );

                        auto result = this->cached_systems.find(argument);
                        if (result == this->cached_systems.end()) {
                                RCLCPP_ERROR(
                                        this->get_logger(),
                                        "System status request for id %d was for unknown system",
                                        argument
                                );
                        } else {
                                TelemMessage tmsg;
                                tmsg.push_system_capacity_message(&result->second, argument);
                                this->send_telemetry(tmsg);
                        }
                } else if (head.msg_type == floattelem::MSG_ID_SET_CONFIG) {
			uint8_t *received_digest = new uint8_t[16];
			uint8_t *generated_digest = new uint8_t[16];

                        message.pop_set_config_message(received_digest);

                        RCLCPP_INFO(
                                this->get_logger(),
				"Got request to set config"
                        );
			
			std::ifstream fs("/opt/stardos/tmp/ftp/buffered_message.json");
			std::string config(
				(std::istreambuf_iterator<char>(fs)),
				(std::istreambuf_iterator<char>())
			);

			fs.close();

			rhash_msg(RHASH_MD5, config.c_str(), config.length(), generated_digest);
			char output[40];

			rhash_print_bytes(output, generated_digest, rhash_get_digest_size(RHASH_MD5), RHPR_HEX | RHPR_UPPERCASE);

			RCLCPP_INFO(this->get_logger(), "Length: %lu; Hash: %s", config.length(), output);

			bool matches = true;
			for (int i = 0; i < 16; i++) {
				if (received_digest[i] != generated_digest[i]) {
					matches = false;
					break;
				}
			}

			delete[] received_digest;
			delete[] generated_digest;

			if (!matches) {
				RCLCPP_ERROR(this->get_logger(), "Buffered set_config message does not match");
				continue;
			} 

			Control ctrl;
			ctrl.options = config;

			set_config_publisher->publish(ctrl);

			if (-1 == remove("/opt/stardos/tmp/ftp/buffered_message.json")) {
				RCLCPP_ERROR(this->get_logger(), "Error deleting buffered message: %s", strerror(errno));
			}
                } else if (head.msg_type == floattelem::MSG_ID_TEMPERATURES) {
                        TemperatureProbes tp;

                        floattelem::SlimTemperatures st = message.pop_temperatures_message();

                        RCLCPP_INFO(this->get_logger(), "Received %lu temperature readings", st.ids.size());

			auto id = st.ids.begin();
			auto reading = st.readings.begin();
                        for (; id != st.ids.end(); id++, reading++) {
				if (this->probes_by_id.find(*id) != this->probes_by_id.end()) {
					tp.ids.push_back(this->probes_by_id[*id]->label);
				} else {
					RCLCPP_WARN(this->get_logger(), "Unknown probe with id=%hhu ", *id);
					tp.ids.push_back("Unknown Sensor");
				}
                                tp.readings.push_back((float) *reading / 100);
                        }

                        this->temperature_publisher->publish(tp);

                        if (!this->starcommand_publisher) continue;

                        StarCommandDownlink down;

                        Json::Value v(Json::objectValue);

                        copy_to_json_array(tp.ids, v["ids"]);
                        copy_to_json_array(tp.readings, v["readings"]);

                        std::ostringstream ss;
			writer->write(v, &ss);
                        down.type = "temperature";
                        down.payload = ss.str();
                        down.origin = "/" + aircraft + "/temperatures";

                        starcommand_publisher->publish(down);
                } else {
                        RCLCPP_ERROR(
                                this->get_logger(),
                                "Unrecognized message ID: %d",
                                head.msg_type);
			break;
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

void Datalink::rc_channels_received_callback(const mavlink_message_t& msg) const {
        mavlink_rc_channels_t rcc;
        mavlink_msg_rc_channels_decode(&msg, &rcc);

        RCChannels ros_message;

        ros_message.time_boot_ms = rcc.time_boot_ms;
        ros_message.chancount = rcc.chancount;
	ros_message.rssi = rcc.rssi;
	
	// i thought i had abandoned this dark art years ago
	// but you leave me no choice...

#define MAYBE_PUSH_CHANNEL_VALUE(CHANNUM) \
	if (ros_message.chancount >= CHANNUM) { \
		ros_message.values.push_back(rcc.chan ## CHANNUM ## _raw); \
	}

	MAYBE_PUSH_CHANNEL_VALUE(1);
	MAYBE_PUSH_CHANNEL_VALUE(2);
	MAYBE_PUSH_CHANNEL_VALUE(3);
	MAYBE_PUSH_CHANNEL_VALUE(4);
	MAYBE_PUSH_CHANNEL_VALUE(5);
	MAYBE_PUSH_CHANNEL_VALUE(6);
	MAYBE_PUSH_CHANNEL_VALUE(7);
	MAYBE_PUSH_CHANNEL_VALUE(8);
	MAYBE_PUSH_CHANNEL_VALUE(9);
	MAYBE_PUSH_CHANNEL_VALUE(10);
	MAYBE_PUSH_CHANNEL_VALUE(11);
	MAYBE_PUSH_CHANNEL_VALUE(12);
	MAYBE_PUSH_CHANNEL_VALUE(13);
	MAYBE_PUSH_CHANNEL_VALUE(14);
	MAYBE_PUSH_CHANNEL_VALUE(15);
	MAYBE_PUSH_CHANNEL_VALUE(16);
	MAYBE_PUSH_CHANNEL_VALUE(17);
	MAYBE_PUSH_CHANNEL_VALUE(18);

#undef MAYBE_PUSH_CHANNEL_VALUE

        rc_channels_publisher->publish(ros_message);
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
                                [id, callback] (std::shared_ptr<T> msg) {
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
