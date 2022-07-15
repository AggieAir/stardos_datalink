#include "nodes/mavlink_camera_capture.hpp"

using namespace std::placeholders;

MAVLinkCameraCapture::MAVLinkCameraCapture(
        const std::string& name,
        const Json::Value& config,
        int32_t id
): BasicDatalinkNode(name, config),
        MAVLinkedNode(mavsdk::ForwardingOption::ForwardingOn),
        id{id}
{
        take_image_publisher = this->create_publisher<Control>("gcs_capture", 10);
        position_subscription = this->create_subscription<GlobalPosition>(
                "global_position",
                10,
                std::bind(&MAVLinkCameraCapture::position_received_callback, this, _1)
        );
}

void MAVLinkCameraCapture::target_passthrough_found_callback() {
	target_passthrough->subscribe_message_async(
                MAVLINK_MSG_ID_COMMAND_LONG,
                std::bind(&MAVLinkCameraCapture::handle_command_long, this, _1)
        );
}

void MAVLinkCameraCapture::position_received_callback(GlobalPosition::SharedPtr pos) {
        buffered_position = *pos;
}

void MAVLinkCameraCapture::send_ack(
                uint8_t to_sysid,
                uint8_t to_compid,
                uint16_t command,
                uint8_t result = MAV_RESULT_ACCEPTED
) {
	mavlink_command_ack_t ack;
	ack.command = command;
	ack.result = result;
	ack.progress = 0;
	ack.target_system = to_sysid;
	ack.target_component = to_compid;

	mavlink_message_t ackmsg;

	mavlink_msg_command_ack_encode(
                sysid,
                compid,
		&ackmsg,
		&ack
	);

	target_passthrough->send_message(ackmsg);
}

void MAVLinkCameraCapture::handle_command_long(const mavlink_message_t& msg) {
        mavlink_command_long_t cmd;
        mavlink_msg_command_long_decode(&msg, &cmd);

        RCLCPP_INFO(this->get_logger(), "Recieved command #%d", cmd.command);

        switch (cmd.command) {
        case MAV_CMD_REQUEST_MESSAGE:
                if (cmd.param1 == MAVLINK_MSG_ID_CAMERA_INFORMATION) {
                        this->handle_request_camera_information(msg, cmd);
                } else if (cmd.param1 == MAVLINK_MSG_ID_CAMERA_SETTINGS) {
                        this->handle_request_camera_information(msg, cmd);
                } else if (cmd.param1 == MAVLINK_MSG_ID_STORAGE_INFORMATION) {
                        this->handle_request_camera_information(msg, cmd);
                } else if (cmd.param1 == MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS) {
                        this->handle_request_capture_status(msg, cmd);
                } else {
                        send_ack(msg.sysid, msg.compid, cmd.command, MAV_RESULT_UNSUPPORTED);
                }
                break;
        case MAV_CMD_REQUEST_CAMERA_INFORMATION:
                this->handle_request_camera_information(msg, cmd);
                break;
        case MAV_CMD_REQUEST_CAMERA_SETTINGS:
                this->handle_request_camera_settings(msg, cmd);
                break;
        case MAV_CMD_REQUEST_STORAGE_INFORMATION:
                this->handle_request_storage_information(msg, cmd);
                break;
        case MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                this->handle_request_capture_status(msg, cmd);
                break;
        case MAV_CMD_IMAGE_START_CAPTURE:
                this->handle_request_start_capture(msg, cmd);
                break;
        case MAV_CMD_IMAGE_STOP_CAPTURE:
                this->handle_request_stop_capture(msg, cmd);
                break;
        default:
                send_ack(msg.sysid, msg.compid, cmd.command, MAV_RESULT_UNSUPPORTED);
                break;
        }

}

void MAVLinkCameraCapture::handle_request_camera_information(
                const mavlink_message_t& msg,
                const mavlink_command_long_t& cmd
) {
        RCLCPP_INFO(this->get_logger(), "Got request for Camera Information");

        send_ack(msg.sysid, msg.compid, cmd.command);

	mavlink_camera_information_t info = {};

	info.time_boot_ms = time_boot_ms();
	info.firmware_version = 0x00000001;
	info.focal_length = 0;
	info.sensor_size_h = 1000;
	info.sensor_size_v = 1000;
	info.flags = CAMERA_CAP_FLAGS_CAPTURE_IMAGE;
	info.resolution_h = 300;
	info.resolution_v = 300;
	info.cam_definition_version = 1;

	strncpy((char*) info.vendor_name, "AggieAir", 32);
	strncpy((char*) info.model_name, "MAVLink capture group", 32);

	mavlink_message_t infomsg;

	mavlink_msg_camera_information_encode(
                sysid,
                compid,
		&infomsg,
		&info);

	target_passthrough->send_message(infomsg);

        RCLCPP_INFO(this->get_logger(), "Camera information message sent");
}

void MAVLinkCameraCapture::handle_request_camera_settings(
                const mavlink_message_t& msg,
                const mavlink_command_long_t& cmd
) {
        RCLCPP_INFO(this->get_logger(), "Got request for Camera Settings");

        send_ack(msg.sysid, msg.compid, cmd.command);

        mavlink_camera_settings_t settings = {};

        settings.time_boot_ms = time_boot_ms();
        settings.mode_id = 0;
        settings.zoomLevel = NAN;
        settings.focusLevel = NAN;

	mavlink_message_t settingsmsg;

	mavlink_msg_camera_settings_encode(
                sysid,
                compid,
		&settingsmsg,
		&settings);

	target_passthrough->send_message(settingsmsg);

        RCLCPP_INFO(this->get_logger(), "Camera information message sent");
}

void MAVLinkCameraCapture::handle_request_storage_information(
                const mavlink_message_t& msg,
                const mavlink_command_long_t& cmd
) {
        RCLCPP_INFO(this->get_logger(), "Got request for Storage Information");

        send_ack(msg.sysid, msg.compid, cmd.command);

        mavlink_storage_information_t storage = {};

        storage.time_boot_ms = time_boot_ms();
        storage.total_capacity = 1024;
        storage.used_capacity = 900;
        storage.available_capacity = 124;
        storage.read_speed = 10;
        storage.write_speed = 10;
        storage.storage_id = 0;
        storage.storage_count = 1;
        storage.status = STORAGE_STATUS_READY;
        storage.type = STORAGE_TYPE_OTHER;
        storage.storage_usage = 0;

        mavlink_message_t storagemsg;

        mavlink_msg_storage_information_encode(
                sysid,
                compid,
                &storagemsg,
                &storage
        );

	target_passthrough->send_message(storagemsg);

        RCLCPP_INFO(this->get_logger(), "Storage information message sent");
}

void MAVLinkCameraCapture::handle_request_capture_status(
                const mavlink_message_t& msg,
                const mavlink_command_long_t& cmd
) {
        RCLCPP_INFO(this->get_logger(), "Got request for camera capture status");

        send_ack(msg.sysid, msg.compid, cmd.command);

        mavlink_camera_capture_status_t status = {};

        status.time_boot_ms = time_boot_ms();
        status.image_interval = 1;
        status.recording_time_ms = 0;
        status.available_capacity = 124;
        status.image_status = 0;
        status.video_status = 0;
        status.image_count = image_count;

        mavlink_message_t statusmsg;

        mavlink_msg_camera_capture_status_encode(
                sysid,
                compid,
                &statusmsg,
                &status
        );

	target_passthrough->send_message(statusmsg);

        RCLCPP_INFO(this->get_logger(), "Camera capture status message sent");
}

void MAVLinkCameraCapture::handle_request_start_capture(
                const mavlink_message_t& msg,
                const mavlink_command_long_t& cmd
) {
        RCLCPP_INFO(this->get_logger(), "Got request to start capture");

        send_ack(msg.sysid, msg.compid, cmd.command);

        float new_interval = cmd.param2;
        uint32_t new_total_images = cmd.param3;
        uint32_t new_sequence_number = cmd.param4;

        if (total_images == 1) {
                bool should_take = false;
                if (new_sequence_number == sequence_number + 1) {
                        sequence_number++;
                        should_take = true;
                } else if (new_sequence_number == sequence_number) {
                        RCLCPP_WARN(
                                this->get_logger(),
                                "Received same sequence number (%d) twice; ignoring second",
                                sequence_number
                        );
                } else if (new_sequence_number > sequence_number) {
                        RCLCPP_WARN(
                                this->get_logger(),
                                "Skipped sequence numbers (%d to %d); adjusting to match",
                                sequence_number,
                                new_sequence_number
                        );
                        sequence_number = new_sequence_number;
                        should_take = true;
                } else {
                        RCLCPP_WARN(
                                this->get_logger(),
                                "Sequence numbers reverted (%d to %d); not taking image",
                                sequence_number,
                                new_sequence_number
                        );
                }

                if (should_take) {
                        take_image();
                }
        } else {
                interval = new_interval;
                total_images = new_total_images;

                image_capture_timer = this->create_wall_timer(
                        std::chrono::microseconds((long) (interval * 1000000)),
                        std::bind(&MAVLinkCameraCapture::capture, this)
                );
        }
}

void MAVLinkCameraCapture::handle_request_stop_capture(
                const mavlink_message_t& msg,
                const mavlink_command_long_t& cmd
) {
        RCLCPP_INFO(this->get_logger(), "Got request to start capture");

        send_ack(msg.sysid, msg.compid, cmd.command);

        interval = 0;
        total_images = 0;

        image_capture_timer = nullptr;
}

void MAVLinkCameraCapture::take_image() {
        Control rosmsg;
        rosmsg.options = id;
        take_image_publisher->publish(rosmsg);

        mavlink_camera_image_captured_t captured = {};
        captured.time_utc = 0;
        captured.time_boot_ms = time_boot_ms();
        captured.lat = buffered_position.lat;
        captured.lon = buffered_position.lon;
        captured.alt = buffered_position.alt;
        captured.q[0] = 1;
        captured.image_index = image_index;
        captured.camera_id = 0;
        captured.capture_result = 1;

        mavlink_message_t capturedmsg;

        mavlink_msg_camera_image_captured_encode(
                sysid,
                compid,
                &capturedmsg,
                &captured
        );

        image_index++;
}

void MAVLinkCameraCapture::capture() {
        take_image();
        image_count++;
        if (!forever && image_count == total_images) {
                image_capture_timer = nullptr;
        }
}
