#include "rclcpp/timer.hpp"
#include "stardos_interfaces/msg/control.hpp"
#include "stardos_interfaces/msg/global_position.hpp"

#include "mavlinked_node.hpp"

using stardos_interfaces::msg::Control;
using stardos_interfaces::msg::GlobalPosition;

class MAVLinkCameraCapture : virtual public MAVLinkedNode {
public:
        MAVLinkCameraCapture(
                const std::string& name,
                const Json::Value& config,
                int32_t id
        );

protected:
        uint32_t image_count;
        int32_t image_index;
        int32_t id;
        
        float interval;
        bool forever;
        uint32_t total_images;
        uint32_t sequence_number;

        GlobalPosition buffered_position;

        rclcpp::Publisher<Control>::SharedPtr take_image_publisher;

        rclcpp::Subscription<GlobalPosition>::SharedPtr position_subscription;
        
        rclcpp::TimerBase::SharedPtr image_capture_timer;

        void target_passthrough_found_callback() override;
        void position_received_callback(GlobalPosition::SharedPtr pos);

        void send_ack(uint8_t to_sysid, uint8_t to_compid, uint16_t command, uint8_t result);

        void handle_command_long(const mavlink_message_t& msg);

        void handle_request_camera_information(const mavlink_message_t& msg, const mavlink_command_long_t& cmd);
        void handle_request_camera_settings(const mavlink_message_t& msg, const mavlink_command_long_t& cmd);
        void handle_request_storage_information(const mavlink_message_t& msg, const mavlink_command_long_t& cmd);
        void handle_request_capture_status(const mavlink_message_t& msg, const mavlink_command_long_t& cmd);
        void handle_request_start_capture(const mavlink_message_t& msg, const mavlink_command_long_t& cmd);
        void handle_request_stop_capture(const mavlink_message_t& msg, const mavlink_command_long_t& cmd);

        void take_image();
        void capture();
};
