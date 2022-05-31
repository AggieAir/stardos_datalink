#ifndef FLOATTELEM_HPP
#define FLOATTELEM_HPP

#include <memory>
#include <stdexcept>
#include <stdint.h>

#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink/v2.0/common/mavlink_msg_debug_float_array.h>

#include "stardos_interfaces/msg/node_heartbeat.hpp"

using stardos_interfaces::msg::NodeHeartbeat;

namespace floattelem {
        constexpr uint8_t MSG_ID_HEARTBEAT = 0x1;
        constexpr uint8_t MSG_LENGTH_HEARTBEAT = 14;

        constexpr uint8_t MSG_ID_CONTROL = 0x2;
        constexpr uint8_t MSG_LENGTH_CONTROL = 3;

        typedef struct {
                uint8_t msg_type;
                uint8_t msg_length;
                uint8_t topic_id;
        } Header;

        class Message {
        public:
                Message(float data[]);

                Header get_header();

                // heartbeat
                static Message pack_heartbeat_message(NodeHeartbeat::SharedPtr in, uint8_t topic_id);
                NodeHeartbeat unpack_heartbeat_message();

                // control
                static Message pack_control_message(uint8_t topic_id);
                uint8_t unpack_control_message();

                float *get_data();
        private:
                Message(Header head);
                Message(uint8_t msg_type, uint8_t msg_length, uint8_t topic_id);

                void populate_header(uint8_t msg_type, uint8_t msg_length, uint8_t topic_id);
                inline uint8_t * data_u8();
                inline uint16_t * data_u16();
                static std::runtime_error wrong_id_error(uint8_t recv, uint8_t want);
                static std::runtime_error wrong_length_error(uint8_t recv, uint8_t want);

                float *data;
        };
}

#endif // FLOATTELEM_HPP
