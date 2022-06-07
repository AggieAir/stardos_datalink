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
        constexpr uint8_t MSG_BASE_LENGTH = 3;

        constexpr uint8_t MSG_ID_HEARTBEAT = 0x1;
        constexpr uint8_t MSG_LENGTH_HEARTBEAT = 12;

        constexpr uint8_t MSG_ID_CONTROL = 0x2;
        constexpr uint8_t MAX_STRING_LENGTH = 13;

        typedef struct {
                uint8_t msg_type;
                uint8_t msg_length;
                uint8_t topic_id;
        } Header;

        class Message {
        public:
                Message(float data[]);
                Message();

                Header next_header();

                bool has_next();
                bool is_empty();
                void reset();

                // heartbeat
                bool push_heartbeat_message(NodeHeartbeat::SharedPtr in, uint8_t topic_id);
                NodeHeartbeat pop_heartbeat_message();

                // control
                bool push_control_message(std::string options, uint8_t topic_id);
                std::string pop_control_message();

                float *get_data();
        private:
                bool check_space(uint8_t bytes);
                void push_header(uint8_t msg_type, uint8_t msg_length, uint8_t topic_id);

                void finalize(int length);
                void forward(int length);

                inline uint8_t * data_u8();
                inline uint16_t * data_u16();
                inline char * data_char();

                inline int number_of_floats(int bytes);
                static std::runtime_error wrong_id_error(uint8_t recv, uint8_t want);
                static std::runtime_error wrong_length_error(uint8_t recv, uint8_t want);

                float *data;
                // offset of where to put things, in number of floats
                uint8_t offset;
        };
}

#endif // FLOATTELEM_HPP
