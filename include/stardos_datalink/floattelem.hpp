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
                // Construct a message using an existing float array
                // This usually means you're on the receiving end and are looking to decode
                Message(float data[]);
                // Construct a message with all zeroes
                // The zeroes are important. If you don't have them, you'll end up inflating
                // your message size with irrelevant data
                Message();

                // Returns a header object representing the next FloatTelem message
                Header next_header();

                // Whether or not there is another FloatTelem message
                bool has_next();
                // True if there's nothing to read in the message
                bool is_empty();
                // Clears all data and resets the offset
                void reset();

                // Some functions to add and remove different kinds of messages
                // These always come in pairs
                // push_* add the message at offset, then move offset forward
                // pop_* read the message at offset, return info, then move offset forward
                // In either case, they take you to the position where the next message should be

                // heartbeat
                bool push_heartbeat_message(NodeHeartbeat::SharedPtr in, uint8_t topic_id);
                NodeHeartbeat pop_heartbeat_message();

                // control
                bool push_control_message(std::string options, uint8_t topic_id);
                std::string pop_control_message();

                // Get the floats themselves
                float *get_data();
        private:
                // True if `bytes` more bytes will fit in the buffer
                bool check_space(uint8_t bytes);
                // Add a new header at offset (this will take THREE bytes)
                void push_header(uint8_t msg_type, uint8_t msg_length, uint8_t topic_id);

                // Go forward and null-terminate
                void finalize(int length);
                // Given a message length, move the offset to the edge of the next word after it ends
                void forward(int length);

                // Get the data as a sequence of 232 unsigned 8-bit integers
                inline uint8_t * data_u8();
                // Get the data as a sequence of 116 unsigned 16-bit integers
                inline uint16_t * data_u16();
                // Get the data as a sequence of 232 chars
                inline char * data_char();

                // The number of floats reqired to hold a number of bytes equal to `bytes`
                inline int number_of_floats(int bytes);
                // Get a runtime error for if the next message has an unexpected ID
                static std::runtime_error wrong_id_error(uint8_t recv, uint8_t want);
                // Get a runtime error for if the next message has an unexpected length
                static std::runtime_error wrong_length_error(uint8_t recv, uint8_t want);

                // the buffer itself
                float *data;
                // offset of where to put things, in number of floats
                uint8_t offset;
        };
}

#endif // FLOATTELEM_HPP
