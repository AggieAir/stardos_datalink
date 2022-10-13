#ifndef FLOATTELEM_HPP
#define FLOATTELEM_HPP

#include <memory>
#include <stdexcept>
#include <stdint.h>

#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <vector>

#include "stardos_interfaces/msg/node_heartbeat.hpp"
#include "stardos_interfaces/msg/system_status.hpp"

using stardos_interfaces::msg::NodeHeartbeat;
using stardos_interfaces::msg::SystemStatus;

namespace floattelem {
        typedef struct {
                uint8_t msg_type;
                uint8_t msg_length;
                uint8_t topic_id;
        } Header;

        constexpr size_t BUFFER_SIZE = 249;
        constexpr uint8_t MSG_BASE_LENGTH = 3;

        constexpr uint8_t MSG_ID_HEARTBEAT = 0x1;
        constexpr uint8_t MSG_LENGTH_HEARTBEAT = 15;

        constexpr uint8_t MSG_ID_CONTROL = 0x2;
        constexpr uint8_t MAX_STRING_LENGTH = 13;

        constexpr uint8_t MSG_ID_SYSTEM_STATUS = 0x3;
        constexpr uint8_t MSG_SYSTEM_STATUS_STATIC_LENGTH = 12;

        constexpr uint8_t MSG_ID_SYSTEM_CAPACITY = 0x4;
        constexpr uint8_t MSG_SYSTEM_CAPACITY_STATIC_LENGTH = 12;

        constexpr uint8_t MSG_ID_SYSTEM_CAPACITY_REQUEST = 0x5;
        constexpr uint8_t MSG_SYSTEM_CAPACITY_REQUEST_LENGTH = 3;

        typedef struct {
                std::vector<uint8_t> cpu_usage;
                uint16_t memory;
                uint16_t swap;
                std::vector<uint16_t> disks;
                std::vector<uint8_t> mounts;
                uint32_t uptime;
        } SlimSystemStatus;

        typedef struct __SystemCapacity {
                uint32_t max_memory_mb;
                uint32_t max_swap_mb;
                std::vector<uint32_t> disks_size_mb;

                bool operator==(struct __SystemCapacity &other) {
                        return this->max_memory_mb == other.max_memory_mb &&
                                this->max_swap_mb == other.max_swap_mb &&
                                this->disks_size_mb == other.disks_size_mb;
                }

                bool operator!=(struct __SystemCapacity &other) {
                        return !(*this == other);
                }
        } SystemCapacity;

        class Message {
        public:
                // Construct a message using an existing octet array
                // This usually means you're on the receiving end and are looking to decode
                Message(uint8_t data[]);
                // Construct a message with all zeroes
                // The zeroes are important. If you don't have them, you'll end up inflating
                // your message size with irrelevant data
                Message();

                // Returns a header object representing the next FloatTelem message
                Header next_header() const;

                // Whether or not there is another FloatTelem message
                bool has_next() const;
                // True if there's nothing to read in the message
                bool is_empty() const;
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

                // system_status
                bool push_system_status_message(SlimSystemStatus *in, uint8_t topic_id);
                SlimSystemStatus pop_system_status_message();

                // system_capacity
                bool push_system_capacity_message(SystemCapacity *in, uint8_t topic_id);
                SystemCapacity pop_system_capacity_message();

                // request_system_capacity
                bool push_system_capacity_request_message(uint8_t topic_id);
                uint8_t pop_system_capacity_request_message();

                // Get the floats themselves
                const uint8_t * get_data() const;
                // Get the floats themselves
                uint8_t get_offset() const;
        private:
                // True if `bytes` more bytes will fit in the buffer
                bool check_space(uint8_t bytes) const;
                // Add a new header at offset (this will take THREE bytes)
                void push_header(uint8_t msg_type, uint8_t msg_length, uint8_t topic_id);

                // Go forward and null-terminate
                void finalize(int length);
                // Given a message length, move the offset to the edge of the next word after it ends
                void forward(int length);

                // Get the data as a sequence of 232 unsigned 8-bit integers
                inline uint8_t * data_u8_mut();
                // Get the data as a sequence of 116 unsigned 16-bit integers
                inline uint16_t * data_u16_mut();
                // Get the data as a sequence of 116 unsigned 16-bit integers
                inline uint32_t * data_u32_mut();
                // Get the data as a sequence of 232 chars
                inline char * data_char_mut();

                // Get the data as a sequence of 232 unsigned 8-bit integers
                inline const uint8_t * data_u8() const;
                // Get the data as a sequence of 116 unsigned 16-bit integers
                inline const uint16_t * data_u16() const;
                // Get the data as a sequence of 116 unsigned 16-bit integers
                inline const uint32_t * data_u32() const;
                // Get the data as a sequence of 232 chars
                inline const char * data_char() const;

                // The number of floats reqired to hold a number of bytes equal to `bytes`
                inline size_t offset_from_length(size_t bytes) const;
                // Get a runtime error for if the next message has an unexpected ID
                static const std::runtime_error wrong_id_error(uint8_t recv, uint8_t want);
                // Get a runtime error for if the next message has an unexpected length
                static const std::runtime_error wrong_length_error(uint8_t recv, uint8_t want);

                // the buffer itself
                uint8_t *data;
                // offset of where to put things, in number of floats
                uint8_t offset;
        };
}

#endif // FLOATTELEM_HPP
