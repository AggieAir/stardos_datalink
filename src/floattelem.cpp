#include <algorithm>
#include <cstring>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <stdint.h>

#include "floattelem.hpp"
#include "splitter.hpp"

#include "stardos_interfaces/msg/node_heartbeat.hpp"

namespace floattelem {
        Message::Message(uint8_t data[]) : data{data}, offset{0} {}

        Message::Message() : Message(new uint8_t[BUFFER_SIZE] {0}) {}

        Header Message::next_header() {
                uint8_t *data8 = data_u8();
                Header head;
                head.msg_type = data8[0];
                head.msg_length = data8[1];
                head.topic_id = data8[2];

                return head;
        }

        bool Message::has_next() {
                return offset < BUFFER_SIZE && data_u8()[0] != 0;
        }

        bool Message::is_empty() {
                return offset == 0 && data_u8()[0] == 0;
        }

        void Message::reset() {
                for (int i = 0; i < offset; i++) {
                        data[i] = 0;
                }
                offset = 0;
        }

        bool Message::push_heartbeat_message(NodeHeartbeat::SharedPtr msg, uint8_t topic_id) {
                /*  bits  bytes  hword
                 *  0:23   0- 2   0- 1  header
                 * 24:31      3         unused
                 * 32:47   4- 5      2  state
                 * 48:63   6- 7      3  errors
                 * 64:79   8- 9      4  requests
                 * 80:95  10-11      5  failures
                 */

                if (!check_space(MSG_LENGTH_HEARTBEAT)) {
                        return false;
                }

                this->push_header(MSG_ID_HEARTBEAT, MSG_LENGTH_HEARTBEAT, topic_id);

                uint16_t *data16 = data_u16();

                data16[2] = msg->state;
                data16[3] = msg->errors;
                data16[4] = msg->requests;
                data16[5] = msg->failures;

                finalize(MSG_LENGTH_HEARTBEAT);

                return true;
        }

        NodeHeartbeat Message::pop_heartbeat_message() {
                Header head = next_header();

                if (head.msg_type != MSG_ID_HEARTBEAT) {
                        throw  wrong_id_error(head.msg_type, MSG_ID_HEARTBEAT);
                }
                
                if (head.msg_length != MSG_LENGTH_HEARTBEAT) {
                        throw wrong_length_error(head.msg_length, MSG_LENGTH_HEARTBEAT);
                }

                uint16_t *data16 = data_u16();

                NodeHeartbeat ret = NodeHeartbeat();

                ret.state = data16[2];
                ret.errors = data16[3];
                ret.requests = data16[4];
                ret.failures = data16[5];

                forward(MSG_LENGTH_HEARTBEAT);

                std::cout << "Popped message, offset=" << (int) offset << "\n";

                return ret;
        }

        bool Message::push_control_message(std::string options, uint8_t topic_id) {
                /*  bits  bytes
                 *  0:23   0- 2  header
                 * 24: n   3- n  action
                 */

                int length = MSG_BASE_LENGTH + (std::min((size_t) MAX_STRING_LENGTH, options.length()));
                if (!check_space(length)) {
                        return false;
                }

                this->push_header(MSG_ID_CONTROL, length, topic_id);

                char *datachar = this->data_char();
                strncpy(datachar + MSG_BASE_LENGTH, options.c_str(), MAX_STRING_LENGTH);

                finalize(length);

                return true;
        }

        std::string Message::pop_control_message() {
                Header head = next_header();

                if (head.msg_type != MSG_ID_CONTROL) {
                        throw  wrong_id_error(head.msg_type, MSG_ID_CONTROL);
                }
                
                if (head.msg_length > MSG_BASE_LENGTH + MAX_STRING_LENGTH) {
                        throw wrong_length_error(head.msg_length, MSG_BASE_LENGTH + MAX_STRING_LENGTH);
                }

                char *datachar = this->data_char();

                std::string ret = std::string(datachar + MSG_BASE_LENGTH, head.msg_length - MSG_BASE_LENGTH);

                forward(head.msg_length);

                return ret;
        }

        bool Message::push_system_status_message(SlimSystemStatus *msg, uint8_t topic_id) {
                /*  bits  bytes  hword   word
                 *  0:23   0- 2   0         0  header
                 * 24:31      3    - 1         cpu_count
                 * 32:47   4- 5      2      1  memory
                 * 48:63   6- 7      3         swap
                 * 64:95   8-11   4- 5      2  uptime
                 * 96: a  12- a   6- a      3  disks
                 *  a: b   a- b   a- b      a  cpu_usage
                 *  a: b   a- b   a- b      a  mounts
                 */

                // There will always be 12 bytes comprising the statically-sized portion.
                // Add one byte for each cpu_usage flag
                // Add three for each mount point (1 each for mounts and 2 each for disks)
                int length = MSG_SYSTEM_STATUS_STATIC_LENGTH + msg->cpu_usage.size() + 3 * msg->mounts.size();

                if (!check_space(length)) {
                        return false;
                }

                this->push_header(MSG_ID_SYSTEM_STATUS, length, topic_id);

                uint8_t *data8 = data_u8();
                uint16_t *data16 = data_u16();
                uint32_t *data32 = data_u32();

                data8[3] = msg->mounts.size();

                data16[2] = msg->memory;
                data16[3] = msg->swap;

                data32[2] = msg->uptime;

                int localoffset = MSG_SYSTEM_STATUS_STATIC_LENGTH; // IN BYTES!!!

                for (auto v = msg->disks.begin(); v != msg->disks.end(); v++) {
                        data16[localoffset / 2] = *v;
                        localoffset += 2;
                }

                for (auto v = msg->cpu_usage.begin(); v != msg->cpu_usage.end(); v++) {
                        data8[localoffset] = *v;
                        localoffset++;
                }

                for (auto v = msg->mounts.begin(); v != msg->mounts.end(); v++) {
                        data8[localoffset] = *v;
                        localoffset++;
                }

                finalize(localoffset);

                return true;
        }

        SlimSystemStatus Message::pop_system_status_message() {
                Header head = next_header();

                if (head.msg_type != MSG_ID_SYSTEM_STATUS) {
                        throw  wrong_id_error(head.msg_type, MSG_ID_SYSTEM_STATUS);
                }
                
                if (head.msg_length < MSG_SYSTEM_STATUS_STATIC_LENGTH) {
                        throw wrong_length_error(head.msg_length, MSG_SYSTEM_STATUS_STATIC_LENGTH);
                }

                uint8_t *data8 = data_u8();
                uint16_t *data16 = data_u16();
                uint32_t *data32 = data_u32();

                SlimSystemStatus ret;

                ret.memory = data16[2];
                ret.swap   = data16[3];
                ret.uptime = data32[2];

                uint8_t cpu_count = head.topic_id;
                uint8_t disk_count = (head.msg_length - MSG_SYSTEM_STATUS_STATIC_LENGTH - cpu_count) / 3;

                int localoffset = MSG_SYSTEM_STATUS_STATIC_LENGTH;

                for (int i = 0; i < disk_count; i++) {
                        ret.disks.push_back(data16[localoffset / 2]);
                        localoffset += 2;
                }

                for (int i = 0; i < cpu_count; i++) {
                        ret.cpu_usage.push_back(data8[localoffset]);
                        localoffset++;
                }

                for (int i = 0; i < disk_count; i++) {
                        ret.mounts.push_back(data8[localoffset]);
                        localoffset++;
                }

                forward(localoffset);

                return ret;
        }

        bool Message::push_system_capacity_message(SystemCapacity *msg, uint8_t topic_id) {
                /*  bits  bytes  hword   word
                 *  0:23   0- 2   0         0  header
                 * 24:31      3    - 1         num_disks
                 * 32:63   4- 7   2- 3      1  max_memory_mb
                 * 64:95   8-11   4- 5      2  max_swap_mb
                 * 96: a  12- a   6- a      3  disks_size_mb
                 */

                // There will always be 12 bytes comprising the statically-sized portion.
                // Add one byte for each cpu_usage flag
                // Add three for each mount point (1 each for mounts and 2 each for disks)
                int length = MSG_SYSTEM_CAPACITY_STATIC_LENGTH + 4 * msg->disks_size_mb.size();

                if (!check_space(length)) {
                        return false;
                }

                this->push_header(MSG_ID_HEARTBEAT, length, topic_id);

                uint8_t *data8 = data_u8();
                uint32_t *data32 = data_u32();

                data8[3] = msg->disks_size_mb.size();

                data32[1] = msg->max_memory_mb;
                data32[2] = msg->max_swap_mb;

                int localoffset = MSG_SYSTEM_CAPACITY_STATIC_LENGTH; // IN BYTES!!!

                for (auto v = msg->disks_size_mb.begin(); v != msg->disks_size_mb.end(); v++) {
                        data32[localoffset / 4] = *v;
                        localoffset += 4;
                }

                finalize(localoffset);

                return true;
        }

        SystemCapacity Message::pop_system_capacity_message() {
                Header head = next_header();

                if (head.msg_type != MSG_ID_SYSTEM_CAPACITY) {
                        throw  wrong_id_error(head.msg_type, MSG_ID_SYSTEM_CAPACITY);
                }
                
                if (head.msg_length < MSG_SYSTEM_CAPACITY_STATIC_LENGTH) {
                        throw wrong_length_error(head.msg_length, MSG_SYSTEM_CAPACITY_STATIC_LENGTH);
                }

                uint8_t *data8 = data_u8();
                uint32_t *data32 = data_u32();

                SystemCapacity ret;

                ret.max_memory_mb = data32[1];
                ret.max_swap_mb   = data32[2];

                uint8_t disk_count = data8[3];

                int localoffset = MSG_SYSTEM_CAPACITY_STATIC_LENGTH;

                for (int i = 0; i < disk_count; i++) {
                        ret.disks_size_mb.push_back(data32[localoffset / 2]);
                        localoffset += 2;
                }

                forward(localoffset);

                return ret;
        }

        uint8_t *Message::get_data() {
                return data;
        }

        uint8_t Message::get_offset() {
                return offset;
        }

        bool Message::check_space(uint8_t bytes) {
                return offset + offset_from_length(bytes) < BUFFER_SIZE;
        }

        void Message::push_header(uint8_t msg_type, uint8_t msg_length, uint8_t topic_id) {
                uint8_t *data8 = data_u8();

                data8[0] = msg_type;
                data8[1] = msg_length;
                data8[2] = topic_id;
        }

        void Message::finalize(int length) {
                forward(length);
        }

        void Message::forward(int length) {
                int f = offset_from_length(length);
                this->offset += f;
        }

        uint8_t * Message::data_u8() {
                return (uint8_t*) data + offset;
        }

        uint16_t * Message::data_u16() {
                return (uint16_t*) data + offset / 2;
        }

        uint32_t * Message::data_u32() {
                return (uint32_t*) data + offset / 4;
        }

        char * Message::data_char() {
                return (char*) data + offset;
        }

        size_t Message::offset_from_length(size_t bytes) {
                return 4 * ((bytes-1) / sizeof(float) + 1);
        }

        std::runtime_error Message::wrong_id_error(uint8_t recv, uint8_t want) {
                std::stringstream buffer = std::stringstream();
                buffer << "Invalid Message ID " << (int) recv << "; expected " << (int) want;
                return std::runtime_error(buffer.str());
        }

        std::runtime_error Message::wrong_length_error(uint8_t recv, uint8_t want) {
                std::stringstream buffer = std::stringstream();
                buffer << "Reported message length "
                        << (int) recv
                        << " too " 
                        << (recv > want ? "long" : "short")
                        << "; expected "
                        << (int) want;
                return std::runtime_error(buffer.str());
        }
}
