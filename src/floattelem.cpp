#include <algorithm>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <stdint.h>
#include "floattelem.hpp"
#include "stardos_interfaces/msg/node_heartbeat.hpp"

namespace floattelem {
        Message::Message(float data[]) : data{data}, offset{0} {}

        Message::Message() : Message(new float[58] {0}) {}

        Header Message::next_header() {
                uint8_t *data8 = data_u8();
                Header head;
                head.msg_type = data8[0];
                head.msg_length = data8[1];
                head.topic_id = data8[2];

                return head;
        }

        bool Message::has_next() {
                return offset < 58 && data_u8()[0] != 0;
        }

        bool Message::is_empty() {
                return offset == 0;
        }

        void Message::reset() {
                offset = 0;
                for (int i = 0; i < 58; i++) {
                        data[i] = 0;
                }
        }

        bool Message::push_heartbeat_message(NodeHeartbeat::SharedPtr msg, uint8_t topic_id) {
                /*  bits  bytes  hword
                 *  0:23   0- 2   0- 1  header
                 * 24:31      3         unused
                 * 32:47   4- 5      2  state
                 * 48:63   6- 7      3  errors
                 * 64:79   8-11      4  requests
                 * 80:95  10-13      5  failures
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

        float *Message::get_data() {
                return data;
        }

        bool Message::check_space(uint8_t bytes) {
                return offset + number_of_floats(bytes) < 58;
        }

        void Message::push_header(uint8_t msg_type, uint8_t msg_length, uint8_t topic_id) {
                uint8_t *data8 = data_u8();

                data8[0] = msg_type;
                data8[1] = msg_length;
                data8[2] = topic_id;
        }

        void Message::finalize(int length) {
                forward(length);

                if (offset < 58) {
                        this->data_u8()[0] = 0;
                }
        }

        void Message::forward(int length) {
                int f = number_of_floats(length);
                this->offset += f;
        }

        uint8_t * Message::data_u8() {
                return (uint8_t*) data + offset * 4;
        }

        uint16_t * Message::data_u16() {
                return (uint16_t*) data + offset * 2;
        }

        char * Message::data_char() {
                return (char*) data + offset * 4;
        }

        int Message::number_of_floats(int bytes) {
                return (bytes-1) / sizeof(float) + 1;
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
