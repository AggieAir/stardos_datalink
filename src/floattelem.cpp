#include <sstream>
#include <stdexcept>
#include <stdint.h>
#include "floattelem.hpp"
#include "stardos_interfaces/msg/node_heartbeat.hpp"

namespace floattelem {
        Message::Message(float data[]) : data{data} {}

        Header Message::get_header() {
                uint8_t *data8 = data_u8();
                return Header {
                        .msg_type = data8[0],
                        .msg_length = data8[1],
                        .topic_id = data8[2],
                };
        }

        Message Message::pack_heartbeat_message(NodeHeartbeat::SharedPtr msg, uint8_t topic_id) {
                /*  bits  bytes  hword
                 *  0:23   0- 2   0- 1  header
                 * 24:31      3         unused
                 * 32:47   4- 5      2  state
                 * 48:63   6- 7      3  errors
                 * 64:79   8-11      4  requests
                 * 80:95  10-13      5  failures
                 */

                Message ret = Message(MSG_ID_HEARTBEAT, MSG_LENGTH_HEARTBEAT, 0);

                uint16_t *data16 = ret.data_u16();

                data16[2] = msg->state;
                data16[3] = msg->errors;
                data16[4] = msg->requests;
                data16[5] = msg->failures;

                return ret;
        }

        NodeHeartbeat Message::unpack_heartbeat_message() {
                Header head = get_header();
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

                return ret;
        }

        float *Message::get_data() {
                return data;
        }

        Message::Message(Header head) : Message(head.msg_type, head.msg_length, head.topic_id) {}

        Message::Message(uint8_t msg_type, uint8_t msg_length, uint8_t topic_id) {
                // msg_length is in bytes. get number of single-precision floats to hold it.
                
                // first, divide by size of float
                int number_of_floats = msg_length / sizeof(float);
                // that truncates though. we may have up to three bytes hanging off.
                // add them back if so.
                if (msg_length % 4 != 0) {
                        number_of_floats++;
                }

                data = new float[number_of_floats];

                populate_header(msg_type, msg_length, topic_id);
        }

        void Message::populate_header(uint8_t msg_type, uint8_t msg_length, uint8_t topic_id) {
                uint8_t *data8 = data_u8();

                data8[0] = msg_type;
                data8[1] = msg_length;
                data8[2] = topic_id;
        }

        uint8_t * Message::data_u8() {
                return (uint8_t*) data;
        }

        uint16_t * Message::data_u16() {
                return (uint16_t*) data;
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
