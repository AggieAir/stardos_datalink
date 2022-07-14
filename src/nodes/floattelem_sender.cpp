#include <fstream>

#include "floattelem.hpp"
#include "nodes/floattelem_sender.hpp"

mavsdk::MavlinkPassthrough::Result FloatTelemSenderNode::send_telemetry(floattelem::Message msg) {
        if (!target_passthrough) {
                return mavsdk::MavlinkPassthrough::Result::ConnectionError;
        }

        if (msg.is_empty()) {
                return mavsdk::MavlinkPassthrough::Result::Unknown;
        }

        mavlink_message_t message;

        if (array_id == UINT16_MAX) array_id = 0;

        mavlink_msg_logging_data_pack(
                target_passthrough->get_our_sysid(), // SystemID
                target_passthrough->get_our_compid(), //My comp ID
                &message, //Message reference
                targetsysid,
                targetcompid,
                array_id++,
                msg.get_offset(),
                0,
                msg.get_data()
        );

        return send_mavlink(message);
}
