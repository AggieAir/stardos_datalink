#ifndef FLOATTELEM_SENDER_NODE_HPP
#define FLOATTELEM_SENDER_NODE_HPP

#include "../floattelem.hpp"
#include "mavlinked_node.hpp"

class FloatTelemSenderNode : virtual public MAVLinkedNode {
protected:
        // A unique ID for the array to be sent by FloatTelem.
        uint16_t array_id;

        // Load the mountpoint enum
        mavsdk::MavlinkPassthrough::Result send_telemetry(floattelem::Message);
};

#endif
