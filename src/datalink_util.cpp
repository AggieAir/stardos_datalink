#include <string>
#include "datalink_util.hpp"

std::string get_node_name(DatalinkScope scope) {
        switch (scope) {
        case SERVER:
                return "server";
        case FLOATTELEM_BRIDGE:
                return "floattelem_bridge";
        case AUTOPILOT_BRIDGE:
                return "autopilot_bridge";
        case STARCOMMAND_SERIALIZER:
                return "starcommand_serializer";
        case CAMERA:
                return "camera";
        case CAMERA2:
                return "camera2";
        case CAMERA3:
                return "camera3";
        case CAMERA4:
                return "camera4";
        case CAMERA5:
                return "camera5";
        case CAMERA6:
                return "camera6";
        default:
                return "unknown";
        }
}

