#ifndef DATALINK_UTIL_HPP
#define DATALINK_UTIL_HPP

#include <string>

#define SYS_ID_PAYLOAD     1
#define COMP_ID_PAYLOAD  243
#define COMP_ID_COPILOT  220

#define SYS_ID_GCS       190
#define COMP_ID_GCS      190

typedef enum __DatalinkScope {
        SERVER = 230,
        FLOATTELEM_BRIDGE = 191,
        AUTOPILOT_BRIDGE = 25,
        STARCOMMAND_SERIALIZER = 26,
        CAMERA  = 100,
        CAMERA2 = 101,
        CAMERA3 = 102,
        CAMERA4 = 103,
        CAMERA5 = 104,
        CAMERA6 = 105,

} DatalinkScope;

static std::string get_node_name(DatalinkScope scope) {
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

typedef struct __DatalinkSystem {
        uint8_t id;
        std::string name;
        std::string topic;
} DatalinkSystem;

#endif // DATALINK_UTIL_HPP
