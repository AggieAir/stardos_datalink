#ifndef DATALINK_UTIL_HPP
#include <mavsdk/mavsdk.h>
#define DATALINK_UTIL_HPP

#define SYS_ID_PAYLOAD     1
#define COMP_ID_PAYLOAD  243
#define COMP_ID_COPILOT  220

#define SYS_ID_GCS       190
#define COMP_ID_GCS      190

typedef enum __DatalinkScope {
        SERVER = 0,
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

#endif // DATALINK_UTIL_HPP
