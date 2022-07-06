#ifndef MOUNTPOINT_READER_NODE_HPP
#define MOUNTPOINT_READER_NODE_HPP
#include "mavlinked_node.hpp"

class MountpointReaderNode : virtual public MAVLinkedNode {
public:
        MountpointReaderNode();

protected:
        // enum specifying where filesystems can be mounted
        std::map<std::string, uint8_t> mountpoints;
        std::vector<std::string> mountpoint_names;

        // Load the mountpoint enum
        void load_mountpoints();
};

#endif
