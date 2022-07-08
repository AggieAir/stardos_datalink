#ifndef SYSTEM_READER_NODE_HPP
#define SYSTEM_READER_NODE_HPP
#include <functional>
#include <stdint.h>

#include "../datalink_util.hpp"
#include "mavlinked_node.hpp"

class SystemReaderNode : virtual public MAVLinkedNode {
protected:
        // Load the mountpoint enum
        void load_systems(std::function<void(const DatalinkSystem&)>);
};

#endif
