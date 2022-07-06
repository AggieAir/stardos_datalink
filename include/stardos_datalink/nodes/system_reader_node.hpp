#ifndef SYSTEM_READER_NODE_HPP
#define SYSTEM_READER_NODE_HPP
#include <stdint.h>

#include "mavlinked_node.hpp"

class SystemReaderNode : virtual public MAVLinkedNode {
public:
        SystemReaderNode();

protected:
        // Load the mountpoint enum
        void load_systems();

        // Callback to call when we add a system
        virtual void add_system(const uint8_t id, const std::string& name, const std::string& topic) = 0;
};

#endif
