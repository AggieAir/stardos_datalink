#ifndef MAVLINKED_NODE_HPP
#define MAVLINKED_NODE_HPP

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink/v2.0/mavlink_types.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

#include "rclcpp/rclcpp.hpp"

#include "./basic_datalink_node.hpp"

class MAVLinkedNode: virtual public BasicDatalinkNode
{
public:
	MAVLinkedNode();

protected:
        /* ************************ *
         * VARIABLE ZONE            *
         * There are a lot of these *
         * ************************ */

        // Instance of MAVSDK -- this models the connection
  	mavsdk::Mavsdk dc;
        // Reference to MAVLink system to send telemetry to
	std::shared_ptr<mavsdk::System> target;

        // Where are we
        uint8_t sysid;
        uint8_t compid;

        // Where to send telemetry messages to
        uint8_t targetsysid;
        uint8_t targetcompid;

        // These allow us to pass messages directly to the systems
	std::shared_ptr<mavsdk::MavlinkPassthrough> target_passthrough;

        // Runs every second; looks for systems
        rclcpp::TimerBase::SharedPtr get_system_timer;

        /* **************************** *
         * FUNCTION ZONE                *
         * Non-callback functions first *
         * **************************** */

        // Wrapper around Mavsdk::set_configuration
	virtual void configure();
        // Bind to the connection_url
	virtual void connect();
        // Check to see if there is another system; connect if so
        virtual void check_systems();

        // Send a mavlink message
        virtual mavsdk::MavlinkPassthrough::Result send_mavlink(mavlink_message_t& msg);

        /* ********************** *
         * ENTERING CALLBACK LAND *
         * ********************** */

        // This is called immediately after target is found and a MavlinkPassthrough is created for it
        virtual void target_passthrough_found_callback() = 0;
};

#endif //MAVLINKED_NODE_HPP
