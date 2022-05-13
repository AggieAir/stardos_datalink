#include <mavsdk/mavsdk.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class AbstractDatalinkNode : public rclcpp::Node {
public:
	AbstractDatalinkNode(std::string name) : Node(name) {
		publisher = this->create_publisher<std_msgs::msg::String>(name + "/output", 10);

                init_mavlink();
	}

        void init_mavlink() {
                configure();
                connect();
        }

        virtual void configure() = 0;
        virtual mavsdk::ConnectionResult connect() = 0;

protected:
        mavsdk::Mavsdk mav;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
};
