#ifndef BASIC_DATALINK_NODE_HPP
#define BASIC_DATALINK_NODE_HPP

#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>

#include "rclcpp/rclcpp.hpp"

class BasicDatalinkNode: public rclcpp::Node
{
public:
	BasicDatalinkNode(
                const std::string& name,
                const Json::Value& config
        );

protected:
        /* ************************ *
         * VARIABLE ZONE            *
         * There are a lot of these *
         * ************************ */

        // ROS node name
        std::string name;
        // Configuration JSON
        Json::Value config;

        // Where to look for additional configuration files
        std::string extra_config_directory;

        // Convenience methods -- takes a list of topics and subscribes/creates publishers to all of them
        // Makes it a lot easier to handle "pub" and "sub" lists from the control listener
        template<typename T>
        void fill_subscriber_list(
                        Json::Value& topics,
                        std::vector<typename rclcpp::Subscription<T>::SharedPtr> &dest,
                        std::map<std::string, uint8_t> &mapping,
                        std::function<void(int, std::shared_ptr<T>)> callback
        ) {
                int id = 0;
                for (auto v = topics.begin(); v != topics.end(); v++) {
                        std::string topic = v->asString();

                        RCLCPP_DEBUG(this->get_logger(), "subscribing to %s", topic.c_str());

                        dest.push_back(
                                this->create_subscription<T>(
                                        topic,
                                        10,
                                        [this, id, callback] (std::shared_ptr<T> msg) {
                                                callback(id, msg);
                                        }
                                )
                        );
                        
                        mapping.insert(std::make_pair(topic, id));

                        id++;
                }
        }

        template<typename T>
        void fill_publisher_list(
                        Json::Value& topics,
                        std::vector<typename rclcpp::Publisher<T>::SharedPtr> &dest,
                        std::map<std::string, uint8_t> &mapping
        ) {
                int id = 0;
                for (auto v = topics.begin(); v != topics.end(); v++) {
                        std::string topic = v->asString();

                        RCLCPP_DEBUG(this->get_logger(), "subscribing to %s", topic.c_str());

                        dest.push_back(
                                this->create_publisher<T>(
                                        topic,
                                        10
                                )
                        );
                        
                        mapping.insert(std::make_pair(topic, id));
                        
                        id++;
                }
        }
};

#endif //BASIC_DATALINK_NODE_HPP
