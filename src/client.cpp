#include <memory>
#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"

#include "datalink.hpp"

int main(int argc, char *argv[]) {
        rclcpp::init(argc, argv);
        Datalink datalink = Datalink("datalink_client", 1, 220, true, "udp://127.0.0.1:14570");
        rclcpp::spin(std::shared_ptr<Datalink>(&datalink));
        rclcpp::shutdown();
        return 0;
}
