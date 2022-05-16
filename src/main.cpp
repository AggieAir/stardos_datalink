#include <memory>
#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"

#include "datalink.hpp"

int main(int argc, char *argv[]) {
        rclcpp::init(argc, argv);
        Datalink datalink = Datalink("datalink", 1, 1, true);
        rclcpp::spin(std::shared_ptr<Datalink>(&datalink));
        rclcpp::shutdown();
        return 0;
}
