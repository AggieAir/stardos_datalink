#include <memory>
#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"

#include "datalink.hpp"
#include "datalink_constants.hpp"

int main(int argc, char *argv[]) {
        rclcpp::init(argc, argv);
        Datalink datalink = Datalink(
                        "datalink",
                        SYS_ID_GCS,
                        COMP_ID_GCS,
                        true,
                        "udp://127.0.0.1:11002",
                        SYS_ID_PAYLOAD,
                        COMP_ID_COPILOT,
                        false);
        rclcpp::spin(std::shared_ptr<Datalink>(&datalink));
        rclcpp::shutdown();
        return 0;
}
