#include "fast_lio_sam.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("fast_lio_sam_node");
    auto fast_lio_sam = std::make_shared<FastLioSam>(node);

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();

    fast_lio_sam.reset();
    rclcpp::shutdown();
    return 0;
}

