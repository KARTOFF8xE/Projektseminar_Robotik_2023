#include "node.hpp"

#include <memory>

// void sigint_callback(int) {
//     rclcpp::shutdown(); //shut down node on keyboardinterrupt
// }

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<Node> node_ptr = std::make_shared<Node>();

    // signal(SIGINT, sigint_callback);

    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    
    return 0;
}
