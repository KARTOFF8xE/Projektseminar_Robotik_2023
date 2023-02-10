#include "node.hpp"

#include <memory>
#include <csignal>

void sigint_callback(int) {
    rclcpp::shutdown(); //shut down node on keyboardinterrupt
}

int main(int argc, char* argv[]) {
    bool do_visualize = (argc > 1 && std::string(argv[1]) == "-v");

    rclcpp::init(argc, argv);
    std::shared_ptr<Node> node_ptr = std::make_shared<Node>(do_visualize);

    signal(SIGINT, sigint_callback);

    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    
    return 0;
}
