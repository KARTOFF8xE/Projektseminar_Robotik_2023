#include "node.hpp"

#include <memory>
#include <fstream>

// void sigint_callback(int) {
//     rclcpp::shutdown(); //shut down node on keyboardinterrupt
// }

int main(int argc, char* argv[]) {
    bool do_visualize = (std::string(argv[1]) == "-v");

    std::ofstream f;
    f.open("lidar_width_data.csv");
    f << "timestamp (in seconds), left_limit, right_limit, width\n";
    f.close();
    rclcpp::init(argc, argv);
    std::shared_ptr<Node> node_ptr = std::make_shared<Node>(do_visualize);

    // signal(SIGINT, sigint_callback);

    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    
    return 0;
}
