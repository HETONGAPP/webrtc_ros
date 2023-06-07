#include <rclcpp/rclcpp.hpp>
#include <webrtc_ros/webrtc_ros_server.h>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("webrtc_ros_server");

  webrtc_ros::WebrtcRosServer server(node);
  server.run();

  // 检查话题是否有订阅者
  size_t num_subscribers = node->count_subscribers("point_cloud");

  // 输出订阅者数量
  std::cout << "Number of subscribers: " << num_subscribers << std::endl;

  rclcpp::executors::MultiThreadedExecutor spinner(rclcpp::ExecutorOptions(),
                                                   10);
  spinner.add_node(node);
  spinner.spin();

  server.stop();

  rclcpp::shutdown();
}
