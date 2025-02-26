#include <QCoreApplication>

#include "waywise_truck.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  QCoreApplication a(argc, argv);

  a.processEvents();

  auto waywise_truck_node = std::make_shared<WayWiseTruck>();
  waywise_truck_node->initialize_node();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(waywise_truck_node);

  while (rclcpp::ok()) {
    exec.spin_some();
    a.processEvents();
  }

  exec.remove_node(waywise_truck_node);
  rclcpp::shutdown();

  return 0;
}
