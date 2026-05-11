#include <QCoreApplication>

#include "waywiser_copter_node_core.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  QCoreApplication app(argc, argv);

  app.processEvents();

  auto waywiser_copter_node = std::make_shared<WaywiserCopter>();
  waywiser_copter_node->initialize_node();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(waywiser_copter_node);

  while (rclcpp::ok()) {
    exec.spin_some();
    app.processEvents();
  }

  exec.remove_node(waywiser_copter_node);
  rclcpp::shutdown();

  return 0;
}