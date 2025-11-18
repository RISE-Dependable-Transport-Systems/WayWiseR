#include <QCoreApplication>

#include "waywiser_truck_node_core.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  QCoreApplication app(argc, argv);

  app.processEvents();

  auto waywiser_truck_node = std::make_shared<WaywiserTruck>();
  waywiser_truck_node->initialize_node();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(waywiser_truck_node);

  while (rclcpp::ok()) {
    exec.spin_some();
    app.processEvents();
  }

  exec.remove_node(waywiser_truck_node);
  rclcpp::shutdown();

  return 0;
}
