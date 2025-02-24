#include <QCoreApplication>

#include "waywise_car_autopilot.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  QCoreApplication a(argc, argv);

  a.processEvents();

  auto waywise_car_autopilot_node = std::make_shared<WaywiseCarAutopilot>();
  waywise_car_autopilot_node->initialize_node();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(waywise_car_autopilot_node);

  while (rclcpp::ok()) {
    exec.spin_some();
    a.processEvents();
  }

  exec.remove_node(waywise_car_autopilot_node);
  rclcpp::shutdown();

  return 0;
}
