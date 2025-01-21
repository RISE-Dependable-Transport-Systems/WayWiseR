#include <QCoreApplication>

#include "waywise_car.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  QCoreApplication a(argc, argv);

  a.processEvents();

  auto waywise_car_node = std::make_shared<WayWiseCar>();
  waywise_car_node->initialize_node();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(waywise_car_node);

  while (rclcpp::ok()) {
    exec.spin_some();
    a.processEvents();
  }

  exec.remove_node(waywise_car_node);
  rclcpp::shutdown();

  return 0;
}
