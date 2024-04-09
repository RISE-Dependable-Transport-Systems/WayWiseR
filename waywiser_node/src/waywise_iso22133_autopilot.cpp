#include "waywise_autopilot.cpp"
#include "WayWise/communication/iso22133vehicleserver.h"
#include "WayWise/sensors/gnss/ubloxrover.h"

class WayWiseIso22133AutoPilot : public WayWiseAutoPilot
{

public:
    WayWiseIso22133AutoPilot()
    : WayWiseAutoPilot()
    {
        auto iso22133Ip = this->declare_parameter(
            "iso-ip",
            "0.0.0.0");

    }

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  QCoreApplication a(argc, argv);

  a.processEvents();

  auto waywiseNode = std::make_shared<WayWiseIso22133AutoPilot>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(waywiseNode);

  while (rclcpp::ok()) {
    exec.spin_some();
    a.processEvents();
  }

  exec.remove_node(waywiseNode);
  rclcpp::shutdown();

  return 0;
}