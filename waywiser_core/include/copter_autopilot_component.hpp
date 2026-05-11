#ifndef COPTER_AUTOPILOT_COMPONENT_HPP_
#define COPTER_AUTOPILOT_COMPONENT_HPP_

#include <string>
#include <QObject>
#include <QString>

#include "geometry_msgs/msg/twist.hpp"

#include "WayWise/autopilot/purepursuitwaypointfollower.h"
#include "WayWise/communication/mavsdkvehicleserver.h"
#include "WayWise/vehicles/copterstate.h"
#include "WayWise/autopilot/followpoint.h"

#include "qobject_node.hpp"
#include "waywiser_core_utils.hpp"

class CopterAutopilotComponent : public QObject
{
  Q_OBJECT

public:
  CopterAutopilotComponent(QObjectNode * parentQObjectNode, const QSharedPointer<CopterState> copterState)
  : QObject(parentQObjectNode), mCopterState(copterState), mParentQObjectNode(parentQObjectNode) {}
  virtual ~CopterAutopilotComponent() {}

  virtual void setupAutopilot(QSharedPointer<EmergencyStopState> emergencyStopState);
  void reset();

  // Setters
  void setEnableMavlinkInterface(bool value) {mEnableMavlinkInterface = value;}
  void setWaywiseControlTowerAddress(std::string value) {mWaywiseControlTowerAddress = value;}
  void setWaywiseControlTowerPort(int value) {mWaywiseControlTowerPort = value;}
  void setAutoLiftOffEnabled(bool value);
  void setAutoLiftOffActive(bool value);
  void setAutoLiftOffHeight(double value) {mAutoLiftOffHeight = value;}
  void setAutoLiftOffSpeed(double value) {mAutoLiftOffSpeed = value;}
  void setAutoLiftOffTolerance(double value) {mAutoLiftOffTolerance = value;}

  // Getters
  bool getEnableMavlinkInterface() const {return mEnableMavlinkInterface;}
  std::string getWaywiseControlTowerAddress() const {return mWaywiseControlTowerAddress;}
  int getWaywiseControlTowerPort() const {return mWaywiseControlTowerPort;}
  bool getAutoLiftOffEnabled() const {return mAutoLiftOffEnabled;}
  bool getAutoLiftOffActive() const {return mAutoLiftOffActive;}
  double getAutoLiftOffHeight() const {return mAutoLiftOffHeight;}
  double getAutoLiftOffSpeed() const {return mAutoLiftOffSpeed;}
  double getAutoLiftOffTolerance() const {return mAutoLiftOffTolerance;}

  // Utility methods
  virtual void provideParametersToParameterServer();
  void cancelAutoLiftOff();
  bool updateAutoLiftOffCommand(geometry_msgs::msg::Twist & output, bool armed, bool inFlight);

protected:
  bool mEnableMavlinkInterface = true;
  std::string mWaywiseControlTowerAddress = "127.0.0.1";
  int mWaywiseControlTowerPort = 14540;
  bool mAutoLiftOffEnabled = false;
  bool mAutoLiftOffActive = false;
  double mAutoLiftOffHeight = 2.0;
  double mAutoLiftOffSpeed = 0.5;
  double mAutoLiftOffTolerance = 0.05;

  QSharedPointer<CopterState> mCopterState;
  QSharedPointer<EmergencyStopState> mEmergencyStopState;
  QSharedPointer<MovementController> mAutopilotMovementController;
  QSharedPointer<MavsdkVehicleServer> mMavsdkVehicleServer;

  QObjectNode * mParentQObjectNode;
};

#endif  // COPTER_AUTOPILOT_COMPONENT_HPP_
