#ifndef COPTER_AUTOPILOT_COMPONENT_HPP_
#define COPTER_AUTOPILOT_COMPONENT_HPP_

#include <string>
#include <QObject>
#include <QString>

#include "geometry_msgs/msg/twist.hpp"

#include "WayWise/autopilot/copterwaypointfollower.h"
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
  void setAutopilotTimerRate(int value) {mAutopilotTimerRate = value;}
  void setEnableMavlinkInterface(bool value) {mEnableMavlinkInterface = value;}
  void setWaywiseControlTowerAddress(std::string value) {mWaywiseControlTowerAddress = value;}
  void setWaywiseControlTowerPort(int value) {mWaywiseControlTowerPort = value;}
  void setMissionPosTypeUsed(PosType value) {mMissionPosTypeUsed = value;}
  void setRequireGnssForMission(bool value) {mRequireGnssForMission = value;}
  void setVehicleInitialized(bool value) {mVehicleInitialized = value;}
  void setGnssFixStatus(GnssFixStatus gnssFixStatus) {mGnssFixStatus = gnssFixStatus;}
  void setWaypointProximity(double value) {mWaypointProximity = value;}
  void setEndGoalAlignmentThreshold(double value) {mEndGoalAlignmentThreshold = value;}
  void setCruiseSpeed(double value) {mCruiseSpeed = value;}
  void setMaxMissionSpeed(double value) {mMaxMissionSpeed = value;}
  void setMinApproachSpeed(double value) {mMinApproachSpeed = value;}
  void setApproachSlowdownRadius(double value) {mApproachSlowdownRadius = value;}
  void setFaceTravelDirection(bool value) {mFaceTravelDirection = value;}
  void setYawGain(double value) {mYawGain = value;}
  void setMaxYawRate(double value) {mMaxYawRate = value;}
  void setPositionAccuracyThresholdForMission(float value)
  {
    mPositionAccuracyThresholdForMission = value;
  }
  void setYawAccuracyThresholdForMission(float value) {mYawAccuracyThresholdForMission = value;}
  void setAutoLiftOffEnabled(bool value);
  void setAutoLiftOffActive(bool value);
  void setAutoLiftOffHeight(double value) {mAutoLiftOffHeight = value;}
  void setAutoLiftOffSpeed(double value) {mAutoLiftOffSpeed = value;}
  void setAutoLiftOffTolerance(double value) {mAutoLiftOffTolerance = value;}

  // Getters
  int getAutopilotTimerRate() const {return mAutopilotTimerRate;}
  bool getEnableMavlinkInterface() const {return mEnableMavlinkInterface;}
  std::string getWaywiseControlTowerAddress() const {return mWaywiseControlTowerAddress;}
  int getWaywiseControlTowerPort() const {return mWaywiseControlTowerPort;}
  PosType getMissionPosTypeUsed() const {return mMissionPosTypeUsed;}
  bool getRequireGnssForMission() const {return mRequireGnssForMission;}
  QList<PosPoint> getWaypointList() const {return mWaypointList;}
  MissionState getCurrentMissionState() const {return currentMissionState;}
  PosPoint getCurrentGoal() const;
  QSharedPointer<MovementController> getAutopilotMovementController() const
  {
    return mAutopilotMovementController;
  }
  geometry_msgs::msg::Twist getAutopilotTwistCommand() const;
  double getWaypointProximity() const {return mWaypointProximity;}
  double getEndGoalAlignmentThreshold() const {return mEndGoalAlignmentThreshold;}
  double getCruiseSpeed() const {return mCruiseSpeed;}
  double getMaxMissionSpeed() const {return mMaxMissionSpeed;}
  double getMinApproachSpeed() const {return mMinApproachSpeed;}
  double getApproachSlowdownRadius() const {return mApproachSlowdownRadius;}
  bool getFaceTravelDirection() const {return mFaceTravelDirection;}
  double getYawGain() const {return mYawGain;}
  double getMaxYawRate() const {return mMaxYawRate;}
  bool getAutoLiftOffEnabled() const {return mAutoLiftOffEnabled;}
  bool getAutoLiftOffActive() const {return mAutoLiftOffActive;}
  double getAutoLiftOffHeight() const {return mAutoLiftOffHeight;}
  double getAutoLiftOffSpeed() const {return mAutoLiftOffSpeed;}
  double getAutoLiftOffTolerance() const {return mAutoLiftOffTolerance;}

  // Utility methods
  virtual void provideParametersToParameterServer();
  void switchAutopilot(bool enable);
  void updateWaypointFollowerRoute(QList<PosPoint> & waypointList);
  void processMissionStateMachine();
  bool isActive();
  virtual void stopWaypointFollower();
  void cancelAutoLiftOff();
  bool updateAutoLiftOffCommand(geometry_msgs::msg::Twist & output, bool armed, bool inFlight);

signals:
  void updatedMissionState(MissionState state);
  void gnssFixAccuracyAssertionFailed(GnssFixStatus gnssFixStatus);

protected:
  virtual void updateMissionState(MissionState state);
  virtual void startWaypointFollower(QList<PosPoint> & waypointList);
  bool assertGnssFixAccuracy();

  int mAutopilotTimerRate = 10; // [hz]
  bool mEnableMavlinkInterface = true;
  std::string mWaywiseControlTowerAddress = "127.0.0.1";
  int mWaywiseControlTowerPort = 14540;
  PosType mMissionPosTypeUsed = PosType::odom;
  bool mRequireGnssForMission = false;
  double mWaypointProximity = 0.5;
  double mEndGoalAlignmentThreshold = 0.25;
  double mCruiseSpeed = 1.0;
  double mMaxMissionSpeed = 2.0;
  double mMinApproachSpeed = 0.1;
  double mApproachSlowdownRadius = 1.5;
  bool mFaceTravelDirection = true;
  double mYawGain = 1.5;
  double mMaxYawRate = 1.0;
  float mPositionAccuracyThresholdForMission = 0.5; // [m]
  float mYawAccuracyThresholdForMission = 5.0; // [deg]
  bool mAutoLiftOffEnabled = false;
  bool mAutoLiftOffActive = false;
  double mAutoLiftOffHeight = 2.0;
  double mAutoLiftOffSpeed = 0.5;
  double mAutoLiftOffTolerance = 0.05;

  QSharedPointer<CopterState> mCopterState;
  QSharedPointer<EmergencyStopState> mEmergencyStopState;
  QSharedPointer<MovementController> mAutopilotMovementController;
  QSharedPointer<CopterWaypointFollower> mWaypointFollower;
  QSharedPointer<MavsdkVehicleServer> mMavsdkVehicleServer;

  QObjectNode * mParentQObjectNode;
  QList<PosPoint> mWaypointList;
  MissionState currentMissionState = MissionState::WaitingForVehicleInit;
  GnssFixStatus mGnssFixStatus;
  bool mVehicleInitialized = false;
};

#endif  // COPTER_AUTOPILOT_COMPONENT_HPP_
