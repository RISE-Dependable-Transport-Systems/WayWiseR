#ifndef CAR_INTERFACE_COMPONENT_HPP_
#define CAR_INTERFACE_COMPONENT_HPP_

#include <memory>
#include <string>
#include <QObject>
#include <QString>
#include <tuple>

#include "WayWise/core/coordinatetransforms.h"
#include "WayWise/logger/logger.h"
#include "WayWise/sensors/imu/bno055orientationupdater.h"
#include "WayWise/sensors/imu/imuorientationupdater.h"
#include "WayWise/sensors/tof/tofsensor.h"
#include "WayWise/sensors/tof/vl53l0xtofsensor.h"
#include "WayWise/vehicles/carstate.h"
#include "WayWise/vehicles/controller/carmovementcontroller.h"
#include "WayWise/vehicles/controller/vescmotorcontroller.h"

#include "qobject_node.hpp"
#include "waywiser_core_utils.hpp"


class CarInterfaceComponent : public QObject
{
  Q_OBJECT

public:
  // Constructor and destructor
  CarInterfaceComponent(
    QObjectNode * parentQObjectNode, const QSharedPointer<CarState> & carState,
    bool autoActuateMotorAndServo = true);
  virtual ~CarInterfaceComponent() {}
  virtual void reset();

  // Setters
  void setLength(float value) {mLength = value; mRearAxleToRearEndOffset.x = -0.25 * value;}
  void setWidth(float value) {mWidth = value;}
  void setWheelbase(float value) {mWheelbase = value;}
  void setMinTurningRadius(float value) {mMinTurningRadius = value;}
  void setRearAxleToBaseOffset(xyz_t value) {mRearAxleToBaseOffset = value;}
  void setRearAxleToCenterOffset(xyz_t value) {mRearAxleToCenterOffset = value;}
  void setRearAxleToRearEndOffset(xyz_t value) {mRearAxleToRearEndOffset = value;}
  void setErpmMin(float value) {mErpmMin = value;}
  void setErpmMax(float value) {mErpmMax = value;}
  void setSpeedToRPMFactor(float value) {mSpeedToRPMFactor = value;}
  void setInvertServoOutput(bool value) {mInvertServoOutput = value;}
  void setServoOffset(float value) {mServoOffset = value;}
  void setServoRange(float value) {mServoRange = value;}
  void setMinBatteryVoltage(float value) {mMinBatteryVoltage = value;}
  void setImuVariant(ImuVariant value) {mImuVariant = value;}
  void setVehicleInterfaceType(VehicleInterfaceType value) {mVehicleInterfaceType = value;}
  void setSpeedControlType(SpeedControlType value) {mSpeedControlType = value;}
  void setVehicleStatePollRate(int value) {mVehicleStatePollRate = value;}
  void setPIDSpeedControllerGains(float kp, float ki, float kd)
  {
    mPIDSpeedControllerKp = kp; mPIDSpeedControllerKi = ki; mPIDSpeedControllerKd = kd;
  }
  void setToFSensorsInfo(std::map<std::string, std::tuple<int, int>> value)
  {
    mToFSensorsInfo = value;
  }

  // Getters
  float getLength() const {return mLength;}
  float getWidth() const {return mWidth;}
  float getWheelbase() const {return mWheelbase;}
  float getMinTurningRadius() const {return mMinTurningRadius;}
  xyz_t getRearAxleToBaseOffset() const {return mRearAxleToBaseOffset;}
  xyz_t getRearAxleToCenterOffset() const {return mRearAxleToCenterOffset;}
  xyz_t getRearAxleToRearEndOffset() const {return mRearAxleToRearEndOffset;}
  float getErpmMin() const {return mErpmMin;}
  float getErpmMax() const {return mErpmMax;}
  float getSpeedToRPMFactor() const {return mSpeedToRPMFactor;}
  bool getInvertServoOutput() const {return mInvertServoOutput;}
  float getServoOffset() const {return mServoOffset;}
  float getServoRange() const {return mServoRange;}
  float getMinBatteryVoltage() const {return mMinBatteryVoltage;}
  ImuVariant getImuVariant() const {return mImuVariant;}
  VehicleInterfaceType getVehicleInterfaceType() const {return mVehicleInterfaceType;}
  SpeedControlType getSpeedControlType() const {return mSpeedControlType;}
  int getVehicleStatePollRate() const {return mVehicleStatePollRate;}
  std::map<std::string, std::tuple<int, int>> getToFSensorsInfo() const
  {
    return mToFSensorsInfo;
  }

  QSharedPointer<EmergencyStopState> getEmergencyStopState() const {return mEmergencyStopState;}
  QSharedPointer<MovementController> getMovementController() const
  {
    return mCarMovementController;
  }
  CarControlCommand getCarControlCommand() const {return mCarControlCommand;}
  QSharedPointer<IMUOrientationUpdater> getIMUOrientationUpdater() const
  {
    return mIMUOrientationUpdater;
  }

  // Callback methods
  void waywise_simulation_timer_callback();

  // Utility methods
  virtual void setup_vehicle_interface();
  void activate_emergency_stop(const std::string & sender_id = "", const std::string & reason = "");
  void clear_emergency_stop(const std::string & sender_id = "");

  void updateControlCommand(double desired_linear_speed, double desired_angular_speed, double dt);
  void executeControlCommand();

signals:
  void battery_voltage_received(double voltage);
  void tof_distance_received(std::string sensor_name, double distance);

public slots:

protected:
  // Parameters
  float mLength = 0.8;   // [m]
  float mWidth = 0.335;   // [m]
  float mWheelbase = 0.48; // [m]
  float mMinTurningRadius = 0.67; // [m]
  float mErpmMin = 2000.0;
  float mErpmMax = 4000.0;
  float mSpeedToRPMFactor = 4123.3;   // default for Traxxas Slash VXL
  bool mInvertServoOutput = false;
  float mServoOffset = 0.5;
  float mServoRange = 1.0;
  float mMinBatteryVoltage = 0.0; // [V]

  ImuVariant mImuVariant = ImuVariant::UNKNOWN;
  VehicleInterfaceType mVehicleInterfaceType = VehicleInterfaceType::WAYWISE_SIMULATED;
  SpeedControlType mSpeedControlType = SpeedControlType::OPEN_LOOP_ERPM_CONTROL;

  bool mAutoActuateMotorAndServo = true;
  int mVehicleStatePollRate = 10; // [Hz]

  float mPIDSpeedControllerKp = 1.0;
  float mPIDSpeedControllerKi = 0.0;
  float mPIDSpeedControllerKd = 0.0;

  xyz_t mRearAxleToBaseOffset;
  xyz_t mRearAxleToCenterOffset;
  xyz_t mRearAxleToRearEndOffset;

  std::map<std::string, std::tuple<int, int>> mToFSensorsInfo;

  // WayWise components
  QSharedPointer<CarState> mCarState;
  QSharedPointer<EmergencyStopState> mEmergencyStopState;
  QSharedPointer<CarMovementController> mCarMovementController;
  QSharedPointer<VESCMotorController> mVESCMotorController;
  QSharedPointer<IMUOrientationUpdater> mIMUOrientationUpdater;

  // Internal variables
  QObjectNode * mParentQObjectNode;
  rclcpp::TimerBase::SharedPtr mWaywiseSimulationTimer = nullptr;
  float mCurrentBatteryVoltage = 0.0;
  QSharedPointer<PIDController> mPIDSpeedController;
  CarControlCommand mCarControlCommand;
  std::map<std::string, QSharedPointer<ToFSensor>> mToFSensors;
};

#endif  // CAR_INTERFACE_COMPONENT_HPP_
