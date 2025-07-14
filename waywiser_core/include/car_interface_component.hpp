#ifndef CAR_INTERFACE_COMPONENT_HPP_
#define CAR_INTERFACE_COMPONENT_HPP_

#include <memory>
#include <string>
#include <QObject>
#include <QString>
#include <tuple>

#include "WayWise/core/coordinatetransforms.h"
#include "WayWise/logger/logger.h"
#include "WayWise/sensors/fusion/sdvpvehiclepositionfuser.h"
#include "WayWise/sensors/gnss/gnssreceiver.h"
#include "WayWise/sensors/gnss/rtcmclient.h"
#include "WayWise/sensors/gnss/ubloxrover.h"
#include "WayWise/sensors/imu/bno055orientationupdater.h"
#include "WayWise/sensors/imu/imuorientationupdater.h"
#include "WayWise/sensors/tof/tofsensor.h"
#include "WayWise/sensors/tof/vl53l0xtofsensor.h"
#include "WayWise/vehicles/carstate.h"
#include "WayWise/vehicles/controller/carmovementcontroller.h"
#include "WayWise/vehicles/controller/vescmotorcontroller.h"

#include "waywiser_core_utils.hpp"


class CarInterfaceComponent : public QObject
{
  Q_OBJECT

public:
  // Constructor and destructor
  CarInterfaceComponent(
    QObject * parent, const QSharedPointer<CarState> & carState,
    bool autoActuateMotorAndServo = true);
  virtual ~CarInterfaceComponent();
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
  void setUseSdvpPositionFusion(bool value) {mUseSdvpPositionFusion = value;}
  void setMinBatteryVoltage(float value) {mMinBatteryVoltage = value;}
  void setImuVariant(ImuVariant value) {mImuVariant = value;}
  void setVehicleInterfaceType(VehicleInterfaceType value) {mVehicleInterfaceType = value;}
  void setGnssReceiverVariant(RECEIVER_VARIANT value) {mGnssReceiverVariant = value;}
  void setSpeedControlType(SpeedControlType value) {mSpeedControlType = value;}
  void setVehicleStatePollRate(int value) {mVehicleStatePollRate = value;}
  void setPositionFusionInputTimerRate(int value) {mPositionFusionInputTimerRate = value;}
  void setGnssPrintVerbose(bool value) {mGnssPrintVerbose = value;}
  void setGnssSensorFusionImuAutoalign(bool value) {mGnssSensorFusionImuAutoalign = value;}
  void setGnssSensorFusionForceRecalibrate(bool value) {mGnssSensorFusionForceRecalibrate = value;}
  void setGnssMeasurementRate(int value) {mGnssMeasurementRate = value;}
  void setGnssPriorityMessageRate(int value) {mGnssPriorityMessageRate = value;}
  void setGnssDynamicModel(DynamicModel value) {mGnssDynamicModel = value;}
  void setGnssAntennaToGnssChipOffset(xyz_t value) {mGnssAntennaToGnssChipOffset = value;}
  void setGnssChipToRearAxleOffset(xyz_t value) {mGnssChipToRearAxleOffset = value;}
  void setGnssChipOrientationOffset(xyz_t value) {mGnssChipOrientationOffset = value;}
  void setEnuReference(llh_t value) {mEnuReference = value;}
  void setPIDSpeedControllerGains(float kp, float ki, float kd)
  {
    mPIDSpeedControllerKp = kp; mPIDSpeedControllerKi = ki; mPIDSpeedControllerKd = kd;
  }
  void setToFSensorsInfo(std::map<std::string, std::tuple<int, int>> value)
  {
    mToFSensorsInfo = value;
  }
  void setGnssTimeout(float value) {mGnssTimeout = value;}

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
  bool getUseSdvpPositionFusion() const {return mUseSdvpPositionFusion;}
  float getMinBatteryVoltage() const {return mMinBatteryVoltage;}
  ImuVariant getImuVariant() const {return mImuVariant;}
  VehicleInterfaceType getVehicleInterfaceType() const {return mVehicleInterfaceType;}
  RECEIVER_VARIANT getGnssReceiverVariant() const {return mGnssReceiverVariant;}
  SpeedControlType getSpeedControlType() const {return mSpeedControlType;}
  int getVehicleStatePollRate() const {return mVehicleStatePollRate;}
  int getPositionFusionInputTimerRate() const {return mPositionFusionInputTimerRate;}
  bool getGnssPrintVerbose() const {return mGnssPrintVerbose;}
  bool getGnssSensorFusionImuAutoalign() const {return mGnssSensorFusionImuAutoalign;}
  bool getGnssSensorFusionForceRecalibrate() const {return mGnssSensorFusionForceRecalibrate;}
  int getGnssMeasurementRate() const {return mGnssMeasurementRate;}
  int getGnssPriorityMessageRate() const {return mGnssPriorityMessageRate;}
  DynamicModel getGnssDynamicModel() const {return mGnssDynamicModel;}
  xyz_t getGnssAntennaToGnssChipOffset() const {return mGnssAntennaToGnssChipOffset;}
  xyz_t getGnssChipToRearAxleOffset() const {return mGnssChipToRearAxleOffset;}
  xyz_t getGnssChipOrientationOffset() const {return mGnssChipOrientationOffset;}
  llh_t getEnuReference() const {return mEnuReference;}
  std::map<std::string, std::tuple<int, int>> getToFSensorsInfo() const
  {
    return mToFSensorsInfo;
  }
  float getGnssTimeout() const {return mGnssTimeout;}

  QSharedPointer<GNSSReceiver> getGnssReceiver() const {return mGNSSReceiver;}
  QSharedPointer<EmergencyStopState> getEmergencyStopState() const {return mEmergencyStopState;}
  QSharedPointer<MovementController> getMovementController() const
  {
    return mMovementController;
  }
  QSharedPointer<RtcmClient> getRtcmClient() const {return mRtcmClient;}
  CarControlCommand getCarControlCommand() const {return mCarControlCommand;}

  // Utility methods
  virtual void setup_vehicle_interface();
  void activate_emergency_stop(const std::string & sender_id = "", const std::string & reason = "");
  void clear_emergency_stop(const std::string & sender_id = "");

  void updateControlCommand(const geometry_msgs::msg::Twist & twist, double dt);
  void executeControlCommand();

signals:
  void battery_voltage_received(double voltage);
  void tof_distance_received(std::string sensor_name, double distance);

public slots:
  void update_fused_position(PosPoint position);
  void on_updated_fused_position_externally(PosPoint position);
  void on_external_fused_position_timeout();

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

  bool mUseSdvpPositionFusion = false;
  ImuVariant mImuVariant = ImuVariant::UNKNOWN;
  VehicleInterfaceType mVehicleInterfaceType = VehicleInterfaceType::WAYWISE_SIMULATED;
  RECEIVER_VARIANT mGnssReceiverVariant = RECEIVER_VARIANT::WAYWISE_SIMULATED;
  SpeedControlType mSpeedControlType = SpeedControlType::OPEN_LOOP_ERPM_CONTROL;

  bool mAutoActuateMotorAndServo = true;
  int mVehicleStatePollRate = 10; // [Hz]
  int mPositionFusionInputTimerRate = 10; // [Hz]
  bool mGnssPrintVerbose = false;
  bool mGnssSensorFusionImuAutoalign = false;
  bool mGnssSensorFusionForceRecalibrate = false;
  int mGnssMeasurementRate = 5; // [Hz]
  int mGnssPriorityMessageRate = 10; // [Hz]
  DynamicModel mGnssDynamicModel = DynamicModel::AUTOMOT;

  float mPIDSpeedControllerKp = 1.0;
  float mPIDSpeedControllerKi = 0.0;
  float mPIDSpeedControllerKd = 0.0;
  float mGnssTimeout = 3.0;

  xyz_t mRearAxleToBaseOffset;
  xyz_t mRearAxleToCenterOffset;
  xyz_t mRearAxleToRearEndOffset;
  xyz_t mGnssAntennaToGnssChipOffset;
  xyz_t mGnssChipToRearAxleOffset;
  xyz_t mGnssChipOrientationOffset;
  llh_t mEnuReference = {57.713805, 12.890088, 203.59}; // [lat, lon, height]

  std::map<std::string, std::tuple<int, int>> mToFSensorsInfo;

  // WayWise components
  QSharedPointer<CarState> mCarState;
  QSharedPointer<GNSSReceiver> mGNSSReceiver;
  QSharedPointer<EmergencyStopState> mEmergencyStopState;
  QSharedPointer<MovementController> mMovementController;
  QSharedPointer<RtcmClient> mRtcmClient;

  QSharedPointer<VESCMotorController> mVESCMotorController;
  QSharedPointer<IMUOrientationUpdater> mIMUOrientationUpdater;
  QSharedPointer<SDVPVehiclePositionFuser> mSDVPVehiclePositionFuser;

  // Internal variables
  QTimer mPositionFusionInputTimer = QTimer();
  QTimer mWaywiseSimulationTimer = QTimer();
  float mCurrentBatteryVoltage = 0.0;
  QSharedPointer<PIDController> mPIDSpeedController;
  CarControlCommand mCarControlCommand;
  std::map<std::string, QSharedPointer<ToFSensor>> mToFSensors;
  QList<QMetaObject::Connection> mExternalFusedPositionBackupConnections;
};

#endif  // CAR_INTERFACE_COMPONENT_HPP_
