#ifndef LOCALIZATION_COMPONENT_HPP_
#define LOCALIZATION_COMPONENT_HPP_

#include <memory>
#include <string>
#include <QObject>
#include <QString>
#include <tuple>
#include <QSerialPortInfo>

#include "WayWise/core/coordinatetransforms.h"
#include "WayWise/logger/logger.h"
#include "WayWise/sensors/fusion/sdvpvehiclepositionfuser.h"
#include "WayWise/sensors/gnss/gnssreceiver.h"
#include "WayWise/sensors/gnss/rtcmclient.h"
#include "WayWise/sensors/gnss/ubloxrover.h"
#include "WayWise/vehicles/objectstate.h"

#include "qobject_node.hpp"
#include "waywiser_core_utils.hpp"


class LocalizationComponent : public QObject
{
  Q_OBJECT

public:
  // Constructor and destructor
  LocalizationComponent(
    QObjectNode * parentQObjectNode, const QSharedPointer<ObjectState> & objectState);
  virtual ~LocalizationComponent();
  virtual void reset();

  virtual void setup_localization();

  // Setters
  void setUseSdvpPositionFusion(bool value) {mUseSdvpPositionFusion = value;}
  void setGnssVariant(RECEIVER_VARIANT value) {mGnssVariant = value;}
  void setPositionFusionInputTimerRate(int value) {mPositionFusionInputTimerRate = value;}
  void setGnssPrintVerbose(bool value) {mGnssPrintVerbose = value;}
  void setGnssFusionOnChip(bool value) {mGnssFusionOnChip = value;}
  void setGnssSensorFusionImuAutoalign(bool value) {mGnssSensorFusionImuAutoalign = value;}
  void setGnssSensorFusionForceRecalibrate(bool value) {mGnssSensorFusionForceRecalibrate = value;}
  void setGnssMessageRate(int value) {mGnssMessageRate = value;}
  void setGnssDynamicModel(DynamicModel value) {mGnssDynamicModel = value;}
  void setGnssAntennaToGnssChipOffset(xyz_t value) {mGnssAntennaToGnssChipOffset = value;}
  void setChipToBaseOffset(xyz_t value) {mChipToBaseOffset = value;}
  void setGnssChipOrientationOffset(xyz_t value) {mGnssChipOrientationOffset = value;}

  // Getters
  bool getUseSdvpPositionFusion() const {return mUseSdvpPositionFusion;}
  RECEIVER_VARIANT getGnssVariant() const {return mGnssVariant;}
  int getPositionFusionInputTimerRate() const {return mPositionFusionInputTimerRate;}
  bool getGnssPrintVerbose() const {return mGnssPrintVerbose;}
  bool getGnssFusionOnChip() const {return mGnssFusionOnChip;}
  bool getGnssSensorFusionImuAutoalign() const {return mGnssSensorFusionImuAutoalign;}
  bool getGnssSensorFusionForceRecalibrate() const {return mGnssSensorFusionForceRecalibrate;}
  int getGnssMessageRate() const {return mGnssMessageRate;}
  DynamicModel getGnssDynamicModel() const {return mGnssDynamicModel;}
  xyz_t getGnssAntennaToGnssChipOffset() const {return mGnssAntennaToGnssChipOffset;}
  xyz_t getGnssChipToReferencePointOffset() const {return mChipToBaseOffset;}
  xyz_t getGnssChipOrientationOffset() const {return mGnssChipOrientationOffset;}

  QSharedPointer<GNSSReceiver> getGnssReceiver() const {return mGNSSReceiver;}
  QSharedPointer<RtcmClient> getRtcmClient() const {return mRtcmClient;}

signals:

public slots:

protected:
  // Parameters
  bool mUseSdvpPositionFusion = false;
  RECEIVER_VARIANT mGnssVariant = RECEIVER_VARIANT::WAYWISE_SIMULATED;
  int mPositionFusionInputTimerRate = 10; // [Hz]
  bool mGnssPrintVerbose = false;
  bool mGnssFusionOnChip = true; // only used for Ublox F9R
  bool mGnssSensorFusionImuAutoalign = false;
  bool mGnssSensorFusionForceRecalibrate = false;
  int mGnssMessageRate = 5; // [Hz]
  DynamicModel mGnssDynamicModel = DynamicModel::AUTOMOT;
  xyz_t mGnssAntennaToGnssChipOffset;
  xyz_t mChipToBaseOffset;
  xyz_t mGnssChipOrientationOffset;

  // WayWise components
  QSharedPointer<ObjectState> mObjectState;
  QSharedPointer<GNSSReceiver> mGNSSReceiver;
  QSharedPointer<RtcmClient> mRtcmClient;
  QSharedPointer<SDVPVehiclePositionFuser> mSDVPVehiclePositionFuser;

  // Internal variables
  QObjectNode * mParentQObjectNode;
  rclcpp::TimerBase::SharedPtr mPositionFusionInputTimer = nullptr;
  std::vector<QMetaObject::Connection> mQMetaObjectConnections;
};

#endif  // LOCALIZATION_COMPONENT_HPP_
