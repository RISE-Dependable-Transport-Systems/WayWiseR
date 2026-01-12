#include "localization_component.hpp"
#include "moc_localization_component.cpp"

#include "WayWise/sensors/gnss/ubloxrover.h"

LocalizationComponent::LocalizationComponent(
  QObjectNode * parentQObjectNode, const QSharedPointer<ObjectState> & objectState)
: QObject(parentQObjectNode)
{
  mParentQObjectNode = parentQObjectNode;
  mObjectState = objectState;
  mGNSSReceiver.reset(new GNSSReceiver(mObjectState));
}

LocalizationComponent::~LocalizationComponent()
{
  if (mGNSSReceiver->getReceiverVariant() == RECEIVER_VARIANT::UBLX_ZED_F9P ||
    mGNSSReceiver->getReceiverVariant() == RECEIVER_VARIANT::UBLX_ZED_F9R)
  {
    QSharedPointer<UbloxRover> mUbloxRover = qSharedPointerDynamicCast<UbloxRover>(mGNSSReceiver);
    if (mUbloxRover) {
      mUbloxRover->aboutToShutdown();
    }
  }
}

void LocalizationComponent::reset()
{
  mObjectState->setVelocity({0.0, 0.0, 0.0});

  auto initial_yaw_offset = mGnssChipOrientationOffset.z;
  auto pospoint = PosPoint();
  pospoint.setYaw(initial_yaw_offset);
  pospoint.setType(PosType::odom);
  mObjectState->setPosition(pospoint);
  pospoint.setType(PosType::IMU);
  mObjectState->setPosition(pospoint);
  pospoint.setType(PosType::fused);
  mObjectState->setPosition(pospoint);

  for (auto connection : mQMetaObjectConnections) {
    QObject::disconnect(connection);
  }
  mQMetaObjectConnections.clear();
  mImuDataAvailable = false;
}

void LocalizationComponent::setup_localization()
{
  // --- GNSS receiver setup ---
  switch (mGnssVariant) {
    case RECEIVER_VARIANT::UBLX_ZED_F9P:
    case RECEIVER_VARIANT::UBLX_ZED_F9R:
      {
        QSharedPointer<UbloxRover> mUbloxRover = QSharedPointer<UbloxRover>::create(mObjectState);
        foreach(const QSerialPortInfo & portInfo, QSerialPortInfo::availablePorts()) {
          // qDebug()<<portInfo.manufacturer();
          if (portInfo.manufacturer().toLower().replace("-", "").contains("ublox")) {
            mUbloxRover->setDynamicModel(mGnssDynamicModel);
            mUbloxRover->setNavPvtMessageRate(mGnssMessageRate);
            mUbloxRover->setReceiverVariant(mGnssVariant);

            if (mGnssVariant == RECEIVER_VARIANT::UBLX_ZED_F9R) {
              mUbloxRover->setPrintVerbose(mGnssPrintVerbose);
              mUbloxRover->setFusionOnChip(mGnssFusionOnChip);
              mUbloxRover->setESFAlgAutoMntAlgOn(mGnssSensorFusionImuAutoalign);
              mUbloxRover->setForceRecalibrateSensors(mGnssSensorFusionForceRecalibrate);
              mUbloxRover->setSpeedDataInputRate(mPositionFusionInputTimerRate);
            }

            if (mUbloxRover->connectSerial(portInfo)) {
              qDebug() << "UbloxRover connected to:" << portInfo.systemLocation();

              if (mGnssVariant == RECEIVER_VARIANT::UBLX_ZED_F9R) {
                mPositionFusionInputTimer = rclcpp::create_timer(
                  mParentQObjectNode->get_node_base_interface(),
                  mParentQObjectNode->get_node_timers_interface(),
                  mParentQObjectNode->get_clock(), // uses sim time if enabled
                  std::chrono::milliseconds(
                    (int)std::round(1000.0 / mPositionFusionInputTimerRate)
                  ),
                  std::bind(&UbloxRover::readObjectSpeedForPositionFusion, mUbloxRover.get()));
              }

              // -- NTRIP/TCP client setup for feeding RTCM data into GNSS receiver
              mRtcmClient.reset(new RtcmClient(this));
              QObject::connect(
                mUbloxRover.get(), &UbloxRover::gotNmeaGga,
                mRtcmClient.get(), &RtcmClient::forwardNmeaGgaToServer);
              QObject::connect(
                mRtcmClient.get(), &RtcmClient::rtcmData,
                mUbloxRover.get(), &UbloxRover::writeRtcmToUblox);
              if (mRtcmClient->connectWithInfoFromFile("./rtcmServerInfo.txt")) {
                qDebug() << "RtcmClient: connected to" << QString(
                  mRtcmClient->getCurrentHost() + ":" +
                  QString::number(mRtcmClient->getCurrentPort()));
              } else {
                qWarning() << "RtcmClient: not connected";
              }

              mGNSSReceiver = mUbloxRover;
            }
          }
        }
        if (!mUbloxRover->isSerialConnected()) {
          qWarning() <<
            "Configured GNSS receiver is not available! Simulating GNSS receiver using WayWise.";
          mGnssVariant = RECEIVER_VARIANT::WAYWISE_SIMULATED;
        }
      } break;
    default:
      break;
  }

  if (mGnssVariant == RECEIVER_VARIANT::WAYWISE_SIMULATED) {
    mGNSSReceiver->setReceiverState(RECEIVER_STATE::READY);

    mQMetaObjectConnections.emplace_back(
      QObject::connect(
        mObjectState.get(), &ObjectState::positionUpdated,
        [&](PosType type) {
          if (type == PosType::odom) {
            mGNSSReceiver->simulationStep();
          }
        }
    ));
  }

  mGNSSReceiver->setAntennaToChipOffset(
    mGnssAntennaToGnssChipOffset.x,
    mGnssAntennaToGnssChipOffset.y,
    mGnssAntennaToGnssChipOffset.z);
  mGNSSReceiver->setChipToBaseOffset(
    mChipToBaseOffset.x,
    mChipToBaseOffset.y,
    mChipToBaseOffset.z);
  mGNSSReceiver->setChipOrientationOffset(
    mGnssChipOrientationOffset.x,
    mGnssChipOrientationOffset.y,
    mGnssChipOrientationOffset.z);
  mGNSSReceiver->setReceiverVariant(mGnssVariant);

  if (mUseSdvpPositionFusion) {
    // Position Fuser
    mSDVPVehiclePositionFuser.reset(new SDVPVehiclePositionFuser(this));

    mQMetaObjectConnections.emplace_back(
      QObject::connect(
        mGNSSReceiver.get(), &GNSSReceiver::updatedGNSSPositionAndOrientation,
        mSDVPVehiclePositionFuser.get(),
        &SDVPVehiclePositionFuser::correctPositionAndYawGNSS));


    mQMetaObjectConnections.emplace_back(
      QObject::connect(
        mObjectState.get(), &ObjectState::positionUpdated,
        [&](PosType type) {
          switch (type) {
            case PosType::odom:
              {
                // Odometry fusion
                PosPoint odomPos = mObjectState->getPosition(type);
                static xyz_t lastOdomXyz;
                mSDVPVehiclePositionFuser->correctPositionAndYawOdom(
                  mObjectState,
                  QLineF(QPointF(lastOdomXyz.x, lastOdomXyz.y), odomPos.getPoint()).length());
                lastOdomXyz = odomPos.getXYZ();
              } break;
            case PosType::IMU:
              {
                // IMU fusion
                mSDVPVehiclePositionFuser->correctPositionAndYawIMU(mObjectState);
              } break;
            default:
              break;
          }
        }
    ));
  } else {
    QObject::connect(
      mGNSSReceiver.get(), &GNSSReceiver::updatedGNSSPositionAndOrientation,
      [&](QSharedPointer<ObjectState> objectState, double distanceMoved,
      GnssFixStatus gnssFixStatus) {
        Q_UNUSED(distanceMoved)

        PosPoint fusedPos = objectState->getPosition(PosType::fused);
        PosPoint gnssPos = objectState->getPosition(PosType::GNSS);
        fusedPos.setXYZ(gnssPos.getXYZ());
        fusedPos.setTime(gnssPos.getTime());
        if (gnssFixStatus.isFusedOnChip) {
          fusedPos.setRPY(gnssPos.getRPY());
        } else if (mImuDataAvailable) {
          fusedPos.setRPY(objectState->getPosition(PosType::IMU).getRPY());
        }
        objectState->setPosition(fusedPos);
      }
    );

    mQMetaObjectConnections.emplace_back(
      QObject::connect(
        mObjectState.get(), &ObjectState::positionUpdated,
        [&](PosType type) {
          if (type == PosType::IMU && !mImuDataAvailable) {
            mImuDataAvailable = true;
          }
        }
    ));
  }
}
