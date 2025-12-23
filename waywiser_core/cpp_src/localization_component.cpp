#include "localization_component.hpp"
#include "moc_localization_component.cpp"

LocalizationComponent::LocalizationComponent(
  QObject * parentQObject, const QSharedPointer<ObjectState> & objectState)
: QObject(parentQObject)
{
  mParentQObject = parentQObject;
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
                connect(
                  &mPositionFusionInputTimer, &QTimer::timeout,
                  mUbloxRover.get(), &UbloxRover::readObjectSpeedForPositionFusion);
                mPositionFusionInputTimer.start(1000 / mPositionFusionInputTimerRate);
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

    QObject::connect(
      mObjectState.get(), &ObjectState::positionUpdated,
      [&](PosType type) {
        if (type == PosType::odom) {
          mGNSSReceiver->simulationStep();
        }
      }
    );
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
    QObject::connect(
      mGNSSReceiver.get(), &GNSSReceiver::updatedGNSSPositionAndYaw,
      mSDVPVehiclePositionFuser.get(),
      &SDVPVehiclePositionFuser::correctPositionAndYawGNSS);

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
    );
  } else {
    QObject::connect(
      mObjectState.get(), &ObjectState::positionUpdated,
      [&](PosType type) {
        if (type == PosType::GNSS) {
          PosPoint fusedPos = mObjectState->getPosition(type);
          fusedPos.setType(PosType::fused);
          mObjectState->setPosition(fusedPos);
        }
      }
    );
  }
}
