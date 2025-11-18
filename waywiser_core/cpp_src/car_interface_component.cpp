#include "car_interface_component.hpp"
#include "moc_car_interface_component.cpp"

CarInterfaceComponent::CarInterfaceComponent(
  QObject * parent, const QSharedPointer<CarState> & carState, bool autoActuateMotorAndServo)
: QObject(parent)
{
  mCarState = carState;
  mAutoActuateMotorAndServo = autoActuateMotorAndServo;
}

CarInterfaceComponent::~CarInterfaceComponent()
{
  if (mGNSSReceiver && (mGNSSReceiver->getReceiverVariant() == RECEIVER_VARIANT::UBLX_ZED_F9P ||
    mGNSSReceiver->getReceiverVariant() == RECEIVER_VARIANT::UBLX_ZED_F9R))
  {
    QSharedPointer<UbloxRover> mUbloxRover = qSharedPointerDynamicCast<UbloxRover>(mGNSSReceiver);
    if (mUbloxRover) {
      mUbloxRover->aboutToShutdown();
    }
  }
}

void CarInterfaceComponent::reset()
{
  mCurrentBatteryVoltage = 0.0;
  if (mSpeedControlType == SpeedControlType::CLOSED_LOOP_PID_SPEED_CONTROL) {
    mPIDSpeedController->reset();
  }
  mCarControlCommand.throttle = 0.0;
  mCarControlCommand.brake = 1.0;
  mCarControlCommand.steering = 0.0;

  mExternalFusedPositionBackupConnections.clear();
  mEmergencyStopState->set_clear();

  mCarState->setVelocity({0.0, 0.0, 0.0});
  mCarState->setSteering(0.0);

  auto initial_yaw_offset = getGnssChipOrientationOffset().z;
  auto pospoint = PosPoint();
  pospoint.setYaw(initial_yaw_offset);
  pospoint.setType(PosType::odom);
  mCarState->setPosition(pospoint);
}

void CarInterfaceComponent::setup_vehicle_interface()
{
  mCarState->setLength(mLength);
  mCarState->setWidth(mWidth);
  mCarState->setAxisDistance(mWheelbase);
  mCarState->setMaxSteeringAngle(atan(mWheelbase / mMinTurningRadius));
  mCarState->setRearAxleToCenterOffset(mRearAxleToCenterOffset);
  mCarState->setRearAxleToRearEndOffset(mRearAxleToRearEndOffset);

  mEmergencyStopState.reset(new EmergencyStopState());

  // --- Movement control setup ---
  switch (mVehicleInterfaceType) {
    case VehicleInterfaceType::VESC:
    case VehicleInterfaceType::WAYWISE_SIMULATED:
      {
        QSharedPointer<CarMovementController> carMovementController =
          QSharedPointer<CarMovementController>(
          new CarMovementController(
            mCarState,
            mAutoActuateMotorAndServo));
        carMovementController->setSpeedToRPMFactor(mSpeedToRPMFactor);

        // setup and connect VESC, simulate movements if unable to connect
        if (mVehicleInterfaceType == VehicleInterfaceType::VESC) {
          mVESCMotorController.reset(new VESCMotorController());
          foreach(const QSerialPortInfo & portInfo, QSerialPortInfo::availablePorts())
          {
            if (portInfo.description().toLower().replace("-", "").contains("chibios")) { // assumption: Serial device with ChibiOS in
                                                                                         // description is VESC
              mVESCMotorController->connectSerial(portInfo);
              qDebug() << "VESCMotorController connected to: " << portInfo.systemLocation();
            }
          }

          if (mVESCMotorController->isSerialConnected()) {
            carMovementController->setMotorController(mVESCMotorController);
            // convert Hz to ms and set VESC polling rate
            mVESCMotorController->setPollValuesPeriod(1000 / mVehicleStatePollRate);

            // VESC is a special case that can also control the servo
            const auto servoController = mVESCMotorController->getServoController();
            servoController->setInvertOutput(mInvertServoOutput);
            servoController->setServoCenter(mServoOffset);
            servoController->setServoRange(mServoRange);
            carMovementController->setServoController(servoController);

            if (mMinBatteryVoltage <= 0.0) {
              qDebug() <<
                "WARNING: Low battery protection is not configured!";
            } else {
              qDebug() << "Low battery protection is enabled with voltage threshold of " <<
                mMinBatteryVoltage << " V.";
            }

            QObject::connect(
              mVESCMotorController.get(), &VESCMotorController::gotStatusValues,
              [&](double rpm, int tachometer, int tachometer_abs, double voltageInput,
              double temperature,
              int errorID) {
                Q_UNUSED(rpm)
                Q_UNUSED(tachometer)
                Q_UNUSED(tachometer_abs)
                Q_UNUSED(temperature)
                Q_UNUSED(errorID)

                static int count = 0;
                if (count++ % mVehicleStatePollRate) { // reduce output rate to 1 Hz
                  return;
                }

                emit battery_voltage_received(voltageInput);
              });
          } else {
            mVehicleInterfaceType = VehicleInterfaceType::WAYWISE_SIMULATED;
            qDebug() <<
              "VESCMotorController is not connected!";
          }
        }

        if (mVehicleInterfaceType == VehicleInterfaceType::WAYWISE_SIMULATED) {
          qDebug() << "Simulating vehicle movement using WayWise.";
          const int pollPeriodMs = 1000 / mVehicleStatePollRate;
          QObject::connect(
            &mWaywiseSimulationTimer, &QTimer::timeout,
            [this, carMovementController, pollPeriodMs]() {
              carMovementController->simulationStep(pollPeriodMs);
            });
          mWaywiseSimulationTimer.start(pollPeriodMs);
        }

        mMovementController = carMovementController;
      } break;
    case VehicleInterfaceType::EXT_SIMULATED:
      {
        mMovementController.reset(new MovementController(mCarState));
      } break;
    default:
      qDebug() << "Unknown vehicle interface type is requested!";
      break;
  }

  switch (mSpeedControlType) {
    case SpeedControlType::CLOSED_LOOP_PID_SPEED_CONTROL:
      {
        mPIDSpeedController.reset(
          new PIDController(
            mPIDSpeedControllerKp, mPIDSpeedControllerKi, mPIDSpeedControllerKd));
      } break;
    default:
      break;
  }

  // --- Positioning setup ---
  // GNSS (with fused IMU when using u-blox F9R)
  switch (mGnssReceiverVariant) {
    case RECEIVER_VARIANT::UBLX_ZED_F9P:
    case RECEIVER_VARIANT::UBLX_ZED_F9R:
      {
        QSharedPointer<UbloxRover> mUbloxRover = QSharedPointer<UbloxRover>::create(mCarState);
        foreach(const QSerialPortInfo & portInfo, QSerialPortInfo::availablePorts()) {
          // qDebug()<<portInfo.manufacturer();
          if (portInfo.manufacturer().toLower().replace("-", "").contains("ublox")) {
            mUbloxRover->setReceiverVariant(mGnssReceiverVariant);

            if (mGnssReceiverVariant == RECEIVER_VARIANT::UBLX_ZED_F9R) {
              mUbloxRover->setDynamicModel(mGnssDynamicModel);
              mUbloxRover->setPrintVerbose(mGnssPrintVerbose);
              mUbloxRover->setESFAlgAutoMntAlgOn(mGnssSensorFusionImuAutoalign);
              mUbloxRover->setForceRecalibrateSensors(mGnssSensorFusionForceRecalibrate);
              mUbloxRover->setGNSSMeasurementRate(mGnssMeasurementRate);
              mUbloxRover->setNavPrioMessageRate(mGnssPriorityMessageRate);
              mUbloxRover->setSpeedDataInputRate(mPositionFusionInputTimerRate);
            }

            if (mUbloxRover->connectSerial(portInfo)) {
              qDebug() << "UbloxRover connected to:" << portInfo.systemLocation();

              mUbloxRover->setAntennaToChipOffset(
                mGnssAntennaToGnssChipOffset.x,
                mGnssAntennaToGnssChipOffset.y,
                mGnssAntennaToGnssChipOffset.z);
              mUbloxRover->setChipToRearAxleOffset(
                mGnssChipToRearAxleOffset.x,
                mGnssChipToRearAxleOffset.y,
                mGnssChipToRearAxleOffset.z);
              mUbloxRover->setChipOrientationOffset(
                mGnssChipOrientationOffset.x,
                mGnssChipOrientationOffset.y,
                mGnssChipOrientationOffset.z);

              if (mUseSdvpPositionFusion) {
                QObject::connect(
                  mUbloxRover.get(), &UbloxRover::updatedGNSSPositionAndYaw,
                  mSDVPVehiclePositionFuser.get(),
                  &SDVPVehiclePositionFuser::correctPositionAndYawGNSS);
              } else {
                QObject::connect(
                  mUbloxRover.get(), &UbloxRover::txNavPvt,
                  [&](const ubx_nav_pvt & ubxPvt) {
                    Q_UNUSED(ubxPvt)

                    PosPoint currentPosition = mCarState->getPosition(PosType::GNSS);
                    currentPosition.setType(PosType::fused);
                    mCarState->setPosition(currentPosition);
                  });
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
                qDebug() << "RtcmClient: not connected";
              }

              mGNSSReceiver = mUbloxRover;
              mGNSSReceiver->setEnuRef(mEnuReference);

              if (mGNSSReceiver->getReceiverVariant() == RECEIVER_VARIANT::UBLX_ZED_F9R) {
                connect(
                  &mPositionFusionInputTimer, &QTimer::timeout,
                  mGNSSReceiver.get(), &GNSSReceiver::readVehicleSpeedForPositionFusion);
                mPositionFusionInputTimer.start(1000 / mPositionFusionInputTimerRate);
              }
            }
          }
        }
        if (mGNSSReceiver == nullptr) {
          qDebug() <<
            "Configured GNSS receiver is not available! Simulating GNSS receiver using WayWise.";
          mGnssReceiverVariant = RECEIVER_VARIANT::WAYWISE_SIMULATED;
        }
      } break;
    case RECEIVER_VARIANT::EXTERNAL:
      {
        mGNSSReceiver.reset(new GNSSReceiver(mCarState));
        mGNSSReceiver->setReceiverVariant(RECEIVER_VARIANT::EXTERNAL);
        mGNSSReceiver->setEnuRef(mEnuReference);

        QObject::connect(
          parent(), SIGNAL(externalFusedPositionTimeout()), this,
          SLOT(on_external_fused_position_timeout()));

        QObject::connect(
          parent(), SIGNAL(updatedFusedPositionExternally(PosPoint)), this,
          SLOT(on_updated_fused_position_externally(PosPoint)));
      } break;
    default:
      break;
  }

  if (mGnssReceiverVariant == RECEIVER_VARIANT::WAYWISE_SIMULATED) {
    mGNSSReceiver.reset(new GNSSReceiver(mCarState));
    mGNSSReceiver->setReceiverVariant(RECEIVER_VARIANT::WAYWISE_SIMULATED);
    mGNSSReceiver->setReceiverState(RECEIVER_STATE::READY);
    mGNSSReceiver->setEnuRef(mEnuReference);
    mGNSSReceiver->setGnssFixAccuracy({0.0, 0.0, 0.0}); // TODO: estimate accuracy

    QObject::connect(
      mMovementController.get(), &MovementController::updatedOdomPositionAndYaw,
      [&](QSharedPointer<VehicleState> vehicleState, double distanceDriven) {
        Q_UNUSED(distanceDriven)

        update_fused_position(vehicleState->getPosition(PosType::odom));
      });

    QObject::connect(
      parent(), SIGNAL(updatedOdomPositionExternally(PosPoint)), this,
      SLOT(update_fused_position(PosPoint)));
  }

  // Position Fuser
  if (mUseSdvpPositionFusion) {
    mSDVPVehiclePositionFuser.reset(new SDVPVehiclePositionFuser(this));

    // IMU
    switch (mImuVariant) {
      case ImuVariant::VESC:
        {
          if (mVESCMotorController->isSerialConnected()) {
            mIMUOrientationUpdater = mVESCMotorController->getIMUOrientationUpdater(mCarState);
            QObject::connect(
              mIMUOrientationUpdater.get(), &IMUOrientationUpdater::updatedIMUOrientation,
              mSDVPVehiclePositionFuser.get(),
              &SDVPVehiclePositionFuser::correctPositionAndYawIMU);
            qDebug() << "Using vesc IMU for position fusion.";
          } else {
            qDebug() <<
              "vesc IMU is configured for position fusion but VESCMotorController is not connected.";
          }
        } break;
      case ImuVariant::BNO055:
        {
          mIMUOrientationUpdater.reset(new BNO055OrientationUpdater(mCarState, "/dev/i2c-1"));
          QObject::connect(
            mIMUOrientationUpdater.get(), &IMUOrientationUpdater::updatedIMUOrientation,
            mSDVPVehiclePositionFuser.get(),
            &SDVPVehiclePositionFuser::correctPositionAndYawIMU);
          qDebug() << "Using bno055 IMU for position fusion.";
        } break;
      default:
        qDebug() << "Unknown IMU variant is requested for position fusion!";
        break;
    }

    // Odometry
    QObject::connect(
      mMovementController.get(), &MovementController::updatedOdomPositionAndYaw,
      mSDVPVehiclePositionFuser.get(),
      &SDVPVehiclePositionFuser::correctPositionAndYawOdom);
  }

  // ToF Sensors
  for (const auto & pair : mToFSensorsInfo) {
    std::string tof_sensor_name = pair.first;
    QSharedPointer<ToFSensor> tof_sensor;
    tof_sensor.reset(new VL53L0XToFSensor());
    QObject::connect(
      tof_sensor.get(), &ToFSensor::updatedDistance, this,
      [this, tof_sensor_name](double distance) {
        emit tof_distance_received(tof_sensor_name, distance);
      });
    mToFSensors[tof_sensor_name] = tof_sensor;
  }
}

void CarInterfaceComponent::on_updated_fused_position_externally(PosPoint position)
{
  Q_UNUSED(position)

  mGNSSReceiver->setReceiverState(RECEIVER_STATE::READY);
  mGNSSReceiver->setGnssFixAccuracy({0.0, 0.0, 0.0});       // TODO: estimate accuracy

  switch (mGNSSReceiver->getReceiverVariant()) {
    case RECEIVER_VARIANT::WAYWISE_SIMULATED:
      {
        qDebug() << "Receiving external position updates.";
        mGNSSReceiver->setReceiverVariant(RECEIVER_VARIANT::EXTERNAL);
        for (const auto & conn : mExternalFusedPositionBackupConnections) {
          QObject::disconnect(conn);
        }
        mExternalFusedPositionBackupConnections.clear();
      } break;
    default:
      break;
  }
}

void CarInterfaceComponent::on_external_fused_position_timeout()
{
  qDebug() <<
    "External position update timed out. Switching to waywise simulated position updater.";
  mGNSSReceiver->setReceiverVariant(RECEIVER_VARIANT::WAYWISE_SIMULATED);
  mGNSSReceiver->setReceiverState(RECEIVER_STATE::READY);
  mGNSSReceiver->setGnssFixAccuracy({0.0, 0.0, 0.0});           // TODO: estimate accuracy
  if (!mExternalFusedPositionBackupConnections.isEmpty()) {
    for (const auto & conn : mExternalFusedPositionBackupConnections) {
      QObject::disconnect(conn);
    }
    mExternalFusedPositionBackupConnections.clear();
  }
  mExternalFusedPositionBackupConnections.append(
    QObject::connect(
      mMovementController.get(), &MovementController::updatedOdomPositionAndYaw,
      [&](QSharedPointer<VehicleState> vehicleState, double distanceDriven) {
        Q_UNUSED(distanceDriven)

        update_fused_position(vehicleState->getPosition(PosType::odom));
      }));
  mExternalFusedPositionBackupConnections.append(
    QObject::connect(
      parent(), SIGNAL(updatedOdomPositionExternally(PosPoint)), this,
      SLOT(update_fused_position(PosPoint))));
}

void CarInterfaceComponent::update_fused_position(PosPoint position)
{
  position.setType(PosType::fused);
  mCarState->setPosition(position);
}

void CarInterfaceComponent::activate_emergency_stop(
  const std::string & sender_id,
  const std::string & reason)
{
  if (!mEmergencyStopState->is_active()) {
    mMovementController->setDesiredSpeed(0.0);
    mMovementController->setDesiredSteering(0.0);
    mEmergencyStopState->set_active();
    qDebug() << QString("Emergency stop ACTIVATED%1.%2")
      .arg(sender_id.empty() ? "" : " by " + QString::fromStdString(sender_id))
      .arg(reason.empty() ? "" : " Reason: " + QString::fromStdString(reason));
  }
}

void CarInterfaceComponent::clear_emergency_stop(const std::string & sender_id)
{
  if (!mEmergencyStopState->is_clear()) {
    mEmergencyStopState->set_clear();
    qDebug() << QString("Emergency stop CLEARED%1")
      .arg(sender_id.empty() ? "" : " by " + QString::fromStdString(sender_id));
  }
}

void CarInterfaceComponent::updateControlCommand(const geometry_msgs::msg::Twist & twist, double dt)
{
  float desired_speed = 0.0;

  if (mEmergencyStopState->is_active()) {
    mCarControlCommand.throttle = 0.0;
    mCarControlCommand.brake = 1.0;
    mCarControlCommand.steering = 0.0;
    if (mSpeedControlType == SpeedControlType::CLOSED_LOOP_PID_SPEED_CONTROL) {
      mPIDSpeedController->reset();
    }
  } else if (mEmergencyStopState->is_clear()) {
    static double max_linear_speed = mErpmMax / mSpeedToRPMFactor;
    static double min_linear_speed = mErpmMin / mSpeedToRPMFactor;

    desired_speed = std::clamp(twist.linear.x, -max_linear_speed, max_linear_speed);
    desired_speed = fabs(desired_speed) >= min_linear_speed ? desired_speed : 0.0;

    switch (mSpeedControlType) {
      case SpeedControlType::OPEN_LOOP_ERPM_CONTROL:
        {
          mCarControlCommand.throttle = desired_speed / max_linear_speed;
          mCarControlCommand.brake = 0.0;
        } break;
      case SpeedControlType::CLOSED_LOOP_PID_SPEED_CONTROL:
        {
          if (fabs(desired_speed) > 0.0) {
            double speed_error = desired_speed - mCarState->getSpeed();
            auto speed_control_signal = std::clamp(
              mPIDSpeedController->compute(speed_error, dt), -1.0, 1.0);
            // qDebug() << "Speed control signal: " << speed_control_signal << ", current speed: " <<
            // mCarState->getSpeed() << ", desired speed: " << desired_speed << ", speed error: " <<
            // speed_error;
            if (desired_speed * speed_control_signal > 0.0) {
              mCarControlCommand.throttle = speed_control_signal;
              mCarControlCommand.brake = 0.0;
            } else {
              mCarControlCommand.throttle = 0.0;
              mCarControlCommand.brake = fabs(speed_control_signal);
            }
          } else {
            mPIDSpeedController->reset();
            mCarControlCommand.throttle = 0.0;
            mCarControlCommand.brake = 1.0;
          }
        }
        break;
      default:
        break;
    }

    float steering_curvature = 0.0; // 1/r = ω/v
    if (fabs(desired_speed) > 0.0) {
      steering_curvature = twist.angular.z / desired_speed;
    } else {
      steering_curvature = twist.angular.z / min_linear_speed;
    }

    // NOTE / TODO: WayWise has a sign error here (curvature in wrong direction)
    mCarControlCommand.steering = std::clamp(
      -atan(mCarState->getAxisDistance() * steering_curvature) / mCarState->getMaxSteeringAngle(),
      -1.0,
      1.0);
  }

  mMovementController->setDesiredSpeed(desired_speed);
  mMovementController->setDesiredSteering(mCarControlCommand.steering);
}

void CarInterfaceComponent::executeControlCommand()
{
  float motorErpm = 0.0;
  if (mCarControlCommand.brake == 0.0) {
    motorErpm = std::clamp(mCarControlCommand.throttle * mErpmMax, -mErpmMax, mErpmMax);
    motorErpm = fabs(motorErpm) >= mErpmMin ? motorErpm : 0.0;
  }
  mMovementController->actuateDriveMotor(motorErpm);
  mMovementController->actuateSteeringServo(mCarControlCommand.steering);
}
