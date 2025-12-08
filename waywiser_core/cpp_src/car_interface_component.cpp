#include "car_interface_component.hpp"
#include "moc_car_interface_component.cpp"

CarInterfaceComponent::CarInterfaceComponent(
  QObject * parentQObject, const QSharedPointer<CarState> & carState,
  bool autoActuateMotorAndServo)
: QObject(parentQObject)
{
  mParentQObject = parentQObject;
  mCarState = carState;
  mAutoActuateMotorAndServo = autoActuateMotorAndServo;
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

  mEmergencyStopState->set_clear();

  mCarState->setVelocity({0.0, 0.0, 0.0});
  mCarState->setSteering(0.0);
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

void CarInterfaceComponent::updateControlCommand(
  double desired_linear_speed, double desired_angular_speed, double dt)
{
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

    desired_linear_speed = std::clamp(desired_linear_speed, -max_linear_speed, max_linear_speed);
    desired_linear_speed = fabs(desired_linear_speed) >=
      min_linear_speed ? desired_linear_speed : 0.0;

    switch (mSpeedControlType) {
      case SpeedControlType::OPEN_LOOP_ERPM_CONTROL:
        {
          mCarControlCommand.throttle = desired_linear_speed / max_linear_speed;
          mCarControlCommand.brake = 0.0;
        } break;
      case SpeedControlType::CLOSED_LOOP_PID_SPEED_CONTROL:
        {
          if (fabs(desired_linear_speed) > 0.0) {
            double speed_error = desired_linear_speed - mCarState->getSpeed();
            auto speed_control_signal = std::clamp(
              mPIDSpeedController->compute(speed_error, dt), -1.0, 1.0);
            // qDebug() << "Speed control signal: " << speed_control_signal << ", current speed: " <<
            // mCarState->getSpeed() << ", desired speed: " << desired_linear_speed << ", speed error: " <<
            // speed_error;
            if (desired_linear_speed * speed_control_signal > 0.0) {
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
    if (fabs(desired_linear_speed) > 0.0) {
      steering_curvature = desired_angular_speed / desired_linear_speed;
    } else {
      steering_curvature = desired_angular_speed / min_linear_speed;
    }

    // NOTE / TODO: WayWise has a sign error here (curvature in wrong direction)
    mCarControlCommand.steering = std::clamp(
      -atan(mCarState->getAxisDistance() * steering_curvature) / mCarState->getMaxSteeringAngle(),
      -1.0, 1.0);
  }

  mMovementController->setDesiredSpeed(desired_linear_speed);
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
