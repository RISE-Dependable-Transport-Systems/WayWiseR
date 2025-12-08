#include "truck_interface_component.hpp"
#include "moc_truck_interface_component.cpp"

TruckInterfaceComponent::TruckInterfaceComponent(
  QObject * parent, const QSharedPointer<TruckState> truckState, bool hasTrailer,
  bool autoActuateMotorAndServo)
: CarInterfaceComponent(parent, truckState, autoActuateMotorAndServo)
{
  mTruckState = truckState;
  mHasTrailer = hasTrailer;
}

TruckInterfaceComponent::~TruckInterfaceComponent()
{
  CarInterfaceComponent::~CarInterfaceComponent();
}

void TruckInterfaceComponent::reset()
{
  mTruckState->setTrailerAngle(0.0);
  CarInterfaceComponent::reset();
}

void TruckInterfaceComponent::setup_vehicle_interface()
{
  CarInterfaceComponent::setup_vehicle_interface();

  if (mHasTrailer) {
    mTrailerState.reset(new TrailerState(mTrailerMavlinkComponentID, Qt::white));
    mTrailerState->setLength(mTrailerLength);
    mTrailerState->setWidth(mTrailerWidth);
    mTrailerState->setWheelBase(mTrailerWheelbase);
    mTrailerState->setRearAxleToCenterOffset(mTrailerRearAxleToTrailerCenterOffset);
    mTrailerState->setRearAxleToRearEndOffset(mTrailerRearAxleToTrailerRearEndOffset);
    mTrailerState->setRearAxleToHitchOffset(mTrailerRearAxleToTrailerHitchOffset);

    mTruckState->setTrailingVehicle(mTrailerState);
    mTruckState->setRearAxleToHitchOffset(mRearAxleToHitchOffset);

    switch (mVehicleInterfaceType) {
      case VehicleInterfaceType::VESC:
      case VehicleInterfaceType::WAYWISE_SIMULATED:
        {
          // Angle Sensor
          mAngleSensorUpdater.reset(new AS5600Updater(mTruckState, mAngleSensorOffset));
          if (!mAngleSensorUpdater->isConnected()) {
            mTruckState->setSimulateTrailer(true);
            qDebug() << "AS5600 not connected. Trailer angle will be simulated.";
          }
        } break;
      default:
        break;
    }
  }
}
