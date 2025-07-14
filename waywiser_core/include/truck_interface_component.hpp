#ifndef TRUCK_INTERFACE_COMPONENT_HPP_
#define TRUCK_INTERFACE_COMPONENT_HPP_

#include "WayWise/vehicles/truckstate.h"
#include "WayWise/vehicles/trailerstate.h"
#include "WayWise/sensors/angle/as5600updater.h"

#include "car_interface_component.hpp"

class TruckInterfaceComponent : public CarInterfaceComponent
{
  Q_OBJECT

public:
  // Constructor and destructor
  TruckInterfaceComponent(
    QObject * parent, const QSharedPointer<TruckState> truckState, bool hasTrailer,
    bool autoActuateMotorAndServo = true);
  virtual ~TruckInterfaceComponent();
  virtual void reset() override;

  // Setters
  void setTrailerLength(float value)
  {
    mTrailerLength = value;
    mTrailerRearAxleToTrailerRearEndOffset.x = -0.1 * value;
    mTrailerRearAxleToTrailerHitchOffset.x = 0.7 * value;
  }
  void setTrailerWidth(float value) {mTrailerWidth = value;}
  void setTrailerWheelbase(float value) {mTrailerWheelbase = value;}
  void setRearAxleToHitchOffset(xyz_t value) {mRearAxleToHitchOffset = value;}
  void setTrailerRearAxleToTrailerBaseOffset(xyz_t value)
  {
    mTrailerRearAxleToTrailerBaseOffset = value;
  }
  void setTrailerRearAxleToTrailerRearEndOffset(xyz_t value)
  {
    mTrailerRearAxleToTrailerRearEndOffset = value;
  }
  void setTrailerRearAxleToTrailerCenterOffset(xyz_t value)
  {
    mTrailerRearAxleToTrailerCenterOffset = value;
  }
  void setTrailerRearAxleToTrailerHitchOffset(xyz_t value)
  {
    mTrailerRearAxleToTrailerHitchOffset = value;
  }
  void setTrailerMavlinkComponentID(int value) {mTrailerMavlinkComponentID = value;}
  void setAngleSensorOffset(float value) {mAngleSensorOffset = value;}

  // Getters
  bool hasTrailer() const {return mHasTrailer;}
  float getTrailerLength() const {return mTrailerLength;}
  float getTrailerWidth() const {return mTrailerWidth;}
  float getTrailerWheelbase() const {return mTrailerWheelbase;}
  xyz_t getRearAxleToHitchOffset() const {return mRearAxleToHitchOffset;}
  xyz_t getTrailerRearAxleToTrailerBaseOffset() const
  {
    return mTrailerRearAxleToTrailerBaseOffset;
  }
  xyz_t getTrailerRearAxleToTrailerRearEndOffset() const
  {
    return mTrailerRearAxleToTrailerRearEndOffset;
  }
  xyz_t getTrailerRearAxleToTrailerCenterOffset() const
  {
    return mTrailerRearAxleToTrailerCenterOffset;
  }
  xyz_t getTrailerRearAxleToTrailerHitchOffset() const
  {
    return mTrailerRearAxleToTrailerHitchOffset;
  }
  int getTrailerMavlinkComponentID() const {return mTrailerMavlinkComponentID;}
  float getAngleSensorOffset() const {return mAngleSensorOffset;}

  // Utility methods
  virtual void setup_vehicle_interface() override;

protected:
  // Parameters
  bool mHasTrailer = false;
  float mTrailerLength = 0.96;   // [m]
  float mTrailerWidth = 0.21;   // [m]
  float mTrailerWheelbase = 0.64; // [m]

  float mAngleSensorOffset = 0.0; // [deg]

  xyz_t mRearAxleToHitchOffset;
  xyz_t mTrailerRearAxleToTrailerBaseOffset;
  xyz_t mTrailerRearAxleToTrailerRearEndOffset;
  xyz_t mTrailerRearAxleToTrailerCenterOffset;
  xyz_t mTrailerRearAxleToTrailerHitchOffset;

  int mTrailerMavlinkComponentID = -1;

  // WayWise components
  QSharedPointer<TruckState> mTruckState;
  QSharedPointer<TrailerState> mTrailerState;
  QSharedPointer<AngleSensorUpdater> mAngleSensorUpdater;

  // Internal variables
};

#endif  // TRUCK_INTERFACE_COMPONENT_HPP_
