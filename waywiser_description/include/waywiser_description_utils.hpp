#ifndef WAYWISER_DESCRIPTION_UTILS_HPP_
#define WAYWISER_DESCRIPTION_UTILS_HPP_

#include <QSharedPointer>

#include "urdf/model.h"

#include "waywiser/waywiser_utils.hpp"


inline QSharedPointer<urdf::Model> getURDFModel(const std::string & urdf_file_)
{
  QSharedPointer<urdf::Model> urdfModel = nullptr;

  if (!urdf_file_.empty()) {
    urdfModel = QSharedPointer<urdf::Model>::create();
    if (urdf_file_.find("xml version=") != std::string::npos) {
      urdfModel->initString(urdf_file_);
    } else {
      urdfModel->initFile(urdf_file_);
    }
  }

  if (urdfModel != nullptr) {
    qDebug() << "URDF model loaded.";
  }

  return urdfModel;
}

inline vector3_t getFramePosition(
  QSharedPointer<urdf::Model> urdfModel,
  const std::string & link_name)
{
  urdf::Vector3 position(0, 0, 0);
  urdf::LinkConstSharedPtr link = urdfModel->getLink(link_name);
  if (!link) {
    if (urdfModel->getRoot()->name != link_name) {
      qWarning() << "Link not found:" << QString::fromStdString(link_name) << " in model:" <<
        QString::fromStdString(urdfModel->getRoot()->name);
    }
  } else {
    while (link && link->parent_joint) {
      const urdf::Pose & joint_pose = link->parent_joint->parent_to_joint_origin_transform;
      position = joint_pose.rotation * position + joint_pose.position;
      link = urdfModel->getLink(link->getParent()->name);
    }
  }
  return vector3_t{position.x, position.y, position.z};
}

inline vector3_t getFramePositionOffset(
  QSharedPointer<urdf::Model> urdfModel, const std::string & frame_A,
  const std::string & frame_B)
{

  return getFramePosition(urdfModel, frame_A) - getFramePosition(urdfModel, frame_B);
}

inline urdf::Rotation getFrameRotation(
  QSharedPointer<urdf::Model> urdfModel,
  const std::string & link_name)
{
  urdf::Rotation rotation(0, 0, 0, 1);  // identity quaternion
  urdf::LinkConstSharedPtr link = urdfModel->getLink(link_name);
  if (!link) {
    qWarning() << "Link not found:" << QString::fromStdString(link_name);
  } else {
    while (link && link->parent_joint) {
      const urdf::Pose & joint_pose = link->parent_joint->parent_to_joint_origin_transform;
      rotation = joint_pose.rotation * rotation;
      link = urdfModel->getLink(link->getParent()->name);
    }
  }
  return rotation;
}

inline vector3_t getFrameRotationOffset(
  QSharedPointer<urdf::Model> urdfModel,
  const std::string & frame_A,
  const std::string & frame_B)
{
  urdf::Rotation rot_A = getFrameRotation(urdfModel, frame_A);
  urdf::Rotation rot_B = getFrameRotation(urdfModel, frame_B);

  urdf::Rotation rot_offset = rot_B.GetInverse() * rot_A;

  double roll, pitch, yaw;
  rot_offset.getRPY(roll, pitch, yaw);

  return vector3_t{roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI};
}
#endif  // WAYWISER_DESCRIPTION_UTILS_HPP_
