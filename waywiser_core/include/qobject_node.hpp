#pragma once

#include <QObject>

#include "rclcpp/rclcpp.hpp"

class QObjectNode : public QObject, public rclcpp::Node
{
  Q_OBJECT

public:
  explicit QObjectNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(), QObject * parent = nullptr);

  virtual ~QObjectNode() = default;

signals:

protected:
};
