#include "qobject_node.hpp"
#include "moc_qobject_node.cpp"

QObjectNode::QObjectNode(
  const std::string & node_name, const rclcpp::NodeOptions & options, QObject * parent)
: QObject(parent), rclcpp::Node(node_name, options)
{
}
