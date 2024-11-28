#include "rp_simulator/landmark_scanner.h"

LandmarkScanner::LandmarkScanner(WorldItem& par, const std::string& ns,
                                 rclcpp::Node::SharedPtr node,
                                 const Eigen::Isometry2f& pos, float f,
                                 float range_max)
    : WorldItem(par, pos),
      _node(node),
      _namespace(ns),
      _frequency(f),
      _period(1. / f),
      _tf_broadcaster(node),
      _range_max(range_max) {
  // Initialize publisher for LandmarkArray messages
  // _landmark_pub =  // TODO_1 (topic_name must be = _namespace +
  // "/landmarks");

  // Initialize publisher for Marker messages
  // _marker_pub =  // TODO_2 (check rviz_configs directory for topic_name);

  // Get the world pointer
  WorldItem* w = this;
  while (w->_parent) {
    w = w->_parent;
  }
  _world = dynamic_cast<World*>(w);

  // Initialize random color
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.0, 1.0);
  _marker_color.r = dis(gen);
  _marker_color.g = dis(gen);
  _marker_color.b = dis(gen);
  _marker_color.a = 1.0;  // Alpha
}

void LandmarkScanner::detectLandmarks() {
  _ids.clear();
  _points.clear();

  // TODO_2: Get the global pose of the scanner

  // Iterate over all landmarks and add those within range to the list
  for (size_t i = 0; i < _world->landmarks().size(); ++i) {
    // TODO_3: Get the landmark position and calculate the distance to the
    // landmark
    float distance = 0.0;
    if (distance < _range_max) {
      _ids.push_back(_world->landmarkIds()[i]);
      // TODO_4: transform the landmark position to the scanner frame (inverse
      // of the global pose * landmark position)
      geometry_msgs::msg::Point point;
      // TODO_5: Set the x and y coordinates of the point
      _points.push_back(point);
    }
  }
}

void LandmarkScanner::publishLandmarks(rclcpp::Time time_now) {
  // Publish the landmark array
  rp_commons::msg::LandmarkArray msg;
  msg.header.frame_id = _namespace;
  msg.header.stamp = time_now;
  // TODO_6: Set the landmarks field of the message (check the
  // LandmarkArray.msg file)

  _landmark_pub->publish(msg);

  // Calculate pose in base link frame
  Eigen::Isometry2f pose_in_base_link = Eigen::Isometry2f::Identity();
  WorldItem* base_link = this;
  while (base_link->_parent->_parent) {
    pose_in_base_link = base_link->_pose_in_parent * pose_in_base_link;
    base_link = base_link->_parent;
  }

  // Publish the transform
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = time_now;
  transform_stamped.header.frame_id = base_link->_namespace;
  transform_stamped.child_frame_id = _namespace;
  transform_stamped.transform.translation.x =
      pose_in_base_link.translation().x();
  transform_stamped.transform.translation.y =
      pose_in_base_link.translation().y();
  transform_stamped.transform.translation.z = 0.0;
  transform_stamped.transform.rotation.z =
      sin(atan2(pose_in_base_link.linear()(1, 0),
                pose_in_base_link.linear()(0, 0)) /
          2);
  transform_stamped.transform.rotation.w =
      cos(atan2(pose_in_base_link.linear()(1, 0),
                pose_in_base_link.linear()(0, 0)) /
          2);
  _tf_broadcaster.sendTransform(transform_stamped);

  // Initialize the marker array
  visualization_msgs::msg::MarkerArray marker_array;

  // Add a delete all marker to clear the previous markers
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_marker);

  // Add a marker for each landmark
  for (size_t i = 0; i < _ids.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    // TODO_7: Set the marker properties
    // (https://docs.ros2.org/galactic/api/visualization_msgs/msg/Marker.html)

    marker_array.markers.push_back(marker);
  }
  _marker_pub->publish(marker_array);
}

void LandmarkScanner::tick(float dt, rclcpp::Time time_now) {
  // TODO_8: Complete the tick function (take inspiration from the
  // LaserScanner class)
}

void LandmarkScanner::draw(Canvas& canvas) const {
  Eigen::Isometry2f gp = globalPose();

  for (size_t i = 0; i < _ids.size(); ++i) {
    Eigen::Vector2f l_in_sensor =
        Eigen::Vector2f::Zero();  // TODO_9: Get the landmark position in the
                                  //  scanner frame;
    Eigen::Vector2f l_in_world =
        Eigen::Vector2f::Zero();  // TODO_10: Get the landmark position in the
                                  //  world frame;
    Eigen::Vector2i l_px =
        Eigen::Vector2i::Zero();  // TODO_11: Convert the landmark position to
                                  // pixel
                                  //  coordinates;
    drawSquareFilled(canvas, l_px, 5, 90);
  }
}

std::vector<Eigen::Vector2f> LandmarkScanner::getPoints() const {
  std::vector<Eigen::Vector2f> points;
  for (size_t i = 0; i < _points.size(); ++i) {
    Eigen::Vector2f point;
    point.x() = _points[i].x;
    point.y() = _points[i].y;
    points.push_back(point);
  }
  return points;
}