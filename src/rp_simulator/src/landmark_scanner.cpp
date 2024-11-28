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
  // TODO_1
  // Initialize publisher for LandmarkArray messages
  _landmark_pub = _node->create_publisher<rp_commons::msg::LandmarkArray>(
      _namespace + "/landmarks", 10);

  // TODO_2
  // Initialize publisher for Marker messages
  _marker_pub = _node->create_publisher<visualization_msgs::msg::MarkerArray>(
      _namespace + "/landmark_markers", 10);

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
  Eigen::Isometry2f gp = globalPose();
  Eigen::Vector2f p = gp.translation();

  // Iterate over all landmarks and add those within range to the list
  for (size_t i = 0; i < _world->landmarks().size(); ++i) {
    // TODO_3: Get the landmark position and calculate the distance to the
    // landmark
    Eigen::Vector2f l = _world->landmarks()[i];
    float distance = (l - p).norm();
    if (distance < _range_max) {
      _ids.push_back(_world->landmarkIds()[i]);
      // TODO_4: transform the landmark position to the scanner frame (inverse
      // of the global pose * landmark position)
      Eigen::Vector2d l_in_sensor = (gp.inverse() * l).cast<double>();
      geometry_msgs::msg::Point point;
      // TODO_5: Set the x and y coordinates of the point
      point.x = l_in_sensor.x();
      point.y = l_in_sensor.y();
      point.z = 0.0;
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
  msg.ids = _ids;
  msg.points = _points;

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
    // (https://docs.ros2.org/galactic/api/visualization_msgs/msg/Marker.html)marker.header.frame_id
    // = _namespace;
    marker.header.stamp = time_now;
    marker.ns = _namespace;
    marker.id = _ids[i];
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = _points[i];
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color = _marker_color;  // Use the randomly chosen color
    marker_array.markers.push_back(marker);
  }
  _marker_pub->publish(marker_array);
}

void LandmarkScanner::tick(float dt, rclcpp::Time time_now) {
  // TODO_8: Complete the tick function (take inspiration from the
  // LaserScanner class)
  _elapsed_time += dt;
  _new_landmarks = false;
  if (_elapsed_time < _period) return;

  detectLandmarks();
  _elapsed_time = 0;
  _new_landmarks = true;

  publishLandmarks(time_now);
}

void LandmarkScanner::draw(Canvas& canvas) const {
  Eigen::Isometry2f gp = globalPose();

  for (size_t i = 0; i < _ids.size(); ++i) {
    Eigen::Vector2f l_in_sensor = Eigen::Vector2f(
        _points[i].x,
        _points[i].y);  // TODO_9: Get the landmark position in the
                        //  scanner frame;
    Eigen::Vector2f l_in_world =
        gp * l_in_sensor;  // TODO_10: Get the landmark position in the
                           //  world frame;
    Eigen::Vector2i l_px =
        _grid_map->worldToGrid(l_in_world);  // TODO_11: Convert the landmark
                                             // position to pixel
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