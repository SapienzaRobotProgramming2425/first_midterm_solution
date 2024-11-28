#pragma once

#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rp_commons/msg/landmark_array.hpp"
#include "world.h"
#include "world_item.h"

class LandmarkScanner : public WorldItem {
 public:
  LandmarkScanner(WorldItem& par, const std::string& ns,
                  rclcpp::Node::SharedPtr node, const Eigen::Isometry2f& pos,
                  float f, float range_max = 15);

  inline bool newLandmarks() const { return _new_landmarks; }

  void detectLandmarks();

  void publishLandmarks(rclcpp::Time time_now);

  void tick(float dt, rclcpp::Time time_now);

  void draw(Canvas& canvas) const;

  inline std::vector<int>& getIds() { return _ids; }

  std::vector<Eigen::Vector2f> getPoints() const;

 protected:
  World* _world;
  std::vector<int> _ids;
  std::vector<geometry_msgs::msg::Point> _points;

  float _elapsed_time = 0;
  bool _new_landmarks = false;
  rclcpp::Node::SharedPtr _node;
  std::string _namespace;
  float _frequency;
  float _period;
  tf2_ros::TransformBroadcaster _tf_broadcaster;
  rclcpp::Publisher<rp_commons::msg::LandmarkArray>::SharedPtr _landmark_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      _marker_pub;
  float _range_max;

  std_msgs::msg::ColorRGBA
      _marker_color;  // Attribute to store the marker color
};
