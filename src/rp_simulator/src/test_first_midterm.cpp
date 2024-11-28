#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <filesystem>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "rp_commons/grid_map.h"
#include "rp_simulator/landmark_scanner.h"
#include "rp_simulator/laser_scanner.h"
#include "rp_simulator/unicycle.h"
#include "rp_simulator/world.h"
#include "rp_simulator/world_item.h"

Eigen::Isometry2f fromCoefficients(float tx, float ty, float alpha) {
  Eigen::Isometry2f iso;
  iso.setIdentity();
  iso.translation() << tx, ty;
  iso.linear() = Eigen::Rotation2Df(alpha).matrix();
  return iso;
}

std::vector<int> ids_true = {42, 69, 420};
std::vector<int> ids_test;

TEST(rp_simulator_test, landmarks_test) {
  // Assert the two sets of IDs are equal
  ASSERT_EQ(ids_true.size(), ids_test.size());

  // Check if all IDs in the true set are in the test set
  for (size_t i = 0; i < ids_true.size(); ++i) {
    for (size_t j = 0; j < ids_test.size(); ++j) {
      if (ids_true[i] == ids_test[j]) {
        break;
      }
      if (j == ids_test.size() - 1) {
        FAIL() << "ID " << ids_true[i] << " not found in test set";
      }
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("simulator_node");

  float resolution = 0.1f;
  bool publish_robots_gt = true;

  // Load the grid map from the image file
  GridMap grid_map;
  grid_map._origin = Eigen::Vector2f(0.0, 0.0);
  grid_map._resolution = resolution;
  grid_map._rows = 2138;
  grid_map._cols = 987;
  grid_map.resize(grid_map._rows, grid_map._cols);

  // Create the WorldNode, which will manage ROS 2 interactions and simulation
  auto world_object = std::make_shared<World>(grid_map, node, 0.1f);

  std::vector<std::shared_ptr<UnicyclePlatform>> robots;
  std::vector<std::shared_ptr<LandmarkScanner>> landmark_scanners;

  // Create robot
  std::string robot_name = "robot_1";
  std::vector<float> robot_in_world = {18, -25, 0};
  float robot_radius = 0.5f;

  auto robot = std::make_shared<UnicyclePlatform>(
      *world_object, robot_name, node,
      fromCoefficients(robot_in_world[0], robot_in_world[1], robot_in_world[2]),
      publish_robots_gt);
  robot->_radius = robot_radius;
  robots.push_back(robot);

  // Create landmark scanner
  std::string landmark_scanner_name = "landmark_scanner_1";
  std::vector<float> landmark_scanner_in_robot = {0, 0, 0};
  float landmark_scanner_radius = 0.5f;
  float landmark_scanner_freq = 1000.0f;

  auto landmark_scanner = std::make_shared<LandmarkScanner>(
      *robot, landmark_scanner_name, node,
      fromCoefficients(landmark_scanner_in_robot[0],
                       landmark_scanner_in_robot[1],
                       landmark_scanner_in_robot[2]),
      landmark_scanner_freq);
  landmark_scanner->_radius = landmark_scanner_radius;
  landmark_scanners.push_back(landmark_scanner);

  // Add landmarks to the world
  std::vector<std::pair<int, std::vector<float>>> landmarks = {
      {42, {18, -25}},
      {69, {18, -26}},
      {420, {18, -27}},
      {314, {100, -100}},
      {271, {50, 50}}};

  for (const auto& landmark : landmarks) {
    world_object->addLandmark(landmark.first, landmark.second[0],
                              landmark.second[1]);
  }

  // Tick the world for initializing the landmarks
  world_object->tick(0.1, node->now());

  // Get the IDs of the landmarks detected by the scanner
  LandmarkScanner* landmark_scanner_1 = landmark_scanners[0].get();
  ids_test = landmark_scanner_1->getIds();

  return RUN_ALL_TESTS();
}
