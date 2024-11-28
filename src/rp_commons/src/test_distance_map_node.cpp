#include <memory>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rp_commons/distance_map.h"
#include "rp_commons/grid_map.h"

class DistanceMapNode : public rclcpp::Node {
 public:
  DistanceMapNode() : Node("distance_map_node") {
    _map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 10,
        std::bind(&DistanceMapNode::map_callback, this, std::placeholders::_1));
    _distance_map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "distance_map", 10);
  }

 private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Map size: [%dx%d] Creating distance map",
                msg->info.width, msg->info.height);
    _distance_map.loadFromOccupancyGrid(*msg);
    _grid_map.loadFromOccupancyGrid(*msg);

    RCLCPP_INFO(this->get_logger(), "Publishing distance map");
    publish_distance_map(*msg);
  }

  void publish_distance_map(const nav_msgs::msg::OccupancyGrid& map) {
    nav_msgs::msg::OccupancyGrid distance_map_msg;
    distance_map_msg.header = map.header;
    distance_map_msg.info = map.info;
    distance_map_msg.data.resize(_distance_map.size());

    std::vector<unsigned int> distances(_distance_map.size());

    for (unsigned int r = 0; r < _distance_map.rows(); ++r) {
      for (unsigned int c = 0; c < _distance_map.cols(); ++c) {
        // RCLCPP_INFO(this->get_logger(), "r: %d, c: %d v=%d", r, c,
        //             _grid_map.at(r, c));
        unsigned int i =
            _distance_map.cols() * (_distance_map.rows() - r - 1) + c;
        if (_distance_map.at(r, c).parent == nullptr or
            _grid_map.at(r, c) == GridMap::UNKNOWN) {
          distances[i] = 0;
          continue;
        }
        const auto dist2 = _distance_map.squaredDistance(
            _distance_map.at(r, c), *_distance_map.at(r, c).parent);
        distances[i] = dist2;
      }
    }
    const auto max_dist = *std::max_element(distances.begin(), distances.end());
    for (unsigned int i = 0; i < distances.size(); i++) {
      if (distances[i] == 0) {
        distance_map_msg.data[i] = 127;
      } else {
        distance_map_msg.data[i] =
            (unsigned char)((1 - (float)distances[i] / (float)max_dist) * 98);
      }
    }

    // const auto max_dist = *std::max_element(distances.begin(),
    // distances.end()); for (unsigned int i = 0; i < distances.size(); i++) {

    //   if (_grid_map.at(i / _grid_map.cols(), i % _grid_map.cols()) ==
    //       GridMap::UNKNOWN) {
    //     distance_map_msg.data[i] = 127;
    //     continue;
    //   }
    //   distance_map_msg.data[i] =
    //       (unsigned char)((1 - (float)distances[i] / (float)max_dist) * 98);
    // }

    _distance_map_pub->publish(distance_map_msg);
  }
  // void publish_distance_map() {
  //   nav_msgs::msg::OccupancyGrid distance_map_msg;
  //   distance_map_msg.header = _map_sub->get_message()->header;
  //   distance_map_msg.info = _map_sub->get_message()->info;
  //   distance_map_msg.data.resize(_distance_map.size());
  //   for (unsigned int i = 0; i < _distance_map.size(); i++) {
  //     distance_map_msg.data[i] = _distance_map[i].parent ? 100 : 0;
  //   }
  //   _distance_map_pub->publish(distance_map_msg);
  // }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _map_sub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _distance_map_pub;
  DistanceMap _distance_map;
  GridMap _grid_map;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistanceMapNode>());
  rclcpp::shutdown();
  return 0;
}