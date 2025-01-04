#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <cmath>
#include <queue>
#include <vector>
#include <limits>
#include <unordered_map>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace robot
{

// ------------------- Supporting Structures -------------------

// 2D grid index
struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};

// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    void initPlanner(double smoothing_factor, int iterations);

    bool planPath(
      double start_world_x,
      double start_world_y,
      double goal_x,
      double goal_y,
      nav_msgs::msg::OccupancyGrid::SharedPtr map
    );

    bool doAStar(
      const CellIndex &start_idx,
      const CellIndex &goal_idx,
      std::vector<CellIndex> &out_path
    );

    // Reconstructs path by backtracking cameFrom
    void reconstructPath(
      const std::unordered_map<CellIndex, CellIndex, CellIndexHash> &cameFrom,
      const CellIndex &current,
      std::vector<CellIndex> &out_path
    );

    // 8-direction neighbors
    std::vector<CellIndex> getNeighbors8(const CellIndex &c);

    // Euclidean heuristic
    double euclideanHeuristic(const CellIndex &a, const CellIndex &b);

    // Step cost for orth vs diagonal moves
    double stepDistance(const CellIndex &a, const CellIndex &b);

    void lineOfSightSmoothing(std::vector<CellIndex> &path_cells);

    // Convert world coordinates to map indices
    bool poseToMap(double wx, double wy, CellIndex &out_idx);

    // Convert map cell to world coordinates
    void mapToPose(const CellIndex &idx, double &wx, double &wy);

    bool lineOfSight(const CellIndex &start, const CellIndex &end);

    bool isCellFree(int x, int y);

    nav_msgs::msg::Path::SharedPtr getPath() const;

  private:
    double smoothing_factor_;
    int iterations_;

    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    nav_msgs::msg::Path::SharedPtr path_;
};

}  

#endif  
