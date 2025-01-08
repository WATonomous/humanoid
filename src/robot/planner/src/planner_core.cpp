#include "planner_core.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) : path_(std::make_shared<nav_msgs::msg::Path>()), map_(std::make_shared<nav_msgs::msg::OccupancyGrid>()), logger_(logger) {}

void PlannerCore::initPlanner(double smoothing_factor, int iterations) {
  smoothing_factor_ = smoothing_factor;
  iterations_ = iterations;
}

bool PlannerCore::planPath(
  double start_world_x,
  double start_world_y,
  double goal_x, 
  double goal_y,
  nav_msgs::msg::OccupancyGrid::SharedPtr map
) {
  map_ = map;

  // Convert current goal to map indices
  CellIndex goal_idx;
  if (!poseToMap(goal_x,
                 goal_y,
                 goal_idx))
  {
    RCLCPP_WARN(logger_, "Goal is out of costmap bounds. Aborting.");
    return false;
  }

  CellIndex start_idx;
  if (!poseToMap(start_world_x, start_world_y, start_idx)) {
    RCLCPP_WARN(logger_, "Start is out of costmap bounds. Aborting.");
    return false;
  }

  RCLCPP_INFO(logger_,
              "Planning from odom start (%0.2f, %0.2f) => cell (%d, %d) to goal (%d, %d).",
              start_world_x, start_world_y,
              start_idx.x, start_idx.y,
              goal_idx.x, goal_idx.y);
  
  // Run A*
  std::vector<CellIndex> path_cells;
  bool success = doAStar(start_idx, goal_idx, path_cells);

  if (!success) {
    RCLCPP_WARN(logger_, "A* failed to find a path.");
    return false;
  }

  // Convert path cells to nav_msgs::Path
  if (path_->poses.size() > 0) {
    path_->poses.clear();
  }

  for (auto &cell : path_cells) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = map_->header;

    double wx, wy;
    mapToPose(cell, wx, wy);

    pose_stamped.pose.position.x = wx;
    pose_stamped.pose.position.y = wy;
    pose_stamped.pose.orientation.w = 1.0; // simple orientation

    path_->poses.push_back(pose_stamped);
  }

  return true;
}

bool PlannerCore::doAStar(
  const CellIndex &start_idx,
  const CellIndex &goal_idx,
  std::vector<CellIndex> &out_path
) {
  const int width  = map_->info.width;
  const int height = map_->info.height;

  // Data structures for A*
  std::unordered_map<CellIndex, double, CellIndexHash> gScore;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> cameFrom;
  std::unordered_map<CellIndex, double, CellIndexHash> fScore;

  auto setScore = [&](auto &storage, const CellIndex &idx, double val){
    storage[idx] = val;
  };
  auto getScore = [&](auto &storage, const CellIndex &idx){
    auto it = storage.find(idx);
    if (it == storage.end()) {
      return std::numeric_limits<double>::infinity();
    }
    return it->second;
  };

  // Helper to retrieve cell cost from costmap
  auto cellCost = [&](const CellIndex &idx) {
    if (idx.x < 0 || idx.x >= width || idx.y < 0 || idx.y >= height) {
      return 127; // out of bounds => treat as high cost
    }
    int map_index = idx.y * width + idx.x;
    int8_t val = map_->data[map_index];
    // Unknown => treat as high cost
    if (val < 0) {
      val = 100;
    }
    return static_cast<int>(val);
  };

  // Initialize start node
  setScore(gScore, start_idx, 0.0);
  double h_start = euclideanHeuristic(start_idx, goal_idx);
  setScore(fScore, start_idx, h_start);

  // Open set (min-heap by f_score)
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> openSet;
  openSet.push(AStarNode(start_idx, h_start));

  while (!openSet.empty()) {
    AStarNode current = openSet.top();
    openSet.pop();
    CellIndex cidx = current.index;

    // Goal check
    if (cidx == goal_idx) {
      reconstructPath(cameFrom, cidx, out_path);
      return true;
    }

    double current_g = getScore(gScore, cidx);

    // Check neighbors (8-way)
    auto neighbors = getNeighbors8(cidx);
    for (auto &nb : neighbors) {
      // Skip out of bounds quickly
      if (nb.x < 0 || nb.x >= width || nb.y < 0 || nb.y >= height) {
        continue;
      }

      // If cell cost is too high => treat as obstacle
      int cost_val = cellCost(nb);
      if (cost_val > 90) {
        continue;
      }

      // Step cost: 1.0 orth, sqrt(2) diag
      double step_cost = stepDistance(cidx, nb);
      // Add a penalty from costmap cell value (simple scale)
      double penalty = cost_val / 25.0;

      double tentative_g = current_g + step_cost + penalty;
      double old_g = getScore(gScore, nb);

      if (tentative_g < old_g) {
        setScore(gScore, nb, tentative_g);
        double h = euclideanHeuristic(nb, goal_idx);
        double f = tentative_g + h;
        setScore(fScore, nb, f);

        cameFrom[nb] = cidx;
        openSet.push(AStarNode(nb, f));
      }
    }
  }

  return false; // No path found
}

void PlannerCore::reconstructPath(
  const std::unordered_map<CellIndex, CellIndex, CellIndexHash> &cameFrom,
  const CellIndex &current,
  std::vector<CellIndex> &out_path)
{
  out_path.clear();
  CellIndex c = current;
  out_path.push_back(c);

  auto it = cameFrom.find(c);
  while (it != cameFrom.end()) {
    c = it->second;
    out_path.push_back(c);
    it = cameFrom.find(c);
  }
  std::reverse(out_path.begin(), out_path.end());
}

std::vector<CellIndex> PlannerCore::getNeighbors8(const CellIndex &c)
{
  std::vector<CellIndex> result;
  result.reserve(8);
  for (int dx = -1; dx <= 1; dx++) {
    for (int dy = -1; dy <= 1; dy++) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      result.push_back(CellIndex(c.x + dx, c.y + dy));
    }
  }
  return result;
}

double PlannerCore::euclideanHeuristic(const CellIndex &a, const CellIndex &b)
{
  double dx = static_cast<double>(a.x - b.x);
  double dy = static_cast<double>(a.y - b.y);
  return std::sqrt(dx * dx + dy * dy);
}

double PlannerCore::stepDistance(const CellIndex &a, const CellIndex &b)
{
  int dx = std::abs(a.x - b.x);
  int dy = std::abs(a.y - b.y);
  if (dx + dy == 2) {
    // diagonal
    return std::sqrt(2.0);
  } else {
    // orth
    return 1.0;
  }
}

bool PlannerCore::poseToMap(double wx, double wy, CellIndex &out_idx)
{
  double origin_x = map_->info.origin.position.x;
  double origin_y = map_->info.origin.position.y;
  double res      = map_->info.resolution;

  double mx = (wx - origin_x) / res;
  double my = (wy - origin_y) / res;

  int ix = static_cast<int>(std::floor(mx));
  int iy = static_cast<int>(std::floor(my));

  if (ix < 0 || ix >= static_cast<int>(map_->info.width) ||
      iy < 0 || iy >= static_cast<int>(map_->info.height))
  {
    return false;
  }

  out_idx.x = ix;
  out_idx.y = iy;
  return true;
}

void PlannerCore::mapToPose(const CellIndex &idx, double &wx, double &wy)
{
  double origin_x = map_->info.origin.position.x;
  double origin_y = map_->info.origin.position.y;
  double res      = map_->info.resolution;

  wx = origin_x + (idx.x + 0.5) * res;
  wy = origin_y + (idx.y + 0.5) * res;
}

nav_msgs::msg::Path::SharedPtr PlannerCore::getPath() const {
  return path_;
}

} 
