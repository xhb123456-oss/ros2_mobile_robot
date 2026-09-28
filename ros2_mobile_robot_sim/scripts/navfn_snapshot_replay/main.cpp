// Offline algorithm probe: no ROS node, publisher or navigation action.
#include <nav2_navfn_planner/navfn.hpp>
#include <algorithm>
#include <iostream>
#include <vector>

int main()
{
  int w, h, cx, cy, sx, sy, gx, gy;
  if (!(std::cin >> w >> h >> cx >> cy >> sx >> sy >> gx >> gy) ||
    w < 3 || h < 3 || w > 10000 || h > 10000)
  {
    return 2;
  }
  auto inside = [w, h](int x, int y) {
      return x > 0 && x < w - 1 && y > 0 && y < h - 1;
    };
  if (!inside(cx, cy) || !inside(sx, sy) || !inside(gx, gy)) {
    std::cerr << "Cells outside interior grid; probe skipped\n";
    return 2;
  }
  std::vector<unsigned char> data(w * h);
  for (auto & cell : data) {
    int value;
    if (!(std::cin >> value) || value < 0 || value > 255) {return 2;}
    cell = static_cast<unsigned char>(value);
  }
  data[cy * w + cx] = 0;  // Costmap2D floor-index start clearing.
  for (bool astar : {false, true}) {
    nav2_navfn_planner::NavFn planner(w, h);
    planner.setCostmap(data.data(), true, false);
    int robot[2] = {sx, sy};
    int goal[2] = {gx, gy};
    // NavfnPlanner intentionally reverses these for potential propagation.
    planner.setStart(goal);
    planner.setGoal(robot);
    bool propagated = astar ? planner.calcNavFnAstar() : planner.calcNavFnDijkstra(true);
    float potential = planner.potarr[gy * w + gx];
    int length = 0;
    if (potential < POT_HIGH) {
      length = planner.calcPath(std::max(w, h) * 4);
    }
    std::cout << (astar ? "ASTAR" : "DIJKSTRA")
              << " propagation_return=" << propagated
              << " goal_potential=" << potential
              << " legal_potential=" << (potential < POT_HIGH)
              << " path_points=" << length << '\n';
  }
  return 0;
}
