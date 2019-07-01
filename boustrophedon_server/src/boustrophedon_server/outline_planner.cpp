#include <CGAL/create_offset_polygons_2.h>
#include <boustrophedon_server/outline_planner.h>

void OutlinePlanner::setParameters(OutlinePlanner::Parameters parameters)
{
  params_ = parameters;
}

void OutlinePlanner::addToPath(Polygon polygon, const Point& robot_position, std::vector<NavPoint>& path,
                               Polygon& innermost_polygon)
{
  shiftPolygonToStartNearRobot(polygon, robot_position);

  if (polygon.is_clockwise_oriented())
  {
    polygon.reverse_orientation();
  }

  // Add outermost outline
  if (params_.outline_layers >= 1)
  {
    for (const auto& point : polygon.container())
    {
      path.emplace_back(PointType::Outline, point);
    }
    path.emplace_back(PointType::Outline, polygon.container().front());
  }
  innermost_polygon = polygon;
  for (auto layer = 1; layer < params_.outline_layers; layer++)
  {
    auto offset_distance = layer * params_.stripe_separation;
    auto inner_polygons = CGAL::create_interior_skeleton_and_offset_polygons_2(offset_distance, polygon);
    assert(!inner_polygons.empty());
    for (const Point& p : inner_polygons.front()->container())
    {
      path.emplace_back(PointType::Outline, p);
    }
    // Close polygon by repeating first point
    path.emplace_back(PointType::Outline, inner_polygons.front()->container().front());
    innermost_polygon = *inner_polygons.front();
  }
}

void OutlinePlanner::shiftPolygonToStartNearRobot(Polygon& polygon, const Point& robot_position)
{
  auto comp_dist_to_robot = [&robot_position](const Point& a, const Point& b) {
    auto comp = CGAL::compare_distance_to_point(robot_position, a, b);
    return (comp == CGAL::SMALLER);
  };

  std::vector<Point> points;
  points.insert(points.end(), polygon.vertices_begin(), polygon.vertices_end());

  auto closest_point_iter = std::min_element(points.begin(), points.end(), comp_dist_to_robot);

  std::rotate(points.begin(), closest_point_iter, points.end());

  polygon.clear();
  std::copy(points.begin(), points.end(), std::back_inserter(polygon));
}
