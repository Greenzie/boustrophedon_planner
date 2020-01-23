#include <CGAL/create_offset_polygons_2.h>
#include "boustrophedon_server/outline_planner.h"

void OutlinePlanner::setParameters(OutlinePlanner::Parameters parameters)
{
  params_ = parameters;
}

bool OutlinePlanner::addToPath(Polygon polygon, const Point& robot_position, std::vector<NavPoint>& path,
                               Polygon& innermost_polygon)
{
  shiftPolygonToStartNearRobot(polygon, robot_position);

  if (polygon.is_clockwise_oriented())
  {
    polygon.reverse_orientation();
  }
  innermost_polygon = polygon;

  if (params_.skip_outlines)
  {
    std::cout << "Skipping outlining!" << std::endl;
    return true;
  }

  if (params_.repeat_boundary)
  {
    if (!addOutermostOutline(path, polygon))
    {
      return false;
    }
  }

  for (auto layer = 1; layer <= params_.outline_layers; layer++)
  {
    auto offset_distance = layer * params_.stripe_separation;
    if (!addInnerOutline(path, polygon, offset_distance, innermost_polygon))
    {
      return false;
    }
  }

  return true;
}

bool OutlinePlanner::addOutermostOutline(std::vector<NavPoint>& path, const Polygon& polygon)
{
  // Add outermost outline
  for (const auto& point : polygon.container())
  {
    path.emplace_back(PointType::Outline, point);
  }

  if (params_.outline_clockwise)
  {
    // reverse the outline path to go clockwise instead, so that
    // grass clippings go inside the boundary, not outside
    std::reverse(++path.begin(), path.end());
  }
  // Close polygon by repeating first point
  path.emplace_back(PointType::Outline, polygon.container().front());
  return true;
}

bool OutlinePlanner::addInnerOutline(std::vector<NavPoint>& path, const Polygon& polygon, const double& offset,
                                     Polygon& innermost_polygon)
{
  auto inner_polygons = CGAL::create_interior_skeleton_and_offset_polygons_2(offset, polygon);
  if (inner_polygons.empty())
  {
    return false;
  }
  for (const Point& p : inner_polygons.front()->container())
  {
    path.emplace_back(PointType::Outline, p);
  }

  if (params_.outline_clockwise)
  {
    // reverse the outline path to go clockwise instead, so that
    // grass clippings go inside the boundary, not outside
    std::reverse(path.end() - inner_polygons.size(), path.end());
  }

  // Close polygon by repeating first point
  path.emplace_back(PointType::Outline, inner_polygons.front()->container().front());
  innermost_polygon = *inner_polygons.front();
  return true;
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
