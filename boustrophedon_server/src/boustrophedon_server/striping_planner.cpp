#include <CGAL/create_offset_polygons_2.h>
#include <CGAL/Quotient.h>
#include <boustrophedon_server/striping_planner.h>
#include <boustrophedon_server/cgal_utils.h>

void StripingPlanner::setParameters(StripingPlanner::Parameters parameters)
{
  params_ = parameters;
}

void StripingPlanner::addToPath(const Polygon& polygon, const Point& robot_position, std::vector<NavPoint>& path)
{
  std::vector<NavPoint> new_path_section;

  const auto& last_point = path.empty() ? robot_position : path.back().point;

  double left_vertex_dist = std::abs(last_point.x() - polygon.left_vertex()->x());
  double right_vertex_dist = std::abs(last_point.x() - polygon.right_vertex()->x());
  bool left_closer = left_vertex_dist < right_vertex_dist;

  if (!left_closer)
  {
    AffineTransform flip_transform(-1, 0, 0, 1);
    transformPoints(polygon.vertices_begin(), polygon.vertices_end(), flip_transform);
  }

  fillPolygon(polygon, new_path_section);

  if (!left_closer)
  {
    AffineTransform flip_transform(-1, 0, 0, 1);
    transformNavPoints(new_path_section.begin(), new_path_section.end(), flip_transform);
  }

  path.insert(path.end(), new_path_section.begin(), new_path_section.end());
}

void StripingPlanner::fillPolygon(const Polygon& polygon, std::vector<NavPoint>& path)
{
  auto left_vertex = *CGAL::left_vertex_2(polygon.container().begin(), polygon.container().end());
  auto min_x = left_vertex.x();
  auto max_x = CGAL::right_vertex_2(polygon.container().begin(), polygon.container().end())->x();

  auto stripe_count = static_cast<int>(std::round(CGAL::to_double(max_x - min_x) / params_.stripe_separation)) + 1;

  auto current_point = left_vertex;

  for (auto stripe_num = 0; stripe_num < stripe_count; stripe_num++)
  {
    auto x = (stripe_num * params_.stripe_separation) + min_x;

    auto intersections = getIntersectionPoints(polygon, Line(Point(x, 0.0), Point(x, 1.0)));

    if (intersections.size() > 1)
    {
      auto comp_dist_to_curr_point = [current_point](const auto& a, const auto& b) {
        auto comp = CGAL::compare_distance_to_point(current_point, a, b);
        return (comp == CGAL::SMALLER);
      };

      std::sort(intersections.begin(), intersections.end(), comp_dist_to_curr_point);

      intersections.erase(std::unique(intersections.begin(), intersections.end()), intersections.end());

      path.emplace_back(PointType::StripeStart, intersections.front());
      path.emplace_back(PointType::StripeEnd, intersections.back());
      current_point = intersections.back();
    }
  }
}

std::vector<Point> StripingPlanner::getIntersectionPoints(const Polygon& polygon, const Line& line)
{
  std::vector<Point> intersections;

  for (auto edge_iterator = polygon.edges_begin(); edge_iterator != polygon.edges_end(); edge_iterator++)
  {
    Segment edge = *edge_iterator;

    auto result = CGAL::intersection(edge, line);

    if (result)
    {
      if (auto seg = boost::get<Segment>(&*result))
      {
        intersections.push_back(seg->source());
        intersections.push_back(seg->target());
      }
      else
      {
        auto pt = boost::get<Point>(&*result);
        intersections.push_back(*pt);
      }
    }
  }
  return intersections;
}
