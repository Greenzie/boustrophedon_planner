#include <CGAL/create_offset_polygons_2.h>
#include <CGAL/Quotient.h>
#include "boustrophedon_server/striping_planner.h"
#include "boustrophedon_server/cgal_utils.h"

void StripingPlanner::setParameters(StripingPlanner::Parameters parameters)
{
  params_ = parameters;
}

void StripingPlanner::addToPath(const Polygon& polygon, const Polygon& sub_polygon, Point& robot_position,
                                std::vector<NavPoint>& path)
{
  std::vector<NavPoint> new_path_section;

  fillPolygon(sub_polygon, new_path_section, robot_position);

  if (params_.travel_along_boundary)
  {
    std::vector<NavPoint> start_to_striping =
        getOutlinePathToPoint(polygon, robot_position, new_path_section.front().point);

    path.insert(path.end(), start_to_striping.begin(), start_to_striping.end());
    path.insert(path.end(), new_path_section.begin(), new_path_section.end());
  }
  else
  {
    path.insert(path.end(), new_path_section.begin(), new_path_section.end());
  }
  robot_position = new_path_section.back().point;
}

void StripingPlanner::addReturnToStart(const Polygon& polygon, const Point& start_position, const Point& robot_position,
                                       std::vector<NavPoint>& path)
{
  std::vector<NavPoint> striping_to_start = getOutlinePathToPoint(polygon, robot_position, start_position);
  path.insert(path.end(), striping_to_start.begin(), striping_to_start.end());
}

void StripingPlanner::fillPolygon(const Polygon& polygon, std::vector<NavPoint>& path, const Point& robot_position)
{
  auto left_vertex = *getLeftVertex(polygon);
  auto min_x = left_vertex.x();
  auto max_x = CGAL::right_vertex_2(polygon.container().begin(), polygon.container().end())->x();

  auto stripe_count = static_cast<int>(std::round(CGAL::to_double(max_x - min_x) / params_.stripe_separation)) + 1;

  StripingDirection stripe_dir = StripingDirection::STARTING;

  bool left_closest = isLeftClosest(polygon, robot_position, min_x, max_x);
  for (auto stripe_num = 0; stripe_num < stripe_count; stripe_num++)
  {
    double x;
    if (left_closest)
    {
      x = min_x + (stripe_num * params_.stripe_separation);
    }
    else
    {
      x = max_x - (stripe_num * params_.stripe_separation);
    }

    auto intersections = getIntersectionPoints(polygon, Line(Point(x, 0.0), Point(x, 1.0)));

    if (intersections.size() > 1)
    {
      // lambda for determining the first striping direction naively
      auto compare_current_point_distance = [&robot_position](const auto& a, const auto& b) {
        auto comp = CGAL::compare_distance_to_point(robot_position, a, b);
        return (comp == CGAL::SMALLER);
      };

      // lambda for sorting points based on current striping direction
      auto compare_striping_direction = [&stripe_dir](const auto& a, const auto& b) {
        if (stripe_dir == StripingDirection::UP)
        {
          return a.y() < b.y();
        }
        else if (stripe_dir == StripingDirection::DOWN)
        {
          return a.y() > b.y();
        }
        else
        {
          std::cout << "Comparing based on STARTING striping direction!" << std::endl;
          return true;
        }
      };

      if (stripe_dir == StripingDirection::STARTING)
      {
        // figure out the first striping direction using naive comparison of the two points
        std::sort(intersections.begin(), intersections.end(), compare_current_point_distance);
        // is the first point lower y than the second?
        stripe_dir =
            intersections.front().y() < intersections.back().y() ? StripingDirection::UP : StripingDirection::DOWN;
      }

      // add in intermediaries, not sorted
      addIntermediaryPoints(intersections);

      // remove any potential duplicates
      intersections.erase(std::unique(intersections.begin(), intersections.end()), intersections.end());

      // re-sort based on the striping direction
      std::sort(intersections.begin(), intersections.end(), compare_striping_direction);

      // here's where we add in the behavior to get from the previous stripe to this stripe
      if (params_.enable_half_y_turn)
      {
        // use the half-y-turn behavior
        addHalfYTurnPoints(path, intersections.front(), stripe_dir);

        // the half-y-turn places its own stripe start points, so we don't want to place this one naively, unless there
        // isn't anything in the path
        if (path.empty())
        {
          // we still need to place the start point in this case
          path.emplace_back(PointType::StripeStart, intersections.front());
        }
      }
      else
      {
        // else use the simply boundary_following behavior
        // follow the boundary to get to the next point
        addBoundaryFollowingPoints(path, intersections.front(), polygon);

        // place the points into the path
        path.emplace_back(PointType::StripeStart, intersections.front());
      }

      for (auto it = intersections.begin() + 1; it < intersections.end() - 1; it++)
      {
        path.emplace_back(PointType::StripeIntermediate, *it);
      }
      path.emplace_back(PointType::StripeEnd, intersections.back());

      // reverse the striping direction for the next stripe
      stripe_dir = stripe_dir == StripingDirection::UP ? StripingDirection::DOWN : StripingDirection::UP;
    }
  }
}

// adds in waypoints between the two waypoints provided in the intersections vector
void StripingPlanner::addIntermediaryPoints(std::vector<Point>& intersections)
{
  Point lower_point = intersections.front();
  Point upper_point = intersections.back();
  double x = lower_point.x();  // this is the x coordinate of all points on this striping line
  double y = lower_point.y();  // this is the base y coordinate

  int intermediary_count =
      static_cast<int>(std::trunc((upper_point.y() - lower_point.y()) / params_.intermediary_separation));

  // start the loop at 1. We don't want to repeat a point on the ends of the line segment.
  for (int i = 1; i < intermediary_count; i++)
  {
    double new_y = i * params_.intermediary_separation + y;
    intersections.emplace_back(x, new_y);
  }
}

void StripingPlanner::addBoundaryFollowingPoints(std::vector<NavPoint>& path, const Point& next_stripe_start,
                                                 Polygon polygon)
{
  // sanity check, don't add a boundary following point if there is no previous stripe
  if (path.empty())
  {
    return;
  }

  // first, insert the last point of the path (the end of last stripe) and the start of the next stripe to the polygon
  insertPointAlongEdge(path.back().point, polygon);
  insertPointAlongEdge(next_stripe_start, polygon);

  // we now have a polygon with all necessary points to find a path between the stripes
  std::vector<NavPoint> boundary_path = getOutlinePathToPoint(polygon, path.back().point, next_stripe_start);

  // check if there's actually a new path we have to follow, instead of just going straight from end to start
  if (boundary_path.size() > 2)
  {
    // if so, add it to the path. the end is already in there, and the start will be added later
    path.insert(path.end(), boundary_path.begin() + 1, boundary_path.end() - 1);

    std::cout << "inserted an extra boundary following path of size: " << boundary_path.size() - 2 << std::endl;
  }
}

void StripingPlanner::addHalfYTurnPoints(std::vector<NavPoint>& path, const Point& next_stripe_start,
                                         StripingDirection& stripe_dir)
{
  // sanity check, don't add a boundary following point if there is no previous stripe
  if (path.empty())
  {
    return;
  }

  Point arc_center_point;
  std::vector<NavPoint> arc;

  // we'll need to do different behaviors based on which direction we are striping in
  // the key differences are whether we are striping left-to-right or right-to-left, and whether we are going up or down
  if (stripe_dir == StripingDirection::DOWN && path.back().point.x() < next_stripe_start.x())
  {
    // we are going left to right
    // the current stripe is going down, so we need an arc from an UP stripe to a DOWN stripe

    // figure out which one of path.back() or next_stripe_start is higher y-coordinate
    if (definitelyGreaterThan(path.back().point.y(), next_stripe_start.y(), EPSILON))
    {
      // the previous stripe's end is higher than the current stripe's start.

      // the arc is centered around a point in the previous stripe
      arc_center_point =
          Point(path.back().point.x(), path.back().point.y() + params_.turn_start_offset - params_.stripe_separation);

      // For this case, we'll use a circular arc, starting from PI/2 and ending at 0 (left to right, higher to lower)
      arc = generateDiscretizedArc(arc_center_point, params_.stripe_separation, CGAL_PI / 2, 0.0,
                                   params_.points_per_turn);

      // the first point in the arc is the zero-turn point, make it end the stripe
      arc.front().type = PointType::StripeEnd;
      // copy the first point in the arc to start the next stripe
      arc.insert(arc.begin() + 1, NavPoint{ PointType::StripeStart, arc.front().point });
    }
    else
    {
      // the previous stripe's end is lower or equal(ish) to the current stripe's start.

      // the arc is centered around a point in the current stripe
      arc_center_point =
          Point(next_stripe_start.x(), next_stripe_start.y() + params_.turn_start_offset - params_.stripe_separation);

      // For this case, we'll use a circular arc, starting from PI and ending at PI/2 (left to right, lower to higher)
      arc = generateDiscretizedArc(arc_center_point, params_.stripe_separation, CGAL_PI, CGAL_PI / 2,
                                   params_.points_per_turn);

      // the last point in the arc is the zero-turn point, make it end the stripe
      arc.back().type = PointType::StripeEnd;
      // copy the last point in the arc to start the next stripe
      arc.insert(arc.end(), NavPoint{ PointType::StripeStart, arc.back().point });
    }
  }
  else if (stripe_dir == StripingDirection::UP && path.back().point.x() < next_stripe_start.x())
  {
    // we are going left to right
    // the current stripe is going up, so we need an arc from a DOWN stripe to an UP stripe.

    // figure out which one of path.back() or next_stripe_start is higher y-coordinate
    if (definitelyLessThan(path.back().point.y(), next_stripe_start.y(), EPSILON))
    {
      // the previous stripe's end is lower than the current stripe's start.

      // the arc is centered around a point in the previous stripe
      arc_center_point =
          Point(path.back().point.x(), path.back().point.y() - params_.turn_start_offset + params_.stripe_separation);

      // For this case, we'll use a circular arc, starting from 3PI/2 and ending at 2PI (left to right, lower to higher)
      arc = generateDiscretizedArc(arc_center_point, params_.stripe_separation, 3 * CGAL_PI / 2, CGAL_PI * 2,
                                   params_.points_per_turn);

      // the first point in the arc is the zero-turn point, make it end the stripe
      arc.front().type = PointType::StripeEnd;
      // copy the first point in the arc to start the next stripe
      arc.insert(arc.begin() + 1, NavPoint{ PointType::StripeStart, arc.front().point });
    }
    else
    {
      // the previous stripe's end is higher or equal(ish) to the current stripe's start.

      // the arc is centered around a point in the current stripe
      arc_center_point =
          Point(next_stripe_start.x(), next_stripe_start.y() - params_.turn_start_offset + params_.stripe_separation);

      // For this case, we'll use a circular arc, starting from PI and ending at 3PI/2 (left to right, higher to lower)
      arc = generateDiscretizedArc(arc_center_point, params_.stripe_separation, CGAL_PI, 3 * CGAL_PI / 2,
                                   params_.points_per_turn);

      // the last point in the arc is the zero-turn point, make it end the stripe
      arc.back().type = PointType::StripeEnd;
      // copy the last point in the arc to start the next stripe
      arc.insert(arc.end(), NavPoint{ PointType::StripeStart, arc.back().point });
    }
  }
  else if (stripe_dir == StripingDirection::DOWN && path.back().point.x() > next_stripe_start.x())
  {
    // we are going right to left
    // the current stripe is going down, so we need an arc from an UP stripe to a DOWN stripe

    // figure out which one of path.back() or next_stripe_start is higher y-coordinate
    if (definitelyGreaterThan(path.back().point.y(), next_stripe_start.y(), EPSILON))
    {
      // the previous stripe's end is higher than the current stripe's start.

      // the arc is centered around a point in the previous stripe
      arc_center_point =
          Point(path.back().point.x(), path.back().point.y() + params_.turn_start_offset - params_.stripe_separation);

      // For this case, we'll use a circular arc, starting from PI/2 and ending at PI (right to left, higher to lower)
      arc = generateDiscretizedArc(arc_center_point, params_.stripe_separation, CGAL_PI / 2, CGAL_PI,
                                   params_.points_per_turn);

      // the first point in the arc is the zero-turn point, make it end the stripe
      arc.front().type = PointType::StripeEnd;
      // copy the first point in the arc to start the next stripe
      arc.insert(arc.begin() + 1, NavPoint{ PointType::StripeStart, arc.front().point });
    }
    else
    {
      // the previous stripe's end is lower or equal(ish) to the current stripe's start.

      // the arc is centered around a point in the current stripe
      arc_center_point =
          Point(next_stripe_start.x(), next_stripe_start.y() + params_.turn_start_offset - params_.stripe_separation);

      // For this case, we'll use a circular arc, starting from 0 and ending at PI/2 (right to left, lower to higher)
      arc = generateDiscretizedArc(arc_center_point, params_.stripe_separation, 0.0, CGAL_PI / 2,
                                   params_.points_per_turn);

      // the last point in the arc is the zero-turn point, make it end the stripe
      arc.back().type = PointType::StripeEnd;
      // copy the last point in the arc to start the next stripe
      arc.insert(arc.end(), NavPoint{ PointType::StripeStart, arc.back().point });
    }
  }
  else if (stripe_dir == StripingDirection::UP && path.back().point.x() > next_stripe_start.x())
  {
    // we are going right to left
    // the current stripe is going up, so we need an arc from a DOWN stripe to an UP stripe.

    // figure out which one of path.back() or next_stripe_start is higher y-coordinate
    if (definitelyLessThan(path.back().point.y(), next_stripe_start.y(), EPSILON))
    {
      // the previous stripe's end is lower than the current stripe's start.

      // the arc is centered around a point in the previous stripe
      arc_center_point =
          Point(path.back().point.x(), path.back().point.y() - params_.turn_start_offset + params_.stripe_separation);

      // For this case, we'll use a circular arc, starting from 3PI/2 and ending at PI (right to left, lower to higher)
      arc = generateDiscretizedArc(arc_center_point, params_.stripe_separation, 3 * CGAL_PI / 2, CGAL_PI,
                                   params_.points_per_turn);

      // the first point in the arc is the zero-turn point, make it end the stripe
      arc.front().type = PointType::StripeEnd;
      // copy the first point in the arc to start the next stripe
      arc.insert(arc.begin() + 1, NavPoint{ PointType::StripeStart, arc.front().point });
    }
    else
    {
      // the previous stripe's end is higher or equal(ish) to the current stripe's start.

      // the arc is centered around a point in the current stripe
      arc_center_point =
          Point(next_stripe_start.x(), next_stripe_start.y() - params_.turn_start_offset + params_.stripe_separation);

      // For this case, we'll use a circular arc, starting from 2PI and ending at 3PI/2 (right to left, higher to lower)
      arc = generateDiscretizedArc(arc_center_point, params_.stripe_separation, CGAL_PI * 2, 3 * CGAL_PI / 2,
                                   params_.points_per_turn);

      // the last point in the arc is the zero-turn point, make it end the stripe
      arc.back().type = PointType::StripeEnd;
      // copy the last point in the arc to start the next stripe
      arc.insert(arc.end(), NavPoint{ PointType::StripeStart, arc.back().point });
    }
  }
  else
  {
    // we shouldn't be able to get here
    std::cout << "addHalfYTurnPoints(): stripe_dir was neither DOWN nor UP!" << std::endl;
    return;
  }
  // remove the previous stripe end point
  path.erase(path.end());

  // insert the arc points onto the end of the path
  path.insert(path.end(), arc.begin(), arc.end());
}

std::vector<NavPoint> StripingPlanner::generateDiscretizedArc(const Point& center_point, const float& radius,
                                                              const float& start_rad, const float& end_rad,
                                                              const int& num_points)
{
  std::vector<NavPoint> points;

  float radian_interval = (end_rad - start_rad) / (num_points - 1);

  for (int i = 0; i < num_points; i++)
  {
    float current_radian = start_rad + (radian_interval * i);
    Point point =
        Point(center_point.x() + (radius * cos(current_radian)), center_point.y() + (radius * sin(current_radian)));
    points.emplace_back(PointType::StripeIntermediate, point);
  }

  return points;
}

bool StripingPlanner::isLeftClosest(const Polygon& polygon, const Point& robot_position, double& min_x, double& max_x)
{
  // lambda for determining the first striping direction naively
  auto compare_current_point_distance = [&robot_position](const auto& a, const auto& b) {
    auto comp = CGAL::compare_distance_to_point(robot_position, a, b);
    return (comp == CGAL::SMALLER);
  };

  std::vector<Point> starting_points = getIntersectionPoints(polygon, Line(Point(min_x, 0.0), Point(min_x, 1.0)));
  std::vector<Point> right_points = getIntersectionPoints(polygon, Line(Point(max_x, 0.0), Point(max_x, 1.0)));
  starting_points.insert(starting_points.end(), right_points.begin(), right_points.end());
  std::sort(starting_points.begin(), starting_points.end(), compare_current_point_distance);

  // if the closest potential starting point is on the left, return true. If not, return false.
  return starting_points.front().x() == min_x;
}

std::vector<NavPoint> StripingPlanner::getOutlinePathToPoint(const Polygon& polygon, const Point& start_point,
                                                             const Point& end_point)
{
  NeighborSearch::Tree tree(polygon.vertices_begin(), polygon.vertices_end());  // Initialize the search tree

  NeighborSearch start_search(tree, start_point, 1);  // find the 1 nearest point to start_point
  Point polygon_start_point = start_search.begin()->first;

  NeighborSearch end_search(tree, end_point, 1);  // find the 1 nearest point to end_point
  Point polygon_end_point = end_search.begin()->first;

  // now we need to figure out the shortest path along the polygon from polygon_start_point to polygon_end_point
  // we only need to try two paths: one going clockwise and one going counter-clockwise
  std::vector<NavPoint> clockwise_path = getPolygonPath(polygon, polygon_start_point, polygon_end_point);
  Polygon reversed_polygon = polygon;
  reversed_polygon.reverse_orientation();
  std::vector<NavPoint> counter_clockwise_path =
      getPolygonPath(reversed_polygon, polygon_start_point, polygon_end_point);
  if (getPathLength(clockwise_path) < getPathLength(counter_clockwise_path))
  {
    return clockwise_path;
  }
  else
  {
    return counter_clockwise_path;
  }
}

std::vector<NavPoint> StripingPlanner::getPolygonPath(const Polygon& polygon, const Point& start_point,
                                                      const Point& end_point)
{
  // get a circulator starting at start_point
  Polygon::Vertex_const_circulator start_circulator = polygon.vertices_circulator();

  while (*start_circulator != start_point)  // loop until the circulator is pointing at the start
  {
    start_circulator++;
  }
  // start with the start_point
  std::vector<NavPoint> path;
  path.emplace_back(PointType::Outline, *start_circulator);

  // now we can add points to the path
  while (*start_circulator++ != end_point)
  {
    path.emplace_back(PointType::Outline, *start_circulator);
  }

  return path;
}

float StripingPlanner::getPathLength(const std::vector<NavPoint>& path)
{
  float squared_length = 0.0;
  for (auto it = path.begin(); it != --path.end(); it++)
  {
    Point point_1 = it->point;
    Point point_2 = (it + 1)->point;
    squared_length += CGAL::sqrt(CGAL::squared_distance(point_1, point_2));
  }
  return squared_length;
}
