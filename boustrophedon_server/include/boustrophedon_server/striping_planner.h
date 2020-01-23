#ifndef SRC_BOUSTROPHEDONPLANNER_H
#define SRC_BOUSTROPHEDONPLANNER_H

#include "boustrophedon_server/cgal_types.h"
#include "boustrophedon_server/cgal_utils.h"
#include "boustrophedon_server/boustrophedon_types.h"

class StripingPlanner
{
public:
  struct Parameters
  {
    // Center-to-center distance (in meters) between passes in the coverage path.
    // Applies to both outline and fill portions.
    double stripe_separation = 0.5;

    // Distance between intermediary waypoints along a stripe
    // Defaults to max double, so it doesn't stripe unless you have a REALLY big boundary
    double intermediary_separation = std::numeric_limits<double>::max();

    // Determines whether the path will travel along the boundary to get to the start and end points of the striping
    // path. Also makes the mower return to the starting point when finished.
    bool travel_along_boundary = true;

    // Determines whether a half-y-turn arc will be added to the path between each stripe, to smooth out turning.
    bool enable_half_y_turn = true;

    // Determines the amount of points in each arc of a half-y-turn.
    int points_per_turn = 20;

    // Determines the offset that a turn starts at -- setting how much room there is for a turn.
    // Larger moves the turn closer to the boundary.
    double turn_start_offset = 0.5;  // meters
  };

  void setParameters(Parameters parameters);

  void addToPath(const Polygon& polygon, const Polygon& sub_polygon, Point& robot_position,
                 std::vector<NavPoint>& path);
  void addReturnToStart(const Polygon& polygon, const Point& start_position, const Point& robot_position,
                        std::vector<NavPoint>& path);

private:
  enum class StripingDirection
  {
    STARTING,
    UP,
    DOWN
  };

  Parameters params_;

  // Constructs the fill portion of the mowing path, which stripes back and
  //  forth over the inner area left after the outline portion is complete.
  void fillPolygon(const Polygon& polygon, std::vector<NavPoint>& path, const Point& robot_position);

  void addIntermediaryPoints(std::vector<Point>& intersection);
  void addBoundaryFollowingPoints(std::vector<NavPoint>& path, const Point& next_stripe_start, Polygon polygon);
  void addHalfYTurnPoints(std::vector<NavPoint>& path, const Point& next_stripe_start, StripingDirection& stripe_dir);
  std::vector<NavPoint> generateDiscretizedArc(const Point& center_point, const float& radius, const float& start_rad,
                                               const float& end_rad, const int& num_points);

  bool isLeftClosest(const Polygon& polygon, const Point& robot_position, double& min_x, double& max_x);
  std::vector<NavPoint> getOutlinePathToPoint(const Polygon& polygon, const Point& start_point, const Point& end_point);
  static std::vector<NavPoint> getPolygonPath(const Polygon& polygon, const Point& start_point, const Point& end_point);
  float getPathLength(const std::vector<NavPoint>& path);
};

#endif  // SRC_BOUSTROPHEDONPLANNER_H
