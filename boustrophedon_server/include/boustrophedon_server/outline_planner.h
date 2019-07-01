#ifndef SRC_OUTLINE_PLANNER_H
#define SRC_OUTLINE_PLANNER_H

#include "cgal_types.h"
#include "boustrophedon_types.h"

class OutlinePlanner
{
public:
  struct Parameters
  {
    /** The number of passes to make in the outline portion
     */
    int outline_layers = 3;
    /** Center-to-center distance (in meters) between passes in the coverage path.
     *
     * Applies to both outline and fill portions.
     */
    double stripe_separation = 0.5;
  };

  void setParameters(Parameters parameters);

  void addToPath(Polygon polygon, const Point& robot_position, std::vector<NavPoint>& path, Polygon& innermost_polygon);

private:
  Parameters params_;

  void shiftPolygonToStartNearRobot(Polygon& polygon, const Point& robot_position);
};

#endif  // SRC_OUTLINE_PLANNER_H
