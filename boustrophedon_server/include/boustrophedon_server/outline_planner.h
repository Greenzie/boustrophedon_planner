#ifndef SRC_OUTLINE_PLANNER_H
#define SRC_OUTLINE_PLANNER_H

#include "boustrophedon_server/cgal_types.h"
#include "boustrophedon_server/boustrophedon_types.h"

class OutlinePlanner
{
public:
  struct Parameters
  {
    /** If true, the path will start by retracing the boundary
     */
    bool repeat_boundary = false;
    /** If true, the path will follow a clockwise outline (including interior outlines). Else, counter-clockwise.
     */
    bool outline_clockwise = true;
    /** If true, the planner will skip any outline tracing and go straight to striping.
     */
    bool skip_outlines = false;

    /** The number of passes to make in the outline portion, where the first interior polygon = 1.
     *  CAUTION: Attempting to set more outline_layers than possible will cause the server to abort.
     */
    int outline_layers = 3;
    /** Center-to-center distance (in meters) between passes in the coverage path.
     *
     * Applies to both outline and fill portions.
     */
    double stripe_separation = 0.5;
  };

  void setParameters(Parameters parameters);

  bool addToPath(Polygon polygon, const Point& robot_position, std::vector<NavPoint>& path, Polygon& innermost_polygon);

private:
  Parameters params_;

  void shiftPolygonToStartNearRobot(Polygon& polygon, const Point& robot_position);
  bool addOutermostOutline(std::vector<NavPoint>& path, const Polygon& polygon);
  bool addInnerOutline(std::vector<NavPoint>& path, const Polygon& polygon, const double& offset,
                       Polygon& innermost_polygon);
};

#endif  // SRC_OUTLINE_PLANNER_H
