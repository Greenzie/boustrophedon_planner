#ifndef SRC_BOUSTROPHEDONPLANNER_H
#define SRC_BOUSTROPHEDONPLANNER_H

#include "cgal_types.h"
#include "boustrophedon_types.h"

class StripingPlanner
{
public:
  struct Parameters
  {
    /** Center-to-center distance (in meters) between passes in the coverage path.
     *
     * Applies to both outline and fill portions.
     */
    double stripe_separation;
  };

  void setParameters(Parameters parameters);

  void addToPath(const Polygon& polygon, const Point& robot_position, std::vector<NavPoint>& path);

private:
  Parameters params_;

  /** Constructs the fill portion of the mowing path, which stripes back and
   * forth over the inner area left after the outline portion is complete.
   *
   */
  void fillPolygon(const Polygon& polygon, std::vector<NavPoint>& path);

  std::vector<Point> getIntersectionPoints(const Polygon& polygon, const Line& line);
};

#endif  // SRC_BOUSTROPHEDONPLANNER_H
