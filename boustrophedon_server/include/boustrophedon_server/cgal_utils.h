#ifndef SRC_CGAL_UTILS_H
#define SRC_CGAL_UTILS_H

#include "cgal_types.h"
#include "boustrophedon_types.h"

AffineTransform preprocessPolygon(Polygon& polygon, const Point& robot_position, const double& striping_angle);

void postprocessPolygonAndPath(const AffineTransform& preprocess_transform, Polygon& polygon,
                               std::vector<NavPoint>& path);

template <typename Iterator_t>
static void transformPoints(const Iterator_t& first, const Iterator_t& last,
                            const CGAL::Aff_transformation_2<Kernel>& transform)
{
  for (auto current = first; current != last; current++)
  {
    *current = transform(*current);
  }
}

void transformNavPoints(const std::vector<NavPoint>::iterator& first, const std::vector<NavPoint>::iterator& last,
                        const CGAL::Aff_transformation_2<Kernel>& transform);

#endif  // SRC_CGAL_UTILS_H
