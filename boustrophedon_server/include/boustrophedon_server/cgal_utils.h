#ifndef SRC_CGAL_UTILS_H
#define SRC_CGAL_UTILS_H

#include "boustrophedon_server/cgal_types.h"
#include "boustrophedon_server/boustrophedon_types.h"

#define EPSILON 0.0001

AffineTransform preprocessPolygon(Polygon& polygon, Point& robot_position, const double& striping_angle);

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

Polygon::Vertex_const_circulator getLeftVertex(const Polygon& polygon);

std::vector<Point> getIntersectionPoints(const Polygon& polygon, const Line& line);

std::vector<Point>::const_iterator findPointInVector(const Point& point, const std::vector<Point>& vector);

std::vector<Point>::const_iterator findPointInVector(const Point& point, const std::vector<Point>::iterator& begin,
                                                     const std::vector<Point>::iterator& end);

void insertPointAlongEdge(const Point& point, Polygon& polygon);

Polygon mergePolygons(const Polygon& poly_1, const Polygon& poly_2);

bool approximatelyEqual(Point a, Point b, double epsilon);

bool approximatelyEqual(double a, double b, double epsilon);

bool definitelyGreaterThan(double a, double b, double epsilon);

bool definitelyLessThan(double a, double b, double epsilon);

#endif  // SRC_CGAL_UTILS_H
