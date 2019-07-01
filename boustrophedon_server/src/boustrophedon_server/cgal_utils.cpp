#include <boustrophedon_server/cgal_utils.h>

AffineTransform preprocessPolygon(Polygon& polygon, const Point& robot_position, const double& striping_angle)
{
  AffineTransform transform(CGAL::IDENTITY);

  AffineTransform rotation_transform(CGAL::ROTATION, sin(striping_angle), cos(striping_angle));
  transform = transform * rotation_transform;

  auto max_x = CGAL::right_vertex_2(polygon.container().begin(), polygon.container().end())->x();
  if (robot_position.x() >= max_x)
  {
    AffineTransform flip_transform(-1, 0, 0, 1);
    transform = transform * flip_transform;
  }

  if (polygon.is_clockwise_oriented())
  {
    polygon.reverse_orientation();
  }

  transformPoints(polygon.vertices_begin(), polygon.vertices_end(), transform);

  return transform;
}

void postprocessPolygonAndPath(const AffineTransform& preprocess_transform, Polygon& polygon,
                               std::vector<NavPoint>& path)
{
  auto transform = preprocess_transform.inverse();

  transformPoints(polygon.vertices_begin(), polygon.vertices_end(), transform);

  transformNavPoints(path.begin(), path.end(), transform);
}

void transformNavPoints(const std::vector<NavPoint>::iterator& first, const std::vector<NavPoint>::iterator& last,
                        const CGAL::Aff_transformation_2<Kernel>& transform)
{
  for (auto current = first; current != last; current++)
  {
    current->point = transform(current->point);
  }
}
