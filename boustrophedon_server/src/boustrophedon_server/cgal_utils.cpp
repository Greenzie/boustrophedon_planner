#include "boustrophedon_server/cgal_utils.h"

AffineTransform preprocessPolygon(Polygon& polygon, Point& robot_position, const double& striping_angle)
{
  AffineTransform transform(CGAL::IDENTITY);

  AffineTransform rotation_transform(CGAL::ROTATION, sin(striping_angle), cos(striping_angle));
  transform = transform * rotation_transform;

  if (polygon.is_clockwise_oriented())
  {
    polygon.reverse_orientation();
  }

  transformPoints(polygon.vertices_begin(), polygon.vertices_end(), transform);
  robot_position = transform(robot_position);

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

// returns a circulator pointing to the leftmost point in the polygon
// Key difference between this and CGAL::left_vertex_2 is that this is inclusive of ALL points
Polygon::Vertex_const_circulator getLeftVertex(const Polygon& polygon)
{
  auto temp_vertex = polygon.vertices_circulator();
  auto left_vertex = temp_vertex;
  do
  {
    temp_vertex++;
    // if next vertex is more LEFT than previous left vertex OR
    //  (if next vertex is equally LEFT as previous left vertex AND next vertex has smaller Y)
    if (temp_vertex->x() < left_vertex->x() ||
        ((temp_vertex->x() == left_vertex->x()) && (temp_vertex->y() < left_vertex->y())))
    {
      left_vertex = temp_vertex;
    }
  } while (temp_vertex != --polygon.vertices_circulator());
  return left_vertex;
}

std::vector<Point> getIntersectionPoints(const Polygon& polygon, const Line& line)
{
  std::vector<Point> intersections;

  for (auto edge_iterator = polygon.edges_begin(); edge_iterator != polygon.edges_end(); edge_iterator++)
  {
    Segment edge = *edge_iterator;
    auto result = CGAL::intersection(edge, line);

    if (result)
    {
      auto edge_y_max = std::max(edge.source().y(), edge.target().y());
      auto edge_y_min = std::min(edge.source().y(), edge.target().y());

      if (auto seg = boost::get<Segment>(&*result))
      {
        // do a sanity check to make sure the intersections are in the range they should be
        if ((definitelyLessThan(seg->source().y(), edge_y_max + 0.1, EPSILON)) &&
            (definitelyGreaterThan(seg->source().y(), edge_y_min - 0.1, EPSILON)) &&
            (definitelyLessThan(seg->target().y(), edge_y_max + 0.1, EPSILON)) &&
            (definitelyGreaterThan(seg->target().y(), edge_y_min - 0.1, EPSILON)))
        {
          intersections.push_back(seg->source());
          intersections.push_back(seg->target());
        }
        else
        {
          std::cout << "An intersection segment was significantly outside of the polygon! Removing..." << std::endl;
        }
      }
      else
      {
        auto pt = boost::get<Point>(&*result);

        // do a sanity check to make sure the point is in the range it should be
        if ((definitelyLessThan(pt->y(), edge_y_max + 0.1, EPSILON)) &&
            (definitelyGreaterThan(pt->y(), edge_y_min - 0.1, EPSILON)))
        {
          intersections.push_back(*pt);
        }
        else
        {
          std::cout << "An intersection point was significantly outside of the polygon! Removing..." << std::endl;
        }
      }
    }
  }
  return intersections;
}

std::vector<Point>::const_iterator findPointInVector(const Point& point, const std::vector<Point>& vector)
{
  // shouldn't use std::find because we need to use approximatelyEqual() because floating point math
  auto it = vector.begin();
  for (; it != vector.end(); it++)
  {
    // ensure the x and y is within tolerances. If we found it, then break out of the loop.
    if (approximatelyEqual(*it, point, EPSILON))
    {
      break;
    }
  }
  return it;
}

// mainly to be used by CGAL Polygons, which don't have accessors to their Point vectors, but rather have iterators.
std::vector<Point>::const_iterator findPointInVector(const Point& point, const std::vector<Point>::iterator& begin,
                                                     const std::vector<Point>::iterator& end)
{
  // shouldn't use std::find because we need to use approximatelyEqual() because floating point math
  auto it = begin;
  for (; it != end; it++)
  {
    // ensure the x and y is within tolerances. If we found it, then break out of the loop.
    if (approximatelyEqual(*it, point, EPSILON))
    {
      break;
    }
  }
  return it;
}

void insertPointAlongEdge(const Point& point, Polygon& polygon)
{
  // check to see if point is a vertex
  if (findPointInVector(point, polygon.vertices_begin(), polygon.vertices_end()) != polygon.vertices_end())
  {
    // it was a vertex, we don't have to do anything
    return;
  }

  // the point intersects an edge, we need to find that edge
  auto edge_iterator = polygon.edges_begin();
  for (; edge_iterator != polygon.edges_end(); edge_iterator++)
  {
    Segment edge = *edge_iterator;
    // check the point's distance to the edge - it might be off by a little bit because of floating point error
    if (definitelyLessThan(CGAL::squared_distance(point, edge), EPSILON, EPSILON))
    {
      // we found the edge!
      break;
    }
  }

  // check to see if we found the edge
  if (edge_iterator == polygon.edges_end())
  {
    std::cout << "couldn't find the edge that point " << point.x() << " , " << point.y() << " lies on!" << std::endl;
  }

  // now, we need to modify the polygon such that we have two edges from source -> point and point -> target
  // we can do this by simply inserting point after the edge's source
  for (auto point_iterator = polygon.vertices_begin(); point_iterator != polygon.vertices_end(); point_iterator++)
  {
    if (approximatelyEqual(*point_iterator, edge_iterator->source(), EPSILON))
    {
      // we found the source point in the points list!
      // we'll insert the point -- this will invalidate the iterator but that's ok because we're done with it
      point_iterator++;
      polygon.insert(point_iterator, point);
      return;
    }
  }

  // weird things happened if we got here
  std::cout << "didn't find a point in polygon that matched source: " << edge_iterator->source().x() << " , "
            << edge_iterator->source().y() << std::endl;
}

/**
 * Merges the two given polygons together, and returns a new polygon that is the two combined polygons
 * Two polygons can be merged together if they share two adjacent vertices
 * @param poly_1
 * @param poly_2
 * @returns the new merged polygon, or the other polygon if one polygon is empty
 */
Polygon mergePolygons(const Polygon& poly_1, const Polygon& poly_2)
{
  // given two polygons that are neighbors, merge them together by creating a new polygon that uses both polygon's
  // points and neighbors
  Polygon new_poly = Polygon();

  // first, check to se eif either polygon is empty
  if (poly_1.size() == 0)
  {
    new_poly = poly_2;
    return new_poly;
  }
  else if (poly_2.size() == 0)
  {
    new_poly = poly_1;
    return new_poly;
  }

  // we need to find the shared vertices
  Polygon::Vertex_const_circulator poly_1_circulator = poly_1.vertices_circulator();
  Polygon::Vertex_const_circulator poly_1_ending;
  // loop to find a particular shared vertex
  do
  {
    // check if this point in cell 1 is also in cell 2
    if (findPointInVector(*poly_1_circulator, poly_2.vertices_begin(), poly_2.vertices_end()) != poly_2.vertices_end())
    {
      // if it is, then it's either the upper or lower intersection. We want whichever comes just after the other, ccw
      if (findPointInVector(*(poly_1_circulator + 1), poly_2.vertices_begin(), poly_2.vertices_end()) !=
          poly_2.vertices_end())
      {
        // the next vertex is the shared vertex immediately after the other shared vertex!
        poly_1_circulator++;
        poly_1_ending = poly_1_circulator;
        break;
      }
    }
    poly_1_circulator++;

  } while (poly_1_circulator != poly_1.vertices_circulator());

  // add all of the points from poly_1_circulator to poly_1_circulator - 1 , inclusive, to new_poly
  // this gives us all the points in polygon 1, ending on the leading edge of the boundary between the two polygons
  do
  {
    new_poly.push_back(*poly_1_circulator);
    poly_1_circulator++;
  } while (!approximatelyEqual(*poly_1_circulator, *poly_1_ending, EPSILON));

  // we've added all of the points in polygon 1

  Polygon::Vertex_const_circulator poly_2_circulator = poly_2.vertices_circulator();
  // we need to get poly_2_circulator to point to the same point that we just added to new_poly
  while (!approximatelyEqual(*poly_2_circulator, *(poly_1_circulator - 1), EPSILON))
  {
    poly_2_circulator++;
  }

  // now add all the points from poly_2_circulator to poly_1_circulator + 1, exclusive, to new_poly
  poly_2_circulator++;
  while (!approximatelyEqual(*poly_2_circulator, *poly_1_circulator, EPSILON))
  {
    new_poly.push_back(*poly_2_circulator);
    poly_2_circulator++;
  }

  // this is the complete new poly!
  return new_poly;
}

bool approximatelyEqual(const Point a, const Point b, const double epsilon)
{
  return approximatelyEqual(a.x(), b.x(), epsilon) && approximatelyEqual(a.y(), b.y(), epsilon);
}

bool approximatelyEqual(const double a, const double b, const double epsilon)
{
  return abs(a - b) <= ((abs(a) < abs(b) ? abs(b) : abs(a)) * epsilon);
}

bool definitelyGreaterThan(const double a, const double b, const double epsilon)
{
  return (a - b) > ((abs(a) < abs(b) ? abs(b) : abs(a)) * epsilon);
}

bool definitelyLessThan(const double a, const double b, const double epsilon)
{
  return (b - a) > ((abs(a) < abs(b) ? abs(b) : abs(a)) * epsilon);
}