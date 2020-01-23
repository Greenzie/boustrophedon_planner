#include "boustrophedon_server/cellular_decomposition/cell.h"

using namespace bcd;

Cell::Cell(const Polygon& polygon)
{
  points.insert(points.begin(), polygon.vertices_begin(), polygon.vertices_end());
}

Polygon Cell::toPolygon() const
{
  Polygon polygon;
  polygon.insert(polygon.vertices_end(), points.begin(), points.end());
  return polygon;
}

void Cell::insertPointOnEdge(const Point& point)
{
  // use the function in cgal_utils
  Polygon polygon = toPolygon();
  insertPointAlongEdge(point, polygon);

  points = std::vector<Point>();
  points.insert(points.begin(), polygon.vertices_begin(), polygon.vertices_end());
}

bool Cell::isPointOnEdge(const Point& point)
{
  Polygon polygon = toPolygon();
  auto edge_iterator = polygon.edges_begin();
  for (; edge_iterator != polygon.edges_end(); edge_iterator++)
  {
    Segment edge = *edge_iterator;
    // check the point's distance to the edge - it might be off by a little bit because of floating point error
    if (definitelyLessThan(CGAL::squared_distance(point, edge), EPSILON, EPSILON))
    {
      // it's on the edge
      return true;
    }
  }
  // it wasn't on any of the edges
  return false;
}

// check if this cell is adjacent to other cell, aka shares two vertices
bool Cell::isAdjacentTo(const bcd::Cell& other)
{
  int same_points_count = 0;
  // loop through every point in this cell
  for (auto& point : points)
  {
    // loop through every point in other cell
    for (const auto& other_point : other.points)
    {
      if (approximatelyEqual(point, other_point, EPSILON))
      {
        same_points_count++;
      }
    }
  }

  // if there were two point shared between these cells, they're adjacent
  if (same_points_count == 2)
  {
    return true;
  }
  else if (same_points_count > 2)
  {
    std::cout << "somehow two cells were adjacent on " << same_points_count << " points!" << std::endl;
  }

  return false;
}

bool Cell::canMergeWith(const std::vector<Cell>& cells)
{
  // check if this cell can merge with the given list of cells
  // we check this by checking if any cell has points >= smallest x of this cell and points <= largest x of this cell
  // in addition, the cell has to be adjacent to at least one other cell

  // check for the case where there are no cells in the current list:
  if (cells.size() == 0)
  {
    return true;
  }

  // get the left and right vertex
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();  // min() is smallest positive value, lowest() is -max()

  for (const Point& point : points)
  {
    if (point.x() < min_x)
    {
      min_x = point.x();
    }
    if (point.x() > max_x)
    {
      max_x = point.x();
    }
  }

  bool has_adjacent_cell = false;
  // check to see if any points in the given vector violate the above rule
  for (const Cell& cell : cells)
  {
    // record to see if there are any points in this cell > min_x and and points < max_x
    bool has_larger_point = false;
    bool has_smaller_point = false;

    for (const Point& point : cell.points)
    {
      if (definitelyGreaterThan(point.x(), min_x, EPSILON))
      {
        has_larger_point = true;
      }
      if (definitelyLessThan(point.x(), max_x, EPSILON))
      {
        has_smaller_point = true;
      }
      // if both conditions have been violated, this cell shares the same x space as the given list, so it can't merge
      if (has_larger_point && has_smaller_point)
      {
        return false;
      }
    }

    // no conflict with this cell, but let's check to see if it's adjacent
    if (isAdjacentTo(cell))
    {
      has_adjacent_cell = true;
    }
  }

  // if we reached here then this cell should be fine to merge...assuming there was at least one adjacent cell!
  return has_adjacent_cell;
}

std::ostream& operator<<(std::ostream& out, const bcd::Cell& cell)
{
  for (const auto& vertex : cell.points)
  {
    out << vertex << ", ";
  }

  return out;
}
