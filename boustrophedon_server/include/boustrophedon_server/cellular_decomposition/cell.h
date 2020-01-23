#ifndef SRC_CELL_H
#define SRC_CELL_H

#include <vector>
#include "boustrophedon_server/cgal_utils.h"
#include "boustrophedon_server/cgal_types.h"
#include <CGAL/squared_distance_2.h>

namespace bcd
{
struct Cell
{
  Cell() = default;
  explicit Cell(const Polygon& polygon);

  void insertPointOnEdge(const Point& point);
  bool isPointOnEdge(const Point& point);
  bool isAdjacentTo(const Cell& other);
  bool canMergeWith(const std::vector<Cell>& cells);

  Polygon toPolygon() const;

  std::vector<Point> points;
  std::vector<Cell*> neighbors;
  bool visited = false;
};
}  // namespace bcd

std::ostream& operator<<(std::ostream& out, const bcd::Cell& cell);
#endif  // SRC_CELL_H
