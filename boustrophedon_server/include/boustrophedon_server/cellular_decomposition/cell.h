#ifndef SRC_CELL_H
#define SRC_CELL_H

#include <boost/functional/hash.hpp>
#include <vector>
#include "../cgal_types.h"

namespace bcd
{
using Circulator = Polygon::Vertex_const_circulator;
enum class Event
{
  Start,
  Ceil,
  Floor,
  End,
  Open,
  Close,
};

struct Vertex
{
  Vertex() = default;
  Vertex(Event type, Circulator point);
  bool isAdjacentTo(const Vertex& other) const;
  Circulator nextPoint() const;
  Circulator previousPoint() const;

  Event type;
  Circulator point;
};

struct Cell
{
  Cell() = default;
  Cell(Vertex start);
  bool connectsToFloor(const Vertex& other) const;
  bool connectsToCeil(const Vertex& other) const;
  bool enclosesVector(const Vertex& other) const;

  const Vertex& currentCeil() const;
  const Vertex& currentFloor() const;

  const Point nextCeilPoint() const;
  const Point nextFloorPoint() const;

  std::vector<Point> toPoints() const;
  Polygon toPolygon() const;

  Vertex start;
  std::vector<Vertex> ceiling_points;
  std::vector<Vertex> floor_points;
  Vertex end;

  std::vector<Cell*> neighbors;
};
}  // namespace bcd

template <>
struct std::hash<const bcd::Cell*>
{
  std::size_t operator()(const bcd::Cell* const& cell) const
  {
    std::size_t seed = 17;
    boost::hash_combine(seed, std::make_pair(cell->start.point->x(), cell->start.point->y()));
    boost::hash_combine(seed, std::make_pair(cell->end.point->x(), cell->end.point->y()));
    return seed;
  }
};

std::ostream& operator<<(std::ostream& out, bcd::Event event);
std::ostream& operator<<(std::ostream& out, const bcd::Vertex& vertex);
std::ostream& operator<<(std::ostream& out, const bcd::Cell& cell);
#endif  // SRC_CELL_H
