#include <boustrophedon_server/cellular_decomposition/cell.h>

using namespace bcd;

Vertex::Vertex(Event type, Circulator point) : type{ type }, point{ point }
{
}

bool Vertex::isAdjacentTo(const Vertex& other) const
{
  return *(point + 1) == *other.point || *(point - 1) == *other.point;
}

Circulator Vertex::nextPoint() const
{
  return point + 1;
}

Circulator Vertex::previousPoint() const
{
  return point - 1;
}

std::ostream& operator<<(std::ostream& out, Event event)
{
  std::string string;
  switch (event)
  {
    case Event::Start:
      string = std::string("Start");
      break;
    case Event::End:
      string = std::string("End");
      break;
    case Event::Open:
      string = std::string("Open");
      break;
    case Event::Close:
      string = std::string("Close");
      break;
    case Event::Ceil:
      string = std::string("Ceil");
      break;
    case Event::Floor:
      string = std::string("Floor");
      break;
  }
  out << string;
  return out;
}

std::ostream& operator<<(std::ostream& out, const Vertex& vertex)
{
  out << *vertex.point << ": " << vertex.type;
  return out;
}

Cell::Cell(Vertex start) : start{ std::move(start) }
{
}

bool Cell::connectsToFloor(const Vertex& other) const
{
  bool connects_to_start_floor = floor_points.empty() && start.isAdjacentTo(other);
  bool connects_to_end_floor = !floor_points.empty() && floor_points.back().isAdjacentTo(other);
  return connects_to_start_floor || connects_to_end_floor;
}

bool Cell::connectsToCeil(const Vertex& other) const
{
  bool connects_to_start_ceil = ceiling_points.empty() && start.isAdjacentTo(other);
  bool connects_to_end_ceil = !ceiling_points.empty() && ceiling_points.back().isAdjacentTo(other);
  return connects_to_start_ceil || connects_to_end_ceil;
}

bool Cell::enclosesVector(const Vertex& other) const
{
  auto top_point = currentCeil().point;
  auto bottom_point = currentFloor().point;

  return bottom_point->y() < other.point->y() && other.point->y() < top_point->y();
}

const Vertex& Cell::currentCeil() const
{
  return ceiling_points.empty() ? start : ceiling_points.back();
}

const Vertex& Cell::currentFloor() const
{
  return floor_points.empty() ? start : floor_points.back();
}

const Point Cell::nextCeilPoint() const
{
  return *(currentCeil().point - 1);
}

const Point Cell::nextFloorPoint() const
{
  return *(currentFloor().point + 1);
}

std::vector<Point> Cell::toPoints() const
{
  std::vector<Point> points;
  points.reserve(2 + floor_points.size() + ceiling_points.size());
  points.emplace_back(*start.point);
  std::transform(floor_points.begin(), floor_points.end(), std::back_inserter(points),
                 [](const Vertex& vertex) { return *vertex.point; });
  points.emplace_back(*end.point);
  std::transform(ceiling_points.rbegin(), ceiling_points.rend(), std::back_inserter(points),
                 [](const Vertex& vertex) { return *vertex.point; });
  return points;
}

Polygon Cell::toPolygon() const
{
  Polygon polygon;
  auto points = toPoints();
  polygon.insert(polygon.vertices_end(), points.begin(), points.end());
  return polygon;
}

std::ostream& operator<<(std::ostream& out, const bcd::Cell& cell)
{
  out << cell.start;
  for (const auto& vertex : cell.floor_points)
  {
    out << ", " << vertex;
  }
  out << ", " << cell.end;
  for (auto vertex = cell.ceiling_points.rbegin(); vertex != cell.ceiling_points.rend(); vertex++)
  {
    out << ", " << *vertex;
  }

  out << std::endl << "neigbours: ";

  for (const auto& neighbour : cell.neighbors)
  {
    out << neighbour->start << " -> " << neighbour->end << std::endl;
  }
  out << std::endl;

  return out;
}
