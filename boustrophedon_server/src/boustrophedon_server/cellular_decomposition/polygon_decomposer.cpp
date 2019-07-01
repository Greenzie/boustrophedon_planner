#include <unordered_set>
#include <boustrophedon_server/cellular_decomposition/polygon_decomposer.h>

using namespace bcd;

void PolygonDecomposer::decompose(Polygon& polygon)
{
  createVertices(polygon);
  createCells();
}

std::vector<Polygon> PolygonDecomposer::getSubPolygons(const Point& position) const
{
  Cell* closest_cell = getClosestCell(position);
  std::vector<const Cell*> cells = visitCells(closest_cell);

  std::vector<Polygon> polygons = toPolygons(cells);
  return polygons;
}

void PolygonDecomposer::createVertices(Polygon& polygon)
{
  insertOpenCloseVertices(polygon);

  auto left_vertex = CGAL::left_vertex_2(polygon.vertices_circulator(), --polygon.vertices_circulator());
  Event current_event = Event::Start;

  vertices_.emplace_back(current_event, left_vertex);

  auto vertex = left_vertex + 1;

  while (vertex != left_vertex)
  {
    current_event = getEvent(vertex, current_event);
    vertices_.emplace_back(current_event, vertex);
    vertex++;
  }
}

void PolygonDecomposer::insertOpenCloseVertices(Polygon& polygon)
{
  auto vertex = CGAL::left_vertex_2(polygon.vertices_circulator(), --polygon.vertices_circulator());
  Event current_event = Event::Start;

  // TODO(Oswin): Use something more efficient. I CBF to write a hash function for now.
  std::vector<Point> visited_vertices;

  auto left_vertex = *vertex;
  vertex++;
  while (*vertex != left_vertex)
  {
    current_event = getEvent(vertex, current_event);
    if ((current_event == Event::Open || current_event == Event::Close) &&
        std::find(visited_vertices.begin(), visited_vertices.end(), *vertex) == visited_vertices.end())
    {
      visited_vertices.emplace_back(*vertex);
      sliceAndInsertIntoPolygon(polygon, *vertex);
      // Start from beginning lmao because iterators are (probably) invalidated.
      vertex = CGAL::left_vertex_2(polygon.vertices_circulator(), --polygon.vertices_circulator());
      current_event = Event::Start;
    }

    vertex++;
  }
}

void PolygonDecomposer::sliceAndInsertIntoPolygon(Polygon& polygon, const Point& point)
{
  auto vertical_line = Line(Point(point.x(), 0), Point(point.x(), 1));

  for (auto it = polygon.vertices_begin(); it != polygon.vertices_end() - 1; it++)
  {
    auto polygon_edge = Segment(*it, *(it + 1));

    auto result = CGAL::intersection(polygon_edge, vertical_line);

    if (result)
    {
      // Only one point => didn't intersect a vertical edge on the polygon.
      // Insert into the polygon between it and it+1
      if (auto found_point = boost::get<Point>(&*result))
      {
        if (*found_point != polygon_edge.source() && *found_point != polygon_edge.target())
        {
          // Fuck pointer invalidation
          it = polygon.insert((it + 1), *found_point) + 1;
        }
      }
    }
  }

  auto polygon_edge = Segment(*polygon.vertices_circulator(), *(polygon.vertices_circulator() - 1));
  // Check the last edge (last point -> first point) as well
  auto result = CGAL::intersection(polygon_edge, vertical_line);

  if (result)
  {
    // Only one point => didn't intersect a vertical edge on the polygon.
    // Insert into the polygon between it and it+1
    if (auto found_point = boost::get<Point>(&*result))
    {
      if (*found_point != polygon_edge.source() && *found_point != polygon_edge.target())
      {
        polygon.push_back(*found_point);
      }
    }
  }
}

void PolygonDecomposer::createCells()
{
  sortVertices();

  std::vector<std::unique_ptr<Cell>> current_cells;
  current_cells.emplace_back(std::make_unique<Cell>(vertices_.front()));

  for (int i = 1; i < vertices_.size(); i++)
  {
    Vertex& current_vertex = vertices_[i];

    bool done = handleVertex(current_vertex, current_cells);
    // TODO(Oswinn): Actually fix this instead of this patch
    // Swap the point with the other point that it's connected with, hopefully everything works out.
    if (!done)
    {
      auto other = [&]() {
        if (current_vertex.point->x() == current_vertex.nextPoint()->x())
        {
          return std::find_if(vertices_.begin(), vertices_.end(),
                              [&](const Vertex& v) { return *v.point == *current_vertex.nextPoint(); });
        }
        if (current_vertex.point->x() == current_vertex.previousPoint()->x())
        {
          return std::find_if(vertices_.begin(), vertices_.end(),
                              [&](const Vertex& v) { return *v.point == *current_vertex.previousPoint(); });
        }
        return vertices_.end();
      }();

      if (other != vertices_.end() && other - vertices_.begin() > i)
      {
        std::swap(current_vertex, *other);
        i--;
        continue;
      }
    }
  }
}

bool PolygonDecomposer::handleVertex(Vertex& current_vertex, std::vector<std::unique_ptr<Cell>>& current_cells)
{
  switch (current_vertex.type)
  {
    case Event::Start:
      return handleStart(current_vertex, current_cells);
    case Event::Ceil:
      return handleCeil(current_vertex, current_cells);
    case Event::Floor:
      return handleFloor(current_vertex, current_cells);
    case Event::End:
      return handleEnd(current_vertex, current_cells);
    case Event::Open:
      return handleOpen(current_vertex, current_cells);
    case Event::Close:
      return handleClose(current_vertex, current_cells);
  }
}

bool PolygonDecomposer::handleFloor(bcd::Vertex& current_vertex, std::vector<std::unique_ptr<Cell>>& current_cells)
{
  for (auto& cell : current_cells)
  {
    if (cell->connectsToFloor(current_vertex))
    {
      cell->floor_points.emplace_back(current_vertex);
      return true;
    }
  }
  return false;
}

bool PolygonDecomposer::handleCeil(Vertex& current_vertex, std::vector<std::unique_ptr<Cell>>& current_cells)
{
  for (auto& cell : current_cells)
  {
    if (cell->connectsToCeil(current_vertex))
    {
      cell->ceiling_points.emplace_back(current_vertex);
      return true;
    }
  }
  return false;
}

bool PolygonDecomposer::handleStart(Vertex& current_vertex, std::vector<std::unique_ptr<Cell>>& current_cells)
{
  current_cells.emplace_back(std::make_unique<Cell>(current_vertex));
  return true;
}

bool PolygonDecomposer::handleEnd(Vertex& current_vertex, std::vector<std::unique_ptr<Cell>>& current_cells)
{
  for (auto it = current_cells.begin(); it != current_cells.end(); it++)
  {
    Cell* cell = it->get();
    if (cell->connectsToCeil(current_vertex) && cell->connectsToFloor(current_vertex))
    {
      cell->end = current_vertex;
      cells_.emplace_back(std::move(*it));
      current_cells.erase(it);
      return true;
    }
  }
  return false;
}

bool PolygonDecomposer::handleOpen(Vertex& current_vertex, std::vector<std::unique_ptr<Cell>>& current_cells)
{
  for (auto it = current_cells.begin(); it != current_cells.end(); it++)
  {
    Cell* closing_cell = it->get();
    if (closing_cell->enclosesVector(current_vertex))
    {
      closing_cell->end = current_vertex;

      auto top = std::make_unique<Cell>(current_vertex);
      top->ceiling_points.emplace_back(closing_cell->ceiling_points.back());

      auto bottom = std::make_unique<Cell>(current_vertex);
      bottom->floor_points.emplace_back(closing_cell->floor_points.back());

      Cell* closed_cell = it->get();
      // Update neighbors list. Counterclockwise from bottom right corner: bottom, top, rest
      closed_cell->neighbors.insert(closed_cell->neighbors.begin(), { bottom.get(), top.get() });

      top->neighbors.emplace_back(closed_cell);
      bottom->neighbors.emplace_back(closed_cell);

      cells_.emplace_back(std::move(*it));
      current_cells.erase(it);

      current_cells.emplace_back(std::move(top));
      current_cells.emplace_back(std::move(bottom));
      return true;
    }
  }
  return false;
}

bool PolygonDecomposer::handleClose(Vertex& current_vertex, std::vector<std::unique_ptr<Cell>>& current_cells)
{
  auto top_cell_it = getTopCell(current_vertex, current_cells);
  if (top_cell_it == current_cells.end())
  {
    return false;
  }
  top_cell_it->get()->end = current_vertex;

  cells_.emplace_back(std::move(*top_cell_it));
  current_cells.erase(top_cell_it);  // Fuck pointer invalidation again
  auto top_cell_ptr = cells_.back().get();

  auto bottom_cell_it = getBottomCell(current_vertex, current_cells);

  if (bottom_cell_it == current_cells.end())
  {
    return false;
  }

  bottom_cell_it->get()->end = current_vertex;
  cells_.emplace_back(std::move(*bottom_cell_it));
  current_cells.erase(bottom_cell_it);
  auto bottom_cell_ptr = cells_.back().get();

  auto new_cell = std::make_unique<Cell>(current_vertex);
  new_cell->ceiling_points.emplace_back(top_cell_ptr->ceiling_points.back());
  new_cell->floor_points.emplace_back(bottom_cell_ptr->floor_points.back());

  // Update neighbors list. Counterclockwise from bottom right corner (back of vector to front): bottom, top
  new_cell->neighbors.insert(new_cell->neighbors.begin(), { top_cell_ptr, bottom_cell_ptr });
  top_cell_ptr->neighbors.emplace_back(new_cell.get());
  bottom_cell_ptr->neighbors.emplace_back(new_cell.get());

  current_cells.emplace_back(std::move(new_cell));

  return true;
}

std::vector<std::unique_ptr<Cell>>::iterator
PolygonDecomposer::getTopCell(Vertex& current_vertex, std::vector<std::unique_ptr<Cell>>& current_cells) const
{
  return std::find_if(current_cells.begin(), current_cells.end(), [&](const std::unique_ptr<Cell>& cell) {
    return cell->nextFloorPoint() == *current_vertex.point;
  });
}

std::vector<std::unique_ptr<Cell>>::iterator
PolygonDecomposer::getBottomCell(Vertex& current_vertex, std::vector<std::unique_ptr<Cell>>& current_cells) const
{
  return std::find_if(current_cells.begin(), current_cells.end(), [&](const std::unique_ptr<Cell>& cell) {
    return cell->nextCeilPoint() == *current_vertex.point;
  });
}

Event PolygonDecomposer::getEvent(const Polygon::Vertex_const_circulator& vertex, Event last_event) const
{
  // Transition out of special events
  if (last_event == Event::Start || last_event == Event::Open)
  {
    last_event = Event::Floor;
  }
  else if (last_event == Event::End || last_event == Event::Close)
  {
    last_event = Event::Ceil;
  }

  auto next_vertex = vertex + 1;

  if (last_event == Event::Floor && next_vertex->x() < vertex->x())
  {
    if (next_vertex->y() < vertex->y())
    {
      return Event::Close;
    }
    return Event::End;
  }
  if (last_event == Event::Ceil && next_vertex->x() > vertex->x())
  {
    if (next_vertex->y() < vertex->y())
    {
      return Event::Start;
    }
    return Event::Open;
  }
  return last_event;
}

void PolygonDecomposer::sortVertices()
{
  std::sort(vertices_.begin(), vertices_.end(), [](const Vertex& lhs, const Vertex& rhs) {
    if (lhs.point->x() != rhs.point->x())
    {
      return lhs.point->x() < rhs.point->x();
    }

    return lhs.type < rhs.type;
  });
}

std::vector<Polygon> PolygonDecomposer::toPolygons(std::vector<const Cell*> cells) const
{
  std::vector<Polygon> polygons;
  polygons.reserve(cells.size());

  std::transform(cells.begin(), cells.end(), std::back_inserter(polygons),
                 [](const Cell* cell) { return cell->toPolygon(); });
  return polygons;
}

bcd::Cell* PolygonDecomposer::getClosestCell(const Point& position) const
{
  auto distance_to_edge = [position](const Polygon::Edge_const_iterator& it) {
    return CGAL::squared_distance(*it, position);
  };

  auto distance_to_cell = [&](const std::unique_ptr<Cell>& cell) {
    auto polygon = cell->toPolygon();
    double smallest_distance = std::numeric_limits<double>::max();
    for (auto edge_it = polygon.edges_begin(); edge_it != polygon.edges_end(); edge_it++)
    {
      smallest_distance = std::min(smallest_distance, distance_to_edge(edge_it));
    }
    return smallest_distance;
  };

  auto compare_distances = [&](const std::unique_ptr<Cell>& lhs, const std::unique_ptr<Cell>& rhs) {
    return distance_to_cell(lhs) < distance_to_cell(rhs);
  };

  auto closest = std::min_element(cells_.begin(), cells_.end(), compare_distances);
  return closest->get();
}

std::vector<const Cell*> PolygonDecomposer::visitCells(const Cell* starting_cell) const
{
  std::unordered_set<const Cell*> visited;
  std::vector<const Cell*> cells;
  const Cell* current_cell = starting_cell;

  auto not_visited = [&](const Cell* cell) { return visited.find(cell) == visited.end(); };

  auto get_next_unvisited_cell = [&](const Cell* cell) {
    return std::find_if(cell->neighbors.begin(), cell->neighbors.end(), not_visited);
  };

  while (cells.size() < cells_.size())
  {
    visited.emplace(current_cell);
    cells.emplace_back(current_cell);

    auto not_visited_cell = get_next_unvisited_cell(current_cell);
    if (not_visited_cell != current_cell->neighbors.end())
    {
      current_cell = *not_visited_cell;
    }
    else
    {
      for (auto it = cells.rbegin(); it != cells.rend(); it++)
      {
        auto not_visited_cell = get_next_unvisited_cell(*it);
        if (not_visited_cell != (*it)->neighbors.end())
        {
          current_cell = *not_visited_cell;
          break;
        }
      }
    }
  }
  return cells;
}
