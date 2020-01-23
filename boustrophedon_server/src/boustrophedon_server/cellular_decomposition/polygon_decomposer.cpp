#include <ros/assert.h>
#include "boustrophedon_server/cellular_decomposition/polygon_decomposer.h"

using namespace bcd;

void PolygonDecomposer::decompose(Polygon& polygon)
{
  // first, find all the points in the polygon that could cause us to have to split it into multiple cells
  std::vector<Point> critical_points = getCriticalInflectionPoints(polygon);

  // sort those critical points for later use
  sortVertices(critical_points);

  // use those critical points and the polygon to create a collection of convex cells
  createCells(critical_points, polygon);
}

std::vector<Polygon> PolygonDecomposer::getSubPolygons(const Point& position)
{
  // get an iterator to the closest cell to position. This will be the root of our depth-first tree
  auto closest_cell = getClosestCell(position);

  // reset the visited state of all the cells, in case we've done this before
  for (Cell cell : cells_)
  {
    cell.visited = false;
  }

  std::vector<Cell> visited_cells = visitCells(closest_cell);

  std::vector<Polygon> polygons = toPolygons(visited_cells);
  return polygons;
}

/**
 * Finds all vertices in the given polygon with an interior angle greater than 180 degrees.
 * @param polygon
 * @returns a list of vertices on the polygon that have greater than 180 degree interior angles.
 */
std::vector<Point> PolygonDecomposer::getCriticalInflectionPoints(Polygon& polygon)
{
  // ensure that the polygon is counter_clockwise oriented. Later functions require this.
  if (polygon.is_clockwise_oriented())
  {
    polygon.reverse_orientation();
  }

  std::vector<Point> critical_inflection_points;
  Polygon::Vertex_const_circulator vertex = polygon.vertices_circulator();

  // loop through all vertices of the polygon
  do
  {
    // for each vertex, check its interior angle. If greater than 180 degrees, add it to the list
    double interior_angle = getInteriorAngleOfVertex(vertex);
    if (interior_angle > 180.0)
    {
      critical_inflection_points.push_back(*vertex);
    }

    vertex++;
  } while (vertex != polygon.vertices_circulator());

  // return the list
  return critical_inflection_points;
}

/**
 * Calculates the interior angle of the given vertex, and returns that angle
 * @param vertex
 * @returns the interior angle in degrees , in the range 0.0 - 360.0
 */
double PolygonDecomposer::getInteriorAngleOfVertex(const Polygon::Vertex_const_circulator& vertex)
{
  // check if vertex is a critical point
  // because the polygon is counter_clockwise oriented, interior angle will be counter clockwise from prev to next
  auto prev_vertex = vertex - 1;
  auto next_vertex = vertex + 1;

  // make vectors from vertex to previous and next points
  Vector v1 = *prev_vertex - *vertex;
  Vector v2 = *next_vertex - *vertex;

  // get the atan2 angle of each vector to x-axis vector, radians in range -PI to PI
  double alpha = std::atan2(v1.y(), v1.x());
  double beta = std::atan2(v2.y(), v2.x());

  // transform alpha and beta to the range 0 to 2PI
  if (alpha < 0)
  {
    alpha += (CGAL_PI * 2);
  }
  if (beta < 0)
  {
    beta += (CGAL_PI * 2);
  }

  // use alpha and beta to calculate the clockwise-angle between alpha and beta
  double angle;
  if (alpha > beta)
  {
    angle = alpha - beta;
  }
  else if (alpha < beta)
  {
    angle = (CGAL_PI * 2) - beta + alpha;
  }
  else
  {
    angle = 0.0;
  }

  // scale angle to 0 to 360 in degrees
  angle *= 180 / CGAL_PI;

  // return angle
  return angle;
}

/**
 * Sorts the given list of vertices by x coordinate. If equal, lower y coordinate is placed first.
 * @param vertices
 */
void PolygonDecomposer::sortVertices(std::vector<Point>& vertices)
{
  std::sort(vertices.begin(), vertices.end(), [](const Point& lhs, const Point& rhs) {
    // sort first using the x coordinate
    if (!approximatelyEqual(lhs.x(), rhs.x(), EPSILON))
    {
      return definitelyLessThan(lhs.x(), rhs.x(), EPSILON);
    }

    // if x coordinate is the same, then sort using y coordinate
    if (!approximatelyEqual(lhs.y(), rhs.y(), EPSILON))
    {
      return definitelyLessThan(lhs.y(), rhs.y(), EPSILON);
    }

    // else just return true, the two points are in the same place
    return true;
  });
}

/**
 * Uses the given list of critical vertices to split the polygon boundary into multiple convex cells.
 * These cells are used by getSubPolygons() to create polygons that can be striped without doubling back
 * @param critical_vertices
 * @param polygon
 */
void PolygonDecomposer::createCells(const std::vector<Point>& critical_vertices, const Polygon& polygon)
{
  cells_.emplace_back(polygon);

  // create all of the new cells using the vertices in critical_vertices
  for (const Point& vertex : critical_vertices)
  {
    std::vector<Point> intersection_points =
        getIntersectionPoints(polygon, Line(Point(vertex.x(), 0.0), Point(vertex.x(), 1.0)));

    // sort the intersection points from lowest y to highest y. The default behavior could have them mixed up.
    // sortVertices() will work because the X coordinate should be the same for all of the intersection points
    sortVertices(intersection_points);

    Point above_point;  // the intersection point just above the critical vertex
    Point below_point;  // the intersection point just below the critical vertex

    // if there is an above point, we make a new cell using above and vertex as endpoints
    if (findAbovePoint(above_point, vertex, intersection_points, polygon))
    {
      sliceNewCell(above_point, vertex);
    }

    // if there is a below point, we make a new cell using vertex and below as endpoints
    if (findBelowPoint(below_point, vertex, intersection_points, polygon))
    {
      sliceNewCell(vertex, below_point);
    }
  }

  // link adjacent cells together now that they've all been created
  // loop through every cell except the last one
  for (auto cell_iterator = cells_.begin(); cell_iterator != cells_.end() - 1; ++cell_iterator)
  {
    // loop through every cell between cell_iterator and the end. This way we check every combination.
    for (auto other_iterator = cell_iterator + 1; other_iterator != cells_.end(); ++other_iterator)
    {
      if (cell_iterator->isAdjacentTo(*other_iterator))
      {
        cell_iterator->neighbors.push_back(&*other_iterator);
        other_iterator->neighbors.push_back(&*cell_iterator);
      }
    }
  }
}
/**
 * Uses the given sorted list of points to find the first unique point above (higher y) the critical_point.
 * If found, places the above point in the above parameter.
 * @param above
 * @param critical_point
 * @param points
 * @returns true if above point is found, false if not
 */
bool PolygonDecomposer::findAbovePoint(Point& above, const Point& critical_point, std::vector<Point>& points,
                                       const Polygon& polygon)
{
  // first find the critical_point in the list of intersection points. Use the cgal_utils function here.
  auto it = findPointInVector(critical_point, points);

  // check if the critical point was found
  if (it == points.end())
  {
    // we didn't find it...probably because of inexact_constructions kernel. Insert it into the list where it should be
    // and continue.
    for (it = points.begin(); it != points.end() - 1; it++)
    {
      if (it->y() < critical_point.y() && critical_point.y() < (it + 1)->y())
      {
        points.insert(it + 1, critical_point);
        break;
      }
    }

    // reset it
    it = findPointInVector(critical_point, points);

    // because points is not const, and findAbovePoint happens before findBelowPoint, we don't have to modify
    // findBelowPoint()
  }

  // the critical point was found, but there might not be a next point in the list
  auto next = it + 1;
  if (next != points.end())
  {
    // the next point exists, but it could be the duplicate or the above point
    if (approximatelyEqual(next->y(), it->y(), EPSILON))
    {
      // next point was the duplicate point! increment next so that it points to the above point, if it exists
      next++;
    }
  }

  // now next should either be pointing to the end or the above point
  if (next != points.end() && next->y() > it->y())
  {
    // do a sanity check to ensure that this line segment lies inside the boundary, use midpoint of it and next
    Point midpoint = Point(it->x(), (it->y() + next->y()) / 2);
    if (CGAL::bounded_side_2(polygon.vertices_begin(), polygon.vertices_end(), midpoint) == CGAL::ON_BOUNDED_SIDE)
    {
      above = *next;
      return true;
    }
  }

  // else return false, we didn't find the above point. It doesn't exist.
  return false;
}

/**
 * Uses the given sorted list of points to find the first unique point below (lower y) the critical_point.
 * If found, places the below point in the below parameter.
 * @param below
 * @param critical_point
 * @param points
 * @returns true if below point is found, false if not
 */
bool PolygonDecomposer::findBelowPoint(Point& below, const Point& critical_point, const std::vector<Point>& points,
                                       const Polygon& polygon)
{
  // first find the critical_point in the list of intersection points. Use the cgal_utils function here.
  auto it = findPointInVector(critical_point, points);

  // check if the critical point was found
  if (it == points.end())
  {
    return false;
  }

  // the critical point was found, but there might not be a previous point in the list
  auto prev = it - 1;
  if (prev != points.begin() - 1 && prev->y() < it->y())
  {
    // do a sanity check to ensure that this line segment lies inside the boundary, use midpoint of it and next
    Point midpoint = Point(it->x(), (it->y() + prev->y()) / 2);
    if (CGAL::bounded_side_2(polygon.vertices_begin(), polygon.vertices_end(), midpoint) == CGAL::ON_BOUNDED_SIDE)
    {
      below = *prev;
      return true;
    }
  }

  // else return false, we didn't find the below point. It doesn't exist.
  return false;
}

/**
 * Uses the given upper and lower intersection points to split an existing cell into two cells
 * Uses and modifies the cells_ vector. Both upper and lower must intersection exactly one existing cell
 * @param upper
 * @param lower
 */
void PolygonDecomposer::sliceNewCell(const Point& upper, const Point& lower)
{
  // before we can slice a cell using this intersection, we have to figure out what cell to work on
  auto working_cell_ptr = findWorkingCell(upper, lower);
  Cell working_cell = *working_cell_ptr;

  // next, we must insert upper and lower into the working cell, assuming they aren't already vertices.
  working_cell.insertPointOnEdge(upper);
  working_cell.insertPointOnEdge(lower);

  Cell new_cell = Cell();

  // first we need to get a circulator of the working cell's vertices, and advance it to upper
  Polygon working_polygon = working_cell.toPolygon();
  Polygon::Vertex_const_circulator it = working_polygon.vertices_circulator();
  while (!approximatelyEqual(*it, upper, EPSILON))
  {
    it++;
  }

  // start from upper and rotate counter-clockwise until we reach lower
  // put every point from upper to lower in the working cell into the new cell
  while (!approximatelyEqual(*it, lower, EPSILON))
  {
    new_cell.points.push_back(*it);
    it++;
  }
  // we also need the lower point, so append it one more time
  new_cell.points.push_back(*it);

  // remove the points of new_cell (except upper and lower!) from the working cell
  auto new_cell_iterator = new_cell.toPolygon().vertices_begin();
  new_cell_iterator++;  // skip upper
  for (; new_cell_iterator != new_cell.toPolygon().vertices_end() - 1; new_cell_iterator++)
  {
    // for each point in the new cell, we need to remove it from the working cell
    working_cell.points.erase(findPointInVector(*new_cell_iterator, working_cell.points));
  }

  // In certain situations, the working cell could potentially only be a line. Just remove it if so.
  if (working_cell.points.size() <= 2)
  {
    // remove the working cell
    cells_.erase(working_cell_ptr);
  }
  else
  {
    // update the current working cell pointer to the new working cell
    *working_cell_ptr = working_cell;
  }

  // the same thing can happen to the new cell. Only add it to cells_ if it's not a line
  if (new_cell.points.size() > 2)
  {
    // put the new cell into the cells vector
    cells_.push_back(new_cell);
  }
}

/**
 * Finds the first (there should only be one) cell in cells_ that both upper and lower exist on.
 * Used by sliceNewCell()
 * @param upper
 * @param lower
 * @returns an iterator of cells_ that points to the cell to be sliced
 */
std::vector<Cell>::iterator PolygonDecomposer::findWorkingCell(const Point& upper, const Point& lower)
{
  // there should only be one cell with both upper and lower in it. We need to return that cell
  auto cell_iterator = cells_.begin();
  for (; cell_iterator != cells_.end(); cell_iterator++)
  {
    // we need to check each cell to see if upper and lower are on it
    bool found_upper = cell_iterator->isPointOnEdge(upper);
    bool found_lower = cell_iterator->isPointOnEdge(lower);

    // if both are true, we found the cell we want to work on! If not, keep looking
    if (found_lower && found_upper)
    {
      return cell_iterator;
    }
  }

  // if we haven't found the working cell by here, something didn't work properly.
  ROS_WARN_STREAM("couldn't find working cell!");
  return cell_iterator;  // this will be cells_end();
}

/**
 * Finds and returns a reference to the closest cell in cells_ from position
 * @param position
 * @returns a iterator pointing to the closest cell in cells_
 */
std::vector<Cell>::iterator PolygonDecomposer::getClosestCell(const Point& position)
{
  // A lambda to calculate the distance between a point and an edge
  auto distance_to_edge = [position](const Polygon::Edge_const_iterator& it) {
    return CGAL::squared_distance(*it, position);
  };

  // A lambda to calculate the minimum distance between a point and a cell
  auto distance_to_cell = [&](const Cell* cell) {
    Polygon polygon = cell->toPolygon();
    double smallest_distance = std::numeric_limits<double>::max();

    // for each edge, check if the distance to that edge is smaller than the previously smallest distance
    for (auto edge_it = polygon.edges_begin(); edge_it != polygon.edges_end(); edge_it++)
    {
      // use the distance_to_edge lambda to get the smallest distance to the edge
      smallest_distance = std::min(smallest_distance, distance_to_edge(edge_it));
    }
    return smallest_distance;
  };

  // A lambda to compare the distances from position to each of two cells
  auto compare_distances = [&](const Cell lhs, const Cell rhs) {
    return distance_to_cell(&lhs) < distance_to_cell(&rhs);
  };

  // get an iterator pointing to the cell closest to position
  return std::min_element(cells_.begin(), cells_.end(), compare_distances);
}

/**
 * Traverses along all the cells in cells_, starting from starting_cell, and finds cells that can be merged together
 * Returns a list of sub-polygons of the original boundary polygon that can be striped
 * @param starting_cell
 * @returns a vector of cells representing sub-polygons to stripe, in order
 */
std::vector<Cell> PolygonDecomposer::visitCells(std::vector<Cell>::iterator starting_cell) const
{
  // we want to visit the cells in an order such that we have to do the least amount of backtracking possible
  // create a rearranged list of cells in the order that we should look to merge them together in
  std::vector<Cell> rearranged_cell_tree = generateCellTree(starting_cell);

  // get lists of cells to combine together
  std::vector<std::vector<Cell>> combined_cells_list = determineCellsToMerge(rearranged_cell_tree);

  std::vector<Cell> final_cells;

  // for each list of cells to combine together...
  for (const std::vector<Cell>& sub_polygon : combined_cells_list)
  {
    // start the new cell with the first one in the list
    Cell final_cell = sub_polygon.front();

    // combine all of the next cells in the list together
    for (auto it = sub_polygon.begin() + 1; it != sub_polygon.end(); ++it)
    {
      final_cell = mergeCells(final_cell, *it);
    }

    // add it to the final_cells list
    final_cells.push_back(final_cell);
  }

  return final_cells;
}

/**
 * Uses a breadth-first search to traverse the neighbors graph of the cells in cells_, using root as the first node
 * Returns a list of cells that is the original cells_ list, but reordered based on the breadth-first search
 * @param root
 * @returns a vector of cells representing the order in which the cells should be traveled to
 */
std::vector<Cell> PolygonDecomposer::generateCellTree(std::vector<Cell>::iterator root) const
{
  // we're going to create a new cells vector where the root (starting point) is the first element, and adjacent cells
  // children
  std::vector<Cell> new_cells_vector;
  std::vector<Cell> queue;
  root->visited = true;
  queue.push_back(*root);
  while (!queue.empty())
  {
    // get the next cell in the queue, put it on the new_cells_vector, and remove it from the queue
    Cell front = queue.front();
    new_cells_vector.push_back(front);
    queue.erase(queue.begin());

    // get all the adjacent cells of front
    for (Cell* neighbor : front.neighbors)
    {
      if (!neighbor->visited)
      {
        neighbor->visited = true;
        queue.push_back(*neighbor);
      }
    }
  }
  return new_cells_vector;
}

/**
 * Uses the given cells list to create a list of list of cells, where each list of cells can be merged together
 * Used to determine cells that can be combined together, to save time striping
 * @param cells
 * @returns a vector of vectors of cells, where all cells in a vector of cells can be merged together safely
 */
std::vector<std::vector<Cell>> PolygonDecomposer::determineCellsToMerge(std::vector<Cell>& cells) const
{
  std::vector<std::vector<Cell>> combined_cells_list;
  std::vector<Cell> first_cell_list;
  combined_cells_list.push_back(first_cell_list);
  // traverse through the rearranged cells list
  for (Cell cell : cells)
  {
    // check if we can merge the current combined cell and the current cell together
    if (cell.canMergeWith(combined_cells_list.back()))
    {
      // the cells can merge! add cell to the current combined cell
      combined_cells_list.back().push_back(cell);
    }
    else
    {
      // the cells can't merge. Create a new combined cell and add current cell to its
      std::vector<Cell> new_cell_list;
      new_cell_list.push_back(cell);
      combined_cells_list.push_back(new_cell_list);
    }
  }
  return combined_cells_list;
}

/**
 * Merges the two given cells together, and returns a new cell that is the two combined cells
 * Two cells can be merged together if they share two adjacent vertices
 * Uses the mergePolygons method of cgal_utils
 * @param cell_1
 * @param cell_2
 * @returns the new merged cell
 */
Cell PolygonDecomposer::mergeCells(const Cell& cell_1, const Cell& cell_2) const
{
  // perform all of the logic on polygons in cgal_utils.cpp
  // previously logic was on cells in this function, moved for generality
  Polygon new_polygon = mergePolygons(cell_1.toPolygon(), cell_2.toPolygon());
  Cell new_cell = Cell(new_polygon);

  // this is the complete new cell! it doesn't have an id or neighbors though.
  return new_cell;
}

/**
 * Turns a vector of cells into a vector of polygons
 * @param cells
 * @returns a vector of polygons of the given cells
 */
std::vector<Polygon> PolygonDecomposer::toPolygons(const std::vector<Cell>& cells)
{
  // create the vector to put all of the polygons into
  std::vector<Polygon> polygons;
  polygons.reserve(cells.size());

  // use each cell's toPolygon() function to place each cell into the polygons vector
  std::transform(cells.begin(), cells.end(), std::back_inserter(polygons),
                 [](const Cell& cell) { return cell.toPolygon(); });
  return polygons;
}
