#ifndef SRC_POLYGON_DECOMPOSER_H
#define SRC_POLYGON_DECOMPOSER_H

#include "boustrophedon_server/cgal_types.h"
#include "boustrophedon_server/cgal_utils.h"
#include "boustrophedon_server/cellular_decomposition/cell.h"

class PolygonDecomposer
{
public:
  /**
   * Decompose the polygon into convex cells, and place them in cells_.
   * @param polygon
   */
  void decompose(Polygon& polygon);

  /**
   * Use the given position to construct a sequence of cells to travel to, combining cells where possible
   * @param polygon
   * @returns a vector of Polygons that can each be striped by the striping_planner in order
   */
  std::vector<Polygon> getSubPolygons(const Point& position);

private:
  std::vector<bcd::Cell> cells_;

  /**
   * Finds all vertices in the given polygon with an interior angle greater than 180 degrees.
   * @param polygon
   * @returns a list of vertices on the polygon that have greater than 180 degree interior angles.
   */
  static std::vector<Point> getCriticalInflectionPoints(Polygon& polygon);

  /**
   * Calculates the interior angle of the given vertex, and returns that angle
   * @param vertex
   * @returns the interior angle in degrees , in the range 0.0 - 360.0
   */
  static double getInteriorAngleOfVertex(const Polygon::Vertex_const_circulator& vertex);

  /**
   * Sorts the given list of vertices by x coordinate. If equal, lower y coordinate is placed first.
   * @param vertices
   */
  static void sortVertices(std::vector<Point>& vertices);

  /**
   * Uses the given list of critical vertices to split the polygon boundary into multiple convex cells.
   * These cells are used by getSubPolygons() to create polygons that can be striped without doubling back
   * @param critical_vertices
   * @param polygon
   */
  void createCells(const std::vector<Point>& critical_vertices, const Polygon& polygon);

  /**
   * Uses the given sorted list of points to find the first unique point above (higher y) the critical_point.
   * If found, places the above point in the above parameter.
   * @param above
   * @param critical_point
   * @param points
   * @param polygon
   * @returns true if above point is found, false if not
   */
  static bool findAbovePoint(Point& above, const Point& critical_point, std::vector<Point>& points,
                             const Polygon& polygon);

  /**
   * Uses the given sorted list of points to find the first unique point below (lower y) the critical_point.
   * If found, places the below point in the below parameter.
   * @param below
   * @param critical_point
   * @param points
   * @param polygon
   * @returns true if below point is found, false if not
   */
  static bool findBelowPoint(Point& below, const Point& critical_point, const std::vector<Point>& points,
                             const Polygon& polygon);

  /**
   * Uses the given upper and lower intersection points to split an existing cell into two cells
   * Uses and modifies the cells_ vector. Both upper and lower must intersection exactly one existing cell
   * @param upper
   * @param lower
   */
  void sliceNewCell(const Point& upper, const Point& lower);

  /**
   * Finds the first (there should only be one) cell in cells_ that both upper and lower exist on.
   * Used by sliceNewCell()
   * @param upper
   * @param lower
   * @returns an iterator of cells_ that points to the cell to be sliced
   */
  std::vector<bcd::Cell>::iterator findWorkingCell(const Point& upper, const Point& lower);

  /**
   * Finds and returns a reference to the closest cell in cells_ from position
   * @param position
   * @returns a const_iterator pointing to the closest cell in cells_
   */
  std::vector<bcd::Cell>::iterator getClosestCell(const Point& position);

  /**
   * Traverses along all the cells in cells_, starting from starting_cell, and finds cells that can be merged together
   * Returns a list of sub-polygons of the original boundary polygon that can be striped
   * @param starting_cell
   * @returns a vector of cells representing sub-polygons to stripe, in order
   */
  std::vector<bcd::Cell> visitCells(std::vector<bcd::Cell>::iterator starting_cell) const;

  /**
   * Uses a breadth-first search to traverse the neighbors graph of the cells in cells_, using root as the first node
   * Returns a list of cells that is the original cells_ list, but reordered based on the breadth-first search
   * @param root
   * @returns a vector of cells representing the order in which the cells should be traveled to
   */
  std::vector<bcd::Cell> generateCellTree(std::vector<bcd::Cell>::iterator root) const;

  /**
   * Uses the given cells list to create a list of list of cells, where each list of cells can be merged together
   * Used to determine cells that can be combined together, to save time striping
   * @param cells
   * @returns a vector of vectors of cells, where all cells in a vector of cells can be merged together safely
   */
  std::vector<std::vector<bcd::Cell>> determineCellsToMerge(std::vector<bcd::Cell>& cells) const;

  /**
   * Merges the two given cells together, and returns a new cell that is the two combined cells
   * Two cells can be merged together if they share two adjacent vertices
   * @param cell_1
   * @param cell_2
   * @returns the new merged cell
   */
  bcd::Cell mergeCells(const bcd::Cell& cell_1, const bcd::Cell& cell_2) const;

  /**
   * Turns a vector of cells into a vector of polygons
   * @param cells
   * @returns a vector of polygons of the given cells
   */
  static std::vector<Polygon> toPolygons(const std::vector<bcd::Cell>& cells);
};

#endif  // SRC_POLYGON_DECOMPOSER_H
