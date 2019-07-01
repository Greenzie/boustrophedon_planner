#ifndef SRC_POLYGON_DECOMPOSER_H
#define SRC_POLYGON_DECOMPOSER_H

#include "../cgal_types.h"
#include "cell.h"

class PolygonDecomposer
{
public:
  void decompose(Polygon& polygon);
  std::vector<Polygon> getSubPolygons(const Point& position) const;

private:
  std::vector<std::unique_ptr<bcd::Cell>> cells_;
  std::vector<bcd::Vertex> vertices_;

  /**
   * Creates vertices and assigns each vertex an event from the Polygon.
   * During the first pass, it finds Open and Close events, and then inserts
   * those intersections of those slices into the polygon.
   * During the second pass, it creates Vertex objects and places them into the
   * vertices_ vector
   * @param polygon
   */
  void createVertices(Polygon& polygon);

  /**
   * Finds Open and close events. For each event, it finds the intersection vertices of the
   * slice and inserts them into the polygon.
   * @param polygon
   */
  void insertOpenCloseVertices(Polygon& polygon);

  /**
   * Inserts all intersections of the slice with the polygon into the polygon.
   * @param polygon
   * @param point
   */
  static void sliceAndInsertIntoPolygon(Polygon& polygon, const Point& point);

  /**
   *
   */
  void createCells();

  bcd::Cell* getClosestCell(const Point& position) const;

  std::vector<const bcd::Cell*> visitCells(const bcd::Cell* starting_cell) const;

  std::vector<Polygon> toPolygons(std::vector<const bcd::Cell*> cells) const;

  /**
   * Dispatcher for the individual event types.
   * @param current_vertex
   * @param current_cells
   * @return
   */
  bool handleVertex(bcd::Vertex& current_vertex, std::vector<std::unique_ptr<bcd::Cell>>& current_cells);

  bool handleFloor(bcd::Vertex& current_vertex, std::vector<std::unique_ptr<bcd::Cell>>& current_cells);
  bool handleCeil(bcd::Vertex& current_vertex, std::vector<std::unique_ptr<bcd::Cell>>& current_cells);
  bool handleStart(bcd::Vertex& current_vertex, std::vector<std::unique_ptr<bcd::Cell>>& current_cells);
  bool handleEnd(bcd::Vertex& current_vertex, std::vector<std::unique_ptr<bcd::Cell>>& current_cells);
  bool handleOpen(bcd::Vertex& current_vertex, std::vector<std::unique_ptr<bcd::Cell>>& current_cells);
  bool handleClose(bcd::Vertex& current_vertex, std::vector<std::unique_ptr<bcd::Cell>>& current_cells);

  std::vector<std::unique_ptr<bcd::Cell>>::iterator
  getTopCell(bcd::Vertex& current_vertex, std::vector<std::unique_ptr<bcd::Cell>>& current_cells) const;
  std::vector<std::unique_ptr<bcd::Cell>>::iterator
  getBottomCell(bcd::Vertex& current_vertex, std::vector<std::unique_ptr<bcd::Cell>>& current_cells) const;

  /**
   * Sorts the vertices first by x coordinate, then by type.
   */
  void sortVertices();
  bcd::Event getEvent(const bcd::Circulator& vertex, bcd::Event last_event) const;
};

#endif  // SRC_POLYGON_DECOMPOSER_H
