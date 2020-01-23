#ifndef SRC_CGAL_TYPES_H
#define SRC_CGAL_TYPES_H

#include <CGAL/Polygon_2.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Arr_linear_traits_2.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_2.h>

using Kernel = CGAL::Cartesian<double>;
using Polygon = CGAL::Polygon_2<Kernel>;
using Point = Kernel::Point_2;
using Vector = Kernel::Vector_2;
using Traits = CGAL::Arr_linear_traits_2<Kernel>;
using Segment = Traits::Segment_2;
using Line = Traits::Line_2;
using AffineTransform = CGAL::Aff_transformation_2<Kernel>;
using NeighborSearch = CGAL::Orthogonal_k_neighbor_search<CGAL::Search_traits_2<Kernel>>;

#endif  // SRC_CGAL_TYPES_H
