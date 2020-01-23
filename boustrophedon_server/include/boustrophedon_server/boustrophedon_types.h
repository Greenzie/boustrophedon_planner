#ifndef SRC_BOUSTROPHEDON_TYPES_H
#define SRC_BOUSTROPHEDON_TYPES_H

#include "cgal_types.h"

enum class PointType
{
  Outline = 0,
  StripeStart = 1,
  StripeEnd = 2,
  StripeIntermediate = 3
};

struct NavPoint
{
  PointType type{};
  Point point{};

  NavPoint(PointType type, Point point);
};

#endif  // SRC_BOUSTROPHEDON_TYPES_H
