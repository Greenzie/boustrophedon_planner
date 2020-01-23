#include "boustrophedon_server/boustrophedon_types.h"

NavPoint::NavPoint(PointType type, Point point) : type{ type }, point{ std::move(point) }
{
}
