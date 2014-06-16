#ifndef CONCAVE_HULL_H
#define CONCAVE_HULL_H

#include <vector>
#include <osrm/Coordinate.h>

std::vector<FixedPointCoordinate> concaveHull(std::set<FixedPointCoordinate>& coordinates);

#endif