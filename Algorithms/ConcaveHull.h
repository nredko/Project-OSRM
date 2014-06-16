#ifndef CONCAVE_HULL_H
#define CONCAVE_HULL_H
#if defined(_MSC_VER) && (_MSC_VER >= 1020)
# pragma once
#endif

#include <vector>
#include <set>


#define MAX_POINTS_DIST     99


void concaveHull(const std::set<FixedPointCoordinate>& coordinates, std::vector<FixedPointCoordinate>& outHull);

#endif