#include "../Util/MercatorUtil.h"
#include "../Util/ComputeAngle.h"
#include <osrm/Coordinate.h>

#include "ConcaveHull.h"

#include <algorithm>  
#include <set>
#include <vector>


void concaveHull(const std::set<FixedPointCoordinate>& coordinates, std::vector<FixedPointCoordinate>& hull){
	std::vector<FixedPointCoordinate> points(coordinates.begin(), coordinates.end());
	hull.clear();
	hull = points;
	return;

	int start = 0;
	for (int i = 0; i < points.size(); i++){
		if (points[i].lat < points[start].lat)
			start = i;
			
	}
	hull.push_back(points[start]);
	int curr = start;
	int next = -1;
	FixedPointCoordinate init(points[start].lat - 1, points[start].lon);
	FixedPointCoordinate *prev = &init;
	while (next != start){
		next = -1;
		double min_angle = 361.0;
		for (int i = 0; i < points.size(); i++){
			if (i != curr && FixedPointCoordinate::ApproximateEuclideanDistance(points[curr], *prev) < 1.5 * MAX_POINTS_DIST){
				double angle = GetAngleBetweenThreeFixedPointCoordinates(*prev, points[curr], points[i]);
				if (angle < min_angle){
					min_angle = angle;
					next = i;
				}
			}
		}
		// BOOST_ASSERT(next >= 0);
		hull.push_back(points[next]);
		prev = &points[curr];
		curr = next;
	}

	return;
};