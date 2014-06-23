#include "../Util/MercatorUtil.h"
#include "../Util/ComputeAngle.h"
#include "../Util/SimpleLogger.h"

#include <osrm/Coordinate.h>

#include "ConcaveHull.h"

#include <algorithm>  
#include <set>
#include <vector>
#include <limits>
#include <valarray>

// area?jsonp=show&time=10&loc=54.83663,83.09185

/// a->b->c angle (0-360°)
float getAngle(const FixedPointCoordinate& prev, const FixedPointCoordinate& curr, const FixedPointCoordinate& next) {
	int prev_x = prev.lon , prev_y = prev.lat;
	int curr_x = curr.lon , curr_y = curr.lat;
	int next_x = next.lon , next_y = next.lat;
	if (next_x == curr_x && next_y == curr_y)
		return -9000.f;
	if (next_x == prev_x && next_y == prev_y)
		return -360.f;

	int a_x = curr_x - prev_x;
	int a_y = curr_y - prev_y;
	int b_x = next_x - curr_x;
	int b_y = next_y - curr_y;
	float vect = a_x*b_y - b_x*a_y;
	float scal = a_x*b_x + a_y*b_y;
	float angle = 0;
	if (scal == 0){
		if (vect > 0)
			angle = 90.f;
		if (vect < 0)
			angle = -90.f;
	}
	else {
		angle = atan(vect / scal) * 180.f / M_PI;
		if (scal < 0){
			if (vect >= 0)
				angle += 180.f;
			if (vect < 0)
				angle -= 180.f;
		}
	}
	if (angle == 360.f)
		angle = 0;
	return 180.f - angle;
}


int ccw(const FixedPointCoordinate& p0, const FixedPointCoordinate& p1, const FixedPointCoordinate& p2) {
	int dx1 = p1.lon - p0.lon;
	int dy1 = p1.lat - p0.lat;
	int dx2 = p2.lon - p0.lon;
	int dy2 = p2.lat - p0.lat;
	int d = dx1*dy2 - dy1*dx2;
	if (d > 0) return 1;
	if (d < 0) return -1;
	if ((dx1*dx2 < 0) || (dy1*dy2 < 0)) return -1;
	if ((dx1*dx1 + dy1*dy1) < (dx2*dx2 + dy2*dy2)) return 1;
	return 0;
}

bool intersect(const FixedPointCoordinate& p1, const FixedPointCoordinate& p2, const FixedPointCoordinate& q1, const FixedPointCoordinate& q2) {
	if ((p1.lon == q1.lon && p1.lat == q1.lat || p2.lon == q2.lon && p2.lat == q2.lat) ||
		(p1.lon == q2.lon && p1.lat == q2.lat || p2.lon == q1.lon && p2.lat == q1.lat))
		return false;

	return (ccw(p1, p2, q1) * ccw(p1, p2, q2) <= 0) && (ccw(q1, q2, p1) * ccw(q1, q2, p2) <= 0);
}


bool intersect(std::vector<FixedPointCoordinate>& hull, const FixedPointCoordinate& next) {
	int l = hull.size() - 1;
	for (int i = 1; i < l; i++){
		if (intersect(hull[i - 1], hull[i], hull[l], next))
			return true;
	}
	return false;
}

inline bool isNear(const FixedPointCoordinate& a, const FixedPointCoordinate& b, int maxLat, int maxLon){
	return (abs(a.lat - b.lat) < maxLat && abs(a.lon - b.lon) < maxLon);
}


void calculateHull(const std::vector<FixedPointCoordinate>& points, std::vector<FixedPointCoordinate>& hull, int maxLat, int maxLon) {
	
	hull.clear();
	//hull = points;
	//return;
	std::vector<int> counts(points.size(), 0);

	int start = 0;
	for (int i = 0; i < points.size(); i++) {
		if (points[i].lat < points[start].lat)
			start = i;
	}
	hull.push_back(points[start]);
	int curr = start;
	int next = -1;
	const FixedPointCoordinate init(points[start].lat - 10, points[start].lon);
	const FixedPointCoordinate *prev = &init;
	

#ifdef DEBUG
	SimpleLogger().Write(logDEBUG) << "Init: " << init;
	SimpleLogger().Write(logDEBUG) << "Start: " << start << ": " << points[start];
#endif
	while (next != start) {
		next = -1;
		float max_angle = -10000.f;
		int min_rot = std::numeric_limits<int>::max();
		for (int i = 0; i < points.size(); i++) {
#ifdef MyDEBUG			
			SimpleLogger().Write(logDEBUG) << i << ": " << points[i] 
				<< " dist: " << FixedPointCoordinate::ApproximateEuclideanDistance(points[i], points[curr]) 
				<< " [" << (FixedPointCoordinate::ApproximateEuclideanDistance(points[i], points[curr]) < 1.2 * MAX_POINTS_DIST ? "v" : " ") << "]"
				<< " rough: lat " << abs(points[i].lat - points[curr].lat) << " lon " << abs(points[i].lon - points[curr].lon)
				<< " [" << (isNear(points[i], points[curr], maxLat, maxLon) ? "v" : " ") << "]"
		
				<< " angle: " << getAngle(*prev, points[curr], points[i]) 
				//<< " rot: " << getRot(*prev, points[curr], points[i])
				<< " intersect:" << intersect(hull, points[i]);
#endif
			if (i != curr && isNear(points[i], points[curr], maxLat, maxLon) /*&& FixedPointCoordinate::ApproximateEuclideanDistance(points[i], points[curr]) < 1.2 * MAX_POINTS_DIST */ && !intersect(hull, points[i])){
				float angle = getAngle(*prev, points[curr], points[i]);
				if (angle > max_angle) {
					max_angle = angle;
					next = i;
				}
			}
		}
		//BOOST_ASSERT(next >= 0);
		if (next < 0) {
			SimpleLogger().Write(logWARNING) << "Error in hull calculations.";
			hull.push_back(points[start]);
			return;
		}
		hull.push_back(points[next]);
		if (++counts[next] > 4){
			SimpleLogger().Write(logWARNING) << "Something goes wrong in hull calculations.";
			hull.push_back(points[start]);
			return;
		}

#ifdef DEBUG
		SimpleLogger().Write(logDEBUG) << curr << " -> " << next;
#endif
		prev = &points[curr];
		curr = next;
	}
	return;
};


void concaveHull(const std::set<FixedPointCoordinate>& coordinates, std::vector<FixedPointCoordinate>& hull) {
	
	FixedPointCoordinate start = *coordinates.begin();
	
	// calculate scale by lat and scale by lon for current geographic area
	FixedPointCoordinate south(start.lat - 100, start.lon);
	FixedPointCoordinate east(start.lat, start.lon - 100);
	int maxLat = 100 * MAX_POINTS_DIST / FixedPointCoordinate::ApproximateDistance(south, start);
	int maxLon = 100 * MAX_POINTS_DIST / FixedPointCoordinate::ApproximateDistance(east, start);

	std::vector<FixedPointCoordinate> points(coordinates.begin(), coordinates.end());
	calculateHull(points, hull, maxLat*1.2, maxLon*1.2);
	SimpleLogger().Write(logINFO) << "First hull: " << hull.size() << " points. Expanding...";


	int dLat = maxLat / 1.4142135623730950488016887242097;
	int dLon = maxLon / 1.4142135623730950488016887242097;
	
	std::vector<FixedPointCoordinate> expanded;
	expanded.reserve((hull.size() - 1) * 8);
	for (int i = 0; i < hull.size() - 1; i++) {
		FixedPointCoordinate p0(hull[i].lat + maxLat, hull[i].lon);
		expanded.push_back(p0);
		FixedPointCoordinate p1(hull[i].lat - maxLat, hull[i].lon);
		expanded.push_back(p1);
		FixedPointCoordinate p2(hull[i].lat, hull[i].lon + maxLon);
		expanded.push_back(p2);
		FixedPointCoordinate p3(hull[i].lat, hull[i].lon - maxLon);
		expanded.push_back(p3);
		FixedPointCoordinate p4(hull[i].lat + dLat, hull[i].lon + dLon);
		expanded.push_back(p4);
		FixedPointCoordinate p5(hull[i].lat - dLat, hull[i].lon - dLon);
		expanded.push_back(p5);
		FixedPointCoordinate p6(hull[i].lat - dLat, hull[i].lon + dLon);
		expanded.push_back(p6);
		FixedPointCoordinate p7(hull[i].lat + dLat, hull[i].lon - dLon);
		expanded.push_back(p7);
	}
	calculateHull(expanded, hull, maxLat*1.2, maxLon*1.2);
	
}
