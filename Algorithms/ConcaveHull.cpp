#include "../Util/MercatorUtil.h"
#include "../Util/ComputeAngle.h"
#include "../Util/SimpleLogger.h"

#include <osrm/Coordinate.h>

#include "ConcaveHull.h"

#include <algorithm>  
#include <set>
#include <vector>
#include <limits>


// area?jsonp=show&time=10&loc=54.83663,83.09185

/// a->b->c angle (0-360°)
float getAngle1(const FixedPointCoordinate& prev, const FixedPointCoordinate& curr, const FixedPointCoordinate& next) {
	float prev_x = prev.lon / COORDINATE_PRECISION, prev_y = lat2y(prev.lat / COORDINATE_PRECISION);
	float curr_x = curr.lon / COORDINATE_PRECISION, curr_y = lat2y(curr.lat / COORDINATE_PRECISION);
	float next_x = next.lon / COORDINATE_PRECISION, next_y = lat2y(next.lat / COORDINATE_PRECISION);
	if (next_x == curr_x && next_y == curr_y)
		return -9000.f;
	if (next_x == prev_x && next_y == prev_y)
		return -360.f;
	
	float a_x = curr_x - prev_x;
	float a_y = curr_y - prev_y;
	float b_x = next_x - curr_x;
	float b_y = next_y - curr_y;
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

int ccw0(const FixedPointCoordinate& p0, const FixedPointCoordinate& p1, const FixedPointCoordinate& p2) {
	const float epsylon = 1e-13;
	int dx1 = (p1.lon - p0.lon) / COORDINATE_PRECISION;
	int dy1 = lat2y(p1.lat / COORDINATE_PRECISION) - lat2y(p0.lat / COORDINATE_PRECISION);
	int dx2 = (p2.lon - p0.lon) / COORDINATE_PRECISION;
	int dy2 = lat2y(p2.lat / COORDINATE_PRECISION) - lat2y(p0.lat / COORDINATE_PRECISION);
	int d = dx1*dy2 - dy1*dx2;
	if (d > epsylon) return 1;
	if (d < -epsylon) return -1;
	if ((dx1*dx2 < -epsylon) || (dy1*dy2 < -epsylon)) return -1;
	if ((dx1*dx1 + dy1*dy1) < (dx2*dx2 + dy2*dy2)) return 1;
	return 0;
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

int getRot(const FixedPointCoordinate& p0, const FixedPointCoordinate& p1, const FixedPointCoordinate& p2){
	return (p1.lon - p0.lon)*(p2.lat - p1.lat) - (p1.lat - p0.lat)*(p2.lon - p1.lon);
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

void concaveHull(const std::set<FixedPointCoordinate>& coordinates, std::vector<FixedPointCoordinate>& hull) {
	std::vector<FixedPointCoordinate> points(coordinates.begin(), coordinates.end());
	hull.clear();
	//hull = points;
	//return;

	int start = 0;
	for (int i = 0; i < points.size(); i++) {
		if (points[i].lat < points[start].lat)
			start = i;
	}
	hull.push_back(points[start]);
	int curr = start;
	int next = -1;
	FixedPointCoordinate init(points[start].lat - 10, points[start].lon);
	FixedPointCoordinate *prev = &init;
#ifdef DEBUG
	SimpleLogger().Write(logDEBUG) << "Init: " << init;
	SimpleLogger().Write(logDEBUG) << "Start: " << start << ": " << points[start];
#endif
	while (next != start) {
		next = -1;
		float max_angle = -10000.f;
		int min_rot = std::numeric_limits<int>::max();
		for (int i = 0; i < points.size(); i++) {
#ifdef DEBUG			
			SimpleLogger().Write(logDEBUG) << i << ": " << points[i] << " dist: " << FixedPointCoordinate::ApproximateEuclideanDistance(points[i], points[curr]) 
				<< " [" << (FixedPointCoordinate::ApproximateEuclideanDistance(points[i], points[curr]) < 1.2 * MAX_POINTS_DIST ? "v" : " ") << "]" 
				<< " angle: " << getAngle(*prev, points[curr], points[i]) 
				<< " rot: " << getRot(*prev, points[curr], points[i])
				<< " intersect:" << intersect(hull, points[i]);
#endif
			if (i != curr && FixedPointCoordinate::ApproximateEuclideanDistance(points[i], points[curr]) < 1.2 * MAX_POINTS_DIST && !intersect(hull, points[i])){
				/*float angle = getAngle(*prev, points[curr], points[i]);
				if (angle > max_angle) {
					max_angle = angle;
					next = i;
				}*/
				int rot = getRot(*prev, points[curr], points[i]);
				if (rot == 0)
					rot = std::numeric_limits<int>::max() - 1;
				if (rot < min_rot) {
					min_rot = rot;
					next = i;
				}
			}
		}
		BOOST_ASSERT(next >= 0);
		hull.push_back(points[next]);
#ifdef DEBUG
		SimpleLogger().Write(logDEBUG) << curr << " -> " << next;
#endif
		prev = &points[curr];
		curr = next;
	}

	return;
};