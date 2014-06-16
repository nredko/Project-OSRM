#include <algorithm>  
#include <set>
#include <vector>
#include <osrm/Coordinate.h>

struct sortByLat
{
	bool operator()(FixedPointCoordinate const & a, FixedPointCoordinate const & b) const
	{
		return a.lat < b.lat;
	}
};

struct sortByLon
{
	bool operator()(FixedPointCoordinate const & a, FixedPointCoordinate const & b) const
	{
		return a.lon < b.lon;
	}
};

std::vector<FixedPointCoordinate> concaveHull(std::set<FixedPointCoordinate>& coordinates){
	std::vector<FixedPointCoordinate> hull(coordinates.begin(), coordinates.end());

	/*
	std::vector<FixedPointCoordinate> byLat(coordinates.begin(), coordinates.end());
	std::sort(byLat.begin(), byLat.end(), sortByLat());
	std::vector<FixedPointCoordinate> byLon(coordinates.begin(), coordinates.end());
	std::sort(byLon.begin(), byLon.end(), sortByLon());
	*/

	return hull;
};