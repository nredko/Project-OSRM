#ifndef ISOCHRONE_HPP
#define ISOCHRONE_HPP

#include "engine/routing_algorithms/routing_base.hpp"
#include "engine/search_engine_data.hpp"
#include "util/typedefs.hpp"

#include <boost/assert.hpp>

#include <limits>
#include <memory>
#include <unordered_map>
#include <vector>

namespace osrm
{
namespace engine
{
namespace routing_algorithms
{

template <class DataFacadeT>
class Isochrone final
    : public BasicRoutingInterface<DataFacadeT, Isochrone<DataFacadeT>>
{
    using super = BasicRoutingInterface<DataFacadeT, Isochrone<DataFacadeT>>;
    using QueryHeap = SearchEngineData::QueryHeap;
    SearchEngineData &engine_working_data;

  public:
    Isochrone(DataFacadeT *facade, SearchEngineData &engine_working_data)
        : super(facade), engine_working_data(engine_working_data)
    {
    }

    ~Isochrone() {}

    std::shared_ptr<std::vector<FixedPointCoordinate>>
    operator()(PhantomNode &start, const int &max_time) const
    {
		std::vector<FixedPointCoordinate> points;
		points.emplace_back(start.location);
		//TODO

		///
		std::shared_ptr<std::vector<FixedPointCoordinate>> result_points = std::make_shared<std::vector<FixedPointCoordinate>>(points);
		return result_points;
    }

};
}
}
}

#endif // ISOCHRONE_HPP
