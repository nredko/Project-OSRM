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
	using EdgeData = typename DataFacadeT::EdgeData;

  public:
    Isochrone(DataFacadeT *facade, SearchEngineData &engine_working_data)
        : super(facade), engine_working_data(engine_working_data)
    {
    }

    ~Isochrone() {}

    std::shared_ptr<std::vector<FixedPointCoordinate>>
    operator()(PhantomNode &source_phantom, const int &max_time) const
    {
		std::vector<FixedPointCoordinate> points;
		points.emplace_back(source_phantom.location);
		//TODO
		engine_working_data.InitializeOrClearFirstThreadLocalStorage(
			super::facade->GetNumberOfNodes());
		QueryHeap &forward_heap = *(engine_working_data.forward_heap_1);
		forward_heap.Clear();
		BOOST_ASSERT(source_phantom.IsValid());

		if (source_phantom.forward_node_id != SPECIAL_NODEID)
		{
			forward_heap.Insert(source_phantom.forward_node_id,
				-source_phantom.GetForwardWeightPlusOffset(),
				source_phantom.forward_node_id);
		}

		NodeID nodeId = source_phantom.forward_node_id;
		for (int i = 0; i < 10; i++)
		for (auto edge = super::facade->BeginEdges(nodeId); edge < super::facade->EndEdges(nodeId); ++edge)
		{
			const EdgeData &data = super::facade->GetEdgeData(edge);
			if (!data.shortcut) 
			{
				GetCoordsForEdgeID(data.id, points);
				nodeId = facade->GetTarget(edge);
			}
		}
		///
		std::shared_ptr<std::vector<FixedPointCoordinate>> result_points = std::make_shared<std::vector<FixedPointCoordinate>>(points);
		return result_points;
    }

private:

	void GetCoordsForEdgeID(NodeID edge_id,	std::vector<FixedPointCoordinate>& coordinates) const
	{
		if (!super::facade->EdgeIsCompressed(edge_id))
		{
			NodeID geom_id = super::facade->GetGeometryIndexForEdgeID(edge_id);
			coordinates.push_back(super::facade->GetCoordinateOfNode(geom_id));
			//return true;
		}
		else
		{
			//bool ret = false;
			std::vector<NodeID> geom_vector;
			super::facade->GetUncompressedGeometry(super::facade->GetGeometryIndexForEdgeID(edge_id), geom_vector);
			for (auto geom_id: geom_vector)
			{
					FixedPointCoordinate coord = super::facade->GetCoordinateOfNode(geom_id);
					coordinates.push_back(coord);
					//ret = true;
			}
			//return ret;
		}
	}
};
}
}
}

#endif // ISOCHRONE_HPP
