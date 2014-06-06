
#ifndef AREA_PLUGIN_H
#define AREA_PLUGIN_H

#include "BasePlugin.h"
#include "../DataStructures/JSONContainer.h"

#include "../Algorithms/ObjectToBase64.h"

#include "../DataStructures/QueryEdge.h"
#include "../DataStructures/SearchEngine.h"
#include "../Descriptors/BaseDescriptor.h"
#include "../Descriptors/GPXDescriptor.h"
#include "../Descriptors/JSONDescriptor.h"
#include "../Util/SimpleLogger.h"
#include "../Util/StringUtil.h"
#include "../Util/TimingUtil.h"

#include <cstdlib>

#include <algorithm>
#include <memory>
#include <unordered_map>
#include <string>
#include <vector>

template <class DataFacadeT> class AreaPlugin : public BasePlugin
{
private:
	std::unordered_map<std::string, unsigned> descriptor_table;
	std::shared_ptr<SearchEngine<DataFacadeT>> search_engine_ptr;

public:
	explicit AreaPlugin(DataFacadeT *facade) : descriptor_string("area"), facade(facade)
	{
		search_engine_ptr = std::make_shared<SearchEngine<DataFacadeT>>(facade);

		descriptor_table.emplace("json", 0);
		descriptor_table.emplace("gpx", 1);
	}

	virtual ~AreaPlugin() {}

	const std::string GetDescriptor() const { return descriptor_string; }

	void HandleRequest(const RouteParameters &route_parameters, http::Reply &reply)
	{
		// check parameters
		if (route_parameters.time == 0 ||
			1 != route_parameters.coordinates.size() ||
			!route_parameters.coordinates[0].isValid())
		{
			reply = http::Reply::StockReply(http::Reply::badRequest);
			SimpleLogger().Write(logDEBUG) << "badRequest";
			return;
		}

		std::vector<FixedPointCoordinate> coordinates(0);
		coordinates.emplace_back(route_parameters.coordinates[0]);

		std::vector<PhantomNode> phantom_node_vector(1);

		facade->FindPhantomNodeForCoordinate(coordinates[0],
				phantom_node_vector[0],
				0);
		
		/*
		PhantomNodes current_phantom_node_pair;
		for (unsigned i = 0; i < phantom_node_vector.size() - 1; ++i)
		{
			current_phantom_node_pair.source_phantom = phantom_node_vector[i];
			current_phantom_node_pair.target_phantom = phantom_node_vector[i + 1];
			raw_route.segment_end_coordinates.emplace_back(current_phantom_node_pair);
		}

		const bool is_alternate_requested = route_parameters.alternate_route;
		const bool is_only_one_segment = (1 == raw_route.segment_end_coordinates.size());
		if (is_alternate_requested && is_only_one_segment)
		{
			search_engine_ptr->alternative_path(raw_route.segment_end_coordinates.front(),
				raw_route);
		}
		else
		{
			search_engine_ptr->shortest_path(raw_route.segment_end_coordinates, raw_route);
		}
		*/
		
		reply.status = http::Reply::ok;
		
		// output
		JSON::Object json_result;
		std::string temp_string;

		JSON::Array json_locations;
		unsigned counter = 0;
		for (const FixedPointCoordinate &coordinate : coordinates)
		{
			JSON::Array json_coordinates;

			json_coordinates.values.push_back(coordinate.lat / COORDINATE_PRECISION);
			json_coordinates.values.push_back(coordinate.lon / COORDINATE_PRECISION);
			json_locations.values.push_back(json_coordinates);
			++counter;
		}
		json_result.values["points"] = json_locations;

		JSON::render(reply.content, json_result);
	}

private:
	std::string descriptor_string;
	DataFacadeT *facade;
};

#endif // AREA_PLUGIN_H