/*

Isochrone plugin by nredko

Request format: "/loc=NN.NNNN,OO.OOOO&t=SS"
loc - starting point for isochrone request
t - isochrone time in seconds.

result
{"start":[NN.NNNN,OO.OOOO], "isochrone":[[NN.NNNN,OO.OOOO],[NN.NNNN,OO.OOOO],........],"status":200}
start - nearest point on road, 
isochrone - json array with coordinates of isochrone area's border.
*/


#ifndef ISOCHRONE_PLUGIN_HPP
#define ISOCHRONE_PLUGIN_HPP

#include "engine/plugins/plugin_base.hpp"

#include "engine/api_response_generator.hpp"
#include "engine/object_encoder.hpp"
#include "engine/search_engine.hpp"
#include "util/for_each_pair.hpp"
#include "util/integer_range.hpp"
#include "util/make_unique.hpp"
#include "util/simple_logger.hpp"
#include "util/timing_util.hpp"
#include "osrm/json_container.hpp"

#include <cstdlib>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace osrm
{
	namespace engine
	{
		namespace plugins
		{

			template <class DataFacadeT> class IsochronePlugin final : public BasePlugin
			{
			private:
				std::string descriptor_string;
				std::unique_ptr<SearchEngine<DataFacadeT>> search_engine_ptr;
				DataFacadeT *facade;
				int max_time_isochrone; // in seconds

			public:
				explicit IsochronePlugin(DataFacadeT *facade, int max_time_isochrone)
					: descriptor_string("isochrone"), facade(facade),
					max_time_isochrone(max_time_isochrone)
				{
					search_engine_ptr = util::make_unique<SearchEngine<DataFacadeT>>(facade);
				}

				virtual ~IsochronePlugin() {}

				const std::string GetDescriptor() const override final { return descriptor_string; }

				Status HandleRequest(const RouteParameters &route_parameters,
					util::json::Object &json_result) override final
				{
					int max_time = -1;
					if (route_parameters.coordinates.size() != 1)
					{
						json_result.values["status_message"] =
							"Isochrone requires only one location parameter";
						return Status::Error;
					}

					if (route_parameters.timestamps.size() != 1)
					{
						json_result.values["status_message"] =
							"Isochrone requires only one time parameter";
						return Status::Error;
					}
					
					if (max_time_isochrone > 0 &&
						(static_cast<int>(route_parameters.timestamps[0]) > max_time_isochrone))
					{
						json_result.values["status_message"] =
							"Time parameter for Isochrone " + std::to_string(route_parameters.timestamps[0]) +
							" is higher than current maximum (" + std::to_string(max_time_isochrone) + ")";
						return Status::Error;
					}
					
					max_time = route_parameters.timestamps[0]*10;

					if (!route_parameters.coordinates[0].IsValid())
					{
						json_result.values["status_message"] = "Invalid coordinate";
						return Status::Error;
					}

					PhantomNodePair phantom_node_pair;
					const bool checksum_OK = (route_parameters.check_sum == facade->GetCheckSum());
					phantom_node_pair = facade->NearestPhantomNodeWithAlternativeFromBigComponent(route_parameters.coordinates[0]);
					
					// we didn't found a fitting node, return error
					if (!phantom_node_pair.first.IsValid(facade->GetNumberOfNodes()))
					{
						json_result.values["status_message"] =
							std::string("Could not find a matching segment");
						return Status::NoSegment;
					}

					auto result_points = search_engine_ptr->isochrone(phantom_node_pair.first, max_time);

					if (!result_points)
					{
						json_result.values["status_message"] = "Can't build isochrone";
						return Status::EmptyResult;
					}

					/// output
					util::json::Array json_start;
					json_start.values.push_back(phantom_node_pair.first.location.lat / COORDINATE_PRECISION);
					json_start.values.push_back(phantom_node_pair.first.location.lon / COORDINATE_PRECISION);
					json_result.values["start"] = std::move(json_start);
					util::json::Array json_result_points;
					for (const auto &point : *result_points)
					{
						util::json::Array json_coord;
						json_coord.values.push_back(point.lat / COORDINATE_PRECISION);
						json_coord.values.push_back(point.lon / COORDINATE_PRECISION);
						json_result_points.values.push_back(json_coord);
					}
					json_result.values["isochrone"] = std::move(json_result_points);

					return Status::Ok;
				}
			};
		}
	}
}

#endif // ISOCHRONE_PLUGIN_HPP
