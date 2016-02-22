#ifndef ISOCHRONE_HPP
#define ISOCHRONE_HPP

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
		if(route_parameters.coordinates.size() != 1)
		{
			json_result.values["status_message"] =
				"Isochrone allows only one location parameter";
			return Status::Error;
		}

		if (route_parameters.timestamps.size() != 1)
		{
			json_result.values["status_message"] =
				"Isochrone allows only one time parameter";
			return Status::Error;
		}

        if (max_time_isochrone > 0 &&
            (static_cast<int>(route_parameters.time) > max_time_isochrone))
        {
            json_result.values["status_message"] =
                "Time parameter for Isochrone " + std::to_string(route_parameters.time) +
                " is higher than current maximum (" + std::to_string(max_time_isochrone) + ")";
            return Status::Error;
        }

        if (!route_parameters.coordinates[0].IsValid()))
        {
            json_result.values["status_message"] = "Invalid coordinate";
            return Status::Error;
        }

        std::vector<PhantomNodePair> phantom_node_pair_list(route_parameters.coordinates.size());
        const bool checksum_OK = (route_parameters.check_sum == facade->GetCheckSum());

        for (const auto i : util::irange<std::size_t>(0, route_parameters.coordinates.size()))
        {
            const int bearing = 0;
            const int range = 180;
            phantom_node_pair_list[i] = facade->NearestPhantomNodeWithAlternativeFromBigComponent(
                route_parameters.coordinates[i], bearing, range);
            // we didn't found a fitting node, return error
            if (!phantom_node_pair_list[i].first.IsValid(facade->GetNumberOfNodes()))
            {
                json_result.values["status_message"] =
                    std::string("Could not find a matching segment for coordinate ") +
                    std::to_string(i);
                return Status::NoSegment;
            }
            BOOST_ASSERT(phantom_node_pair_list[i].first.IsValid(facade->GetNumberOfNodes()));
            BOOST_ASSERT(phantom_node_pair_list[i].second.IsValid(facade->GetNumberOfNodes()));
        }
		/*
        auto snapped_phantoms = snapPhantomNodes(phantom_node_pair_list);

        InternalRouteResult raw_route;
        auto build_phantom_pairs =
            [&raw_route](const PhantomNode &first_node, const PhantomNode &second_node)
        {
            raw_route.segment_end_coordinates.push_back(PhantomNodes{first_node, second_node});
        };
        util::for_each_pair(snapped_phantoms, build_phantom_pairs);

        if (1 == raw_route.segment_end_coordinates.size())
        {
            if (route_parameters.alternate_route)
            {
                search_engine_ptr->alternative_path(raw_route.segment_end_coordinates.front(),
                                                    raw_route);
            }
            else
            {
                search_engine_ptr->direct_shortest_path(raw_route.segment_end_coordinates,
                                                        route_parameters.uturns, raw_route);
            }
        }
        else
        {
            search_engine_ptr->shortest_path(raw_route.segment_end_coordinates,
                                             route_parameters.uturns, raw_route);
        }

        // we can only know this after the fact, different SCC ids still
        // allow for connection in one direction.
        if (raw_route.is_valid())
        {
            auto generator = MakeApiResponseGenerator(facade);
            generator.DescribeRoute(route_parameters, raw_route, json_result);
            json_result.values["status_message"] = "Found route between points";
        }
        else
        {
            auto first_component_id = snapped_phantoms.front().component.id;
            auto not_in_same_component =
                std::any_of(snapped_phantoms.begin(), snapped_phantoms.end(),
                            [first_component_id](const PhantomNode &node)
                            {
                                return node.component.id != first_component_id;
                            });

            if (not_in_same_component)
            {
                json_result.values["status_message"] = "Impossible route between points";
                return Status::EmptyResult;
            }
            else
            {
                json_result.values["status_message"] = "No route found between points";
                return Status::Error;
            }
        }*/
        return Status::Ok;
    }
};
}
}
}

#endif // ISOCHRONE_HPP
