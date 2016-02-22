#ifndef AREA_PLUGIN_H
#define AREA_PLUGIN_H
#if defined(_MSC_VER) && (_MSC_VER >= 1020)
# pragma once
#endif

#include "BasePlugin.h"
#include "../DataStructures/JSONContainer.h"

#include "../Algorithms/ObjectToBase64.h"
#include "../Algorithms/ConcaveHull.h"

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
#include <boost/unordered_map.hpp>
#include <string>
#include <vector>
#include <queue>

#define RANGE_LIMIT_SHIFT   10
#define RANGE_LIMIT_MINUS_ONE   ((1 << RANGE_LIMIT_SHIFT)-1)


template <class DataFacadeT> class AreaPlugin : public BasePlugin
{
private:
	struct _AddedData {
		int						m_time;
		FixedPointCoordinate	m_coords_start;
		FixedPointCoordinate	m_coords_end;

		_AddedData() : m_time(0) {}
		_AddedData(int time, const FixedPointCoordinate& start, const FixedPointCoordinate& end) : m_time(time), m_coords_start(start), m_coords_end(end) {}
	};

	struct _QueueNodeData {
		NodeID				m_nodeId;
		struct _AddedData	m_data;

		_QueueNodeData(NodeID nodeId, 
					const FixedPointCoordinate& start, 
					const FixedPointCoordinate& end,
					int time) : m_nodeId(nodeId), m_data(time, start, end) {
		};

		_QueueNodeData(NodeID nodeId, const struct _AddedData& data) : m_nodeId(nodeId), m_data(data) {
		};
	};

	struct AllSessionInfo {
		int									m_max_time;
		std::set<FixedPointCoordinate>		m_coordinates;
		boost::unordered_map<NodeID, int>	m_added;
		boost::unordered_map<NodeID, struct _AddedData> m_nodes;
		int									m_ranges_num;
		std::queue<struct _QueueNodeData>*	m_node_queues_ptr;

		AllSessionInfo() : m_max_time(0), m_ranges_num(0), m_node_queues_ptr(0) {}
	};

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



    bool GetCoordsForEdgeID(NodeID edge_id, 
                            FixedPointCoordinate& coord_out_start, 
                            FixedPointCoordinate& coord_out_end,
                            std::set<FixedPointCoordinate>& coordinates)
    {
        if (!facade->EdgeIsCompressed(edge_id))
        {
            NodeID to_g = facade->GetGeometryIndexForEdgeID(edge_id);
            coord_out_start = facade->GetCoordinateOfNode(to_g);
            coord_out_end = coord_out_start;
            return true;
        }
        else
        {
            ///// пока хз что делать с пачкой координат, отдаем 0-ую
            std::vector<unsigned> id_vector;
            facade->GetUncompressedGeometry(facade->GetGeometryIndexForEdgeID(edge_id),
                                            id_vector);
            if (!id_vector.empty()) {
                coord_out_start = facade->GetCoordinateOfNode(id_vector[0]);
                
                FixedPointCoordinate coord_prev = coord_out_start;
                for (int ii = 1; ii < id_vector.size(); ++ii) {
                    FixedPointCoordinate to_g_coord = facade->GetCoordinateOfNode(id_vector[ii]);
                    coordinates.insert(to_g_coord);

                    check_distance(coord_prev, to_g_coord, coordinates);
                    coord_prev = to_g_coord;
                }

                coord_out_end = coord_prev;

                return true;
            }
        }

        return false;
    }

    void HandleRequest(const RouteParameters &route_parameters, http::Reply &reply)
    {
        // check parameters
        if (route_parameters.time == 0 ||
            1 != route_parameters.coordinates.size() ||
            !route_parameters.coordinates[0].isValid())
        {
            reply = http::Reply::StockReply(http::Reply::badRequest);
            SimpleLogger().Write(logINFO) << "bad request";
            return;
        }
		clock_t begin = clock();

        unsigned short route_time_limit = route_parameters.time * 10;

        //// DEBUG ONLY
//      route_time_limit = 100;
        ////

		AllSessionInfo session_info;
		session_info.m_max_time = route_time_limit;


        PhantomNode startPhantomNode;
        // A phantom node is a point on the closest edge based node, where the route starts or ends
        facade->FindPhantomNodeForCoordinate(route_parameters.coordinates[0],
											startPhantomNode,
                                            route_parameters.zoom_level);
        
//		NodeID nodeId = startPhantomNode.forward_node_id;
//        if (UINT_MAX == nodeId)
//			nodeId = startPhantomNode.reverse_node_id;

		/////////
		//// мы разбиваем время на куски по RANGE_LIMIT и не переходим к следующей очереди пока в предыдущих есть чего
        session_info.m_ranges_num = (route_parameters.time + RANGE_LIMIT_MINUS_ONE) >> RANGE_LIMIT_SHIFT;
		session_info.m_node_queues_ptr = new std::queue<struct _QueueNodeData>[session_info.m_ranges_num];
        
        //
		if (UINT_MAX != startPhantomNode.forward_node_id || UINT_MAX != startPhantomNode.reverse_node_id)
        {
			FixedPointCoordinate first_node_start = startPhantomNode.location;
			FixedPointCoordinate first_node_end = startPhantomNode.location;

			if (startPhantomNode.packed_geometry_id != UINT_MAX) 
			{
				std::vector<unsigned> id_vector;
				facade->GetUncompressedGeometry(startPhantomNode.packed_geometry_id, id_vector);
				if (!id_vector.empty()) {
					int vector_size = id_vector.size();
					for (int ii = 0; ii < vector_size; ++ii) {
						FixedPointCoordinate to_g_coord = facade->GetCoordinateOfNode(id_vector[ii]);
						session_info.m_coordinates.insert(to_g_coord);
						if (0 == ii)
							first_node_start = to_g_coord;
						if (vector_size-1 == ii)
							first_node_end = to_g_coord;
					}
				}
			}
			else {
				session_info.m_coordinates.insert(startPhantomNode.location);
			}

			//
			if (UINT_MAX != startPhantomNode.forward_node_id) {
				add_to_queue(startPhantomNode.forward_node_id,
							first_node_start, first_node_end, 0,
							session_info);
			}
			if (UINT_MAX != startPhantomNode.reverse_node_id) {
				add_to_queue(startPhantomNode.reverse_node_id,
							first_node_start, first_node_end, 0,
							session_info);
			}
        }

		///////////////////////////////////////////////////////
#ifdef _DEBUG
        int n = 0;
#endif

        int cur_queue_index = 0;
		while (cur_queue_index < session_info.m_ranges_num)
        {
            std::queue<struct _QueueNodeData>* cur_queue_ptr = &(session_info.m_node_queues_ptr[cur_queue_index]);
            cur_queue_index++;

            // процессим все точки в очереди
            while (!cur_queue_ptr->empty())
            {
                struct _QueueNodeData qd = cur_queue_ptr->front();
                cur_queue_ptr->pop();

                AddNextNode(qd.m_nodeId, qd.m_data, session_info);

#ifdef _DEBUG
                n++;
//              if (n == 6)
//                  session_info.m_coordinates.clear();
//              if (n > 6)
//                  break;
#endif
            }
        }
        
        // добавляем точки в список для ответа
    
        // TODO: тут надо пробежаться и те точки которые уже в points засунуьт в set чтобы они повторно не залезли
        
        //
		for (typename boost::unordered_map<NodeID, struct _AddedData>::iterator it = session_info.m_nodes.begin(); it != session_info.m_nodes.end(); it++) {
            const FixedPointCoordinate& coord_start = (*it).second.m_coords_start;
			if (session_info.m_coordinates.find(coord_start) == session_info.m_coordinates.end())
				session_info.m_coordinates.insert(coord_start);
            
            const FixedPointCoordinate& coord_end = (*it).second.m_coords_end;
            if (coord_start == coord_end)
                continue;
			if (session_info.m_coordinates.find(coord_end) == session_info.m_coordinates.end())
				session_info.m_coordinates.insert(coord_end);
        }

        //
		delete[] session_info.m_node_queues_ptr; session_info.m_node_queues_ptr = 0;
        
		clock_t end = clock();
		double elapsed = double(end - begin) / CLOCKS_PER_SEC;

		SimpleLogger().Write(logINFO) << "Found " << session_info.m_coordinates.size() << " points in " << elapsed << " s.";

        
        ///// Reply to request
        reply.status = http::Reply::ok;
        
        // output
        JSON::Object json_result;
        std::string temp_string;

        JSON::Array json_locations;
        unsigned counter = 0;

		std::vector<FixedPointCoordinate> hull;
		if (route_parameters.zoom_level != 1) {
			concaveHull(session_info.m_coordinates, hull);
			clock_t end1 = clock();
			elapsed = double(end1 - begin) / CLOCKS_PER_SEC;
			double elapsed1 = double(end1 - end) / CLOCKS_PER_SEC;

			SimpleLogger().Write(logINFO) << "Hull : " << hull.size() << " points in " << elapsed1 << " s. Total "<< elapsed << " s.";

			for (int i = 0; i < hull.size(); i++)
			{
				JSON::Array json_coordinates;

				json_coordinates.values.push_back(hull[i].lat / COORDINATE_PRECISION);
				json_coordinates.values.push_back(hull[i].lon / COORDINATE_PRECISION);
				json_locations.values.push_back(json_coordinates);
				++counter;
			}
		}
		else {

			for (auto c : session_info.m_coordinates){
				JSON::Array json_coordinates;
				json_coordinates.values.push_back(c.lat / COORDINATE_PRECISION);
				json_coordinates.values.push_back(c.lon / COORDINATE_PRECISION);
				json_locations.values.push_back(json_coordinates);
				++counter;
			}
		}
		json_result.values["points"] = json_locations;

        JSON::render(reply.content, json_result);
        
    }

private:
    
    void AddNextNode(NodeID nodeId,
					const _AddedData& coord_data,
					AllSessionInfo& session_info)
    {
		int cur_time = coord_data.m_time;

		if (session_info.m_added.find(nodeId) != session_info.m_added.end()) {
			if (session_info.m_added[nodeId] <= cur_time)
                return;
        }
		session_info.m_added[nodeId] = cur_time;

        //// проверим что у нас есть хотя бы одна edge по которой можно пойти
        bool check_forward = false;
//      for (StaticGraph<QueryEdge::EdgeData, true>::EdgeIterator edge_check = facade->BeginEdges(nodeId); edge_check < facade->EndEdges(nodeId); ++edge_check) {
//          const QueryEdge::EdgeData & data = facade->GetEdgeData(edge_check);
//          if (data.forward) {
//              check_forward = true;
//              break;
//          }
//      }

        //
        for (StaticGraph<QueryEdge::EdgeData>::EdgeIterator edge = facade->BeginEdges(nodeId); edge < facade->EndEdges(nodeId); ++edge)
        {
            const QueryEdge::EdgeData & data = facade->GetEdgeData(edge);
            const NodeID target_node_id = facade->GetTarget(edge);
            const NodeID to = data.id;

//          // не понял надо это или нет, без этой проверки вроде потив шерсти просчитывает
//          if (check_forward && !data.forward) {
//              continue;       //!!! заказчик попросил игнорировать "против шерсти"
//          }

            if (!data.shortcut)
            {
                //// ищем координаты в map'е, если их еще нет, то получаем и сохраняем.
                //// Насколько я понимаю это координаты для target_node_id, который пойдет в add_to_queue, т.к. по самому target_node_id не найти координаты - передаем в add_to_queue(...)
                FixedPointCoordinate coord_of_node_start;
                FixedPointCoordinate coord_of_node_end;
		typename boost::unordered_map<NodeID, struct _AddedData>::iterator mit = session_info.m_nodes.find(to);
		if (session_info.m_nodes.end() == mit)
                {
		    if (GetCoordsForEdgeID(to, coord_of_node_start, coord_of_node_end, session_info.m_coordinates))
                    {
                        struct _AddedData add_data(cur_time + data.distance, coord_of_node_start, coord_of_node_end);
						session_info.m_nodes[to] = add_data;

						check_distance(coord_data, add_data, session_info.m_coordinates);
                    }
                    else {
                        //// такого не должно случиться, если попали сюда - где то косяк
                        continue;
                    }
                }
                else {
                    coord_of_node_start = mit->second.m_coords_start;
                    coord_of_node_end = mit->second.m_coords_end;

                    // вроде и тут нужен check_distance(баг про разрывы)
					check_distance(coord_data, 
								_AddedData(0, coord_of_node_start, coord_of_node_end), 
								session_info.m_coordinates);
                }

                //
                if (cur_time + data.distance < session_info.m_max_time)
					add_to_queue(target_node_id, 
								coord_of_node_start, coord_of_node_end, 
								cur_time + data.distance, 
								session_info);
            }
            else
            {
                FixedPointCoordinate out_coord_of_target_start;        // здесь должна оказаться координата для target, чтобы правильно ее добавить вместе с самим target_node_id в add_to_queue(...)
				FixedPointCoordinate out_coord_of_target_end;
                ProcessShortcut(nodeId,
								coord_data.m_coords_start,
								coord_data.m_coords_end,
								data.id, 
								target_node_id,
								out_coord_of_target_start,
								out_coord_of_target_end,
								cur_time,
								session_info);

                ////////// т.к. мы в ProcessShortcut добавляем точки в очередь, то вот это добавление возможно и не нужно(тем более что не ясно как получать coord_of_target)
                // между nodeId и шоткатом тоже указывается расстояние, которое потом складывается из внутренних растояний
				if (cur_time + data.distance < session_info.m_max_time)
					add_to_queue(target_node_id, 
								out_coord_of_target_start, out_coord_of_target_end, 
								cur_time + data.distance, 
								session_info);
            }
        }
    }

    
    void add_to_queue(NodeID nodeId,
					const FixedPointCoordinate& start, 
					const FixedPointCoordinate& end,
					int cur_time,
					AllSessionInfo& session_info)
    {
#ifdef _DEBUG
        if (nodeId == 4294967295) {
            int zzz = 1;
        }
#endif

		if (session_info.m_added.find(nodeId) != session_info.m_added.end()) {
			if (session_info.m_added[nodeId] <= cur_time)
                return;
        }

		int range = get_range_by_cur_time(session_info.m_ranges_num, cur_time);

		struct _QueueNodeData qd(nodeId, start, end, cur_time);
		session_info.m_node_queues_ptr[range].push(qd);
        
    }

    void ProcessShortcut(NodeID start_node_id,
						const FixedPointCoordinate& coord_of_start_node_start,
						const FixedPointCoordinate& coord_of_start_node_end,
						NodeID shortcut_node_id,
						NodeID end_node_id,
						FixedPointCoordinate& out_coord_of_end_start,       // <<< out
						FixedPointCoordinate& out_coord_of_end_end,       // <<< out
						int cur_time,
						AllSessionInfo& session_info)
    {
        //    std::cout << "------" << std::endl;
        //    std::cout << "PROCESS shortcut: startId=" << start_node_id << " (" << coord_of_start_node_start.lat << "," << coord_of_start_node_start.lon << ")" << "   middleId=" << shortcut_node_id << "  endId=" << end_node_id << std::endl;

        FixedPointCoordinate coords_of_shortcut_start;     // координаты для центральной точки ищем в первом if, используем во втором
		FixedPointCoordinate coords_of_shortcut_end;

        //    std::cout << " search smallest edge between start_node_id and middle" << std::endl;
		unsigned eid2 = facade->FindEdgeInEitherDirection(start_node_id, shortcut_node_id);
        if (eid2 != UINT_MAX)
        {
            const QueryEdge::EdgeData & data2 = facade->GetEdgeData(eid2);
            const NodeID target2 = facade->GetTarget(eid2);

            if (!data2.shortcut)
            {
                //// ищем координаты в map'е, если их еще нет, то получаем и сохраняем.
                //// Насколько я понимаю это координаты для target2, который пойдет в add_to_queue, т.к. по самому target2 не найти координаты - передаем в add_to_queue(...)
                FixedPointCoordinate coord_of_node_start;
                FixedPointCoordinate coord_of_node_end;
                typename boost::unordered_map<NodeID, struct _AddedData>::iterator mit = session_info.m_nodes.find(data2.id);
				if (session_info.m_nodes.end() == mit)
                {
					if (GetCoordsForEdgeID(data2.id, coord_of_node_start, coord_of_node_end, session_info.m_coordinates))
                    {
                        struct _AddedData add_data(cur_time + data2.distance, coord_of_node_start, coord_of_node_end);
						session_info.m_nodes[data2.id] = add_data;

						check_distance(_AddedData(0, coord_of_start_node_start, coord_of_start_node_end), 
									add_data, 
									session_info.m_coordinates);
                    }
                    else {
                        assert(false);
                        return;
                    }
                }
                else {
                    coord_of_node_start = mit->second.m_coords_start;
                    coord_of_node_end = mit->second.m_coords_end;

					// хз надо ли?
					check_distance(_AddedData(0, coord_of_start_node_start, coord_of_start_node_end),
								_AddedData(0, coord_of_node_start, coord_of_node_end),
								session_info.m_coordinates);
                }

				coords_of_shortcut_start = coord_of_node_start;
				coords_of_shortcut_end = coord_of_node_end;

				if (cur_time + data2.distance < session_info.m_max_time)
					add_to_queue(target2, 
								coord_of_node_start, coord_of_node_end, 
								cur_time + data2.distance, 
								session_info);
            }
            else {
				ProcessShortcut(start_node_id,
								coord_of_start_node_start,
								coord_of_start_node_end,
								data2.id, 
								shortcut_node_id,
								coords_of_shortcut_start,		// out 
								coords_of_shortcut_end,		// out
								cur_time,
								session_info);
            }

            // если здесь, то путь до шотката прописали, учитываем это расстояние
            cur_time += data2.distance;
        }

        // нет смысла двигаться дальше полученой точки если время до нее уже более максимума
		if (cur_time >= session_info.m_max_time)
            return;

        //
        //    std::cout << " search smallest edge between middle and end_node_id" << std::endl;

		eid2 = facade->FindEdgeInEitherDirection(shortcut_node_id, end_node_id);
        if (eid2 != UINT_MAX)
        {
            const QueryEdge::EdgeData & data2 = facade->GetEdgeData(eid2);
            const NodeID target2 = facade->GetTarget(eid2);

            if (!data2.shortcut)
            {
                //// ищем координаты в map'е, если их еще нет, то получаем и сохраняем.
                //// Насколько я понимаю это координаты для target2, который пойдет в add_to_queue, т.к. по самому target2 не найти координаты - передаем в add_to_queue(...)
                FixedPointCoordinate coord_of_node_start;
                FixedPointCoordinate coord_of_node_end;
				typename boost::unordered_map<NodeID, struct _AddedData>::iterator mit = session_info.m_nodes.find(data2.id);
				if (session_info.m_nodes.end() == mit)
                {
					if (GetCoordsForEdgeID(data2.id, coord_of_node_start, coord_of_node_end, session_info.m_coordinates))
                    {
                        struct _AddedData add_data(cur_time + data2.distance, coord_of_node_start, coord_of_node_end);
						session_info.m_nodes[data2.id] = add_data;

						check_distance(_AddedData(0, coords_of_shortcut_start, coords_of_shortcut_end), 
									add_data, 
									session_info.m_coordinates);
                    }
                    else {
                        return;
                    }
                }
                else {
                    coord_of_node_start = mit->second.m_coords_start;
                    coord_of_node_end = mit->second.m_coords_end;

					// хз надо ли?
					check_distance(_AddedData(0, coords_of_shortcut_start, coords_of_shortcut_end),
								_AddedData(0, coord_of_node_start, coord_of_node_end),
								session_info.m_coordinates);
                }

				out_coord_of_end_start = coord_of_node_start;
                out_coord_of_end_end = coord_of_node_end;

				if (cur_time + data2.distance < session_info.m_max_time)
					add_to_queue(target2, 
								coord_of_node_start, coord_of_node_end, 
								cur_time + data2.distance, 
								session_info);
            }
            else {
				ProcessShortcut(shortcut_node_id,
								coords_of_shortcut_start,
								coords_of_shortcut_end,
								data2.id, 
								end_node_id,
								out_coord_of_end_start,
								out_coord_of_end_end,
								cur_time,
								session_info);
            }

            // если здесь, то путь до шотката прописали, учитываем это расстояние
            cur_time += data2.distance;
        }
    }

	inline double quick_sortdist(const FixedPointCoordinate& c1, const FixedPointCoordinate& c2)
	{
		if (abs(c1.lat - c2.lat) > 40000 || abs(c1.lon - c2.lon) > 40000)		// пїЅпїЅпїЅпїЅпїЅ пїЅпїЅ пїЅпїЅпїЅпїЅпїЅпїЅпїЅпїЅпїЅпїЅ пїЅпїЅпїЅпїЅпїЅпїЅпїЅпїЅпїЅпїЅ
			return 9999999.0;

		return (c1.lat - c2.lat)*(c1.lat - c2.lat) + (c1.lon - c2.lon)*(c1.lon - c2.lon);
	}

	///////////////////////////////////////////////////////////////////////////////////////////
	//// Search nearest points and call check_distance for them
	void check_distance(const _AddedData& data1, const _AddedData& data2, std::set<FixedPointCoordinate>& coordinates)
	{
		double ranges[4];
		ranges[0] = quick_sortdist(data1.m_coords_start, data2.m_coords_start);
		ranges[1] = quick_sortdist(data1.m_coords_start, data2.m_coords_end);
		ranges[2] = quick_sortdist(data1.m_coords_end, data2.m_coords_start);
		ranges[3] = quick_sortdist(data1.m_coords_end, data2.m_coords_end);

		double* min_pos = std::min_element(&ranges[0], &ranges[4]);
		int min_index = min_pos - &ranges[0];

		switch (min_index)
		{
			case 0:
				check_distance(data1.m_coords_start, data2.m_coords_start, coordinates);
				break;
			case 1:
				check_distance(data1.m_coords_start, data2.m_coords_end, coordinates);
				break;
			case 2:
				check_distance(data1.m_coords_end, data2.m_coords_start, coordinates);
				break;
			case 3:
				check_distance(data1.m_coords_end, data2.m_coords_end, coordinates);
				break;
		}
	}

	///////////////////////////////////////////////////////////////////////////////
	//// fill range from c1 coordinate to c2 coordinate with points
    void check_distance(const FixedPointCoordinate& c1, const FixedPointCoordinate& c2, std::set<FixedPointCoordinate>& coordinates)
    {
        if (c1.lat == INT_MIN)      // что то пошло не так и точку не нашли
            return;
        if (c2.lat == INT_MIN)      // что то пошло не так и точку не нашли
            return;

        double r = FixedPointCoordinate::ApproximateEuclideanDistance(c1, c2);
        int rn = (int)r;
        if (r <= MAX_POINTS_DIST)
            return;

        //// здесь надо разбивать на несколько частей
        int n = (rn + MAX_POINTS_DIST - 1) / MAX_POINTS_DIST;
        int dX = (c2.lat - c1.lat) / n;
        int dY = (c2.lon - c1.lon) / n;

        for (int ii = 1; ii < n; ++ii)
        {
            FixedPointCoordinate test_coord(c1.lat + ii*dX, c1.lon + ii*dY);
            if (coordinates.find(test_coord) == coordinates.end())
            {
                coordinates.insert(test_coord);
            }
        }
    }

	int get_range_by_cur_time(int ranges_num, int cur_time)
	{
		int rang = cur_time >> RANGE_LIMIT_SHIFT;
		if (rang >= ranges_num)
			rang = ranges_num - 1;
		return rang;
	}


private:
    std::string     descriptor_string;
    DataFacadeT *   facade;
};

#endif // AREA_PLUGIN_H