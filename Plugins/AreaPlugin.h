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

struct _QueueNodeData {
    NodeID      m_nodeId;
    FixedPointCoordinate m_coords;
    int         m_time;

    _QueueNodeData(NodeID nodeId, const FixedPointCoordinate& c, int time) : m_nodeId(nodeId), m_time(time), m_coords(c) {};
};

struct _AddedData {
    int         m_time;
    FixedPointCoordinate m_coords_start;
    FixedPointCoordinate m_coords_end;

    _AddedData() : m_time(0) {}
    _AddedData(int time, FixedPointCoordinate& start, FixedPointCoordinate& end) : m_time(time), m_coords_start(start), m_coords_end(end) {}
};

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

        m_start_phantom_is_reverse = false;
    }

    virtual ~AreaPlugin() {}

    const std::string GetDescriptor() const { return descriptor_string; }

    bool GetCoordsForEdgeID(NodeID edge_id, 
                            FixedPointCoordinate& coord_out_start, 
                            FixedPointCoordinate& coord_out_end,
                            std::set<FixedPointCoordinate>& coordinates, 
                            bool& compressed_flag)      // std::set<FixedPointCoordinate>& coordinates)
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
            compressed_flag = true;

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

        unsigned short route_time_limit = route_parameters.time * 20;

        //// DEBU ONLY
//      route_time_limit = 100;
        ////

        m_start_phantom_is_reverse = false;
        PhantomNode startPhantom;
        // A phantom node is a point on the closest edge based node, where the route starts or ends
        facade->FindPhantomNodeForCoordinate(route_parameters.coordinates[0],
                                            startPhantom,
                                            route_parameters.zoom_level);
        
        std::set<FixedPointCoordinate> coordinates;
        coordinates.insert(startPhantom.location);

        NodeID nodeId = startPhantom.forward_node_id;
        if (UINT_MAX == nodeId) {
            nodeId = startPhantom.reverse_node_id;
            
            m_start_phantom_is_reverse = true;                  // в этом случае у нас артефакты прут, надо думать почему и чинить. пока заткнул флагом
        }

        if (UINT_MAX != nodeId) {
            if (startPhantom.packed_geometry_id != UINT_MAX) {
                std::vector<unsigned> id_vector;
                facade->GetUncompressedGeometry(startPhantom.packed_geometry_id, id_vector);
                if (!id_vector.empty()) {
                    for (int ii = 0; ii < id_vector.size(); ++ii) {
                        FixedPointCoordinate to_g_coord = facade->GetCoordinateOfNode(id_vector[ii]);
                        coordinates.insert(to_g_coord);
                    }
                }
            }
        }

        boost::unordered_map<NodeID, int> added;
        boost::unordered_map<NodeID, struct _AddedData> nodes;

        // мы разбиваем время на куски по RANGE_LIMIT и не переходим к следующей очереди пока в предыдущих есть чего

        int ranges_num = (route_parameters.time + RANGE_LIMIT_MINUS_ONE) >> RANGE_LIMIT_SHIFT;
        std::queue<struct _QueueNodeData>* node_queues_ptr = new std::queue<struct _QueueNodeData>[ranges_num];
        
        //
        // TODO: надо оптимизировать передачу параметров через структуру
        if (UINT_MAX != nodeId)
        {
            add_to_queue(nodeId,
                startPhantom.location,
                0,
                ranges_num,
                node_queues_ptr,
                added);
        }

#ifdef _DEBUG
        int n = 0;
#endif

        int cur_queue_index = 0;
        while (cur_queue_index < ranges_num)
        {
            std::queue<struct _QueueNodeData>* cur_queue_ptr = &node_queues_ptr[cur_queue_index];
            cur_queue_index++;

            // процессим все точки в очереди
            while (!cur_queue_ptr->empty())
            {
                struct _QueueNodeData qd = cur_queue_ptr->front();
                cur_queue_ptr->pop();

                // TODO: надо оптимизировать передачу параметров через структуру
                AddNextNode(qd.m_nodeId,
                    qd.m_coords,
                    nodes,
                    qd.m_time,
                    route_time_limit,
                    added,
                    ranges_num,
                    node_queues_ptr, 
                    coordinates);

#ifdef _DEBUG
                n++;
//              if (n == 6)
//                  coordinates.clear();
//              if (n > 6)
//                  break;
#endif
            }
        }
        
        // добавляем точки в список для ответа
    
        // TODO: тут надо пробежаться и те точки которые уже в points засунуьт в set чтобы они повторно не залезли
        
        //
        for (boost::unordered_map<NodeID, struct _AddedData>::iterator it = nodes.begin(); it != nodes.end(); it++) {
            const FixedPointCoordinate& coord_start = (*it).second.m_coords_start;
            if (coordinates.find(coord_start) == coordinates.end())
                coordinates.insert(coord_start);
            
            const FixedPointCoordinate& coord_end = (*it).second.m_coords_end;
            if (coord_start == coord_end)
                continue;
            if (coordinates.find(coord_end) == coordinates.end())
                coordinates.insert(coord_end);
        }

        //
        delete[] node_queues_ptr; node_queues_ptr = 0;
        
        
        SimpleLogger().Write(logINFO) << "Num points: " << coordinates.size();

        
        ///// Reply to request
        reply.status = http::Reply::ok;
        
        // output
        JSON::Object json_result;
        std::string temp_string;

        JSON::Array json_locations;
        unsigned counter = 0;
		std::vector<FixedPointCoordinate> hull;
		concaveHull(coordinates, hull);
		for (int i = 0; i < hull.size(); i++)
        {
            JSON::Array json_coordinates;

			json_coordinates.values.push_back(hull[i].lat / COORDINATE_PRECISION);
			json_coordinates.values.push_back(hull[i].lon / COORDINATE_PRECISION);
            json_locations.values.push_back(json_coordinates);
            ++counter;
        }
        json_result.values["points"] = json_locations;

        JSON::render(reply.content, json_result);
        
    }

private:
    
    void AddNextNode(NodeID nodeId,
        const FixedPointCoordinate& coord_of_nodeId,
        boost::unordered_map<NodeID, struct _AddedData>& nodes,
        int cur_time,
        int max_time,
        boost::unordered_map<NodeID, int>& added,
        int ranges_num,
        std::queue<struct _QueueNodeData>* node_queues_ptr,
        std::set<FixedPointCoordinate>& coordinates)
    {
        if (added.find(nodeId) != added.end()) {
            if (added[nodeId] <= cur_time)
                return;
        }
        added[nodeId] = cur_time;

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
                boost::unordered_map<NodeID, struct _AddedData>::iterator mit = nodes.find(to);
                if (nodes.end() == mit)
                {
                    //coord_of_node = facade->GetCoordinateOfNode(to);
                    bool compressed_flag = false;
                    if (GetCoordsForEdgeID(to, coord_of_node_start, coord_of_node_end, coordinates, compressed_flag))
                    {
                        struct _AddedData add_data(cur_time + data.distance, coord_of_node_start, coord_of_node_end);
                        nodes[to] = add_data;

                        //                if (cur_time+data.distance < max_time)
                        if (!compressed_flag) {
                            if (m_start_phantom_is_reverse)
                                m_start_phantom_is_reverse = false;
                            else
                                check_distance(coord_of_nodeId, coord_of_node_end, coordinates);
                        }
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
                }

                //
                if (cur_time + data.distance < max_time)
                    add_to_queue(target_node_id, coord_of_node_end, cur_time + data.distance, ranges_num, node_queues_ptr, added);
            }
            else
            {
                FixedPointCoordinate coord_of_target;        // здесь должна оказаться координата для target, чтобы правильно ее добавить вместе с самим target_node_id в add_to_queue(...)
                ProcessShortcut(nodeId,
                    coord_of_nodeId,
                    data.id, target_node_id,
                    coord_of_target,
                    nodes,
                    cur_time,
                    max_time,
                    added,
                    ranges_num,
                    node_queues_ptr,
                    coordinates);

                ////////// т.к. мы в ProcessShortcut добавляем точки в очередь, то вот это добавление возможно и не нужно(тем более что не ясно как получать coord_of_target)
                // между nodeId и шоткатом тоже указывается расстояние, которое потом складывается из внутренних растояний
                if (cur_time + data.distance < max_time)
                    add_to_queue(target_node_id, coord_of_target, cur_time + data.distance, ranges_num, node_queues_ptr, added);
            }
        }
    }

    
    void add_to_queue(NodeID nodeId,
        const FixedPointCoordinate& coord_of_nodeId,
        int cur_time,
        int ranges_num,
        std::queue<struct _QueueNodeData>* node_queues_ptr,
        boost::unordered_map<NodeID, int>& added)
    {
#ifdef _DEBUG
        if (nodeId == 4294967295) {
            int zzz = 1;
        }
#endif

        if (added.find(nodeId) != added.end()) {
            if (added[nodeId] <= cur_time)
                return;
        }

        int range = get_range_by_cur_time(ranges_num, cur_time);

        struct _QueueNodeData qd(nodeId, coord_of_nodeId, cur_time);
        node_queues_ptr[range].push(qd);
        
    }

    int get_range_by_cur_time(int ranges_num, int cur_time)
    {
        int rang = cur_time >> RANGE_LIMIT_SHIFT;
        if (rang >= ranges_num)
            rang = ranges_num - 1;
        return rang;
    }

    void ProcessShortcut(NodeID start,
        const FixedPointCoordinate& coord_of_start,
        NodeID shortcut, NodeID end,
        FixedPointCoordinate& out_coord_of_end,       // <<< out
        boost::unordered_map<NodeID, struct _AddedData>& nodes,
        int cur_time,
        int max_time,
        boost::unordered_map<NodeID, int>& added,
        int ranges_num,
        std::queue<struct _QueueNodeData>* node_queues_ptr,
        std::set<FixedPointCoordinate>& coordinates)
    {
        //    std::cout << "------" << std::endl;
        //    std::cout << "PROCESS shortcut: startId=" << start << " (" << coord_of_start.lat << "," << coord_of_start.lon << ")" << "   middleId=" << shortcut << "  endId=" << end << std::endl;

        FixedPointCoordinate coords_of_shortcut;     // координаты для центральной точки ищем в первом if, используем во втором

        //    std::cout << " search smallest edge between start and middle" << std::endl;
        unsigned eid2 = facade->FindEdgeInEitherDirection(start, shortcut);
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
                boost::unordered_map<NodeID, struct _AddedData>::iterator mit = nodes.find(data2.id);
                if (nodes.end() == mit)
                {
                    bool compressed_flag = false;
                    if (GetCoordsForEdgeID(data2.id, coord_of_node_start, coord_of_node_end, coordinates, compressed_flag))
                    {
                        struct _AddedData add_data(cur_time + data2.distance, coord_of_node_start, coord_of_node_end);
                        nodes[data2.id] = add_data;

                        if (!compressed_flag) {
                            if (m_start_phantom_is_reverse)
                                m_start_phantom_is_reverse = false;
                            else
                                check_distance(coord_of_start, coord_of_node_start, coordinates);
                        }
                    }
                    else {
                        assert(false);
                        return;
                    }
                }
                else {
                    coord_of_node_start = mit->second.m_coords_start;
                    coord_of_node_end = mit->second.m_coords_end;
                }

                coords_of_shortcut = coord_of_node_end;

                if (cur_time + data2.distance < max_time)
                    add_to_queue(target2, coord_of_node_end, cur_time + data2.distance, ranges_num, node_queues_ptr, added);
            }
            else {
                ProcessShortcut(start,
                    coord_of_start,
                    data2.id, shortcut,
                    coords_of_shortcut,
                    nodes,
                    cur_time,
                    max_time,
                    added,
                    ranges_num,
                    node_queues_ptr,
                    coordinates);
            }

            // если здесь, то путь до шотката прописали, учитываем это расстояние
            cur_time += data2.distance;
        }

        // нет смысла двигаться дальше полученой точки если время до нее уже более максимума
        if (cur_time >= max_time)
            return;

        //
        //    std::cout << " search smallest edge between middle and end" << std::endl;

        eid2 = facade->FindEdgeInEitherDirection(shortcut, end);
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
                boost::unordered_map<NodeID, struct _AddedData>::iterator mit = nodes.find(data2.id);
                if (nodes.end() == mit)
                {
                    bool compressed_flag = false;
                    if (GetCoordsForEdgeID(data2.id, coord_of_node_start, coord_of_node_end, coordinates, compressed_flag))
                    {
                        struct _AddedData add_data(cur_time + data2.distance, coord_of_node_start, coord_of_node_end);
                        nodes[data2.id] = add_data;

                        if (!compressed_flag) {
                            if (m_start_phantom_is_reverse)
                                m_start_phantom_is_reverse = false;
                            else
                                check_distance(coords_of_shortcut, coord_of_node_start, coordinates);
                        }
                    }
                    else {
                        return;
                    }
                }
                else {
                    coord_of_node_start = mit->second.m_coords_start;
                    coord_of_node_end = mit->second.m_coords_end;
                }

                out_coord_of_end = coord_of_node_end;        // !!! вроде бы это координаты конца(то что пришло в качестве параметра end), но не уверен

                if (cur_time + data2.distance < max_time)
                    add_to_queue(target2, coord_of_node_end, cur_time + data2.distance, ranges_num, node_queues_ptr, added);
            }
            else {
                ProcessShortcut(shortcut,
                    coords_of_shortcut,
                    data2.id, end,
                    out_coord_of_end,
                    nodes,
                    cur_time,
                    max_time,
                    added,
                    ranges_num,
                    node_queues_ptr,
                    coordinates);
            }

            // если здесь, то путь до шотката прописали, учитываем это расстояние
            cur_time += data2.distance;
        }
    }

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


private:
    std::string     descriptor_string;
    DataFacadeT *   facade;
    bool            m_start_phantom_is_reverse;
};

#endif // AREA_PLUGIN_H