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
            ///// ���� �� ��� ������ � ������ ���������, ������ 0-��
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
        
		NodeID nodeId = startPhantomNode.forward_node_id;
        if (UINT_MAX == nodeId)
			nodeId = startPhantomNode.reverse_node_id;

		/////////
		//// �� ��������� ����� �� ����� �� RANGE_LIMIT � �� ��������� � ��������� ������� ���� � ���������� ���� ����
        session_info.m_ranges_num = (route_parameters.time + RANGE_LIMIT_MINUS_ONE) >> RANGE_LIMIT_SHIFT;
		session_info.m_node_queues_ptr = new std::queue<struct _QueueNodeData>[session_info.m_ranges_num];
        
        //
        if (UINT_MAX != nodeId)
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

            add_to_queue(nodeId,
						first_node_start, first_node_end, 0,
						session_info);
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

            // ��������� ��� ����� � �������
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
        
        // ��������� ����� � ������ ��� ������
    
        // TODO: ��� ���� ����������� � �� ����� ������� ��� � points �������� � set ����� ��� �������� �� �������
        
        //
		for (boost::unordered_map<NodeID, struct _AddedData>::iterator it = session_info.m_nodes.begin(); it != session_info.m_nodes.end(); it++) {
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
        
        
		SimpleLogger().Write(logINFO) << "Num points: " << session_info.m_coordinates.size();

        
        ///// Reply to request
        reply.status = http::Reply::ok;
        
        // output
        JSON::Object json_result;
        std::string temp_string;

        JSON::Array json_locations;
        unsigned counter = 0;

		std::vector<FixedPointCoordinate> hull;
		concaveHull(session_info.m_coordinates, hull);

		SimpleLogger().Write(logINFO) << "Hull points: " << hull.size();

		for (int i = 0; i < hull.size(); i++)
        {
            JSON::Array json_coordinates;

			json_coordinates.values.push_back(hull[i].lat / COORDINATE_PRECISION);
			json_coordinates.values.push_back(hull[i].lon / COORDINATE_PRECISION);
            json_locations.values.push_back(json_coordinates);
            ++counter;
        }
        json_result.values["points"] = json_locations;

		JSON::Array json_locations1;
		for (auto c : session_info.m_coordinates){
			JSON::Array json_coordinates;

			json_coordinates.values.push_back(c.lat / COORDINATE_PRECISION);
			json_coordinates.values.push_back(c.lon / COORDINATE_PRECISION);
			json_locations.values.push_back(json_coordinates);
			++counter;
		}
		json_result.values["debug_points"] = json_locations;

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

        //// �������� ��� � ��� ���� ���� �� ���� edge �� ������� ����� �����
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

//          // �� ����� ���� ��� ��� ���, ��� ���� �������� ����� ����� ������ ������������
//          if (check_forward && !data.forward) {
//              continue;       //!!! �������� �������� ������������ "������ ������"
//          }

            if (!data.shortcut)
            {
                //// ���� ���������� � map'�, ���� �� ��� ���, �� �������� � ���������.
                //// ��������� � ������� ��� ���������� ��� target_node_id, ������� ������ � add_to_queue, �.�. �� ������ target_node_id �� ����� ���������� - �������� � add_to_queue(...)
                FixedPointCoordinate coord_of_node_start;
                FixedPointCoordinate coord_of_node_end;
				boost::unordered_map<NodeID, struct _AddedData>::iterator mit = session_info.m_nodes.find(to);
				if (session_info.m_nodes.end() == mit)
                {
					if (GetCoordsForEdgeID(to, coord_of_node_start, coord_of_node_end, session_info.m_coordinates))
                    {
                        struct _AddedData add_data(cur_time + data.distance, coord_of_node_start, coord_of_node_end);
						session_info.m_nodes[to] = add_data;

						check_distance(coord_data, add_data, session_info.m_coordinates);
                    }
                    else {
                        //// ������ �� ������ ���������, ���� ������ ���� - ��� �� �����
                        continue;
                    }
                }
                else {
                    coord_of_node_start = mit->second.m_coords_start;
                    coord_of_node_end = mit->second.m_coords_end;

                    // ����� � ��� ����� check_distance(��� ��� �������)
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
                FixedPointCoordinate out_coord_of_target_start;        // ����� ������ ��������� ���������� ��� target, ����� ��������� �� �������� ������ � ����� target_node_id � add_to_queue(...)
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

                ////////// �.�. �� � ProcessShortcut ��������� ����� � �������, �� ��� ��� ���������� �������� � �� �����(��� ����� ��� �� ���� ��� �������� coord_of_target)
                // ����� nodeId � �������� ���� ����������� ����������, ������� ����� ������������ �� ���������� ���������
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

        FixedPointCoordinate coords_of_shortcut_start;     // ���������� ��� ����������� ����� ���� � ������ if, ���������� �� ������
		FixedPointCoordinate coords_of_shortcut_end;

        //    std::cout << " search smallest edge between start_node_id and middle" << std::endl;
		unsigned eid2 = facade->FindEdgeInEitherDirection(start_node_id, shortcut_node_id);
        if (eid2 != UINT_MAX)
        {
            const QueryEdge::EdgeData & data2 = facade->GetEdgeData(eid2);
            const NodeID target2 = facade->GetTarget(eid2);

            if (!data2.shortcut)
            {
                //// ���� ���������� � map'�, ���� �� ��� ���, �� �������� � ���������.
                //// ��������� � ������� ��� ���������� ��� target2, ������� ������ � add_to_queue, �.�. �� ������ target2 �� ����� ���������� - �������� � add_to_queue(...)
                FixedPointCoordinate coord_of_node_start;
                FixedPointCoordinate coord_of_node_end;
                boost::unordered_map<NodeID, struct _AddedData>::iterator mit = session_info.m_nodes.find(data2.id);
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

            // ���� �����, �� ���� �� ������� ���������, ��������� ��� ����������
            cur_time += data2.distance;
        }

        // ��� ������ ��������� ������ ��������� ����� ���� ����� �� ��� ��� ����� ���������
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
                //// ���� ���������� � map'�, ���� �� ��� ���, �� �������� � ���������.
                //// ��������� � ������� ��� ���������� ��� target2, ������� ������ � add_to_queue, �.�. �� ������ target2 �� ����� ���������� - �������� � add_to_queue(...)
                FixedPointCoordinate coord_of_node_start;
                FixedPointCoordinate coord_of_node_end;
				boost::unordered_map<NodeID, struct _AddedData>::iterator mit = session_info.m_nodes.find(data2.id);
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

            // ���� �����, �� ���� �� ������� ���������, ��������� ��� ����������
            cur_time += data2.distance;
        }
    }

	///////////////////////////////////////////////////////////////////////////////////////////
	//// Search nearest points and call check_distance for them
	void check_distance(const _AddedData& data1, const _AddedData& data2, std::set<FixedPointCoordinate>& coordinates)
	{
		double ranges[4];
		ranges[0] = FixedPointCoordinate::ApproximateEuclideanDistance(data1.m_coords_start, data2.m_coords_start);
		ranges[1] = FixedPointCoordinate::ApproximateEuclideanDistance(data1.m_coords_start, data2.m_coords_end);
		ranges[2] = FixedPointCoordinate::ApproximateEuclideanDistance(data1.m_coords_end, data2.m_coords_start);
		ranges[3] = FixedPointCoordinate::ApproximateEuclideanDistance(data1.m_coords_end, data2.m_coords_end);

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
        if (c1.lat == INT_MIN)      // ��� �� ����� �� ��� � ����� �� �����
            return;
        if (c2.lat == INT_MIN)      // ��� �� ����� �� ��� � ����� �� �����
            return;

        double r = FixedPointCoordinate::ApproximateEuclideanDistance(c1, c2);
        int rn = (int)r;
        if (r <= MAX_POINTS_DIST)
            return;

        //// ����� ���� ��������� �� ��������� ������
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