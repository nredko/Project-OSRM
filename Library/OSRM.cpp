/*
    open source routing machine
    Copyright (C) Dennis Luxen, 2010

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU AFFERO General Public License as published by
the Free Software Foundation; either version 3 of the License, or
any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
or see http://www.gnu.org/licenses/agpl.txt.
 */

#include "OSRM.h"

OSRM::OSRM(const char * server_ini_path) {
    if( !testDataFile(server_ini_path) ){
        throw OSRMException("server.ini not found");
    }

    BaseConfiguration serverConfig(server_ini_path);
    objects = new QueryObjectsStorage(
        serverConfig.GetParameter("hsgrData"),
        serverConfig.GetParameter("ramIndex"),
        serverConfig.GetParameter("fileIndex"),
        serverConfig.GetParameter("nodesData"),
        serverConfig.GetParameter("edgesData"),
        serverConfig.GetParameter("namesData"),
        serverConfig.GetParameter("timestamp")
    );

    RegisterPlugin(new HelloWorldPlugin());
    RegisterPlugin(new LocatePlugin(objects));
    RegisterPlugin(new NearestPlugin(objects));
    RegisterPlugin(new TimestampPlugin(objects));
    RegisterPlugin(new ViaRoutePlugin(objects));
}

OSRM::~OSRM() {
    BOOST_FOREACH(PluginMap::value_type & plugin_pointer, pluginMap) {
        delete plugin_pointer.second;
    }
    delete objects;
}

void OSRM::RegisterPlugin(BasePlugin * plugin) {
    std::cout << "[plugin] " << plugin->GetDescriptor() << std::endl;
    pluginMap[plugin->GetDescriptor()] = plugin;
}

void OSRM::RunQuery(RouteParameters & route_parameters, http::Reply & reply) {
    const PluginMap::const_iterator & iter = pluginMap.find(route_parameters.service);
    if(pluginMap.end() != iter) {
        reply.status = http::Reply::ok;
        iter->second->HandleRequest(route_parameters, reply );
    } else {
        reply = http::Reply::stockReply(http::Reply::badRequest);
    }
}

namespace http {

boost::asio::const_buffer ToBuffer(Reply::status_type status) {
	switch (status) {
	case Reply::ok:
		return boost::asio::buffer(okString);
	case Reply::internalServerError:
		return boost::asio::buffer(internalServerErrorString);
	default:
		return boost::asio::buffer(badRequestString);
	}
}

 std::string ToString(Reply::status_type status) {
	switch (status) {
	case Reply::ok:
		return okHTML;
	case Reply::badRequest:
		return badRequestHTML;
	default:
		return internalServerErrorHTML;
	}
}

std::vector<boost::asio::const_buffer> Reply::toBuffers(){
	std::vector<boost::asio::const_buffer> buffers;
	buffers.push_back(ToBuffer(status));
	for (std::size_t i = 0; i < headers.size(); ++i) {
		Header& h = headers[i];
		buffers.push_back(boost::asio::buffer(h.name));
		buffers.push_back(boost::asio::buffer(seperators));
		buffers.push_back(boost::asio::buffer(h.value));
		buffers.push_back(boost::asio::buffer(crlf));
	}
	buffers.push_back(boost::asio::buffer(crlf));
	buffers.push_back(boost::asio::buffer(content));
	return buffers;
}

 std::vector<boost::asio::const_buffer> Reply::HeaderstoBuffers(){
    std::vector<boost::asio::const_buffer> buffers;
    buffers.push_back(ToBuffer(status));
    for (std::size_t i = 0; i < headers.size(); ++i) {
        Header& h = headers[i];
//        std::cout << h.name << ": " << h.value << std::endl;
        buffers.push_back(boost::asio::buffer(h.name));
        buffers.push_back(boost::asio::buffer(seperators));
        buffers.push_back(boost::asio::buffer(h.value));
        buffers.push_back(boost::asio::buffer(crlf));
    }
    buffers.push_back(boost::asio::buffer(crlf));
    return buffers;
}

 Reply Reply::stockReply(Reply::status_type status) {
	Reply rep;
	rep.status = status;
	rep.content = ToString(status);
	rep.headers.resize(3);
	rep.headers[0].name = "Access-Control-Allow-Origin";
	rep.headers[0].value = "*";
	rep.headers[1].name = "Content-Length";

    std::string s;
    intToString(rep.content.size(), s);

    rep.headers[1].value = s;
	rep.headers[2].name = "Content-Type";
	rep.headers[2].value = "text/html";
	return rep;
}
} // namespace http

