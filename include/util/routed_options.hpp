#ifndef ROUTED_OPTIONS_HPP
#define ROUTED_OPTIONS_HPP

#include "util/version.hpp"
#include "util/exception.hpp"
#include "util/simple_logger.hpp"

#include <boost/any.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/convenience.hpp>

#include <unordered_map>
#include <fstream>
#include <string>

namespace osrm
{
namespace util
{
const static unsigned INIT_OK_START_ENGINE = 0;
const static unsigned INIT_OK_DO_NOT_START_ENGINE = 1;
const static unsigned INIT_FAILED = -1;

inline void
populate_base_path(std::unordered_map<std::string, boost::filesystem::path> &server_paths)
{
    // populate the server_path object
    auto path_iterator = server_paths.find("base");

    // if a base path has been set, we populate it.
    if (path_iterator != server_paths.end())
    {
        const std::string base_string = path_iterator->second.string();
        SimpleLogger().Write() << "populating base path: " << base_string;

        server_paths["hsgrdata"] = base_string + ".hsgr";
        BOOST_ASSERT(server_paths.find("hsgrdata") != server_paths.end());
        server_paths["nodesdata"] = base_string + ".nodes";
        BOOST_ASSERT(server_paths.find("nodesdata") != server_paths.end());
        server_paths["coredata"] = base_string + ".core";
        BOOST_ASSERT(server_paths.find("coredata") != server_paths.end());
        server_paths["edgesdata"] = base_string + ".edges";
        BOOST_ASSERT(server_paths.find("edgesdata") != server_paths.end());
        server_paths["geometries"] = base_string + ".geometry";
        BOOST_ASSERT(server_paths.find("geometries") != server_paths.end());
        server_paths["ramindex"] = base_string + ".ramIndex";
        BOOST_ASSERT(server_paths.find("ramindex") != server_paths.end());
        server_paths["fileindex"] = base_string + ".fileIndex";
        BOOST_ASSERT(server_paths.find("fileindex") != server_paths.end());
        server_paths["namesdata"] = base_string + ".names";
        BOOST_ASSERT(server_paths.find("namesdata") != server_paths.end());
        server_paths["timestamp"] = base_string + ".timestamp";
        BOOST_ASSERT(server_paths.find("timestamp") != server_paths.end());
    }

    // check if files are give and whether they exist at all
    path_iterator = server_paths.find("hsgrdata");
    if (path_iterator == server_paths.end() ||
        !boost::filesystem::is_regular_file(path_iterator->second))
    {
        throw exception(".hsgr not found");
    }

    path_iterator = server_paths.find("nodesdata");
    if (path_iterator == server_paths.end() ||
        !boost::filesystem::is_regular_file(path_iterator->second))
    {
        throw exception(".nodes not found");
    }

    path_iterator = server_paths.find("edgesdata");
    if (path_iterator == server_paths.end() ||
        !boost::filesystem::is_regular_file(path_iterator->second))
    {
        throw exception(".edges not found");
    }

    path_iterator = server_paths.find("geometries");
    if (path_iterator == server_paths.end() ||
        !boost::filesystem::is_regular_file(path_iterator->second))
    {
        throw exception(".geometry not found");
    }

    path_iterator = server_paths.find("ramindex");
    if (path_iterator == server_paths.end() ||
        !boost::filesystem::is_regular_file(path_iterator->second))
    {
        throw exception(".ramIndex not found");
    }

    path_iterator = server_paths.find("fileindex");
    if (path_iterator == server_paths.end() ||
        !boost::filesystem::is_regular_file(path_iterator->second))
    {
        throw exception(".fileIndex not found");
    }

    path_iterator = server_paths.find("namesdata");
    if (path_iterator == server_paths.end() ||
        !boost::filesystem::is_regular_file(path_iterator->second))
    {
        throw exception(".namesIndex not found");
    }

    SimpleLogger().Write() << "HSGR file:\t" << server_paths["hsgrdata"];
    SimpleLogger().Write(logDEBUG) << "Nodes file:\t" << server_paths["nodesdata"];
    SimpleLogger().Write(logDEBUG) << "Edges file:\t" << server_paths["edgesdata"];
    SimpleLogger().Write(logDEBUG) << "Geometry file:\t" << server_paths["geometries"];
    SimpleLogger().Write(logDEBUG) << "RAM file:\t" << server_paths["ramindex"];
    SimpleLogger().Write(logDEBUG) << "Index file:\t" << server_paths["fileindex"];
    SimpleLogger().Write(logDEBUG) << "Names file:\t" << server_paths["namesdata"];
    SimpleLogger().Write(logDEBUG) << "Timestamp file:\t" << server_paths["timestamp"];
}

// generate boost::program_options object for the routing part
inline unsigned
GenerateServerProgramOptions(const int argc,
                             const char *argv[],
                             std::unordered_map<std::string, boost::filesystem::path> &paths,
                             std::string &ip_address,
                             int &ip_port,
                             int &requested_num_threads,
                             bool &use_shared_memory,
                             bool &trial,
                             int &max_locations_trip,
                             int &max_locations_viaroute,
                             int &max_locations_distance_table,
                             int &max_locations_map_matching,
							 int &max_time_isochrone)
{
    using boost::program_options::value;
    using boost::filesystem::path;

    // declare a group of options that will be allowed only on command line
    boost::program_options::options_description generic_options("Options");
    generic_options.add_options()                                         //
        ("version,v", "Show version")("help,h", "Show this help message") //
        ("config,c", value<boost::filesystem::path>(&paths["config"])->default_value("server.ini"),
         "Path to a configuration file") //
        ("trial", value<bool>(&trial)->implicit_value(true), "Quit after initialization");

    // declare a group of options that will be allowed on command line
    boost::program_options::options_description config_options("Configuration");
    config_options.add_options()                                                             //
        ("hsgrdata", value<boost::filesystem::path>(&paths["hsgrdata"]), ".hsgr file")       //
        ("nodesdata", value<boost::filesystem::path>(&paths["nodesdata"]), ".nodes file")    //
        ("edgesdata", value<boost::filesystem::path>(&paths["edgesdata"]), ".edges file")    //
        ("geometry", value<boost::filesystem::path>(&paths["geometries"]), ".geometry file") //
        ("ramindex", value<boost::filesystem::path>(&paths["ramindex"]), ".ramIndex file")   //
        ("fileindex", value<boost::filesystem::path>(&paths["fileindex"]),
         "File index file") //
        ("namesdata", value<boost::filesystem::path>(&paths["namesdata"]),
         ".names file") //
        ("timestamp", value<boost::filesystem::path>(&paths["timestamp"]),
         ".timestamp file") //
        ("ip,i", value<std::string>(&ip_address)->default_value("0.0.0.0"),
         "IP address") //
        ("port,p", value<int>(&ip_port)->default_value(5000),
         "TCP/IP port") //
        ("threads,t", value<int>(&requested_num_threads)->default_value(8),
         "Number of threads to use") //
        ("shared-memory,s",
         value<bool>(&use_shared_memory)->implicit_value(true)->default_value(false),
         "Load data from shared memory") //
        ("max-viaroute-size", value<int>(&max_locations_viaroute)->default_value(500),
         "Max. locations supported in viaroute query") //
        ("max-trip-size", value<int>(&max_locations_trip)->default_value(100),
         "Max. locations supported in trip query") //
        ("max-table-size", value<int>(&max_locations_distance_table)->default_value(100),
         "Max. locations supported in distance table query") //
		("max-time-isochrone", value<int>(&max_time_isochrone)->default_value(3600),
			"Max. time for calculating isochrone area") //
        ("max-matching-size", value<int>(&max_locations_map_matching)->default_value(100),
         "Max. locations supported in map matching query");

    // hidden options, will be allowed on command line, but will not be shown to the user
    boost::program_options::options_description hidden_options("Hidden options");
    hidden_options.add_options()("base,b", value<boost::filesystem::path>(&paths["base"]),
                                 "base path to .osrm file");

    // positional option
    boost::program_options::positional_options_description positional_options;
    positional_options.add("base", 1);

    // combine above options for parsing
    boost::program_options::options_description cmdline_options;
    cmdline_options.add(generic_options).add(config_options).add(hidden_options);

    boost::program_options::options_description visible_options(
        boost::filesystem::basename(argv[0]) + " <base.osrm> [<options>]");
    visible_options.add(generic_options).add(config_options);

    // parse command line options
    boost::program_options::variables_map option_variables;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv)
                                      .options(cmdline_options)
                                      .positional(positional_options)
                                      .run(),
                                  option_variables);

    if (option_variables.count("version"))
    {
        SimpleLogger().Write() << OSRM_VERSION;
        return INIT_OK_DO_NOT_START_ENGINE;
    }

    if (option_variables.count("help"))
    {
        SimpleLogger().Write() << visible_options;
        return INIT_OK_DO_NOT_START_ENGINE;
    }

    boost::program_options::notify(option_variables);

    if (1 > requested_num_threads)
    {
        throw exception("Number of threads must be a positive number");
    }
    if (2 > max_locations_distance_table)
    {
        throw exception("Max location for distance table must be at least two");
    }
    if (max_locations_map_matching > 0 && 2 > max_locations_map_matching)
    {
        throw exception("Max location for map matching must be at least two");
    }

    if (!use_shared_memory && option_variables.count("base"))
    {
        return INIT_OK_START_ENGINE;
    }
    else if (use_shared_memory && !option_variables.count("base"))
    {
        return INIT_OK_START_ENGINE;
    }
    else if (use_shared_memory && option_variables.count("base"))
    {
        SimpleLogger().Write(logWARNING) << "Shared memory settings conflict with path settings.";
    }

    SimpleLogger().Write() << visible_options;
    return INIT_OK_DO_NOT_START_ENGINE;
}
}
}

#endif // ROUTED_OPTIONS_HPP
