cmake_minimum_required(VERSION 2.6)
project(OSRM)
include(FindPackageHandleStandardArgs)

TRY_RUN(SHARED_LIBRARY_PATH_TYPE SHARED_LIBRARY_PATH_INFO_COMPILED ${PROJECT_BINARY_DIR}/CMakeTmp ${PROJECT_SOURCE_DIR}/cmake/size.cpp OUTPUT_VARIABLE IS_64_SYSTEM)
if(IS_64_SYSTEM)
	message(STATUS "System supports 64 bits.")
else(IS_64_SYSTEM)
	MESSAGE(WARNING "Compiling on a 32 bit system is unsupported!")
endif(IS_64_SYSTEM)


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(BOOST_COMPONENTS filesystem regex system thread)

file(GLOB ExtractorGlob Extractor/*.cpp)
set(ExtractorSources extractor.cpp ${ExtractorGlob})
add_executable(osrm-extract ${ExtractorSources})

file(GLOB PrepareGlob Contractor/*.cpp)
set(PrepareSources createHierarchy.cpp ${PrepareGlob})
add_executable(osrm-prepare ${PrepareSources})

add_executable(osrm-routed routed.cpp)
set_target_properties(osrm-routed PROPERTIES COMPILE_FLAGS -DROUTED)

file(GLOB DescriptorGlob Descriptors/*.cpp)
file(GLOB LibOSRMGlob Library/*.cpp)
file(GLOB SearchEngineSource DataStructures/SearchEngine*.cpp)
file(GLOB ServerStructureGlob Server/DataStructures/*.cpp)

set(OSRMSources ${LibOSRMGlob} ${DescriptorGlob} ${SearchEngineSource} ${ServerStructureGlob})
add_library(OSRM STATIC ${OSRMSources})

# Check the release mode
if(NOT CMAKE_BUILD_TYPE MATCHES Debug)
	set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE MATCHES Debug)
if(CMAKE_BUILD_TYPE MATCHES Debug)
    message(STATUS "Configuring OSRM in debug mode")
endif(CMAKE_BUILD_TYPE MATCHES Debug)
if(CMAKE_BUILD_TYPE MATCHES Release)
    message(STATUS "Configuring OSRM in release mode")
endif(CMAKE_BUILD_TYPE MATCHES Release)

#Configuring compilers
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
	# using Clang
	set(CMAKE_CXX_FLAGS "-Wall -Wno-unknown-pragmas -Wno-unneeded-internal-declaration")
	message(STATUS "OpenMP parallelization not available using clang++")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
	# using GCC
	set(CMAKE_CXX_FLAGS "-Wall -fopenmp -pedantic")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
	# using Intel C++
	set(CMAKE_CXX_FLAGS "-static-intel -wd10237 -Wall -openmp -ipo")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
	# using Visual Studio C++
	add_definitions(-D_CRT_SECURE_NO_WARNINGS)
	add_definitions(-DNOMINMAX) # avoid min and max macros that can break compilation
	SET(STXXL_USE_BOOST 1)
endif()

if(STXXL_USE_BOOST) 
        LIST(APPEND BOOST_COMPONENTS date_time chrono iostreams)
	add_definitions(-DSTXXL_BOOST_THREADS)
	add_definitions(-DSTXXL_BOOST_TIMESTAMP)
	add_definitions(-DSTXXL_BOOST_RANDOM)
	add_definitions(-DSTXXL_BOOST_CONFIG)
	add_definitions(-DSTXXL_BOOST_FILESYSTEM)
endif()

if(APPLE)
	SET(CMAKE_OSX_ARCHITECTURES "x86_64")
	message("Set Architecture to x64 on OS X")
endif()

#Check Boost
set(BOOST_MIN_VERSION "1.44.0")
find_package( Boost ${BOOST_MIN_VERSION} COMPONENTS ${BOOST_COMPONENTS} )
if (NOT Boost_FOUND)
      set(BOOST_ROOT "NOT FOUND: please choose" CACHE PATH "")
      set(Boost_USE_STATIC_LIBS OFF CACHE BOOL "")
      MARK_AS_ADVANCED(CLEAR BOOST_ROOT)
      message(FATAL_ERROR "Fatal error: Boost (version >= 1.44.0) required. Please install it or specify correct BOOST_ROOT and Boost_USE_STATIC_LIBS\n")
endif(NOT Boost_FOUND)

include_directories(${Boost_INCLUDE_DIRS})


find_package ( BZip2 REQUIRED )
include_directories(${BZIP2_INCLUDE_DIR})
target_link_libraries (osrm-extract ${BZIP2_LIBRARIES})

find_package( ZLIB REQUIRED )
include_directories(${ZLIB_INCLUDE_DIRS})
target_link_libraries (osrm-extract ${ZLIB_LIBRARY})
target_link_libraries (osrm-routed ${ZLIB_LIBRARY})

find_package( Threads REQUIRED )
target_link_libraries (osrm-extract ${Threads_LIBRARY})

find_package( LuaJIT )
IF( LUAJIT_INCLUDE_DIR AND LUAJIT_LIBRARIES )
	include_directories(${LUAJIT_INCLUDE_DIR})
	target_link_libraries( osrm-extract ${LUAJIT_LIBRARIES} )
	target_link_libraries( osrm-prepare ${LUAJIT_LIBRARIES} )
ELSE( LUAJIT_INCLUDE_DIR )
	find_package( Lua51 REQUIRED AND LUAJIT_LIBRARIES )
	include_directories(${LUA_INCLUDE_DIR})
	target_link_libraries( osrm-extract ${LUA_LIBRARY} )
	target_link_libraries( osrm-prepare ${LUA_LIBRARY} )
ENDIF( LUAJIT_INCLUDE_DIR AND LUAJIT_LIBRARIES )

find_package( LibXml2 REQUIRED )
include_directories(${LIBXML2_INCLUDE_DIR})
target_link_libraries (osrm-extract ${LIBXML2_LIBRARIES})

find_package( Luabind REQUIRED )
include_directories(${LUABIND_INCLUDE_DIR})
target_link_libraries (osrm-extract ${LUABIND_LIBRARY})
target_link_libraries (osrm-prepare ${LUABIND_LIBRARY})

find_package( Protobuf REQUIRED )
include_directories(${PROTOBUF_INCLUDE_DIRS})
target_link_libraries (osrm-extract ${PROTOBUF_LIBRARY})
target_link_libraries (osrm-prepare ${PROTOBUF_LIBRARY})

find_package( STXXL REQUIRED )
include_directories(${STXXL_INCLUDE_DIR})
target_link_libraries (osrm-extract ${STXXL_LIBRARY})
target_link_libraries (osrm-prepare ${STXXL_LIBRARY})
target_link_libraries (OSRM ${STXXL_LIBRARY})

find_package( OSMPBF REQUIRED )
include_directories(${OSMPBF_INCLUDE_DIR})
target_link_libraries (osrm-extract ${OSMPBF_LIBRARY})
target_link_libraries (osrm-prepare ${OSMPBF_LIBRARY})

target_link_libraries( OSRM ${Boost_LIBRARIES})
target_link_libraries( osrm-extract ${Boost_LIBRARIES} )
target_link_libraries( osrm-prepare ${Boost_LIBRARIES} )
target_link_libraries( osrm-routed ${Boost_LIBRARIES} OSRM)

if(WITH_TOOLS)
	message("-- Activating OSRM internal tools")
	find_package( GDAL )
	if(GDAL_FOUND)
		add_executable(osrm-components Tools/componentAnalysis.cpp)
		include_directories(${GDAL_INCLUDE_DIR})
		target_link_libraries( osrm-components ${GDAL_LIBRARIES} )
		target_link_libraries( osrm-components ${Boost_LIBRARIES} )
	endif(GDAL_FOUND)
	add_executable ( osrm-cli Tools/simpleclient.cpp)
	target_link_libraries( osrm-cli ${Boost_LIBRARIES} OSRM)
endif(WITH_TOOLS)
