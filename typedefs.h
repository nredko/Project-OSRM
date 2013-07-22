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

#ifndef TYPEDEFS_H_
#define TYPEDEFS_H_

#include <climits>
#include <cstdlib>

// To fix long and long long woes
#include <boost/integer.hpp>
#include <boost/integer_traits.hpp>

#ifdef __APPLE__
#include <signal.h>
#endif

#include <iostream>

#define INFO(x) do {std::cout << "[i " << __FILE__ << ":" << __LINE__ << "] " << x << std::endl;} while(0);
#define ERR(x) do {std::cerr << "[! " << __FILE__ << ":" << __LINE__ << "] " << x << std::endl; std::exit(-1);} while(0);
#define WARN(x) do {std::cerr << "[? " << __FILE__ << ":" << __LINE__ << "] " << x << std::endl;} while(0);

#ifdef NDEBUG
#define DEBUG(x)
#else
#define DEBUG(x) do {std::cout << "[d " << __FILE__ << ":" << __LINE__ << "] " << x << std::endl;} while(0);
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Necessary workaround for Windows as VS doesn't implement C99
#ifdef _MSC_VER
#if _MSC_VER >= 1600
#include <stdint.h>
#else 
#if (_MSC_VER < 1300)
   typedef signed char       int8_t;
   typedef signed short      int16_t;
   typedef signed int        int32_t;
   typedef unsigned char     uint8_t;
   typedef unsigned short    uint16_t;
   typedef unsigned int      uint32_t;
#else
   typedef signed __int8     int8_t;
   typedef signed __int16    int16_t;
   typedef signed __int32    int32_t;
   typedef unsigned __int8   uint8_t;
   typedef unsigned __int16  uint16_t;
   typedef unsigned __int32  uint32_t;
#endif
typedef signed __int64       int64_t;
typedef unsigned __int64     uint64_t;
#endif

template<typename digitT>
digitT round(digitT x) {
    return std::floor(x + 0.5);
}
#endif


typedef unsigned int NodeID;
typedef unsigned int EdgeID;
typedef unsigned int EdgeWeight;

static const NodeID SPECIAL_NODEID = boost::integer_traits<uint32_t>::const_max;
static const EdgeID SPECIAL_EDGEID = boost::integer_traits<uint32_t>::const_max;

#endif /* TYPEDEFS_H_ */
