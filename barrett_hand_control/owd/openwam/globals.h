/*
  Copyright 2006 Simon Leonard

  This file is part of openwam.

  openwam is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation; either version 3 of the License, or (at your
  option) any later version.

  openwam is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _GLOBALS_H_
#define _GLOBALS_H_

#include <math.h>

# define TWOPI 6.283185

# define OW_SUCCESS 0
# define OW_FAILURE -1

namespace OWD {

  inline double clip(double val, double min, double max) {
    if (isnan(val)) {
      throw "NAN passed to OWD::clip";
    } else if (isinf(val)) {
      throw "INF passed to OWD::clip";
    } else return (val<min) ? min
	     :    (max<val) ? max
	     : val;
  }

  inline bool is_in_range(double val, double min, double max) {
    if (isnan(val)) {
      throw "NAN passed to OWD::is_in_range";
    } else if (isinf(val)) {
      throw "INF passed to OWD::is_in_range";
    } else return (min <= val) && (val <= max);
  }

};

#endif // GLOBALS_H




