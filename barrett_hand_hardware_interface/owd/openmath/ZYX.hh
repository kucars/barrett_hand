/*
  Copyright 2006 Simon Leonard

  This file is part of openwam.

  openman is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation; either version 3 of the License, or (at your
  option) any later version.

  openman is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __ZYX_HH__
#define __ZYX_HH__

class SO3;

class ZYX{
private:
  double x, y, z;

public:

  ZYX(){x=y=z=0.0;}
  ZYX(double x, double y, double z){this->x=x; this->y=y; this->z=z;}

  operator SO3() const
  {
    double cx, sx, cy, sy, cz, sz;
    
    cx = cos(x); sx = sin(x);
    cy = cos(y); sy = sin(y);
    cz = cos(z); sz = sin(z);
    
    return SO3( R3( cz*cy,           sz*cy,         -sy),
                R3(-sz*cx+cz*sy*sx,  cz*cx+sz*sy*sx, cy*sx),
                R3( sz*sx+cz*sy*cx, -cz*sx+sz*sy*cx, cy*cx) );
  };
};

#endif
