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

#include "DH.hh"

#include "../openmath/R3.hh"
#include "../openmath/SE3.hh"
#include "../openmath/Inertia.hh"

#ifndef __LINK_HH__
#define __LINK_HH__

using namespace std;

namespace OWD {
  class Link{
  public:

    DH dh;
    R3 cog;
    double mass;
    Inertia inertia;


    static const int L0 = 0;         // link 0 index
    static const int L1 = 1;         // link 1 index
    static const int L2 = 2;         // link 2 index
    static const int L3 = 3;         // link 3 index
    static const int L4 = 4;         // link 4 index
    static const int L5 = 5;         // link 5 index
    static const int L6 = 6;         // link 6 index
    static const int L7 = 7;         // link 7 index
#ifdef WRIST
    static const int Ln = Link::L7;
#else
    static const int Ln = Link::L4;
#endif

    Link(){}
    Link(const DH& dh, double m, const R3& cog, const Inertia& inertia){
      this->dh = dh;  this->mass = m;  this->inertia = inertia;  this->cog = cog;
    }

    operator SE3() const { return (SE3)dh; }
    operator SO3() const { return (SO3)dh; }

    R3      s()  const {return cog;}
    R3      ps() const {return dh.ps();}
    double  m()  const {return mass;}
    Inertia I()  const {return inertia;}
  
    SE3 theta (double t) {dh.t(t); return (SE3)dh;}
  };

}; // namespace OWD
#endif
