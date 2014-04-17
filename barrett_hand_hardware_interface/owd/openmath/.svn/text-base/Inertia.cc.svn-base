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
#include "Inertia.hh"

Inertia operator * (const Inertia& I, const SO3& R){
  Inertia IR;
  double* ir = (double*)IR;

  *ir++ = (I[Inertia::Ixx]*R[SO3::R11] + 
	   I[Inertia::Ixy]*R[SO3::R21] + 
	   I[Inertia::Ixz]*R[SO3::R31]);
  
  *ir++ = (I[Inertia::Ixx]*R[SO3::R12] + 
	   I[Inertia::Ixy]*R[SO3::R22] + 
	   I[Inertia::Ixz]*R[SO3::R32]);

  *ir++ = (I[Inertia::Ixx]*R[SO3::R13] + 
	   I[Inertia::Ixy]*R[SO3::R23] + 
	   I[Inertia::Ixz]*R[SO3::R33]);

  *ir++ = (I[Inertia::Iyx]*R[SO3::R11] + 
	   I[Inertia::Iyy]*R[SO3::R21] + 
	   I[Inertia::Iyz]*R[SO3::R31]);

  *ir++ = (I[Inertia::Iyx]*R[SO3::R12] + 
	   I[Inertia::Iyy]*R[SO3::R22] + 
	   I[Inertia::Iyz]*R[SO3::R32]);

  *ir++ = (I[Inertia::Iyx]*R[SO3::R13] + 
	   I[Inertia::Iyy]*R[SO3::R23] + 
	   I[Inertia::Iyz]*R[SO3::R33]);

  *ir++ = (I[Inertia::Izx]*R[SO3::R11] + 
	   I[Inertia::Izy]*R[SO3::R21] + 
	   I[Inertia::Izz]*R[SO3::R31]);

  *ir++ = (I[Inertia::Izx]*R[SO3::R12] + 
	   I[Inertia::Izy]*R[SO3::R22] + 
	   I[Inertia::Izz]*R[SO3::R32]);

  *ir   = (I[Inertia::Izx]*R[SO3::R13] + 
	   I[Inertia::Izy]*R[SO3::R23] + 
	   I[Inertia::Izz]*R[SO3::R33]);
  return IR;
}

Inertia operator * (const SO3& R, const Inertia& I){
  Inertia RI;
  double* ri = (double*)RI;

  *ri++ = (I[Inertia::Ixx]*R[SO3::R11]+
	   I[Inertia::Ixy]*R[SO3::R12]+
	   I[Inertia::Ixz]*R[SO3::R13]);

  *ri++ = (I[Inertia::Ixy]*R[SO3::R11]+
	   I[Inertia::Iyy]*R[SO3::R12]+
	   I[Inertia::Iyz]*R[SO3::R13]);

  *ri++ = (I[Inertia::Ixz]*R[SO3::R11]+
	   I[Inertia::Iyz]*R[SO3::R12]+
	   I[Inertia::Izz]*R[SO3::R13]);

  *ri++ = (I[Inertia::Ixx]*R[SO3::R21]+
	   I[Inertia::Iyx]*R[SO3::R22]+
	   I[Inertia::Izx]*R[SO3::R23]);

  *ri++ = (I[Inertia::Ixy]*R[SO3::R21]+
	   I[Inertia::Iyy]*R[SO3::R22]+
	   I[Inertia::Iyz]*R[SO3::R23]);

  *ri++ = (I[Inertia::Ixz]*R[SO3::R21]+
	   I[Inertia::Iyz]*R[SO3::R22]+
	   I[Inertia::Izz]*R[SO3::R23]);

  *ri++ = (I[Inertia::Ixx]*R[SO3::R31]+
	   I[Inertia::Iyx]*R[SO3::R32]+
	   I[Inertia::Izx]*R[SO3::R33]);

  *ri++ = (I[Inertia::Ixy]*R[SO3::R31]+
	   I[Inertia::Iyy]*R[SO3::R32]+
	   I[Inertia::Izy]*R[SO3::R33]);

  *ri++ = (I[Inertia::Ixz]*R[SO3::R31]+
	   I[Inertia::Iyz]*R[SO3::R32]+
	   I[Inertia::Izz]*R[SO3::R33]);
  return RI;
}

ostream& operator <<  (ostream& s, const Inertia& i){
  int p;
  p = cout.precision();
  cout.precision(12);
  cout.setf(ios::fixed, ios::floatfield);
  cout.setf(ios::right, ios::adjustfield);
  cout<<setw(17)<<i.I[0][0]  <<setw(17)<<i.I[0][1]  <<setw(17)<<i.I[0][2]<<endl
      <<setw(17)<<i.I[1][0]  <<setw(17)<<i.I[1][1]  <<setw(17)<<i.I[1][2]<<endl
      <<setw(17)<<i.I[2][0]  <<setw(17)<<i.I[2][1]  <<setw(17)<<i.I[2][2];
  cout.setf(ios::left, ios::adjustfield);
  cout.precision(p);
  return s;
}
