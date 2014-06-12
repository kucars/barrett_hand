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
#include "SE3.hh"

void SE3::setR3(const R3& t){
    H[0][3] = t[R3::X];
    H[1][3] = t[R3::Y];
    H[2][3] = t[R3::Z];
}

void SE3::setSO3(const SO3& R){
    H[0][0]=R[SO3::R11];   H[0][1]=R[SO3::R12];   H[0][2]=R[SO3::R13];
    H[1][0]=R[SO3::R21];   H[1][1]=R[SO3::R22];   H[1][2]=R[SO3::R23];
    H[2][0]=R[SO3::R31];   H[2][1]=R[SO3::R32];   H[2][2]=R[SO3::R33];
}

SE3::operator so3() const{
  double theta = acos( (H[0][0]+H[1][1]+H[2][2] - 1.0)/2);

  if(so3::EPSILON < fabs(theta))
    return so3( R3( (H[2][1]-H[1][2])/(2.0*sin(theta)),
		    (H[0][2]-H[2][0])/(2.0*sin(theta)),
		    (H[1][0]-H[0][1])/(2.0*sin(theta)) ), theta );
  else return so3(R3(), theta); 
}

SE3 operator * (const SE3& h1, const SE3& h2){
  SE3 h3;
  double *ptr = (double*)h3;

  *ptr++ = (h1[SE3::R11]*h2[SE3::R11] + 
	    h1[SE3::R12]*h2[SE3::R21] + 
	    h1[SE3::R13]*h2[SE3::R31]); 

  *ptr++ = (h1[SE3::R11]*h2[SE3::R12] + 
	    h1[SE3::R12]*h2[SE3::R22] + 
	    h1[SE3::R13]*h2[SE3::R32]); 

  *ptr++ = (h1[SE3::R11]*h2[SE3::R13] + 
	    h1[SE3::R12]*h2[SE3::R23] + 
	    h1[SE3::R13]*h2[SE3::R33]); 

  *ptr++ = (h1[SE3::R11]*h2[SE3::TX] + 
	    h1[SE3::R12]*h2[SE3::TY] + 
	    h1[SE3::R13]*h2[SE3::TZ] + h1[SE3::TX]); 

  *ptr++ = (h1[SE3::R21]*h2[SE3::R11] + 
	    h1[SE3::R22]*h2[SE3::R21] + 
	    h1[SE3::R23]*h2[SE3::R31]); 

  *ptr++ = (h1[SE3::R21]*h2[SE3::R12] + 
	    h1[SE3::R22]*h2[SE3::R22] + 
	    h1[SE3::R23]*h2[SE3::R32]); 

  *ptr++ = (h1[SE3::R21]*h2[SE3::R13] + 
	    h1[SE3::R22]*h2[SE3::R23] + 
	    h1[SE3::R23]*h2[SE3::R33]); 

  *ptr++ = (h1[SE3::R21]*h2[SE3::TX] + 
	    h1[SE3::R22]*h2[SE3::TY] + 
	    h1[SE3::R23]*h2[SE3::TZ] + h1[SE3::TY]); 

  *ptr++ = (h1[SE3::R31]*h2[SE3::R11] + 
	    h1[SE3::R32]*h2[SE3::R21] + 
	    h1[SE3::R33]*h2[SE3::R31]); 

  *ptr++ = (h1[SE3::R31]*h2[SE3::R12] + 
	    h1[SE3::R32]*h2[SE3::R22] + 
	    h1[SE3::R33]*h2[SE3::R32]); 

  *ptr++ = (h1[SE3::R31]*h2[SE3::R13] + 
	    h1[SE3::R32]*h2[SE3::R23] + 
	    h1[SE3::R33]*h2[SE3::R33]); 

  *ptr++ = (h1[SE3::R31]*h2[SE3::TX] + 
	    h1[SE3::R32]*h2[SE3::TY] + 
	    h1[SE3::R33]*h2[SE3::TZ] + h1[SE3::TZ]); 

  return h3;
}

SE3 operator ^ (const SE3& E, int i){
  if(i == -1){
    R3 p = -1.0*((R3)E);
    SO3 Rt = !((SO3)E);
    return SE3(Rt, Rt*p); 
  }
  return SE3();
}

SE3 SE3::operator*(double a) const {
  so3 rot = (so3)(SO3)(*this);  // get the rotation
  // make the new SE3
  return SE3(so3(rot.w(),a*rot.t()), // new rotation
	     a * (R3)(*this)); // new translation
}

SE3 operator * (double a, const SE3 &s) {
  return s*a;
}

SE3 SE3::operator +(const SE3 &rhs) {
  R3 t = (R3)(*this) + (R3)rhs;    // add the translations
  SO3 R = (SO3)rhs * (SO3)(*this); // multiply the rotations
  return SE3(R, t);
}

SE3 SE3::operator -(const SE3 &rhs) {
  R3 t = (R3)(*this) - (R3)rhs;
  SO3 R = (SO3)(*this) * (!(SO3)rhs);
  return SE3(R,t);
}



ostream& operator <<  (ostream& s, const SE3& h){
  int p;
  p = s.precision();
  s.precision(12);
  s.setf(ios::fixed, ios::floatfield);
  s.setf(ios::right, ios::adjustfield);

  s << setw(17) << h[SE3::R11] 
    << setw(17) << h[SE3::R12] 
    << setw(17) << h[SE3::R13]
    << setw(17) << h[SE3::TX]  << endl
    << setw(17) << h[SE3::R21] 
    << setw(17) << h[SE3::R22] 
    << setw(17) << h[SE3::R23]
    << setw(17) << h[SE3::TY]  << endl
    << setw(17) << h[SE3::R31] 
    << setw(17) << h[SE3::R32] 
    << setw(17) << h[SE3::R33]
    << setw(17) << h[SE3::TZ]  << endl
    << setw(17) << 0 
    << setw(17) << 0 
    << setw(17) << 0
    << setw(17) << 1;

  s.setf(ios::left, ios::adjustfield);
  s.precision(p);
  return s;
}

