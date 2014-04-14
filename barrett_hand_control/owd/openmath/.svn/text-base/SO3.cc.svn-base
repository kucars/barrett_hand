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
#include "SO3.hh"

so3::operator SO3() const{
  double ct, st, vt, w1, w2, w3;
  
  ct = cos(theta);    st = sin(theta);    vt = 1.0-ct;
  w1 = omega[R3::X];  w2 = omega[R3::Y];  w3 = omega[R3::Z];

  return SO3( R3(w1*w1*vt + ct,      w1*w2*vt + w3*st,   w1*w3*vt - w2*st),
	      R3(w1*w2*vt - w3*st,   w2*w2*vt + ct,      w2*w3*vt + w1*st),
	      R3(w1*w3*vt + w2*st,   w2*w3*vt - w1*st,   w3*w3*vt + ct   ));
}


SO3::SO3(){
    this->eye();
}

SO3::SO3(const R3 rx, const R3 ry, const R3 rz){
    this->R[0][0] = rx[0];  this->R[0][1] = ry[0];  this->R[0][2] = rz[0];
    this->R[1][0] = rx[1];  this->R[1][1] = ry[1];  this->R[1][2] = rz[1];
    this->R[2][0] = rx[2];  this->R[2][1] = ry[2];  this->R[2][2] = rz[2];
    
    normalize();
}

void SO3::eye(){
    bzero(&R[0][0], 3*3*sizeof(double)); 
    R[0][0] = R[1][1] = R[2][2] = 1.0;
}

SO3::operator so3() const{
  double sum=(this->R[0][0]+this->R[1][1]+this->R[2][2] - 1.0)/2.0;
  if (sum > 1.0) {
    // ocassionally happens due to floating point imprecision
    sum = 1.0;
  } else if (sum < -1.0) {
    sum = -1.0;
  }
  double theta = acos(sum);

  if(so3::EPSILON < fabs(theta)){
    return so3(R3( (R[2][1]-R[1][2])/(2.0*sin(theta)),
		   (R[0][2]-R[2][0])/(2.0*sin(theta)),
		   (R[1][0]-R[0][1])/(2.0*sin(theta)) ), theta);
  }
  else return so3(R3(), theta); 
}

R3 operator * (const SO3& R,  const R3& p) {
    return R3( R.R[0][0]*p[0] + R.R[0][1]*p[1] + R.R[0][2]*p[2],
	       R.R[1][0]*p[0] + R.R[1][1]*p[1] + R.R[1][2]*p[2],
	       R.R[2][0]*p[0] + R.R[2][1]*p[1] + R.R[2][2]*p[2]);
}
  
SO3 operator ! (const SO3& R){
    return SO3( R3(R.R[0][0], R.R[0][1], R.R[0][2]),
		R3(R.R[1][0], R.R[1][1], R.R[1][2]),
		R3(R.R[2][0], R.R[2][1], R.R[2][2]) );
}

SO3 operator * (const SO3& r1, const SO3& r2){
  SO3 R;

  double* r = (double*)R;

  *r++ = (r1[SO3::R11]*r2[SO3::R11] +
	  r1[SO3::R12]*r2[SO3::R21] +
	  r1[SO3::R13]*r2[SO3::R31]);

  *r++ = (r1[SO3::R11]*r2[SO3::R12] +
	  r1[SO3::R12]*r2[SO3::R22] +
	  r1[SO3::R13]*r2[SO3::R32]);

  *r++ = (r1[SO3::R11]*r2[SO3::R13] +
	  r1[SO3::R12]*r2[SO3::R23] +
	  r1[SO3::R13]*r2[SO3::R33]);

  *r++ = (r1[SO3::R21]*r2[SO3::R11] +
	  r1[SO3::R22]*r2[SO3::R21] +
	  r1[SO3::R23]*r2[SO3::R31]);

  *r++ = (r1[SO3::R21]*r2[SO3::R12] +
	  r1[SO3::R22]*r2[SO3::R22] +
	  r1[SO3::R23]*r2[SO3::R32]);

  *r++ = (r1[SO3::R21]*r2[SO3::R13] +
	  r1[SO3::R22]*r2[SO3::R23] +
	  r1[SO3::R23]*r2[SO3::R33]);

  *r++ = (r1[SO3::R31]*r2[SO3::R11] +
	  r1[SO3::R32]*r2[SO3::R21] +
	  r1[SO3::R33]*r2[SO3::R31]);

  *r++ = (r1[SO3::R31]*r2[SO3::R12] +
	  r1[SO3::R32]*r2[SO3::R22] +
	  r1[SO3::R33]*r2[SO3::R32]);

  *r   = (r1[SO3::R31]*r2[SO3::R13] +
	  r1[SO3::R32]*r2[SO3::R23] +
	  r1[SO3::R33]*r2[SO3::R33]);

  return R;
}

void SO3::normalize() {
  // assume that x, y, and z are all the wrong length and aren't
  // necessarily orthogonal
  R3 x(R[0][0], R[1][0], R[2][0]);
  x.normalize();
  R3 y(R[0][1], R[1][1], R[2][1]);
  R3 z = x^y;  // z is now orthogonal to x and y
  z.normalize();
  y = z^x;
  R[0][0] = x[0];   R[1][0] = x[1];  R[2][0] = x[2];
  R[0][1] = y[0];   R[1][1] = y[1];  R[2][1] = y[2];
  R[0][2] = z[0];   R[1][2] = z[1];  R[2][2] = z[2];
}

ostream& operator <<  (ostream& s, const SO3& r){
  int p;
  p = s.precision();
  s.precision(12);
  s.setf(ios::fixed, ios::floatfield);
  s.setf(ios::right, ios::adjustfield);
  s << setw(17) << r[SO3::R11] 
       << setw(17) << r[SO3::R12] 
       << setw(17) << r[SO3::R13] << endl
       << setw(17) << r[SO3::R21] 
       << setw(17) << r[SO3::R22] 
       << setw(17) << r[SO3::R23] << endl
       << setw(17) << r[SO3::R31] 
       << setw(17) << r[SO3::R32] 
       << setw(17) << r[SO3::R33];
  s.setf(ios::left, ios::adjustfield);
  s.precision(p);
  return s;
}

