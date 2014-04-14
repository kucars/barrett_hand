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

#include "Sigmoid.hh"

/*
 * y(t1) = x1/(1+exp(-x3*t1)) + x2
 * y(t2) = x1/(1+exp(-x3*t2)) + x2
 * yd(0) = x1*x3/4
 * t1 = -4/x3
 * t2 =  4/x3
*/

void Sigmoid::init(double y1, double y2){
  double A[2][2], det;

  // Solve the equations for x1 and x3
  A[0][0] = 1.0/(1.0+exp( 4.0));   A[0][1] = 1.0;
  A[1][0] = 1.0/(1.0+exp(-4.0));   A[1][1] = 1.0;
  det = A[0][0]*A[1][1] - A[1][0]*A[0][1];

  A[0][0] =  1.0/det;                   A[0][1] = -1.0/det;
  A[1][0] = -1.0/(1.0+exp(-4.0))/det;   A[1][1] = 1.0/(1.0+exp( 4.0))/det;
  
  x1 = A[0][0]*y1 + A[0][1]*y2;
  x2 = A[1][0]*y1 + A[1][1]*y2;

  if(x1 != 0.0 && x2 != 0.0){
    x3 = ydmax*4.0/x1;  // solve x3

    t = t1 = 0.0;
    ts = -4.0/x3;      // offset to shift the sigmoid to the right
    t2 =  2.0*4.0/x3;  // stop time
  }
  else{}
}

void Sigmoid::evaluate(double& y, double& yd, double& ydd, double dt){
  t+=dt;

  // if the time is within the bounds and the profile is still running
  if( t1<=t && t<=t2 && 0.001<t2 && state() == Profile::RUN ){

    double term1 = exp(-x3*(t+ts));
    double term2 = 1+term1;
    y =   x1                      /term2 + x2;
    yd =  x1*x3   *term1          /(term2*term2);
    ydd = x1*x3*x3*term1*(term1-1)/(term2*term2*term2);

  }

  else{
    
    stop();
    y   =  x1/(1.0+exp(-x3*(t2+ts))) + x2;;
    yd  = 0;
    ydd = 0;

  }  
}

