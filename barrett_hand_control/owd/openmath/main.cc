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
#include <R3.hh>
#include <R4.hh>
#include <R6.hh>
#include <SO3.hh>
#include <SE3.hh>
#include <Inertia.hh>

int main(){
  SE3 h1( ZYX(1.215, 0.781, 2.190), R3( -1.125, -2.268, 0.156) );
  SE3 h2( ZYX(0.251, 2.016, -1.260), R3(2.561, 1.261, -1.215)) ;

  cout << h1 << endl << endl;

  cout << h2 << endl << endl;
  
  cout << h1*h2 << endl << endl;

  cout << (h1^-1)*h2 << endl << endl;
  cout << h1*(h2^-1) << endl << endl;

  /*
  R6 x1(1.215, 0.781, 2.190, -1.125, -2.268, 0.156);
  R6 x2(0.251, 2.016, -1.260, 2.561, 1.261, -1.215);

  cout << x1 << endl << x2 << endl;

  cout << x1*2.1 << endl;
  cout << x1+x2 << endl;
  cout << x1-x2 << endl;
  cout << x1.norm() << endl;
  */
  /*
  R4 x1(1.215, 0.781, 2.190, -1.125);
  R4 x2(0.251, 2.016, -1.260, 2.561);

  cout << x1 << endl << x2 << endl;

  cout << x1*2.1 << endl << x1+x2 << endl << x1-x2 << endl << x1*x2 << endl;
  */

  /*
  ZYX zyx1(1.215, 0.781, 2.190);
  ZYX zyx2(0.251, 2.016, -1.260);
  SO3 R1 = (SO3)zyx1;
  SO3 R2 = (SO3)zyx2;
  
  cout << R1 << endl << endl;
  cout << R2 << endl << endl;

  cout << R1*R2 << endl << endl;
  cout << R2*R1 << endl << endl;
  */
  /*
  Inertia I(95157.4294,   246.1404,    -95.0183,
	    92032.3524,  -962.6725,  59290.5997, 0.000001);

  ZYX zyx(M_PI/1.215, M_PI/0.781, M_PI/2.190);
  SO3 R = (SO3)zyx;

  cout << I << endl << endl;
  cout << R << endl << endl;
  cout << I*R << endl << endl;    
  cout << R*I << endl << endl;
  */

  /*
  R3 vb(1.25, -0.52, 0.93);
  R3 wb(0.25, -1.11, 2.34);
  
  SE3 H1;
  SE3 H2;//1.5, M_PI-2.2, -M_PI_2/3, 1.51);
  
  cout << H1 << endl << endl;
  cout << H2 << endl << endl;
  
  cout << H1*H2 << endl;
  */

  /*
  cout << atb << endl << endl;
  cout << H << endl << endl;

  cout << "adjoint" << endl;
  Adjoint Ad(H);
  cout << Ad << endl << endl;

  cout << Ad*atb << endl << endl;

  awb = atb;
  cout << awb << endl << endl;
  cout << (!Ad)*awb << endl << endl;
  */

  /*
  Transformation H(0.5, 0.15, -M_PI_2/3, 1.51);
  Rotation R;

  cout << H << endl << endl;
  R = (Rotation)H;
  cout << R << endl;
  */

  /*
  R3 w(2.5, 3.2, -1.2);
  R3 wd(0.4447, 0.6154, 0.7919);
  R3 z0(0, 0, 1);
  Rotation R(M_PI/4-0.3, M_PI/3, -M_PI/6);
  float qd=0;
  float qdd=0;

  cout << R << endl << endl;
  cout << (R^-1) << endl << endl;
  cout << R*(!R) << endl << endl;
  cout << R*(R^-1) << endl << endl;

  cout << R*( wd + z0*qdd + (w^z0*qd)  ) << endl;
  */

  /*
  cout << R << endl;
  
  cout << wd << endl;
  wd = R*(wd + z0*qdd + (w^z0*qd));
  cout << wd << endl;
  */

  return 0;
}





/*
  R3 a(1.0, 2.0, 3.0);
  R4 b(a);
  Rotation R(M_PI/4, M_PI/3, -M_PI/6);
  Transformation H(M_PI/4, M_PI/3, -M_PI/6, 1.0, 2.0, 3.0);

  cout << a << endl << endl;
  cout << R << endl << endl;
  cout << H << endl << endl;

  cout << (R^-1) << endl << endl;
  cout << R*(!R) << endl << endl;
  cout << H*(H^-1) << endl <<endl;
  cout << (H^-1)*H << endl <<endl;

  cout << H*b << endl;
  cout << H*(H^-1)*b << endl;

  cout << (R^-1) << endl << endl << (!R)*a << endl << endl;

  cout << a*(a^a) << endl << endl;

*/
