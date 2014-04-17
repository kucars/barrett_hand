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

#include <iostream>

#include <openwam/WAM.hh>

using namespace std;

int main(){
  //                     #1     #2     #3   #4    #5   #6   #7
  double zero[8] = {0.0, 0.0, -M_PI_2, 0.0, M_PI, 0.0, 0.0, 0.0};

  CANbus bus;
  WAM wam(&bus);

  if(bus.init() == OW_FAILURE){
    cerr<<"main: init failed."<<endl;
    return OW_FAILURE;
  }

  if(wam.init() == OW_FAILURE){
    cerr<<"main: init failed."<<endl;
    return OW_FAILURE;
  }

  cout << "ENTER to zero the WAM." << endl;
  getchar();
  wam.set_jpos(zero);

  cout << "ENTER to start the control loop." <<endl;
  getchar();
  if(wam.start() == OW_FAILURE){
    cerr << "main: start failed." << endl;
    return OW_FAILURE;
  }

  while(1){
    cout << wam.FK() << endl;
    getchar();
  }

  cout << "ENTER to stop the control loop bus." << endl;
  getchar();  

  wam.stop();

  return 0;
}

