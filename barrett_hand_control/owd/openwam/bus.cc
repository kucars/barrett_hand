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
#include <openwam/CANbus.hh>

using namespace std;

RT_TASK task;
CANbus bus;
double mpos[8];

void thread(void *cookie)
{
	int *done = (int *) cookie;
	
  if(bus.init() == OW_FAILURE){
    cerr << "bus::init failed."<<endl;
    *done = 1;
    return;
  }
  bus.dump();

  while(1){
    getchar();
    bus.read_positions(mpos);
    cout << mpos[1] << " " << mpos[2] << " " << mpos[3] << " " <<mpos[4]<<endl;
  }
}

int main(){
	int retvalue, done = 0;
  
	if(mlockall(MCL_CURRENT | MCL_FUTURE) == -1){
	  return -1;
	}
	
	rt_task_set_mode(0, T_PRIMARY, NULL);
	retvalue = rt_task_create(&task, "CANBUS", 0, 99, T_JOINABLE);
	retvalue = rt_task_start(&task, &thread, &done);

	while(!done){	usleep(1000); }

	rt_task_suspend(&task);
	rt_task_delete(&task);
  return 0;
}

