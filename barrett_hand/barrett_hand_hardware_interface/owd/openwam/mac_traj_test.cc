/***********************************************************************

  Copyright 2007-2010 Carnegie Mellon University and Intel Corporation
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

  This file is part of owd.

  owd is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation; either version 3 of the License, or (at your
  option) any later version.

  owd is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ***********************************************************************/

#include "MacJointTraj.hh"
#include <stdio.h>

int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: mac_traj_test <trajectory_file>\n");
        exit(1);
    }
    char *filename = argv[1];
    FILE *csv = fopen(filename,"r");
    if (!csv) {
        printf("Cannot open csv file %s\n",argv[1]);
        exit(1);
    }
    TrajPoint tp;
    TrajType traj;
    tp.resize(7);
    while (fscanf(csv,"%lf: %lf, %lf, %lf, %lf, %lf, %lf, %lf\n",
                  &tp.blend_radius, &tp[0],&tp[1],&tp[2],&tp[3],&tp[4],&tp[5],&tp[6]) == 8) {
        traj.push_back(tp);
    }
    fclose(csv);

    // wipe out the previous output file, if any
    unlink("traj.csv");

    JointPos max_v; 

//        max_v.push_back(40);
//        max_v.push_back(30);
//        max_v.push_back(30);
//        max_v.push_back(60);
//        max_v.push_back(80);
//        max_v.push_back(80);
//        max_v.push_back(90);
    	    max_v.push_back(60);
    	    max_v.push_back(60);
    	    max_v.push_back(60);
    	    max_v.push_back(60);
    	    max_v.push_back(120);
    	    max_v.push_back(120);
    	    max_v.push_back(60);
	    //    	    max_v *= 4;
    max_v *= 3.141592654/180; // degrees to radians
    JointPos max_a(max_v);  // accels as a function of vels

    MacJointTraj *mjt;
    try {
      mjt = new MacJointTraj(traj,max_v,max_a,true,false,1);
    } catch (char *errstr) {
      printf("Error: %s\n",errstr);
      exit(1);
    }

    printf("Trajectory computation done:\n");
    mjt->dump();

    try {
      FILE *csv = fopen("traj.csv","w");
      if (!csv) {
        printf("Cannot output to traj.csv\n");
        exit(1);
      }
      double y[8],yd[8],ydd[8];
      double delta_t = mjt->traj_duration/1000;
      double elapsed_time=0;
      double path_vel, path_accel;
      double max_path_vel,max_path_accel;
      mjt->run();
      mjt->evaluate(y,yd,ydd,0);
      mjt->get_path_values(&path_vel,&path_accel);
      mjt->get_limits(&max_path_vel,&max_path_accel);
      double y_sum = *y - *yd*delta_t;  // initial position at -delta_t
      double yd_sum = *yd - *ydd*delta_t; // initial velocity at -delta_t
      while (mjt->state() != MacJointTraj::DONE) {
	y_sum += *(yd+1) * delta_t;  // integrate the velocity
	yd_sum += *(ydd+1) * delta_t;  // integrate the acceleration
	fprintf(csv,"%3.4f, ",elapsed_time);
	for (unsigned int j=1; j<=max_v.size(); ++j) {
	  fprintf(csv,"%2.8f, ",y[j]);
	}
	for (unsigned int j=1; j<=max_v.size(); ++j) {
	  fprintf(csv,"%2.8f, ",yd[j]);
	}
	for (unsigned int j=1; j<=max_v.size(); ++j) {
	  fprintf(csv,"%2.8f, ",ydd[j]);
	}
	fprintf(csv,"%2.8f, ",y_sum);  // double-check vs. y[0]
	fprintf(csv,"%2.8f, ",yd_sum);  // double_check vs. yd[0]
	fprintf(csv,"%2.8f, %2.8f, ",path_vel, path_accel);
	fprintf(csv,"%2.8f, %2.8f\n",max_path_vel, max_path_accel);

	// next sample
	mjt->evaluate(y,yd,ydd,delta_t);
	mjt->get_path_values(&path_vel,&path_accel);
	mjt->get_limits(&max_path_vel,&max_path_accel);
	elapsed_time += delta_t;
      }
      fclose(csv);
    } catch (char *errstr) {
      printf("Error during eval loop: %s\n",errstr);
      exit(1);
    }

    exit(0);
}
    
