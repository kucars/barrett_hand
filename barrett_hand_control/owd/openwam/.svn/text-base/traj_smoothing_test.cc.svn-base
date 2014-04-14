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

#include "ParaJointTraj.hh"
#include <stdio.h>

int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: traj_smoothing_test <input.csv>\n");
        exit(1);
    }
    FILE *csv = fopen(argv[1],"r");
    if (!csv) {
        printf("Cannot open csv file %s\n",argv[1]);
        exit(1);
    }
    TrajPoint tp;
    TrajType traj;
    tp.resize(7);
    while (fscanf(csv,"%lf, %lf, %lf, %lf, %lf, %lf, %lf\n",
                  &tp[0],&tp[1],&tp[2],&tp[3],&tp[4],&tp[5],&tp[6]) == 7) {
        traj.push_back(tp);
    }
    fclose(csv);
    ParaJointTraj *pjt;
    vector<double> joint_vels, joint_accels;
    joint_vels.resize(7); joint_accels.resize(7);
    joint_vels[0]= 60.0; joint_accels[0]=30.0;
    joint_vels[1]= 40.0; joint_accels[1]=20.0;
    joint_vels[2]= 80.0; joint_accels[2]=40.0;
    joint_vels[3]= 80.0; joint_accels[3]=40.0;
    joint_vels[4]= 40.0; joint_accels[4]=20.0;
    joint_vels[5]= 40.0; joint_accels[5]=20.0;
    joint_vels[6]= 40.0; joint_accels[6]=20.0;
    try {
        pjt= new ParaJointTraj(traj,joint_vels,joint_accels,true,false,0);
    } catch(int e) {
        printf("Could not construct ParaJointTraj from these points\n");
        exit(1);
    }
    printf("Trajectory created; %zd points, %zd segments, %3.3g secs\n",traj.size(),pjt->parsegs[0].size(),pjt->traj_duration);
    csv = fopen("smoothed.csv","w");
    if (!csv) {
        printf("Cannot output to smoothed.csv\n");
        exit(1);
    }
    double y[8],yd[8],ydd[8];
    pjt->run();
    int stopped=50;
    double stop_time=pjt->traj_duration/2;
    double elapsed_time = 0.0f;
    while (pjt->state() != ParaJointTraj::DONE) {
        if (elapsed_time > stop_time) {
            stopped++;
        }
        if (stopped==1 || stopped==21) {
            pjt->stop();
            printf("Trajectory stopped at time %3.3f\n",elapsed_time);
        } else if (stopped==6 || stopped==26) {
            pjt->run();
            printf("Trajectory restarted at time %3.3f\n",elapsed_time);
        }
        pjt->evaluate(y,yd,ydd,0.01);
        elapsed_time += .01f;
        fprintf(csv,"%3.3f, ",elapsed_time);
        for (int j=1; j<8; ++j) {
            fprintf(csv,"%2.8f, ",y[j]);
        }
        for (int j=1; j<8; ++j) {
            fprintf(csv,"%2.8f, ",yd[j]);
        }
        for (int j=1; j<8; ++j) {
            fprintf(csv,"%2.8f, ",ydd[j]);
        }
        fprintf(csv,"0.0 0.0 0.0 0.0 0.0 0.0 0.0\n"); // torq
    }
    fclose(csv);
    csv = fopen("orig.csv","w");
    if (!csv) {
        printf("Cannot output to orig.csv");
        exit(1);
    }
    int segment=0;
    for (int i=0; i < traj.size(); ++i) {
        // we'll just find the time that joint one hit each original
        // joint value, and output all the points at that time, so that we
        // can plot to see how well they line up with the traj curves

        // first, find what segment each point is in
        while ((segment+1 < pjt->parsegs[0].size()) && 
               (i>pjt->parsegs[0][segment+1].start_index)) {
            segment++;
        }
        // now find an appropriate joint to use (one without a CONST segment)
        int jnt=0;
        int max_jump_jnt=0;
        double max_jump=0.0f;
        for (jnt=0; jnt<7; jnt++) {
            double jump = fabs(pjt->parsegs[jnt][segment].end_pos - 
                     pjt->parsegs[jnt][segment].start_pos);
            if (jump > max_jump) {
                max_jump = jump;
                max_jump_jnt = jnt;
            }
        }
        //        while ((pjt->parsegs[jnt][segment].dir == ParabolicSegment::CONST) &&
        //        (jnt<6)) {
        //            jnt++;
        //        } // if they're all CONST we'll just end up using jnt=6.
        double time = pjt->parsegs[max_jump_jnt][segment].calc_time(traj[i][max_jump_jnt]);
        if (time == -1.0f) {
            printf("Warning: no time available for point %d within segment %d of joint %d\n",i,segment,max_jump_jnt);
        }
        fprintf(csv,"%3.3f, ",time);
        for (int j=0; j<6; j++) {
            fprintf(csv,"%2.8f, ",traj[i][j]);
        }
        fprintf(csv,"%2.8f\n",traj[i][6]);
    }
    fclose(csv);
    delete pjt;
    exit(0);
}
    
    
