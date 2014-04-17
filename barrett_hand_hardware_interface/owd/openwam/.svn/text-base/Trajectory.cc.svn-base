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

#include "Trajectory.hh"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

namespace OWD {


  // the default evaluate function is really just here for backwards
  // compatibility, but new Trajectory classes should focus on implementing
  // evaluate_abs(), which is required for multi-controller synchronization.
  void Trajectory::evaluate(OWD::Trajectory::TrajControl &tc, double dt) {
    // if we're running, then increment the time.  Otherwise, stay where we are
    if ((runstate == RUN) || (runstate == LOG)) {
      time += dt;
    }
    evaluate_abs(tc,time);
  }
  
  bool Trajectory::log(const char *trajname) {
    if ((runstate != Trajectory::STOP) && (runstate != Trajectory::DONE)) {
      // can't log a running trajectory; it would mess up the times
      return false;
    }
    double oldtime=time;

    char *simfname = (char *) malloc(strlen(trajname) +9);
    // start with the logfilename
    sprintf(simfname,"%s_sim.csv",trajname);
    FILE *csv = fopen(simfname,"w");
    if (csv) {
      Trajectory::TrajControl tc(end_position.size());
      runstate=LOG;
      time=0.0;
      unsigned int DOF = end_position.size();
      while (runstate == LOG) {
	evaluate(tc,0.01);
	fprintf(csv,"%3.8f, ",time);
	for (unsigned int j=0; j<DOF; ++j) {
	  fprintf(csv,"%2.8f, ",tc.q[j]);
	}
	for (unsigned int j=0; j<DOF; ++j) {
	  fprintf(csv,"%2.8f, ",tc.qd[j]);
	}
	for (unsigned int j=0; j<DOF; ++j) {
	  fprintf(csv,"%2.8f, ",tc.qdd[j]);
	}
	for (unsigned int j=0; j<DOF; ++j) {
	  fprintf(csv,"%2.8f, ",tc.t[j]);
	}
      }
      fclose(csv);
    }
    reset(oldtime);
    runstate=STOP;
    free(simfname);
    return true;
  }

  void Trajectory::ForceFeedback(double ft[]) {
    forcetorque = R6(ft[0],ft[1],ft[2],
		     ft[3],ft[4],ft[5]);
    valid_ft=true;
   
    // check force threshold
    if ((runstate==RUN) &&
	CancelOnForceInput &&
	((forcetorque.v * forcetorque_threshold_direction) 
	 > forcetorque_threshold)) {
      runstate=ABORT;
    }
    // check torque threshold
    if ((runstate==RUN) &&
	CancelOnForceInput &&
	((forcetorque.w * forcetorque_torque_threshold_direction) 
	 > forcetorque_torque_threshold)) {
      runstate=ABORT;
    }
  }

  void Trajectory::TactileFeedback(float tactile[], int repetitions) {
    // check whether we should stop the trajectory based on
    // tactile sensor values.
    // args:
    //   tactile: array of 96 readings
    //   repetitions: number of times that this function will be called
    //       before every  cell has been updated.  OWD gets just a partial
    //       update of the tactile data in each CAN message, so the
    //       repetitions argument allows this function to take that
    //       into account.
    // preset values:
    //   tactile_pad: the pad number (0 through 3) that we are watching
    //   tactile_threshold: the reading that is considered "pressed"
    //   tactile_minimum_cells: the number of cells that have to meet
    //      or exceed the threshold in a single cycle for the pad to be
    //      considered "pressed"
    //   tactile_minimum_readings: the number of consecutive times the
    //      cell should be "pressed" to stop the trajectory
    static int tactilecount(0);
    static int reps_counter = 0;
    static JointPos tact_data_filtered(96);

    const double DIFF_LAST_THRESH = 2.0;

    static double tactile_last[96]={0.0};


    JointPos tact_data(96);
    for (int i=0; i < 24*4; i++)
    {
      tact_data[i] = (double)tactile[i];
    }


    //if gone through whole cycle, update the old values
    reps_counter++;
    if (reps_counter % repetitions == 0)
    {
      tact_data_filtered =  tactile_filter.eval(tact_data);
    }

    //subtract off lowpass
    //also, since sensors have random spikes, simply threshhold to remove if the difference is too large
    for (int i=0; i < 24*4; i++)
    {
      tact_data[i] -= tact_data_filtered[i];
      if (abs(tact_data[i] - tactile_last[i]) > DIFF_LAST_THRESH)
      {
        tact_data[i] = 0.0;
      }
    }

    if (reps_counter % repetitions == 0)
    {
      for (int i=0; i < 24*4; i++)
      {
        tactile_last[i] = tact_data[i];
      }
    }


    // count the number of responses
    if ((runstate==RUN) && CancelOnTactileInput) 
    {
      int cellcount(0);
      double total_response(0.0);

      for (int i=0; i<24; ++i) {
        if (tact_data[i + tactile_pad*24] > tactile_threshold) {
          ++cellcount; 
        }
        
        total_response+= tact_data[i+tactile_pad*24];
      }
      //if (cellcount > tactile_minimum_cells) 
      if (total_response > tactile_threshold) 
      {
        ++tactilecount;
        if (tactilecount > tactile_minimum_readings * repetitions) {
          runstate = ABORT;
        }
      } else {
        tactilecount=0;
      }

    }

  }

  BinaryData Trajectory::serialize(int firstdof, int lastdof) {
    if (lastdof == -1) {
      lastdof = start_position.size()-1;
    }
    if (firstdof > (int)start_position.size()) {
      throw "Invalid firstdof";
    }
    if (lastdof >= (int)start_position.size()) {
      throw "Invalid lastdof";
    }
    // create scaled-down versions of our start and end position
    // vectors with just the DOFS that were requested
    JointPos sp,ep;
    sp.insert(sp.begin(),
	      start_position.begin()+firstdof,
	      start_position.begin()+lastdof+1);
    ep.insert(ep.begin(),
	      end_position.begin()+firstdof,
	      end_position.begin()+lastdof+1);
    
    BinaryData bd;
    bd.PutDoubleVector(sp);
    bd.PutDoubleVector(ep);
    bd.PutString(      id);
    bd.PutString(      type);
    bd.PutDouble(      duration);
    bd.PutBoolean(     CancelOnStall);
    bd.PutBoolean(     WaitForStart);
    bd.PutBoolean(     Synchronize);
    bd.PutBoolean(     CancelOnForceInput);
    bd.PutBoolean(     CancelOnTactileInput);
    
    return bd;
  }

  Trajectory::Trajectory(BinaryData &bd)
    : runstate(STOP), time(0),
      CancelOnStall(false),WaitForStart(false),
      Synchronize(false),
      CancelOnForceInput(false),
      CancelOnTactileInput(false),
      valid_ft(false),
      tactile_filter(3,5)
  {
    start_position       =bd.GetDoubleVector();
    end_position         =bd.GetDoubleVector();
    id                   =bd.GetString();
    type                 =bd.GetString();
    duration             =bd.GetDouble();
    CancelOnStall        =bd.GetBoolean();
    WaitForStart         =bd.GetBoolean();
    Synchronize          =bd.GetBoolean();
    CancelOnForceInput   =bd.GetBoolean();
    CancelOnTactileInput =bd.GetBoolean();
    
    pthread_mutex_init(&mutex,NULL);
    
  }

  std::string Trajectory::random_id() {
    char idstr[20];
    snprintf(idstr,20,"%x", rand() * 1<<16 / RAND_MAX);
    return std::string(idstr);
  }

  Trajectory::TrajControl::TrajControl(unsigned int nDOF) :
    q(nDOF), qd(nDOF), qdd(nDOF), t(nDOF)
  {
  }

  R3 Trajectory::forcetorque_threshold_direction(0,0,-1);  // negative Z (towards palm)
  double Trajectory::forcetorque_threshold(6.0);    // 6 newtons
  R3 Trajectory::forcetorque_torque_threshold_direction(1,0,0);
  double Trajectory::forcetorque_torque_threshold(999); // disabled at start

  int Trajectory::tactile_pad(3);
  float Trajectory::tactile_threshold(3.5);
  int Trajectory::tactile_minimum_readings(0);
}; // namespace OWD

