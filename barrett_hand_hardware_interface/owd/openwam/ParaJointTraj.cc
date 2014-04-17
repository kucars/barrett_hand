/***********************************************************************

  Copyright 2007-2011 Carnegie Mellon University and Intel Corporation
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
#include <ros/ros.h>

namespace OWD {

ParaJointTraj::~ParaJointTraj() {
    delete [] parsegs;
    delete [] current_segment;
}

/* non-synchronized changes:
 1. fit each joint start to finish, based on its own
    inflection points (ignore bends).
 2. find the slowest joint, and rescale segments of each
    of the other joints to match overall end time.

if that doesn't work (collisions between a particular pair
of RAVE points), add an inflection point to the joint that's
furthest from the linear path.  Could find which joint has
a value furthers from the segment endpoints, and put an
inflection for that joint at that endpoint.  Or maybe add
the inflection at the point that's closest to the collision.

keep adding inflection points until no collisions.


*/

/*
 basic algorithm:
 1. go through trajectory point-by-point.  If we're not
      at the end, create a ps for each joint, and watch
      for inflection points
 2. if an inflection point is found for any joint, start over at
    joint 1 and finalize the current segment for each joint.  Keep track of
    the max segment time.
 3. Finally, go through each joint again, rescaling the segments to
    match the max time.
 4. start a new Ps and continue with next point.
*/


ParaJointTraj::ParaJointTraj(TrajType &vtraj, 
                             const std::vector<double> &mjv, 
                             const std::vector<double> &mja,
                             bool bWaitForStart,
                             bool bCancelOnStall,
			     bool bCancelOnForceInput,
			     bool bCancelOnTactileInput) :
  Trajectory("ParaJointTraj",""),max_joint_vel(mjv),
  max_joint_accel(mja), restart(false)
{
  WaitForStart=bWaitForStart;
  CancelOnStall=bCancelOnStall;
  CancelOnForceInput=bCancelOnForceInput;
  CancelOnTactileInput=bCancelOnTactileInput;
  pthread_mutex_init(&mutex, NULL);
  runstate=STOP;
  time=0.0;
    if (vtraj.size() < 2) {
      throw "Trajectories must have 2 or more points; do a move instead";
    }

    DOF = vtraj[0].size();
    parsegs = new std::vector<ParabolicSegment>[DOF];
    current_segment = new std::vector<ParabolicSegment>::iterator[DOF];

    int numpoints = vtraj.size();
    start_position = vtraj.front();
    
    // start by initializing a PS for each joint
    for (int j = 0; j<DOF; ++j) {
        ParabolicSegment ps(0,0.0,vtraj[0][j],vtraj[1][j]);
        parsegs[j].push_back(ps);
    }
    
    // now, go through all points, extending the PS when possible
    // and creating a new one otherwise
    for (int i = 2; i < numpoints; ++i) {        
        bool bend=check_for_bend(parsegs,vtraj[i-1],vtraj[i]);
#ifdef INFLECT_CHECK
        bool inflect=false;
        // for each joint
        for (int j = 0; j<DOF; ++j) {
            if (parsegs[j].back().inflection(vtraj[i-1][j],vtraj[i][j])) {
                inflect=true;
               ROS_DEBUG_NAMED("trajectory","ParaJointTraj: Inflection found: joint %d was %s, now going from %2.8f to %2.8f\n",
                       j,parsegs[j].back().dir==ParabolicSegment::CONST?"const":
                       parsegs[j].back().dir==ParabolicSegment::UP?"up":"down",
                       vtraj[i-1][j],vtraj[i][j]);
                break;
            }
        }
        ROS_WARN_COND(inflect && !bend,
		      "ParaJointTraj: bend didn't detect inflection, point %d",i);
	ROS_WARN_COND(bend && !inflect,
		      "ParaJointTraj: EXTRA BEND FOUND AT POINT %d, SEGMENT %zd",i,parsegs[0].size());
#endif // INFLECT_CHECK
        if (bend) {
            // finalize PS for each joint
            // while we're at it, keep track of the slowest joint
            double max_end_time=parsegs[0].back().start_time;
            double accel_time=0.0f;
            int slowest_joint = 0;
            for (int k=0; k<DOF; ++k) {
                // finalize up to the inflection point
                parsegs[k].back().end_index = i-1;
                parsegs[k].back().end_pos = vtraj[i-1][k];
                
                parsegs[k].back().fit_curve(max_joint_vel[k],
                                            max_joint_accel[k]);
                ROS_DEBUG_NAMED("trajectory","ParaJointTraj: first fitting J%d: ",k);
                parsegs[k].back().dump();

                if (parsegs[k].back().end_time > max_end_time) {
                    max_end_time = parsegs[k].back().end_time;
                    accel_time = parsegs[k].back().time_a;
                    slowest_joint = k;
                }
            }
            ROS_DEBUG_NAMED("trajectory","ParaJointTraj: After first pass, max_end=%2.3f, ta=%2.3f, j=%d\n",
                   max_end_time,accel_time,slowest_joint);
            if (accel_time == 0.0f) {
                // all the segments must have been constant.
                // therefore, we can eliminate all of them, and just start
                // new segments from here.
	      ROS_WARN("Discarding constant segments between points %d and %d",parsegs[0].back().start_index,i-1);
                for (int k=0; k<DOF; ++k) {
                    parsegs[k].pop_back();
                }
                ROS_DEBUG_NAMED("trajectory","ParaJointTraj: Last segment was const for all joints; wiped out\n");
            // otherwise, we'll rescale all the segments to match the slowest
            } else if (rescale_to_slowest(slowest_joint,max_end_time,accel_time,
                                   max_joint_vel,max_joint_accel)) {
	      ROS_ERROR("ParaJointTraj: error occured at traj point %d\n",i+1);
            }
            
            // start new segments for each joint
            for (int k=0; k<DOF; ++k) {
                ParabolicSegment ps(i-1,max_end_time,
                                    vtraj[i-1][k],vtraj[i][k]);
                parsegs[k].push_back(ps);
            }
        }
    }
    int i = numpoints-1; // finalize the last open segment
    double max_end_time=parsegs[0].back().start_time;
    double accel_time = 0.0f;
    int slowest_joint = 0;
    for (int k=0; k<DOF; ++k) {
        // finalize up to the last point
        parsegs[k].back().end_index = i;
        parsegs[k].back().end_pos = vtraj[i][k];
        
        parsegs[k].back().fit_curve(max_joint_vel[k],
                                    max_joint_accel[k]);
        ROS_DEBUG_NAMED("trajectory","ParaJointTraj: first fitting J%d: ",k);
        parsegs[k].back().dump();
        if (parsegs[k].back().end_time > max_end_time) {
            max_end_time = parsegs[k].back().end_time;
            accel_time = parsegs[k].back().time_a;
            slowest_joint = k;
        }
    }
    ROS_DEBUG_NAMED("trajectory","ParaJointTraj: After first pass, max_end=%2.3f, ta=%2.3f, j=%d\n",
           max_end_time,accel_time,slowest_joint);
    if (accel_time == 0.0f) {
        // all the segments must have been constant.
        // therefore, we can eliminate all of them and end at the previous segment
        if (parsegs[0].size() == 1) {
            // if there was only one segment, and it has all constant joint values, we
            // don't even need the trajectory.  But it's probably better to keep it and
            // let it eval for one cycle, so that the calling function doesn't freak out.
            duration = parsegs[0].back().end_time;
            end_position = vtraj.back();
            // set the current_segment[] iterators
            for (int j = 0; j<DOF; ++j) {
                current_segment[j]=parsegs[j].begin();
            }
            return;
        }
        for (int k=0; k<DOF; ++k) {
            parsegs[k].pop_back();
        }
        ROS_DEBUG_NAMED("trajectory","ParaJointTraj: Last segment was const for all joints; wiped out\n");
    // otherwise, rescale the final segments to match the slowest one
    } else if (rescale_to_slowest(slowest_joint,max_end_time,accel_time,
                           max_joint_vel,max_joint_accel)) {
      ROS_ERROR("ParaJointTraj: error occured at traj point %d\n",i+1);
    }
    duration = parsegs[0].back().end_time;
    end_position = vtraj.back();

    // set the current_segment[] iterators
    for (int j = 0; j<DOF; ++j) {
        current_segment[j]=parsegs[j].begin();
    }
    return;
}

bool ParaJointTraj::check_for_bend(std::vector<ParabolicSegment> *ps,
                                   TrajPoint& p1, TrajPoint &p2) {
    double biggest_slope=0.0f;
    int biggest_slope_j=-1;
    double slope[DOF];
    for (int j=0; j<DOF; j++) {
        slope[j]=p1[j]-ps[j].back().start_pos;  // current pt minus seg start
        if (fabs(slope[j])>biggest_slope) {
            biggest_slope=fabs(slope[j]);
            biggest_slope_j=j;
        }
    }
    if (biggest_slope < TRAJ_TOLERANCE) {
        // all segments are CONST, so just compare points
        for (int j=0; j<DOF; j++) {
            if (fabs(p2[j] - p1[j]) > TRAJ_TOLERANCE) {
                return true;
            }
        }
    }
    double run = (p2[biggest_slope_j]-p1[biggest_slope_j])
        / slope[biggest_slope_j];
    if (run<0.0f) {
        return true; // we switched signs
    }
    for (int j=0; j<DOF; j++) {
        if ((j != biggest_slope_j) &&
            (fabs(p1[j] + slope[j] * run -p2[j]) > TRAJ_TOLERANCE)) {
            return true;
        }
        if (sgn(p2[j]-p1[j]) != sgn(slope[j])) {
            return true;  // inflection point
        }
    }
            
    return false;
}

int ParaJointTraj::rescale_to_slowest(int slowest_joint,double max_end_time,double accel_time,const std::vector<double> &max_joint_vel, const std::vector<double> &max_joint_accel) {
    // Make one more pass through the segments, rescaling
    // most of them to match the slowest one
    for (int k=0; k<DOF; ++k) {
        if (k != slowest_joint) {
            try {
                parsegs[k].back().refit_curve(max_joint_vel[k],
                                              max_joint_accel[k],
                                              max_end_time,
                                              accel_time);
                ROS_DEBUG_NAMED("trajectory","ParaJointTraj: second fitting J%d: ",k);
                parsegs[k].back().dump();
            } catch (const char *msg) {
	      ROS_ERROR("ParaJointTraj: error in joint %d: %s",k+1,msg);
	      ROS_ERROR("ParaJointTraj: Limiting joint %d, max time %3.3f accel time %3.3f\n",slowest_joint+1,max_end_time,accel_time);
	      ROS_ERROR("ParaJointTraj: dump of all joints for this seg:\n");
                for (int k2 = 0; k2 < DOF; k2++) {
		  ROS_ERROR("ParaJointTraj: joint %d: ",k2+1);
                    parsegs[k2].back().dump();
                }
                return -1;
            }
        }
    }
    return 0;
}

void ParaJointTraj::evaluate_abs(Trajectory::TrajControl &tc, double t) {

  // DANGER: cannot call any ROSOUT functions (ROS_DEBUG, etc) from within
  // this function, or it will kill our realtime performance.  Instead, the
  // message should be pushed onto a queue or a RT fifo, so that the non-RT
  // thread can get them out and print them.  For now they are all commented
  // out.

  if (tc.q.size() < (unsigned int)DOF) {
    runstate=DONE;
    return;
  }

  // if we're running, then increment the time.  Otherwise, stay where we are
  if ((runstate == RUN) || (runstate == LOG)) {
    // check for restart segment
    if (restart) { 
      restart_time = t - current_segment[0]->end_time;
      if (restart_time > restart_parsegs[0].end_time) {
	// we're done with the restart segment, so switch
	// back to the regular segment
	restart_parsegs.clear();
	restart=false;
      } 
    }
    time = t;
    //    ROS_DEBUG_NAMED("trajectory","ParaJointTraj eval: time=%3.3f",time);
  }
  int finished_joints = 0;
  for (int j = 0; j < DOF; j++) {
    // check to see if we need to go forward
    while ((time > current_segment[j]->end_time) &&
	   (current_segment[j] != parsegs[j].end())) {
      // keep skipping segments until we find the one that includes this time
      //	ROS_DEBUG_NAMED("trajectory","ParaJointTraj: incrementing segment for joint %d, time=%2.2f, et=%2.2f\n",j,time,current_segment[j]->end_time);
      ++current_segment[j];
    }
    if (current_segment[j] == parsegs[j].end()) {
      // we're done
      tc.q[j] = parsegs[j].back().end_pos;
      tc.qd[j]=tc.qdd[j]=0.0;
      finished_joints++;
      continue;
    }
    // check to see if we need to go backwards
    while ((time < current_segment[j]->start_time) &&
	   (current_segment[j] != parsegs[j].begin())) {
      // keep skipping segments until we find the one that includes this time
      //	ROS_DEBUG_NAMED("trajectory","ParaJointTraj: decrementing segment for joint %d, time=%2.2f, et=%2.2f\n",j,time,current_segment[j]->end_time);
      --current_segment[j];
    }
    if (time < current_segment[j]->start_time) {
      //	ROS_ERROR_NAMED("trajectory","Could not locate proper segment for trajectory.");
      //	ROS_ERROR_NAMED("trajectory","time %2.2f, current segment start %2.2f, end %2.2f",time,current_segment[j]->start_time,current_segment[j]->end_time);
      throw "Could not locate matching segment";
    }
    try {
      // now that we've found the right segments, ask each segment
      // to evaluate itself.
      if (restart) {
	// if we stopped during a previous restart, use the
	// restart segments
	restart_parsegs[j].evaluate(tc.q[j],tc.qd[j],tc.qdd[j],restart_time);
      } else {
	// otherwise, use the standard segments
	current_segment[j]->evaluate(tc.q[j],tc.qd[j],tc.qdd[j],time);
      }
    } catch(const char *msg) {
      if (restart) {
	  //	  ROS_ERROR("ParaJointTraj: error occurred within joint %d, restart segment, time %3.3f of %3.3f\n",
	  //		    j+1,restart_time,restart_parsegs[0].end_time);
	  //	  ROS_ERROR("ParaJointTraj: Dump of all joints for that segment:\n");
	  for (int k = 0; k < DOF; k++) {
	    //	    ROS_ERROR("ParaJointTraj: %d: ",k+1);
	    restart_parsegs[k].dump();
	  }
      } else {
	  //	  ROS_ERROR("ParaJointTraj: Error occurred within joint %d, time %3.3f of %3.3f\n",
	  //		    j+1,time,duration);
	  //	  ROS_ERROR("ParaJointTraj: Dump of all joints for that segment:\n");
	  for (int k = 0; k < DOF; k++) {
	    //	    ROS_ERROR("ParaJointTraj: %d: ",k+1);
	    current_segment[k]->dump();
	  }
      }
      runstate = STOP;
      ROS_ERROR("Error in ParaJointTraj evaluate(): %s",msg);
      throw "problem in ParaJointTraj evaluate()";
	
    }
    if (runstate == STOP) {
      // if we're supposed to be stationary, keep the position values we
      // calculated, but zero out the vel and accel
      for (int j=0; j < DOF; j++) {
	tc.qd[j]=tc.qdd[j]=0.0;
      }
    }
  }
  if (finished_joints) {
    if (finished_joints== DOF) {
      runstate = DONE;
    } else {
      //      ROS_DEBUG_NAMED("trajectory","ParaJointTraj: only %d joints have finished\n",finished_joints);
    }
  }
  return;
}

void ParaJointTraj::run() {
    if (runstate==DONE || runstate==RUN) {
        return;
    }
    if (time == 0.0f) {
        runstate=RUN;
        return;
    }

    // still need to fix non-synchronized restarts
    runstate=RUN;
    return;

    // since the time was non-zero, it means we were stopped mid-trajectory.
    // instead of just restarting, we need to calculate a smooth start from
    // the current position, assuming zero velocity.
    
    double max_end_time=0.0f;
    double accel_time;
    int slowest_joint;
    double endpos[DOF];
    OWD::Trajectory::TrajControl tc(DOF);
    for (int j=0; j<DOF; j++) {
        // get the current and end positions
        if (restart) {
            // if we were already in a restart segment and got stopped again,
            // use the restart segment and time for the current position
            restart_parsegs[j].evaluate(tc.q[j],tc.qd[j],tc.qdd[j],restart_time);
            endpos[j] = restart_parsegs[j].end_pos;
        } else {
            // otherwise, use the original segment
	  current_segment[j]->evaluate(tc.q[j],tc.qd[j],tc.qdd[j],time);
	  endpos[j] = current_segment[j]->end_pos;
        }
    }
    if (restart_parsegs.size() > 0) {
      // we're done with the previous restart segs, so delete them
      restart_parsegs.clear();
    }
    for (int j=0; j<DOF; j++) {
      // create our replacement segment
      ParabolicSegment ps(0,0.0f,tc.q[j],endpos[j]);
        ps.end_index = 1;
        ps.end_pos = endpos[j];
        ps.fit_curve(max_joint_vel[j],max_joint_accel[j]);
        if (ps.end_time > max_end_time) {
            max_end_time=ps.end_time;
            accel_time = ps.time_a;
            slowest_joint=j;
        }
        restart_parsegs.push_back(ps);
    }
    if (accel_time == 0.0f) {
        // all segments were constant, so we must already be at the end point.
        // just restart at the next segment.
        restart=false;
        restart_parsegs.clear();
        if (current_segment[0] == parsegs[0].end()) {
            // we're already at the last segment, so just move to the end
            time = parsegs[0].back().end_time;
            runstate=RUN; // need to allow one more call to evaluate
            return;
        } else {
            current_segment[0]++;
            time=current_segment[0]->start_time;
            runstate=RUN;
            return;
        }
    }
    // otherwise, rescale most of the joints to match the slowest joint
    for (int j=0; j<DOF; ++j) {
        if (j != slowest_joint) {
            try {
                restart_parsegs[j].refit_curve(max_joint_vel[j],
                                               max_joint_accel[j],
                                               max_end_time,
                                               accel_time);
            } catch (const char *msg) {
	      ROS_ERROR("ParaJointTraj: ERROR in joint %d: %s",j+1,msg);
	      ROS_ERROR("ParaJointTraj: Limiting joint %d, max time %3.3f accel time %3.3f\n",slowest_joint+1,max_end_time,accel_time);
	      ROS_ERROR("ParaJointTraj: Dump of all joints for this seg:\n");
	      for (int k2 = 0; k2 < DOF; k2++) {
		ROS_ERROR("ParaJointTraj: joint %d: ",k2+1);
		restart_parsegs[k2].dump();
	      }
	      throw "Could not rescale joint speeds"; // give up
            }
        }
    }
    restart_time=0.0;
    restart=true;
    runstate = RUN;
}

void ParaJointTraj::stop() {
    runstate = STOP;
}

int ParaJointTraj::state() {
    return runstate;
}

bool ParaJointTraj::log(const char *trajname) {
    if ((runstate != STOP) && (runstate != DONE)) {
        // can't log a running trajectory; it would mess up the times
        return false;
    }
    double oldtime=time;
    char *simfname = (char *) malloc(strlen(trajname) +9);
    // start with the logfilename
    sprintf(simfname,"%s_sim.csv",trajname);
    FILE *csv = fopen(simfname,"w");
    if (csv) {
      Trajectory::TrajControl tc(DOF);
        runstate=LOG;
        time=0.0;
        while (runstate == LOG) {
            evaluate(tc,0.01);
            fprintf(csv,"%3.8f, ",time);
            for (int j=0; j<DOF; ++j) {
                fprintf(csv,"%2.8f, ",tc.q[j]);
            }
            for (int j=0; j<DOF; ++j) {
                fprintf(csv,"%2.8f, ",tc.qd[j]);
            }
            for (int j=0; j<DOF; ++j) {
                fprintf(csv,"%2.8f, ",tc.qdd[j]);
            }
            for (int j=0; j<DOF; ++j) {
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

void ParaJointTraj::reset(double t) {
  time=t;
  for (int j=0; j<DOF; ++j) {
    current_segment[j]=parsegs[j].begin();
  }
}

}; // namespace OWD
