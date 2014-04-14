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

/* Modified 2007-2010 by:
      Mike Vande Weghe <vandeweg@cmu.edu>
      Robotics Institute
      Carnegie Mellon University
*/

#include <sys/time.h>
#include "Dynamics.hh"
#include "Plugin.hh"

extern "C" {
  void dpotrf_(char *uplo, int *n, double *a, int *lda, int *info);
  void dpotri_(char *uplo, int *n, double *a, int *lda, int *info);
  void dsymm_(char *side, char *uplo, int *m, int *n, double *alpha, double *a,
	      int *lda, double* b, int *ldb, double *beta, double *c,int *ldc);
  void dgemm_(char *transa, char *transb, int *m, int *n, int *k,double *alpha,
	      double *a, int *lda, double *b, int *ldb, double *beta,double *c,
	      int *ldc);
  void dsymv_(char *uplo, int *n, double *alpha, double *a, int *lda,double *x,
	      int *incx, double *beta, double *y, int *incy);
  void dgemv_(char *trans, int *m, int *n, double *alpha, double *a, int *lda,
	      double *x, int *incx, double *beta, double *y, int *incy);
}

namespace Dynamics {
   R3 z0(0,0,1);
   R3 up(0,0,1);
}

#ifdef WRIST
int dyn_active_link=7;
#else
int dyn_active_link=4;
#endif // WRIST

/*
 * Recursive Newton-Euler method (Luh, Walker, Paul)
 */
void RNE(double tau[Link::Ln+1], 
	 Link links[Link::Ln+1], 
	 double qd[Link::Ln+1], 
	 double qdd[Link::Ln+1], R6& fext){

  R3 w, wd, v, vd, vdhat;
  R3 n, f, N[Link::Ln+1], F[Link::Ln+1];

  vd = OWD::Plugin::gravity*Dynamics::up;

  for(int i=Link::L1; i<=Link::Ln; i++){

    SO3 A      = !((SO3)links[i]);
    Inertia I  =        links[i].I();
    R3 ps      =        links[i].ps();
    R3 s       =        links[i].s();
    double m   =        links[i].m();
    
    wd = A*( wd + (Dynamics::z0*qdd[i]) + (w^(Dynamics::z0*qd[i])) );
    w  = A*( w  + (Dynamics::z0*qd [i]) );
    vd = (wd^ps) + (w^(w^ps)) + A*vd;

    vdhat = (wd^s) + (w^(w^s)) + vd;
    F[i] = m*vdhat;
    N[i] = (I*wd) + (w^(I*w));
  }

  /*
  f[R3::X] = fext[Wrench::FX]; 
  f[R3::Y] = fext[Wrench::FY];
  f[R3::Z] = fext[Wrench::FZ];
  n[R3::X] = fext[Wrench::MX];
  n[R3::Y] = fext[Wrench::MY];
  n[R3::Z] = fext[Wrench::MZ];
  */

  for(int i=Link::Ln; Link::L1<=i; i--){
    SO3 A;
    R3 ps = links[i].ps();
    R3 s  = links[i].s();

    if(i != Link::Ln)
      A = (SO3)links[i+1];

    //n = A*( n + (((!A)*ps)^f) ) + ((ps+s)^F[i]) + N[i];
    //f = A*f + F[i];

    f = A*f + F[i];
    n = A*n + (ps^f) + (s^F[i]) + N[i];  // ask me for this simplification
    A = !((SO3)links[i]);
    tau[i] = n*(A*Dynamics::z0);
  }
}

/*
 * Use to get the Coriolis/Centrifugal + gravity load
 * Based on RNE with all acceleration equal to 0 (Walker, Orin)
 */
void CCG(double ccg[Link::Ln+1], Link links[Link::Ln+1],double qd[Link::Ln+1]){
    /* inputs: ccg - pointer to array of drive torques, to be filled by this function
               links - pointer to array of links
               qd - pointer to array of joint velocities */

    R3 w; //angular velocity of this link
    R3 wd; //angular acceleration of this link
    R3 v; //linear velocity of this link at the link origin
    R3 vd; //linear acceleration of this link at the link origin
    R3 vdhat; //linear acceleration of center of mass of this link
    R3 n; //torque applied to this link by the previous link (backward recursing)
    R3 f; //force applied to this link by the previous link (backward recursing)
    R3 N[Link::Ln+1]; //torques applied to links (forward recursing)
    R3 F[Link::Ln+1]; //forces applied to links (forward recursing)
  
    vd = OWD::Plugin::gravity*Dynamics::up; //acceleration from gravity

  //forward recursive computation for kinematic variables
  for(int i=Link::L1; i<=Link::Ln; i++){

      SO3 A      = !((SO3)links[i]); //the inverse of the rotation of this link, this is used to transform all quantities from the previous link's coordinates to this link's coordinates (remember that (SO3)links[i] is the transform from this link to the previous link)
      Inertia I  = links[i].I(); //the inertia of this link
      R3 ps      = links[i].ps(); //position vector from origin of previous link to origin of this link
      R3 s       = links[i].s(); //position vector from this link origin to cog of this link
      double m   = links[i].m(); //mass of this link

      wd = A*( wd + (w^(Dynamics::z0*qd[i])) ); //calculate angular acceleration on this link
      w  = A*( w  + (Dynamics::z0*qd [i]) ); //calculate angular velocity on this link
      vd = (wd^ps) + (w^(w^ps)) + A*vd; //calculate linear acceleration on this link

      vdhat = (wd^s) + (w^(w^s)) + vd; //calculate linear acceleration at the cog
      F[i] = m*vdhat; //force at the cog
      N[i] = (I*wd) + (w^(I*w)); //torque about the joint
  }

  R3 f_ip1;

  //backward recursive computation for dynamic variables
  for(int i=Link::Ln; Link::L1<=i; i--){
    SO3 A;
    R3 ps = links[i].ps();
    R3 s  = links[i].s();

    if(i != Link::Ln)
        A = (SO3)links[i+1]; //rotation from link above this one

    f_ip1 = f; //save the old f
    f = A*f + F[i];
    n = A*n + (ps^f) + (s^F[i]) + N[i]; //original

    //n = A*( n + (((!A)*ps)^f_ip1) ) + ((ps+s)^F[i]) + N[i]; //from RNE calc above
    //n = A*(n + (ps^(A*f_ip1)) + (s^F[i]) + N[i]; //more correct but too loose
    //    n = A*n  
    //        + (ps^f)         // torque created from subsequent links pushing
    //                         // on this link's origin
    //        + ((ps-s)^F[i])  // torque created by applying F[i] force at this link's COG
    //        + N[i];

    //n = A*(n + (ps)^f_ip1) + (ps+s)^F[i] + N[i]; //from Luh, Walker, Paul '80 eq 44 (doesn't work)
    A = !((SO3)links[i]);
    ccg[i] = n*(A*Dynamics::z0);
  }
}

/*
 * Use to compute the wd and vd. Would be cleaner if it returned a twist...
 */
R6 acceleration( Link links[Link::Ln+1], 
		 double qd[Link::Ln+1], 
		 double qdd[Link::Ln+1] ){
  R3 w, wd, vd;

  for(int i=Link::L1; i<=Link::Ln; i++){

    SO3 A      = !((SO3)links[i]);
    R3 ps      =        links[i].ps();
    
    wd = A*( wd + (Dynamics::z0*qdd[i]) + (w^(Dynamics::z0*qd[i])) );
    w  = A*( w  + (Dynamics::z0*qd [i]) );
    vd = (wd^ps) + (w^(w^ps)) + A*vd;
  }

  return R6(vd, wd);
}

/*
 * Computes the bias acceleration Jd*qd (as in [vdd;wdd] = Jd*qd + J*qdd
 */
R6 bias_acceleration(Link links[Link::Ln+1], double qd[Link::Ln+1]){
  R3 w, wd, vd;

  for(int i=Link::L1; i<=Link::Ln; i++){

    SO3 A      = !((SO3)links[i]);
    R3 ps      =        links[i].ps();
    
    wd = A*( wd + ( w^(Dynamics::z0*qd[i]) ) );
    w  = A*( w  + (    Dynamics::z0*qd[i]  ) );
    vd = (wd^ps) + (w^(w^ps)) + A*vd;
  }

  return R6(vd, wd);
}

/*
 * Computation of the joint space inertia matrix (Walker, Orin)
 * Because the it is symmetric, only the top half is filled.
 * So in fortran (column major) that becomes the lower half
 * Takes about 26usec (17usec with Intel's compiler)
 */
void JSinertia(double A[Link::Ln][Link::Ln], Link links[Link::Ln+1]){
  Inertia I(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);
  int j = Link::Ln;
  SO3 iAj = (SO3)links[j];
  R3 jsj =       links[j].s();
  R3 jpsj =      links[j].ps();
  double mj =    links[j].m();
  double Mj = mj;

  Inertia jEj = links[j].I();
  R3 icj = iAj*(jsj + jpsj);

  Inertia iEj = iAj*jEj*(!iAj);

  R3 iFj = Dynamics::z0^(Mj*icj);
  R3 iNj = iEj*Dynamics::z0;
  
  R3 ifj = iFj;
  R3 inj = iNj + (icj^iFj);
  
  A[j-1][j-1] = inj[R3::Z];

  for(int i=j-1; Link::L1<=i; i--){
    iAj     = (SO3)links[i];
    R3 ipsi =      links[i].ps();

    inj = iAj * (inj + (ipsi^ifj));  // order is important
    ifj = iAj * ifj;

    A[i-1][j-1] = inj[R3::Z];    // the -1 is to ajust the indexes
  }

  for(j=Link::Ln-1; Link::L1<=j; j--){
    double Mk = Mj;
    R3 jck = icj;
    Inertia jEk = iEj;

    iAj         = (SO3)links[j];
    jsj         =      links[j].s();
    Inertia jJj =      links[j].I();
    jpsj        =      links[j].ps();

    mj = links[j].m();
    Mj = Mj + mj;

    R3 jcj = (1.0/Mj) * ( mj*(jsj+jpsj) + Mk*(jck+jpsj) );
    icj = iAj * jcj;

    R3 v1(jck+jpsj-jcj);
    R3 v2(jsj+jpsj-jcj);
    Inertia J1(v1[R3::X]*v1[R3::X], v1[R3::X]*v1[R3::Y], v1[R3::X]*v1[R3::Z],
	       v1[R3::Y]*v1[R3::Y], v1[R3::Y]*v1[R3::Z], v1[R3::Z]*v1[R3::Z]);
    Inertia J2(v2[R3::X]*v2[R3::X], v2[R3::X]*v2[R3::Y], v2[R3::X]*v2[R3::Z],
	       v2[R3::Y]*v2[R3::Y], v2[R3::Y]*v2[R3::Z], v2[R3::Z]*v2[R3::Z]);

    iEj = iAj * ( jEk + Mk*( I*(v1*v1) - J1 ) + 
		  jJj + mj*( I*(v2*v2) - J2 ) ) * (!iAj);
    
    iFj = Dynamics::z0^(Mj*icj);
    iNj = iEj*Dynamics::z0;

    ifj = iFj;
    inj = iNj + (icj^iFj);
    A[j-1][j-1] = inj[R3::Z];

    for(int i=j-1; Link::L1<=i; i--){
      iAj     = (SO3)links[i];
      R3 ipsi =      links[i].ps();
     
      inj = iAj * (inj + (ipsi^ifj)); // order is important
      ifj = iAj * ifj;

      A[i-1][j-1] = inj[R3::Z];
    }
  }
}

/*
 * Computation of the worspace space inertia matrix (Khatib)
 * Only the upper half is computed (lower part in fortran)
 * Takes about 48usec (42 with Intel's compiler)
 */
void WSinertia(double Ac[6][6], Link links[Link::Ln+1]){
  char SIDE = 'R';
  char UPLO = 'L';
  char TRANST = 'T';  
  char TRANSN = 'N';
  double ALPHA = 1.0; 
  double BETA = 0.0;
  int NEQS = 6; 
  int NJOINTS = Joint::Jn;
  int INFO;

  double A[Joint::Jn][Joint::Jn];       // Joint space inertia matrix
  int LDA = Joint::Jn;

  JSinertia(A, links);

  int LDJ = 6;

  // A^-1
  // Factorize the symmetric positive definite matrix
  dpotrf_(&UPLO, &NJOINTS, &A[0][0], &LDA, &INFO);
  if(INFO<0)
    cerr << "WSinertia::dpotrf1: The " << INFO << " argument is illegal.\n";
  else if(0<INFO)
    cerr << "WSinertia::dpotrf1: The matrix is not positive definite.\n";
  // invert
  dpotri_(&UPLO, &NJOINTS, &A[0][0], &LDA, &INFO);
  if(INFO<0)
    cerr << "WSinertia::dpotri1: The " << INFO << " argument is illegal.\n";
  else if(0<INFO)
    cerr << "WSinertia::dpotri1: The matrix is singular.\n";

  // J*A^-1
  double JAi [Joint::Jn][6];
  int LDJAi = 6;
  dsymm_(&SIDE, &UPLO, &NEQS, &NJOINTS, &ALPHA, 
	 &A  [0][0], &LDA, 
	 &OWD::Kinematics::JacobianEE  [0][0], &LDJ, &BETA, 
	 &JAi[0][0], &LDJAi);
  
  // J*A^-1*J'
  int LDAc = 6;
  dgemm_(&TRANSN, &TRANST, &NEQS, &NEQS, &NJOINTS, &ALPHA, 
	 &JAi[0][0], &LDJAi, 
	 &OWD::Kinematics::JacobianEE  [0][0], &LDJ, &BETA, 
	 &Ac [0][0], &LDAc);

  // (J*A^-1*J')^-1
  // Factorize the symmetric matrix
  dpotrf_(&UPLO, &NEQS, &Ac[0][0], &LDAc, &INFO);
  if(INFO<0)
    cerr << "WSinertia::dpotrf2: The " << INFO << " argument is illegal.\n";
  else if(0<INFO)
    cerr << "WSinertia::dpotrf2: The matrix is not positive definite.\n";
  // invert
  dpotri_(&UPLO, &NEQS, &Ac[0][0], &LDAc, &INFO);
  if(INFO<0)
    cerr << "WSinertia::dpotri2: The " << INFO << " argument is illegal.\n";
  else if(0<INFO)
    cerr << "WSinertia::dpotri2: The matrix is singular.\n";
}

/*
 * Operation Space dynamics with MKL libraries
 * Takes about 72us (47usec with Intel's compiler!). And that kind of suck. 
 * Because the control loop of the WAM takes about 930usec just to receive 
 * joint positions and send torques, which brings the total just above 1ms 
 * with this control scheme.
 * Therefore, the control loop must run at 500Hz instead of 1kHz.
 * So, either spend time figuring out a better/faster driver/library that will
 * speed up the CAN stuff. Or figure out how to shave a few extra usecs from 
 * this (but I think it is already fairly optimized)
 * I mean, it's not that bad, most of the control iterations are about 990usec,
 * however too many of them go over 1ms which then screws up RTAI and pump
 * even more CPU.
 */

void WSdynamics(double trq[Link::Ln+1], 
		Link links[Link::Ln+1], 
		double qd[Link::Ln+1], R6& F){

  /*
  char UPLO = 'L';
  char TRANST = 'T';
  int NJOINTS = Joint::Jn;
  int NEQS = 6;
  int INC = 1;

  double ALPHA =  1.0;
  double BETA  =  0.0;
  double GAMMA = -1.0;
  */

  for(int j=Joint::J1; j<=Joint::Jn; j++)
    trq[j] = 0.0;

  double ccg[Joint::Jn+1];           // coriolis/centrifugal + gravity
  CCG(ccg, links, qd);

  // This is little hack to avoid singularities in Ac when using the WAM
  // in interactive mode (no forces). Otherwise the WAM will jerk when q4=0
  // note: the hack is the "if" statement
  // apply endpoint force
  if(0.0 < F.norm()){
    /*
    double Ac[6][6];                 // Cartesian inertia matrix (lower half)
    int LDAc = 6;
    double J[Joint::Jn][6];          // Column major Jacobian
    int LDJ = 6;

    WSinertia(Ac, links);
    JacobianNF(J, links);

    // Ac*F
    R6 AcF;
    dsymv_(&UPLO, &NEQS, &ALPHA,      // symmetric matrix * vector
	   &Ac[0][0],          &LDAc,
	   (double*)F,   &INC, &BETA,
	   (double*)AcF,       &INC);

    
    // J'*Ac*F                       // matrix * vector
    dgemv_(&TRANST, &NEQS,  &NJOINTS, &ALPHA,
	   &J[0][0],        &LDJ,
	   (double*)AcF,    &INC, &BETA,
	   &trq[Joint::J1], &INC);
    */


    //    OWD::Kinematics::JacobianDB(J, links);
    
    //J'*F
    //    dgemv_(&TRANST, &NEQS,  &NJOINTS, &ALPHA,
    //	   &J[0][0],        &LDJ,
    //	   (double*)F,    &INC, &BETA,
    //	   &trq[Joint::J1], &INC);


    /*

    // Ac*h
    R6 h = bias_acceleration(links, qd);
    R6 Ach;                          // symmetric matrix * vector
    dsymv_(&UPLO, &NEQS, &ALPHA,
	   &Ac[0][0],    &LDAc,
	   (double*)h,   &INC, &BETA,
	   (double*)Ach, &INC);
      
    // ccg - J'*Ac*h
    dgemv_(&TRANST, &NEQS,  &NJOINTS, &GAMMA,
	   &J[0][0],        &LDJ,
	   (double*)Ach,    &INC, &ALPHA,
	   &ccg[Joint::J1], &INC);
    */
  }

  for(int j=Joint::J1; j<=Joint::Jn; j++)    trq[j] += ccg[j];  
}

// calculated values for J1 were s=2.0, v=3.3, but experimental data on 4/5 yielded
//  the values below
double fs[8] = {0.0,2.7,2.5,1.5,1.10,0.4,0.4,0.1};
double fv[8] = {0.0,1.5,0.9,0.5,0.25,0.0,0.1,0.0};

void Friction(double qd[Link::Ln+1], double F[Link::Ln+1]) {
    for (int j = Joint::J1; j <= Joint::Jn; ++j) {
        if (qd[j] >0.0) {
            F[j]=fs[j]+qd[j]*fv[j];
        } else if (qd[j] < 0.0) {
            F[j]=-fs[j]+qd[j]*fv[j];
        } else {
            F[j]=0.0;
        }
    }
}
            

std::vector<double> JSdynamics(Link   links[Link::Ln+1], 
			       double qd[Link::Ln+1], 
			       double qdd[Link::Ln+1]){

  double ssq = 0.0;
  std::vector<double> trq(Joint::Jn,0);
  for(int j=Joint::J1; j<=Joint::Jn; j++){
    ssq += qdd[j]*qdd[j];
  }

  double ccg[Joint::Jn+1];           // coriolis/centrifugal + gravity
  CCG(ccg, links, qd);

  if(0.0 < ssq){
    char UPLO = 'L';
    int N = Joint::Jn;
    int LDA = Joint::Jn;
    int INC = 1;
    double ALPHA = 1.0;
    double BETA = 0.0;

    double A[Joint::Jn][Joint::Jn];       // Joint space inertia matrix
    JSinertia(A, links);

    dsymv_(&UPLO,           &N,  &ALPHA,   // symmetric matrix * vector
	  &A[0][0],        &LDA,
	  &qdd[Joint::J1], &INC, &BETA,
	  &trq[0], &INC);
  }
  double F[Link::Ln+1];  // array to hold friction torques
  Friction(qd,F);        // calculate friction torques based on velocities
  for(int j=Joint::J1; j<=Joint::Jn; j++) {
    trq[j-1] += ccg[j] + F[j];   // add in the ccg and friction torques
  }
  return trq;
}
