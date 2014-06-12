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

#include "Kinematics.hh"
#include <stdlib.h>
#include "Plugin.hh" // for debugging the Jacobian
#include <stdio.h>
// #define ADD_
//#include <cblas_f77.h>

extern "C" {
  void dgemm_(char *transa, char *transb, int *m, int *n, int *k,double *alpha,
	      double *a, int *lda, double *b, int *ldb, double *beta,double *c,
	      int *ldc);
  void dgemv_(char *trans, int *m, int *n, double *alpha, double *a, int *lda,
	      double *x, int *incx, double *beta, double *y, int *incy);
  void dgesvd_(char *jobu, char *jobvt, int *m, 
	      int *n, double *a, int *lda, 
	      double *s, double *u, int *ldu, double *vt,
	      int *ldvt, double *work, int *lwork, int *info);
}

namespace OWD {

  // allocate and initialize the static members
  //
  // note: use of the TRANST flag basically says to blas that the
  // input matrix has been transposed, and it should be "untransposed"
  // before the operation.  So the number of rows/columns reflect the
  // native size of the matrix before it was transposed, or
  // after it is untransposed.
  char Kinematics::TRANST = 'T';  
  char Kinematics::TRANSN = 'N';
  double Kinematics::ALPHA = 1.0; 
  double Kinematics::BETA = 0.0;
  char Kinematics::UPLO = 'L';
  int Kinematics::LD_Jacobian=NDIMS;
  int Kinematics::LD_JacobianPseudoInverse=NJOINTS;
  int Kinematics::LD_Nullspace_matrix=NJOINTS;
  int Kinematics::INC=1;
  bool Kinematics::valid_Jacobian(false);
  bool Kinematics::valid_JacobianPseudoInverse(false);
  bool Kinematics::valid_Nullspace_matrix(false);
  double Kinematics::JacobianEE[NJOINTS][NDIMS];
  double Kinematics::Jacobian0[NJOINTS][NDIMS];
  double Kinematics::Jacobian0PseudoInverse[NDIMS][NJOINTS];
  double Kinematics::Nullspace_matrix[NJOINTS][NJOINTS];
  std::stringstream Kinematics::last_error;

  void Kinematics::Jacobian0_times_vector(double *q, double *out) {
    if (!valid_Jacobian) {
      throw "Current Jacobian matrix not available";
    }
    // multiply by the supplied vector
    int M=NDIMS;
    int N=NJOINTS;
    dgemv_(&TRANSN, &M,  &N, &ALPHA,
	      &Jacobian0[0][0], &LD_Jacobian,
	      (double*)q, &INC,
	      &BETA, out, &INC);
  }

  void Kinematics::JacobianPseudoInverse_times_vector(R6 &v, double *out) {
    if (!valid_Jacobian) {
      throw "Current Jacobian matrix not available";
    }
    if (!valid_JacobianPseudoInverse) {
      // must be at a singularity, so cannot use the Pseudo Inverse.
      // calling function can catch this as a const char *.
      std::string error = "No valid Jacobian Pseudo Inverse available: ";
      error += last_error.str();
      throw (const char *) error.c_str();
    }
    // multiply by the supplied vector
    int M=NJOINTS;
    int N=NDIMS;
    double B[6];
    B[0]=v.v[0]; B[1]=v.v[1]; B[2]=v.v[2]; B[3]=v.w[0]; B[4]=v.w[1]; B[5]=v.w[2];
    dgemv_(&TRANSN, &M,  &N, &ALPHA,
	      &Jacobian0PseudoInverse[0][0], &LD_JacobianPseudoInverse,
	      B,  &INC, &BETA,
	      out, &INC);
  }

  void Kinematics::Jacobian0Transpose_times_vector(R6 &v, double *out) {
    if (!valid_Jacobian) {
      throw "Current Jacobian matrix not available";
    }
    int M=NDIMS;
    int N=NJOINTS;
    double B[6];
    B[0]=v.v[0]; B[1]=v.v[1]; B[2]=v.v[2]; B[3]=v.w[0]; B[4]=v.w[1]; B[5]=v.w[2];
    dgemv_(&TRANST, &M,  &N, &ALPHA,
	      &Jacobian0 [0][0], &LD_Jacobian,
	      B, &INC,
	      &BETA, out, &INC);
  }
 
  // Computes the product of Jacobian (in EE frame) transpose with the
  // input vector
  void Kinematics::JacobianEETranspose_times_vector(R6 &v, double *out) {
    if (!valid_Jacobian) {
      throw "Current Jacobian matrix not available";
    }
    int M=NDIMS;
    int N=NJOINTS;
    double B[6];
    B[0]=v.v[0]; B[1]=v.v[1]; B[2]=v.v[2]; B[3]=v.w[0]; B[4]=v.w[1]; B[5]=v.w[2];
    dgemv_(&TRANST, &M,  &N, &ALPHA,
	      &JacobianEE [0][0], &LD_Jacobian,
	      B, &INC,
	      &BETA, out, &INC);
  }
    
  SE3 Kinematics::forward_kinematics(Link* link){
    SE3 H = (SE3)link[Link::L0];

    for(int l=Link::L1; l<=Link::Ln; l++)
      H = H * (SE3)link[l];

    return H;
  }

  void Kinematics::update_jacobians(Link *links) {
    // all the matrices are in column-major order, for Fortran

    // as soon as we start changing the Jacobians, the Pseudo-Inverse
    // be out of date, so go ahead and mark it now
    valid_JacobianPseudoInverse=false;
    valid_Nullspace_matrix=false;

    // calculate the Jacobian in the End-Effector frame and
    // the Link 0 frame
    Jacobians(JacobianEE, Jacobian0, links);
    valid_Jacobian=true;

    PseudoInverse(Jacobian0PseudoInverse,Jacobian0);

    Nullspace_Matrix(Nullspace_matrix,Jacobian0PseudoInverse,Jacobian0);
  }

  void Kinematics::PseudoInverse(double JPI[][NJOINTS],
				 double J0[][NDIMS]) {

    static double A[NJOINTS][NDIMS];
    static double S[NDIMS];
    static int LD_U=NDIMS;
    static int LD_VT=NJOINTS;
    static int LD_DUT=NJOINTS;
    static int L_WORK=5*NDIMS;
    static double U[NDIMS][NDIMS];
    static double VT[NJOINTS][NJOINTS];
    static double DUT[NDIMS][NJOINTS];
    static double WORK[5*NDIMS];
    int INFO;
    static const double maxConditionNumber = 24;

    last_error.str() = std::string("");

    // copy over the Jacobian (since the A matrix will be changed)
    memcpy(A,Jacobian0,NJOINTS*NDIMS*sizeof(double));

    // find SVD of Jacobian
    char JOBU='A';
    char JOBVT='A';
    int M=NDIMS;
    int N=NJOINTS;
    dgesvd_(&JOBU, &JOBVT, &M, &N,
	   &A[0][0], &LD_Jacobian,
	   &S[0],
	   &U[0][0], &LD_U,
	   &VT[0][0], &LD_VT,
	   &WORK[0], &L_WORK,
	   &INFO);
    if (INFO<0) {
      last_error.str() = std::string("Bad argument number ");
      last_error << -INFO;
      last_error << " to dgesvd call";
    } else if (INFO > 0) {
      last_error.str() = std::string("dgesvd:  ");
      last_error << INFO;
      last_error << " superdiagonals did not converge to zero";
    }
    // Calculate the pseudo-inverse of D by inverting the S values,
    // thresholding them as we go.  We'll calculate them in-place,
    // reusing the same storage.
    thresholded_count=0;
    zero_count=0;
    double max_eigen = S[0];
    for (int i=0; i<NDIMS; ++i) {
      if (S[i] > 0) {
	if ((i>0) && (max_eigen / S[i] > maxConditionNumber)) {
	  S[i] = maxConditionNumber / max_eigen; // threshold and invert
	} else {
	  S[i] = 1.0/S[i]; // just invert
	}
	max_condition=max_eigen * S[i];
      } else {
	zero_count++;
      }
    }
	
    // Calculate D * U**T
    // top rows come from U
    for (int r=0; r<NDIMS; ++r) {
      for (int c=0; c<NDIMS; ++c) {
	DUT[c][r] = U[r][c] * S[r];
      }
    }
    // remaining rows are all zero
    for (int r=NDIMS; r<NJOINTS; ++r) {
      for (int c=0; c<NDIMS; ++c) {
	DUT[c][r] = 0;
      }
    }

    // finally, we calculate  V * DUT
    {
      int M=NJOINTS;
      int N=NDIMS;
      int K=NJOINTS;
      dgemm_(&TRANST, &TRANSN, &M, &N, &K,
		&ALPHA, &VT[0][0], &LD_VT,
		&DUT[0][0], &LD_DUT, &BETA,
		&Jacobian0PseudoInverse[0][0], &LD_JacobianPseudoInverse);
    }
    
    valid_JacobianPseudoInverse=true;
    return;
  }

  void Kinematics::Nullspace_projection(double *v, double *result) {
    if (!valid_Nullspace_matrix) {
      throw "No valid nullspace matrix available";
    }
    int M=NJOINTS;
    int N=NJOINTS;
    dgemv_(&TRANSN, &M, &N, &ALPHA,
	   &Nullspace_matrix[0][0], &LD_Nullspace_matrix,
	   v, &INC, &BETA,
	   result, &INC);
  }

  /*
   * Body manipulator Jacobian
   * Paul IEEE SMC 11(6) 1981
   * 
   * BIG FAT WARNING: The jacobians are in column major (for Fortran)
   */
  void Kinematics::Jacobians(double JN[][NDIMS], double J0[][NDIMS], Link *links){
    SE3 U;
    double local_JN[NJOINTS][NDIMS];
    if (!JN) {
      JN=local_JN;  // use our temp storage so that we can successfully
                    // compute J0 even if JN isn't defined
    }

    for(int l=Link::Ln; l>=Link::L1; --l){
      U = ((SE3)links[l]) * U;

      JN[l-1][0] = U[SE3::TX]*U[SE3::NY] - U[SE3::TY]*U[SE3::NX];
      JN[l-1][1] = U[SE3::TX]*U[SE3::OY] - U[SE3::TY]*U[SE3::OX];
      JN[l-1][2] = U[SE3::TX]*U[SE3::AY] - U[SE3::TY]*U[SE3::AX];
      JN[l-1][3] = U[SE3::NZ];
      JN[l-1][4] = U[SE3::OZ];
      JN[l-1][5] = U[SE3::AZ];
    }

    // Rotate the end-effector Jacobian to the base frame
    if (J0) {
      for (int i=0; i<Kinematics::NJOINTS; ++i) {
	// upper three rows, translation
	J0[i][0] = JN[i][0]*U[SE3::R11] + JN[i][1]*U[SE3::R12] + JN[i][2]*U[SE3::R13];
	J0[i][1] = JN[i][0]*U[SE3::R21] + JN[i][1]*U[SE3::R22] + JN[i][2]*U[SE3::R23];
	J0[i][2] = JN[i][0]*U[SE3::R31] + JN[i][1]*U[SE3::R32] + JN[i][2]*U[SE3::R33];

	// bottom three rows, rotation
	J0[i][3] = JN[i][3]*U[SE3::R11] + JN[i][4]*U[SE3::R12] + JN[i][5]*U[SE3::R13];
	J0[i][4] = JN[i][3]*U[SE3::R21] + JN[i][4]*U[SE3::R22] + JN[i][5]*U[SE3::R23];
	J0[i][5] = JN[i][3]*U[SE3::R31] + JN[i][4]*U[SE3::R32] + JN[i][5]*U[SE3::R33];
      }
    }
  }

  R3 Kinematics::Elbow_Velocity(double q[], double qd[]) {
    // calculate the Jacobian for the Elbow in this config
    static double Jacobian[3][3];
    SE3 U;
    for (int l=Link::L3; l>=Link::L1; --l) {
      vlinks[l].theta(q[l-1]);  // set the joint angle
      U = ((SE3)vlinks[l])*U;   // update the transform
      Jacobian[l-1][0] = U[SE3::TX]*U[SE3::NY] - U[SE3::TY]*U[SE3::NX];
      Jacobian[l-1][1] = U[SE3::TX]*U[SE3::OY] - U[SE3::TY]*U[SE3::OX];
      Jacobian[l-1][2] = U[SE3::TX]*U[SE3::AY] - U[SE3::TY]*U[SE3::AX];
    }
    // multiply the jacobian by the joint velocities
    R3 xyz_vel;
    int M(3), N(3), LD_J(3);
    dgemv_(&TRANSN, &M, &N, &ALPHA,
	   &Jacobian[0][0], &LD_J,
	   qd, &INC,
	   &BETA, xyz_vel.x, &INC);
    return xyz_vel;
  }

  R3 Kinematics::Endpoint_Velocity(double q[], double qd[]) {
    // calculate the Jacobian for the Endpoint (350mm past elbow)
    static double Jacobian[4][3];
    SE3 U;
    for (int l=Link::L4; l>=Link::L1; --l) {
      vlinks[l].theta(q[l-1]);  // set the joint angle
      U = ((SE3)vlinks[l])*U;   // update the transform
      Jacobian[l-1][0] = U[SE3::TX]*U[SE3::NY] - U[SE3::TY]*U[SE3::NX];
      Jacobian[l-1][1] = U[SE3::TX]*U[SE3::OY] - U[SE3::TY]*U[SE3::OX];
      Jacobian[l-1][2] = U[SE3::TX]*U[SE3::AY] - U[SE3::TY]*U[SE3::AX];
    }
    // multiply the jacobian by the joint velocities
    R3 xyz_vel;
    int M(3), N(4), LD_J(3);
    dgemv_(&TRANSN, &M, &N, &ALPHA,
	   &Jacobian[0][0], &LD_J,
	   qd, &INC,
	   &BETA, xyz_vel.x, &INC);
    return xyz_vel;
  }

  void Kinematics::Nullspace_Matrix(double Nullspace_matrix[][NJOINTS],
				    double JPI[][NJOINTS],
				    double J[][NDIMS]) {
    if (!valid_Jacobian) {
      throw "No valid Jacobian available";
    } else if (!valid_JacobianPseudoInverse) {
      throw "No valid Jacobian Pseudo-Inverse available";
    }
    int M=NJOINTS;
    int N=NJOINTS;
    int K=NDIMS;
    dgemm_(&TRANSN, &TRANSN, &M, &N, &K,
	   &ALPHA, &JPI[0][0], &LD_JacobianPseudoInverse,
	   &J[0][0], &LD_Jacobian, &BETA,
	   &Nullspace_matrix[0][0], &LD_Nullspace_matrix);
    // subtract the result from the identity matrix
    for (int r=0; r<NJOINTS; ++r) {
      for (int c=0; c<NJOINTS; ++c) {
	if (r==c) {
	  // diagonal term
	  Nullspace_matrix[c][r] = 1-Nullspace_matrix[c][r];
	} else {
	  // off-diagonal term
	  Nullspace_matrix[c][r] = 0-Nullspace_matrix[c][r];
	}
      }
    }
    valid_Nullspace_matrix = true;
  }

  void Kinematics::JacobianDB(double J[][NDIMS], Link *links){
    SE3 U,EE;
    R3 v,axis,anchor,tEE;

    /*
      for(int l=Link::L1; l <= Link::Ln; l++)
      {        
      EE = (((SE3)links[l])^-1) * EE;

      }*/

    EE = forward_kinematics(links);
    tEE = R3(EE[3],EE[7],EE[11]);

    U = (SE3)links[Link::L0];
    for(int l=Link::L1; l <= Link::Ln; l++)
      {

        axis = R3(U[2],U[6],U[10]);
        anchor = R3(U[3],U[7],U[11]);
        
        v = axis^(anchor - tEE);

        J[l-1][0] = v[0];
        J[l-1][1] = v[1];
        J[l-1][2] = v[2];
        J[l-1][3] = 0;
        J[l-1][4] = 0;
        J[l-1][5] = 0;

        U = U * (SE3)links[l];

      }
  }



  // row-major version of the EE Jacobian

  void Kinematics::JacobianN(double J[][NJOINTS], Link *links){
    SE3 U;

    for(int l=Link::Ln; Link::L1<=l; l--){
      U = ((SE3)links[l]) * U;

      J[0][l-1] = U[SE3::TX]*U[SE3::NY] - U[SE3::TY]*U[SE3::NX];
      J[1][l-1] = U[SE3::TX]*U[SE3::OY] - U[SE3::TY]*U[SE3::OX];
      J[2][l-1] = U[SE3::TX]*U[SE3::AY] - U[SE3::TY]*U[SE3::AX];
      J[3][l-1] = U[SE3::NZ];
      J[4][l-1] = U[SE3::OZ];
      J[5][l-1] = U[SE3::AZ];
    }
  }

  void Kinematics::InitializeVelocityLinks() {
    vlinks[Link::L0]=Link( DH(  0.0000,   0.0000, 0.0000,   0.0000), 0, 
			   R3(  0.0000,   0.0000, 0.0000), 
			   Inertia(0,0,0,0,0,0));

    vlinks[Link::L1]=Link( DH( -M_PI_2,   0.0000, 0.0000,   0.0000), 0,
			   R3(  0.0000,   0.0000, 0.0000), 
			   Inertia(0,0,0,0,0,0));
    
    vlinks[Link::L2]=Link( DH(  M_PI_2,   0.0000, 0.0000,   0.0000), 0,
			   R3(  0.0000,   0.0000, 0.0000), 
			   Inertia(0,0,0,0,0,0));

    vlinks[Link::L3]=Link( DH( -M_PI_2,   0.0450, 0.5500,   0.0000), 0,
			   R3(  0.0000,   0.0000, 0.0000), 
			   Inertia(0,0,0,0,0,0));

    vlinks[Link::L4]=Link( DH(  M_PI_2,   0.3529, 0.0000,  -M_PI_2-0.1279), 0,
			   R3(  0.0000,   0.0000, 0.0000), 
			   Inertia(0,0,0,0,0,0));  // origin 350mm beyond elbow
  }

  int Kinematics::thresholded_count=0;
  int Kinematics::zero_count=0;
  double Kinematics::max_condition=0;
  double Kinematics::thresholded_values[NJOINTS];

  OWD::Link Kinematics::vlinks[5];

}; // namespace OWD
