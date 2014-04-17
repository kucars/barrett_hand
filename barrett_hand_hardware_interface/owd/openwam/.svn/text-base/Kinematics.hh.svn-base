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

#include "Link.hh"
#include "../openmath/R6.hh"
#include <sstream>

#ifndef __KINEMATICS_HH__
#define __KINEMATICS_HH__

class WAM;

namespace OWD {

  class Kinematics {

    friend class ::WAM;
    
  public:

    static const int NJOINTS=Link::Ln;
    static const int NDIMS=6;

    /// The column-major Jacobian matrix in the EE frame (NJOINTS x 6)
    static double JacobianEE[][NDIMS];

    /// The column-major Jacobian matrix in the %Link 0 frame (NJOINTS x 6)
    static double Jacobian0[][NDIMS];

    /// The Pseudo-Inverse of the Jacobian in the %Link 0 frame (6 x NJOINTS)
    static double Jacobian0PseudoInverse[][NJOINTS];

    /// The matrix for projecting a jointspace move into the nullspace
    static double Nullspace_matrix[][NJOINTS];

    /// \brief Multiply the base-frame Jacobian by the supplied vector
    ///
    /// \param q an array of NJOINTS doubles for the joint values
    /// \param {out} out A pointer to a previously-allocated array of
    ///              NDIMS doubles to hold the result
    ///
    /// This function is typically used to compute the workspace velocities
    /// resulting from the supplied joint velocities
    static void Jacobian0_times_vector(double *q, double *out);

    /// \brief Multiply the base-frame Jacobian Transpose by the
    /// supplied vector
    ///
    /// \param v an R6 vector
    /// \param {out} out A pointer to a previously-allocated array of
    ///              NJOINTS doubles to hold the result
    ///
    /// This function is typically used to compute the joint torques
    /// that will yield the requested workspace forces/torques
    static void Jacobian0Transpose_times_vector(R6 &v, double *out);

    /// \brief Multiply the EE-frame Jacobian Transpose by the
    /// supplied vector
    ///
    /// \param v an R6 vector
    /// \param {out} out A pointer to a previously-allocated array of
    ///              NJOINTS doubles to hold the result
    ///
    /// This function is typically used to compute the joint torques
    /// that will yield the requested forces/torques in EE frame
    static void JacobianEETranspose_times_vector(R6 &v, double *out);

    /// \brief Multiply the Pseudo-Inverse of the base-frame Jacobian
    /// by the supplied vector.
    ///
    /// \attention If the WAM is in a singular configuration, this function
    /// will throw a const char *, so it should only be called from
    /// within a try/catch block.
    static void JacobianPseudoInverse_times_vector(R6 &v, double *out);

    /// \brief Project a jointspace movement into the nullspace
    ///
    /// The resultant jointspace movement will not affect the 
    /// endpoint pose.
    static void Nullspace_projection(double *v, double *result);

    /// \brief True if the Jacobian is up-to-date
    static bool valid_Jacobian;

    /// \brief True if the Pseudo-Inverse can be used
    static bool valid_JacobianPseudoInverse;

    /// \brief True if the Nullspace Projection Matrix can be used
    static bool valid_Nullspace_matrix;

    // for debugging
    static int thresholded_count;
    static int zero_count;
    static double thresholded_values[NJOINTS];
    static double max_condition;

    /// \brief Calculate the endpoint pose
    ///
    /// Runs through the current links, multiplying their
    /// transformation matrices by one another.
    ///
    /// \param links A pointer to the first element in the array of links
    /// \returns The transformation matrix of link7 in terms of link0
    static SE3 forward_kinematics(Link* links);

  private:
    /// \brief Pre-calculate the Jacobian and friends
    ///
    /// Calculates the Jacobian and derived matrices based on the
    /// current link values, The resulting column-major matrices are
    /// stored so that multiple functions can ask for them without
    /// having to repeat the calculations each time.
    ///
    /// \param links A pointer to the first element in the array of links
    ///
    /// \attention This function should be called from WAM.cc every time
    ///            we receive updated joint values from the arm.
    static void update_jacobians(Link *links);

    /// \brief body Jacobian for Fortran (column major)
    static void Jacobians(double JEE[][NDIMS], double J0[][NDIMS], Link *links);

    static R3 Elbow_Velocity(double q[], double qd[]);

    static R3 Endpoint_Velocity(double q[], double qd[]);

    /// \brief Jacobian Pseudo-Inverse from the Jacobian
    static void PseudoInverse(double JPI[][NJOINTS], double J0[][NDIMS]);

    /// \brief Nullspace projection matrix from Jacobian Pseudo-Inverse
    ///        and Jacobian
    static void Nullspace_Matrix(double Nullspace_matrix[][NJOINTS],
				 double Jacobian0PseudoInverse[][NJOINTS],
				 double Jacobian0[][NDIMS]);

    /// \brief 3DOF body Jacobian (column major)
    static void JacobianDB(double J[][NDIMS], Link *links);

    /// \brief body Jacobian for C (row major) 
    static void JacobianN(double J[][NJOINTS], Link *links);

    /// \brief Set up the internal links that are used just for the
    ///        elbow and endpoint velocities
    static void InitializeVelocityLinks();

    static char TRANST;
    static char TRANSN;
    static double ALPHA;
    static double BETA;
    static char UPLO;
    static int LD_Jacobian;
    static int LD_JacobianPseudoInverse;
    static int LD_Nullspace_matrix;
    static int INC;
    static OWD::Link vlinks[5]; // for elbow and endpoint velocities

    static std::stringstream last_error;
  };

}; // namespace OWD

#endif // __KINEMATICS_HH__
