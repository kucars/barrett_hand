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

#ifndef TRAJTYPE_H
#define TRAJTYPE_H

#include <vector>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <execinfo.h>

namespace OWD {

  const char *owd_backtrace(const char *msg);

// a vector of joint angles defines a single Wam position
class JointPos : public std::vector<double> {
public:
  JointPos() {}
  explicit JointPos(int s) : std::vector<double>(s) {}
  JointPos(const std::vector<double> &vd) : std::vector<double>(vd) {}
  double length() const;
  void cpy(double *out) const;
  void dump() const;
  const char* sdump() const;
  void SetFromArray(const unsigned int, const double *);

  inline bool operator!=(const JointPos &rhs) const {
    for (unsigned int i = 0; i < this->size(); ++i) {
      if (fabs(this->operator[](i) - rhs[i]) > 0.01f ) {
	return true;
      }
    }
    return false;
  }

  inline bool closeto(const JointPos &rhs) const {
    for (unsigned int i = 0; i < this->size(); ++i) {
      if ((i<3) && (fabs(this->operator[](i) - rhs[i]) > 0.1f)) {
	return false;  // J1-J3
      } else if ((i==3) && (fabs(this->operator[](i) - rhs[i]) > 0.2f)) {
        return false; // J4
      } else if ((i>3) && (fabs(this->operator[](i) - rhs[i]) > 0.4f )) {
        return false; // J5-7
      }
    }
    return true;
  }

  inline bool verycloseto(const JointPos &rhs) const {
    for (unsigned int i = 0; i < this->size(); ++i) {
      if (fabs(this->operator[](i) - rhs[i]) > 0.005f) {
	return false;
      }
    }
    return true;
  }

  inline bool operator==(const JointPos &rhs) const {
    return ! operator!=(rhs);
  }

  

  // add two vectors
  inline JointPos operator+(const JointPos &rhs) const {
    if (size() != rhs.size()) {
      throw owd_backtrace("Error: mismatched sizes when calling JointPos::operator+");
    }
    JointPos jp(*this);
    for (unsigned int i=0; i<jp.size(); ++i) {
      jp[i]+=rhs[i];
    }
    return jp;
  }

  // subtract two vectors
  inline JointPos operator-(const JointPos &rhs) const {
    if (size() != rhs.size()) {
      throw owd_backtrace("Error: mismatched sizes when calling JointPos::operator-");
    }
    JointPos jp(*this);
    for (unsigned int i=0; i<jp.size(); ++i) {
      jp[i]-=rhs[i];
    }
    return jp;
  }

  // add two vectors in place
  inline JointPos &operator+=(const JointPos &rhs) {
    if (size() != rhs.size()) {
      throw owd_backtrace("Error: mismatched sizes when calling JointPos::operator+=");
    }
    for (unsigned int i=0; i<size(); ++i) {
      this->operator[](i) += rhs[i];
    }
    return *this;
  }

  // subtract two vectors in place
  inline JointPos &operator-=(const JointPos &rhs) {
    if (size() != rhs.size()) {
      throw owd_backtrace("Error: mismatched sizes when calling JointPos::operator-=");
    }
    for (unsigned int i=0; i<size(); ++i) {
      this->operator[](i) -= rhs[i];
    }
    return *this;
  }

  // multiply by a scalar on the right
  inline JointPos operator*(const double &rhs) const {
    JointPos jp(*this);
    for (unsigned int i=0; i<jp.size(); ++i) {
      jp[i] *= rhs;
    }
    return jp;
  }

  // divide by a scalar
  inline JointPos operator/(const double &rhs) const {
    JointPos jp(*this);
    for (unsigned int i=0; i<jp.size(); ++i) {
      jp[i] /= rhs;
    }
    return jp;
  }

  // multiply by a scalar in place
  inline JointPos &operator*=(const double &rhs) {
    for (unsigned int i=0; i<size(); ++i) {
      this->operator[](i) *= rhs;
    }
    return *this;
  }

  // divide by a scalar in place
  inline JointPos &operator/=(const double &rhs) {
    for (unsigned int i=0; i<size(); ++i) {
      this->operator[](i) /= rhs;
    }
    return *this;
  }

  // dot product
  inline double operator*(const JointPos &rhs) {
    if (size() != rhs.size()) {
      throw owd_backtrace("Error: mismatched sizes when calling JointPos::operator*");
    }
    double result = 0.0;
    for (unsigned int i=0; i<size(); ++i) {
      result += this->operator[](i) * rhs[i];
    }
    return result;
  }
  
};  

// a JointPos along with timing info defines a single trajectory point
class TrajPoint : public JointPos {
 public:
  TrajPoint():blend_radius(0.0) {}
  TrajPoint(int size) : JointPos(size), blend_radius(0.0) {}
  TrajPoint(const JointPos &jp) : JointPos(jp) , blend_radius(0.0) {}
  TrajPoint(const JointPos &jp, const double br) : JointPos(jp) , blend_radius(br) {}
  ~TrajPoint() {}
  bool operator!=(const TrajPoint &rhs);
  TrajPoint& operator=(const TrajPoint &rhs);
  
  double absolute_time;
  double blend_radius;
  void fprint(FILE *fout) const;
  
};

// a trajectory is a vector of trajectory points
typedef std::vector<TrajPoint> TrajType;

  // multiply by a scalar on the left
inline JointPos operator*(const double &lhs, const JointPos &rhs) {
  JointPos jp(rhs);
  jp *= lhs;
  return jp;
}

}; // namespace OWD

#endif //TRAJTYPE_H
