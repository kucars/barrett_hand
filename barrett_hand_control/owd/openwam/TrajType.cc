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

#include "TrajType.hh"
#include <string.h>

namespace OWD {

  const char *owd_backtrace(const char *msg) {
    static char errmsg[2000];
    void **buf = (void **)malloc(20*sizeof(void*));
    int tracecount = backtrace(buf,20);
    char **bt_syms = backtrace_symbols(buf, tracecount);
    strcpy(errmsg,msg);
    strcat(errmsg,"\nbacktrace:\n");
    for (int i=0; i<tracecount; ++i) {
      snprintf(errmsg+strlen(errmsg),1999-strlen(errmsg)," %d: %s\n",
	       i, bt_syms[i]);
    }
    free(bt_syms); free(buf);
    return (const char *) errmsg;
  }

TrajPoint& TrajPoint::operator=(const TrajPoint &rhs) {
    *(JointPos*)this = *(const JointPos*)&rhs;
    absolute_time = rhs.absolute_time;
    blend_radius = rhs.blend_radius;
    return *this;
}

bool TrajPoint::operator!=(const TrajPoint &rhs) {
  if (fabs(blend_radius - rhs.blend_radius) > 0.01f) {
    return true;
  }
  for (unsigned int i = 0; i < this->size(); ++i) {
    if (fabs(this->operator[](i) - rhs[i]) > 0.01f ) {
      return true;
    }
  }
  return false;
}

void TrajPoint::fprint(FILE *fout) const {
    fprintf(fout,"%2.8f",absolute_time);
    for (unsigned int i = 0; i < this->size(); ++i) {
        fprintf(fout,", % 2.8f",this->operator[](i));
    }
    fprintf(fout,"\n");
}

void JointPos::cpy(double *out) const {
  for (unsigned int i=0; i<size(); ++i) {
    out[i]=this->operator[](i);
  }
}

double JointPos::length() const {
  double sum=0;
  for (unsigned int i=0; i<size(); ++i) {
    sum += pow(this->operator[](i),2);
  }
  return sqrt(sum);
}

void JointPos::dump() const {
  printf("[ ");
  for (unsigned int i=0; i<size(); ++i) {
    printf("%2.3f ",this->operator[](i));
  }
  printf("]\n");
}

const char *JointPos::sdump() const {
  static char outstring[400];
  snprintf(outstring,400,"[ ");
  for (unsigned int i=0; i<size(); ++i) {
    int length=strlen(outstring);
    snprintf(outstring+length,400-length,"%2.3f ",this->operator[](i));
  }
  if (strlen(outstring) < 399) {
    strcat(outstring,"]");
  }
  return outstring;
}

void JointPos::SetFromArray(const unsigned int arraysize, const double *p) {
  this->resize(arraysize);
  for (unsigned int i=0; i<arraysize; ++i) {
    this->operator[](i) = p[i];
  }
}

}; // namespace OWD
