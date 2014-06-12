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
#include "Puck.hh"
#include "globals.h"

#ifndef __GROUP_H__
#define __GROUP_H__

#define GROUP_ID_MIN     1
#define GROUP_ID_MAX     7
#define NUM_GROUPS       (GROUP_ID_MAX - GROUP_ID_MIN + 1)

#define GROUP_INVALID   -1

using namespace std;

class Group{
 private:
   int ID;                           // The ID number for this group.
   Puck* pucks[NUM_ORDERS+1];        // An array of puck pointers

 public:
   Group();

   int id(){return ID;}
   Puck* puck(int p){return pucks[p];}

   int insert(Puck* p);

   friend ostream& operator << (ostream& s, Group& g);
};

#endif
