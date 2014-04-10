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
#include "globals.h"

#ifndef __PUCK_H__
#define __PUCK_H__

using namespace std;

#define PUCK_ORDER_MIN 1
#define PUCK_ORDER_MAX 4
#define NUM_ORDERS     (PUCK_ORDER_MAX - PUCK_ORDER_MIN + 1)

enum{STATUS_OFFLINE = -1,
     STATUS_RESET,
     STATUS_ERR,
     STATUS_READY
};

class Puck{
 public:
   int ID;
   int group_id;
   int group_order; 
   int motor_id;   
   int cpr;             // Encoder counts per revolution
   int j_cpr;           // Secondary ("joint") encoder CPR


  static const int MAX_TRQ[7];
  static const int MAX_CLIPPABLE_TRQ_EXPECTED[7];
  static int MAX_CLIPPABLE_TRQ[7];

  Puck(){ID=-1;}
  
  int id(){return ID;}
  int group(){return group_id;}
  int order(){return group_order;}
  int motor(){return motor_id;}
  int CPR(){return cpr;}
  int J_CPR(){return j_cpr;}
  
  friend ostream& operator << (ostream& s, Puck& p){
    s << "Puck (id#): "     << p.id()     << "; "
      << "Motor#: "         << p.motor()  << "; "
      << "Group (id#): "    << p.group()  << "; "
      << "Group (order#): " << p.order()  << "; "
      << "Counts/Rev: "     << p.cpr;
    return s;
  }
}; 

#endif
