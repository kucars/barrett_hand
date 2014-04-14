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

#include "Group.hh"

ostream& operator << (ostream& s, Group& g){
   Puck* puck;
   int p;
   
   cout << "GROUP (id#): " << g.id() << endl;
   for(p=PUCK_ORDER_MIN; p<=PUCK_ORDER_MAX; p++){
      puck = g.puck(p);
      if(puck) cout<< *puck << endl;
      else cout << "No puck (order# " << p << ")." << endl;
   }   
   return s;
}

Group::Group(){
   ID = GROUP_INVALID;
   for(int p=0; p<=PUCK_ORDER_MAX; p++) pucks[p] = NULL;
}

int Group::insert(Puck* p){
   int order, group;

   order = p->order();
   group = p->group();
   if(order<PUCK_ORDER_MIN || PUCK_ORDER_MAX<order){
      cerr<<"Group::insert: invalid group order."<<endl;
      return OW_FAILURE;
   }
   if(group<GROUP_ID_MIN || GROUP_ID_MAX<group){ 
      cerr << "Group::insert: invalid group number."<<endl;
      return OW_FAILURE;
   }
   
   if(pucks[order] != NULL){
      cerr << "Group::insert: pucks " << pucks[order]->id()
	   << " and " << p->id() << " have the same group and order."<<endl;
      return OW_FAILURE;
   }

   if(ID == GROUP_INVALID)  // inserting for the first time?
     ID = group;              // initialize the ID of the group
   if(ID != group){         // is there a mismatch?
      cerr << "Group::insert: group mismatch " << ID << " " << group << endl;
      return OW_FAILURE;
   }
   
   pucks[order] = p;

   return OW_SUCCESS;
}

