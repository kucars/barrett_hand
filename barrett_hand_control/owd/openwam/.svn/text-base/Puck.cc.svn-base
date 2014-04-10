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

#include "Puck.hh"

/* This is constant, and sent to the pucks on startup by CANbus::check(). */
const int Puck::MAX_TRQ[7] = {4860,  4860,  4860,  4320,  3900,  3900,  3200};

/* This is calculated automatically in WAM::init();
 * it will probably be MAX_TRQ +/-1 for J1, J4, and J7,
 * and 2*MAX_TRQ +/- 1 for the differential joints,
 * but it also depends on IPNM. */
const int Puck::MAX_CLIPPABLE_TRQ_EXPECTED[7] = {4860,  9720,  9720,  4320,  7800,  7800,  3200};
int Puck::MAX_CLIPPABLE_TRQ[7] = {0, 0, 0, 0, 0, 0, 0};
