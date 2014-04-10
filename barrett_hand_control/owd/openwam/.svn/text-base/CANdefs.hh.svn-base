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

#ifndef __CANDEFS_HH__
#define __CANDEFS_HH__

#define mbxID               (0)
#define BASE_ID             (0)

#define MAX_BUS             (4)
#define SAFETY_MODULE       (10)

#define isAlpha(c) ( ((c >= 'A') && (c <= 'Z')) ? 1 : 0 )
#define isSpace(c) ( (c == ' ') ? 1 : 0 )
#define isDigit(c) ( ((c >= '0') && (c <= '9')) ? 1 : 0 )

#define ADDR2NODE(x) ((((x) >> 5) & 0x001F) - BASE_ID)
#define NODE2ADDR(x) (((mbxID + BASE_ID) << 5) | ((x) + BASE_ID))
#define GROUPID(n)   (((mbxID + BASE_ID) << 5) | (0x0400 + (n)))
#define BROADCAST    (GROUPID(0))

// enumeration for CANbus::handstate[]
enum {HANDSTATE_UNINIT=0,
      HANDSTATE_DONE,
      HANDSTATE_MOVING,
      HANDSTATE_STALLED};

// typedef unsigned long DWORD;

#define TX_QUEUE_SIZE       (32)
#define RX_QUEUE_SIZE       (32)
#define TX_TIMEOUT          (50)
#define RX_TIMEOUT          (50)

// initialize everything to -10.  Only some of these will get set to
// proper values based on the firmware version.  If CANbus::compile is
// called with an unset property for that puck version, it will complain.

extern int VERS;
extern int ROLE;
extern int SN;
extern int ID;
extern int ERROR;
extern int STAT;
extern int ADDR;
extern int VALUE;
extern int MODE;
extern int TEMP;
extern int PTEMP;
extern int OTEMP;
extern int BAUD;
extern int _LOCK;
extern int DIG0;
extern int DIG1;
extern int FET0;
extern int FET1;
extern int ANA0;
extern int ANA1;
extern int THERM;
extern int VBUS;
extern int IMOTOR;
extern int VLOGIC;
extern int ILOGIC;
extern int SG;
extern int GRPA;
extern int GRPB;
extern int GRPC;
extern int CMD;
extern int SAVE;
extern int LOAD;
extern int DEF;
extern int FIND;
extern int X0;
extern int X1;
extern int X2;
extern int X3;
extern int X4;
extern int X5;
extern int X6;
extern int X7;
extern int T;
extern int MT;
extern int V;
extern int MV;
extern int MCV;
extern int MOV;
extern int P;
extern int P2;
extern int DP;
extern int DP2;
extern int E;
extern int E2;
extern int OT;
extern int OT2;
extern int CT;
extern int CT2;
extern int M;
extern int M2;
extern int _DS;
extern int MOFST;
extern int IOFST;
extern int UPSECS;
extern int OD;
extern int MDS;
extern int MECH;
extern int MECH2;
extern int CTS;
extern int CTS2;
extern int PIDX;
extern int HSG;
extern int LSG;
extern int IVEL;
extern int IOFF;
extern int IOFF2;
extern int MPE;
extern int HOLD;
extern int TSTOP;
extern int KP;
extern int KD;
extern int KI;
extern int ACCEL;
extern int TENST;
extern int TENSO;
extern int JIDX;
extern int IPNM;
extern int HALLS;
extern int HALLH;
extern int HALLH2;
extern int POLES;
extern int IKP;
extern int IKI;
extern int IKCOR;
extern int EN;
extern int EN2;
extern int JP;
extern int JP2;
extern int JOFST;
extern int JOFST2;
extern int TIE;
extern int ECMAX;
extern int ECMIN;
extern int LFLAGS;
extern int LCTC;
extern int LCVC;
extern int TACT;
extern int TACTID;
extern int IHIT;
extern int PROP_END;
extern int AP;
extern int TENSION;
extern int BRAKE;

/* Safety puck properties */
extern int ZERO;
extern int PEN;
extern int SAFE;
extern int VL1;
extern int VL2;
extern int TL1;
extern int TL2;
extern int VOLTL1;
extern int VOLTL2;
extern int VOLTH1;
extern int VOLTH2;
extern int PWR;
extern int MAXPWR;
extern int IFAULT;
extern int VNOM;

/* Force/Torque properties */
extern int FT;
extern int A;

// older properties (firmware < 40)
extern int D;
extern int TORQ;
extern int B;
extern int MD;
extern int AP2;
extern int SAMPLE;
extern int UNITS;
extern int RATIO;
extern int LOG;
extern int DUMP;
extern int LOG1;
extern int LOG2;
extern int LOG3;
extern int LOG4;
extern int GAIN1;
extern int GAIN2;
extern int GAIN3;
extern int OFFSET1;
extern int OFFSET2;
extern int OFFSET3;

#endif
