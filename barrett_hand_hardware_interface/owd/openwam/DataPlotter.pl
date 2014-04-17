#! /usr/bin/perl

# ***********************************************************************
# *                                                                     *
# * Copyright 2010 Carnegie Mellon University and Intel Corporation *
# * Author: Mike Vande Weghe <vandeweg@cmu.edu>                         *
# *                                                                     *
# ***********************************************************************

use IO::Handle;

$GP[0] = "GNUPLOT0";
$GP[1] = "GNUPLOT1";
open($GP[0] , "| gnuplot -geometry 600x280");
open($GP[1] , "| gnuplot -geometry 600x280");
# set autoflush
$GP[0]->autoflush(1);
$GP[1]->autoflush(1);
# use lines for plots
print {$GP[0]} "set style data lines\n";
print {$GP[1]} "set style data lines\n";
print {$GP[1]} "set y2tics\n";

$linenum = -1;
while (<>) {
    if (/b|B|p/) { # back / previous
	$linenum = $linenum-1;
	if ($linenum < 0) {
	    $linenum = 0;
	}
    } elsif (/(\d+)/) {
	$linenum = $1;
    } elsif (/LP/) {
	print {$GP[1]} "set style data linespoints\n";
    } elsif (/P/) {
	print {$GP[1]} "set style data points\n";
    } elsif (/L/) {
	print {$GP[1]} "set style data lines\n";
    } else {
	$linenum = $linenum+1;
    }
    $f = sprintf("wamstats%04d.csv",$linenum);
    system("scp wam:/tmp/$f .");
    print "Plotting $f\n";
    
    if (-e $f) {
	$f = "\"$f\"";
	print {$GP[1]} "set title $f\n";
#print {$GP[1]} "plot $f using 2\n"; # timestep_factor
#print {$GP[1]} "replot $f using 3\n"; #traj_time
#	print {$GP[1]} "plot $f using 4 title \"J1 targ\"\n";
#	print {$GP[1]} "replot $f using 5 title \"J1 act\"\n";
#	print {$GP[1]} "replot $f using 6 axes x1y2 title \"J1 PID\"\n";
#	print {$GP[1]} "replot $f using 7 title \"J2 targ\"\n";
#	print {$GP[1]} "replot $f using 8 title \"J2 act\"\n";
#	print {$GP[1]} "replot $f using 9 axes x1y2 title \"J2 PID\"\n";
#print {$GP[1]} "replot $f using 10\n"; # J3 target
#print {$GP[1]} "replot $f using 11\n"; # J3 actual
#print {$GP[1]} "replot $f using 12 axes x1y2 title \"J3 PID\"\n";
#print {$GP[1]} "replot $f using 13\n"; # J4 target
#print {$GP[1]} "replot $f using 14\n"; # J4 actual
#print {$GP[1]} "replot $f using 16\n"; # J5 target
#print {$GP[1]} "replot $f using 17\n"; # J5 actual
#print {$GP[1]} "replot $f using 19\n"; # J6 target
#print {$GP[1]} "replot $f using 20\n"; # J6 actual
print {$GP[1]} "plot $f using 22 title \"J7 targ\"\n";
print {$GP[1]} "replot $f using 23 title \"J7 act\"\n";
print {$GP[1]} "replot $f using 24 axes x1y2 title \"J7 PID\"\n";
#	print {$GP[1]} "replot $f using 25 axes x1y2 title \"J1 dyn\"\n";
#	print {$GP[1]} "replot $f using 26 axes x1y2 title \"J2 dyn\"\n";
#for ($col=3; $col<21; ++$col) {
#  print {$GP[1]} "replot $f using $col\n";
#}

#	print {$GP[0]} "set title \"Actual Values\"\n";
#
#	print {$GP[0]} "plot $f using 5 title \"J1\"\n";
#	print {$GP[0]} "replot $f using 8 title \"J2\"\n";
#	print {$GP[0]} "replot $f using 11 title \"J3\"\n";
#	print {$GP[0]} "replot $f using 14 title \"J4\"\n";
#	print {$GP[0]} "replot $f using 17 title \"J5\"\n";
#	print {$GP[0]} "replot $f using 20 title \"J6\"\n";
#	print {$GP[0]} "replot $f using 23 title \"J7\"\n";

	print {$GP[0]} "set title \"Target Values\"\n";

	print {$GP[0]} "plot $f using 4 title \"J1\"\n";
	print {$GP[0]} "replot $f using 7 title \"J2\"\n";
	print {$GP[0]} "replot $f using 10 title \"J3\"\n";
	print {$GP[0]} "replot $f using 13 title \"J4\"\n";
	print {$GP[0]} "replot $f using 16 title \"J5\"\n";
	print {$GP[0]} "replot $f using 19 title \"J6\"\n";
	print {$GP[0]} "replot $f using 22 title \"J7\"\n";

    } else {
	print "$f not found";
	print {$GP[1]} "clear\n";
    }
}


close {$GP[0]};
