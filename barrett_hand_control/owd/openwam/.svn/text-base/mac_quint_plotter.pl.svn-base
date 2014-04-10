#! /usr/bin/perl

# ***********************************************************************
# *                                                                     *
# * Copyright 2010 Carnegie Mellon University and Intel Corporation *
# * Author: Mike Vande Weghe <vandeweg@cmu.edu>                         *
# *                                                                     *
# ***********************************************************************

use IO::Handle;

$GP[0] = "GNUPLOT0";
open($GP[0] , "| gnuplot -geometry 600x280");
# set autoflush
$GP[0]->autoflush(1);
    
# use lines for plots
print {$GP[0]} "set style data line\n";

$trajnum = -1;
while (<>) {
    if (/b|B|p|P/) { # back / previous
	$trajnum = $trajnum-1;
	if ($trajnum < 0) {
	    $trajnum = 0;
	}
    } elsif (/(\d+)/) {
	$trajnum = $1;
    } else {
	$trajnum = $trajnum+1;
    }
    $trajname = sprintf("testdata/traj%03d.csv",$trajnum);
    print "Plotting file $trajname\n";
    
    if (-e $trajname) {
	print {$GP[0]} "set title \"$trajname\"\n";
	# pos
	print {$GP[0]} "plot \"$trajname\" using 1:2\n";
	# vel
	print {$GP[0]} "replot \"$trajname\" using 1:3\n";
	# accel
	print {$GP[0]} "replot \"$trajname\" using 1:4\n";

    } else {
	print "$trajname not found";
	print {$GP[0]} "clear\n";
    }
}

close {$GP[0]};
