#! /usr/bin/perl

# ***********************************************************************
# *                                                                     *
# * Copyright 2010 Carnegie Mellon University and Intel Corporation *
# * Author: Mike Vande Weghe <vandeweg@cmu.edu>                         *
# *                                                                     *
# ***********************************************************************

use IO::Handle;

# first, start up 3 different gnuplots
for ($i = 0; $i < 5; $i++) {
    $GP[$i] = "GNUPLOT$i";
#    $loc = "+1100+" . ($i-4) * 300;
#    open($GP[$i] , "| gnuplot -geometry 600x280$loc");
    open($GP[$i] , "| gnuplot -geometry 600x280");
    # set autoflush
    $GP[$i]->autoflush(1);
    
    # use lines for plots
    print {$GP[$i]} "set style data line\n";
}
print {$GP[0]} "set title \"Position\"\n";
print {$GP[1]} "set title \"Velocity\"\n";
print {$GP[2]} "set title \"Acceleration\"\n";
print {$GP[3]} "set title \"Path Velocity and Limit\"\n";
print {$GP[4]} "set title \"Path Acceleration and Limit\"\n";

while (<>) {
    if (/(.*)/) {
        system "~/openrave_local/playerdrivers/openwam/mac_traj_test $1"; # creates traj.csv
        
	if (-e "traj.csv") {
	    # pos
	    print {$GP[0]} "plot \"traj.csv\" using 1:2\n";
	    print {$GP[0]} "replot \"traj.csv\" using 1:23\n";
	    
	    # vel
	    print {$GP[1]} "plot \"traj.csv\" using 1:9\n";
	    print {$GP[1]} "replot \"traj.csv\" using 1:24\n";
	    
	    # accel
	    print {$GP[2]} "plot \"traj.csv\" using 1:16\n";

	    # path vel
	    print {$GP[3]} "plot \"traj.csv\" using 1:25\n";
	    print {$GP[3]} "replot \"traj.csv\" using 1:27\n";

	    # path accel
	    print {$GP[4]} "plot \"traj.csv\" using 1:26\n";
	    print {$GP[4]} "replot \"traj.csv\" using 1:28\n";


	    for ($j=3; $j<9; $j++) {
		print {$GP[0]} "replot \"traj.csv\" using 1:$j\n";
		$k=$j+7;
		print {$GP[1]} "replot \"traj.csv\" using 1:$k\n";
		$k=$j+14;
		print {$GP[2]} "replot \"traj.csv\" using 1:$k\n";
	    }

	} else {
	    print {$GP[0]} "clear\n";
	    print {$GP[1]} "clear\n";
	    print {$GP[2]} "clear\n";
	}
    }
}

for ($i = 0; $i < 8; $i++) {
    close {$GP[$i]};
}
