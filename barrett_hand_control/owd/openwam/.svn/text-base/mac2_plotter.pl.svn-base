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
    print {$GP[$i]} "set style line 1 linewidth 4\n";
    print {$GP[$i]} "set style line 2 linewidth 4\n";
    print {$GP[$i]} "set style increment user\n";
    print {$GP[$i]} "set key off\n";

}
#print {$GP[0]} "set title \"Position\"\n";
print {$GP[0]} "set title \"J2 vs J1\" font \"helvetica,18\"\n";
print {$GP[1]} "set title \"Velocity\" font \"helvetica,18\"\n";
print {$GP[2]} "set title \"Acceleration\" font \"helvetica,18\"\n";
print {$GP[3]} "set title \"Path Velocity\" font \"helvetica,18\"\n";
print {$GP[4]} "set title \"Path Acceleration\" font \"helvetica,18\"\n";

while (<>) {
    if (/(.*)/) {
        print "running mac_traj2_test on $1\n";
        system "mac_traj2_test $1"; # creates traj.csv
        
	if (-e "traj.csv") {
	    print "Plotting traj.csv\n";
	    # pos
#	    print {$GP[0]} "plot \"traj.csv\" using 1:2\n";
#	    print {$GP[0]} "replot \"traj.csv\" using 1:7\n";
	    print {$GP[0]} "plot \"traj.csv\" using 2:3\n";
	    # traj 2 pos
	    print {$GP[0]} "replot \"traj.csv\" using 15:16\n";
            print {$GP[0]} "set style line 1 linewidth 8\n";
            print {$GP[0]} "set style line 2 linewidth 8\n";
	    
	    # vel
	    print {$GP[1]} "plot \"traj.csv\" using 1:4\n";
	    print {$GP[1]} "replot \"traj.csv\" using 1:8\n";
	    
	    # accel
	    print {$GP[2]} "plot \"traj.csv\" using 1:7\n";

	    # path vel
	    print {$GP[3]} "plot \"traj.csv\" using 1:10\n";
#	    print {$GP[3]} "replot \"traj.csv\" using 1:12\n";
	    # traj 2 path vel
	    print {$GP[3]} "replot \"traj.csv\" using 1:17\n";

	    # path accel
	    print {$GP[4]} "plot \"traj.csv\" using 1:11\n";
#	    print {$GP[4]} "replot \"traj.csv\" using 1:13\n";
#	    print {$GP[4]} "replot \"traj.csv\" using 1:14\n";
	    # traj 2 path accel
	    print {$GP[4]} "replot \"traj.csv\" using 1:18\n";


	    for ($j=3; $j<4; $j++) {
#		print {$GP[0]} "replot \"traj.csv\" using 1:$j\n";
		$k=$j+2;
		print {$GP[1]} "replot \"traj.csv\" using 1:$k\n";
		$k=$j+2;
		print {$GP[2]} "replot \"traj.csv\" using 1:$k\n";
	    }

	} else {
	    print "Could not find $1\n";
	    print {$GP[0]} "clear\n";
	    print {$GP[1]} "clear\n";
	    print {$GP[2]} "clear\n";
	}
    }
}

for ($i = 0; $i < 8; $i++) {
    close {$GP[$i]};
}
