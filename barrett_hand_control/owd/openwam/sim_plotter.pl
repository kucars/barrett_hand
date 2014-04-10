#! /usr/bin/perl

# ***********************************************************************
# *                                                                     *
# * Copyright 2010 Carnegie Mellon University and Intel Corporation *
# * Author: Mike Vande Weghe <vandeweg@cmu.edu>                         *
# *                                                                     *
# ***********************************************************************

# run this by doing a "tail -f" of the current wamdriver log and piping it
# to this script


use IO::Handle;



# first, start up 4 different gnuplots
for ($i = 0; $i < 4; $i++) {
    $GP[$i] = "GNUPLOT$i";
    $loc = "+1100+" . ($i-4) * 300;
#    open($GP[$i] , "| gnuplot -geometry 600x280$loc");
    open($GP[$i] , "| gnuplot -geometry 600x280");
    # set autoflush
    $GP[$i]->autoflush(1);
    
    # use lines for plots 1-7
    print {$GP[$i]} "set style data line\n";
}
print {$GP[0]} "set title \"Position\"\n";
print {$GP[1]} "set title \"Velocity\"\n";
print {$GP[2]} "set title \"Acceleration\"\n";
# print {$GP[3]} "set title \"Torque\"\n";

# while (sleep 1) {
while (<>) {
    if (/(.*)/) {
        system "/root/raveplayerdrivers/openwam/traj_smoothing_test $1"; # creates smoothed.csv and orig.csv
        
        # orig points, J1
        print {$GP[0]} "plot \"orig.csv\" using 1:2 title \"RAVE J1\" with points pt 5\n";

        # smoothed traj, J1
        print {$GP[1]} "plot \"smoothed.csv\" using 1:9 title \"J1\"\n";
        print {$GP[2]} "plot \"smoothed.csv\" using 1:16 title \"J1\"\n";
#        print {$GP[3]} "plot \"smoothed.csv\" using 1:23 title \"Joint 1\"\n";

        for ($j = 2; $j < 8; $j++) {
            $pos = $j+1;
            $vel = $pos+7;
            $accel = $vel+7;
            $torq = $accel+7;
            #orig points, J2-J7
            print {$GP[0]} "replot \"orig.csv\" using 1:$pos title \"RAVE J$j\" with points pt 5\n";

            # vel, J2-J7
            print {$GP[1]} "replot \"smoothed.csv\" using 1:$vel title \"J$j\"\n";
            #accel, J2-J7
            print {$GP[2]} "replot \"smoothed.csv\" using 1:$accel title \"J$j\"\n";
#            print {$GP[3]} "replot \"smoothed.csv\" using 1:$torq title \"Joint $j\"\n";
        }
        # sim points, J1
        print {$GP[0]} "replot \"smoothed.csv\" using 1:2 title \"OWD J1\" lt 1\n";

        for ($j = 2; $j < 8; $j++) {
            $pos = $j+1;
            # smoothed traj, J2-J7
            print {$GP[0]} "replot \"smoothed.csv\" using 1:$pos title \"OWD J$j\" lt $j\n";
        }
    }
}

for ($i = 0; $i < 8; $i++) {
    close {$GP[$i]};
}
