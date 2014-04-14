#/usr/bin/perl

#########################################################################
#
#  Copyright 2011 Carnegie Mellon University and Intel Corporation
#  Author: Mike Vande Weghe <vandeweg@cmu.edu>
#
#  This file is part of owd.
#
#  owd is free software; you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation; either version 3 of the License, or (at your
#  option) any later version.
#
#  owd is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
#########################################################################


$old=0;
while (<>) {
   if (/(\d+),(\d+).*G04 GET 048/) {
       $new=$1*1000000+$2;
       $delta=$new-$old;
       if ($old > 0) {
	   print "$new $delta\n";
       }
       $old=$new;
   }
}
