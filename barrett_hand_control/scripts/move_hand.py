#!/usr/bin/env python

from owd_msgs.srv import *
import rospy

def move_hand_client():
    rospy.wait_for_service('/bhd/MoveHand')
    try:
        move_hand = rospy.ServiceProxy('/bhd/MoveHand', MoveHand)

        resp1 = move_hand(movetype=movetype_position, positions=[0.0,0.0,0.0,0.1])
        return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    move_hand_client(x, y)
    print "%s + %s = %s"%(x, y, move_hand_client(x, y))
