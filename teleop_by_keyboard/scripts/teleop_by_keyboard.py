#!/usr/bin/env python 


from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!    
---------------------------
Moving around: 
   r  t   y
   f  g   h
   v  b   n

For Holonomic mode (strafing), hold down the shift key:
---------------------------
1: +x +y
2: +x -y
3: -x -y
4: -x +y

5 : up (+z)
6 : down (-z)

anything else : stop

u/i : increase/decrease max speeds by 10%
o/p : increase/decrease only linear speed by 10%
k/l : increase/decrease only angular speed by 10%

CTRL-C to quit
"""
auto_track = {'a':(0,0,0,0)}
moveBindings ={
        't':(1,0,0,0),
        'z':(1,0,0,-1), 
        'f':(0,0,0,1),
        'h':(0,0,0,-1),
        'r':(1,0,0,1),
        'b':(-1,0,0,0),
        'n':(-1,0,0,1),
        'v':(-1,0,0,-1),
        '2':(1,-1,0,0),
        '1':(1,1,0,0),
        '3':(-1,-1,0,0),
        '4':(-1,1,0,0),
        '5':(0,0,1,0),
        '6':(0,0,-1,0),
    }

speedBindings={
        'u':(1.1,1.1),
        'i':(.9,.9),
        'o':(1.1,1),
        'p':(.9,1),
        'k':(1,1.1),
        'l':(1,.9),
    }

def type_key():
    tty.setraw(sys.stdin.fileno())              #set input terminal to raw
    select.select([sys.stdin], [], [], 0)	#wait to read input
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings) #change terminal settings
    print(key)
    return key


def vels(speed,turn):
    return "currently:\tlinear speed %s\tangular speed %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    laser_switch = Bool()
    laser_switch = False
    pub1 = rospy.Publisher('/vrep/cmd_vel', Twist, queue_size = 1)  
    pub2 = rospy.Publisher('/vrep/laser_switch', Bool, queue_size = 1) 	
    rospy.init_node('teleop_by_keyboard')
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        print(msg)
        print(vels(speed,turn))
        while(1):           
            key = type_key()
            if (status == 14):
                print(msg)
            status = (status + 1) % 15
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                print(vels(speed,turn))
            elif key in auto_track.keys():
                laser_switch = not laser_switch    
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            T = Twist()
            T.linear.x = x*speed
            T.linear.y = y*speed
            T.linear.z = z*speed
            T.angular.x = 0
            T.angular.y = 0
            T.angular.z = th*turn
            if laser_switch == False:
                pub1.publish(T)
            else:
                pub2.publish(laser_switch)
    except Exception as e:
        print(e)

    finally:
        T = Twist()
        T.linear.x = 0
        T.linear.y = 0
        T.linear.z = 0
        T.angular.x = 0
        T.angular.y = 0 
        T.angular.z = 0
        pub1.publish()
        pub2.publish(laser_switch)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
