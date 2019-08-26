#!/usr/bin/env python  
import roslib
import rospy
import tf

#Using GUI interface for capturing key inputs
import curses
import os

def main(win):
    win.clear()                
    win.addstr("Launched test_frame_broadcaster ros node\n")
    win.addstr("- W : Pitch Up\n")
    win.addstr("- S : Pitch Down\n")
    win.addstr("- A : Yaw Up\n")
    win.addstr("- D : Yaw Down\n")
    win.addstr("- E : Roll Up\n")
    win.addstr("- Q : Roll Down\n")

    rospy.init_node('test_frame_broadcaster')
    parent_frame = rospy.get_param('parent_frame', 'map')
    frame = rospy.get_param('frame', 'base_link')
    step = float(rospy.get_param('step', '0.25'))

    br = tf.TransformBroadcaster()
    roll = 0.0
    pitch = 0.0
    yaw = 0.0

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:                 
            key = str(win.getkey())

            if key == 'w':
                pitch += step
            if key == 's':
                pitch -= step
            if key == 'a':
                yaw += step
            if key == 'd':
                yaw -= step
            if key == 'e':
                roll += step
            if key == 'q':
                roll -= step

        except Exception as e:
           pass         

        br.sendTransform(
            (0, 0, 0),
            tf.transformations.quaternion_from_euler(roll, pitch, yaw),
            rospy.Time.now(),
            frame,
            parent_frame)
        r.sleep()

curses.wrapper(main)