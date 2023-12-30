#!/usr/bin/env python

import rospy
import serial
import math
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3

ser = serial.Serial(port="/dev/ttyUSB0", baudrate=115200,
                    rtscts=True, timeout=0.1)

send_data = "n" + "\n"
last_catch = False
has_flushed_recieve_buffer = False
omelette_flag = Bool()
omelette_flag.data = False


def cb_destination(destination):
    global send_data
    global last_catch
    global omelette_flag
    global has_flushed_recieve_buffer
    plus = 160

    catched = Bool()

    if ser.cts:
        if not has_flushed_recieve_buffer:
            ser.reset_input_buffer()
            has_flushed_recieve_buffer = True

        if destination.z == 0:
            send_data = "x" + str(int(math.floor(destination.y*1000))) + "\n"
            rospy.loginfo("move destination x:%f", destination.y)
            catched.data = False

            if abs(destination.y) >= 0.12:
                last_catch = False

        elif destination.z == 1:
            if last_catch:
                send_data = "c" + \
                    str(int(math.floor(destination.x*1000))+plus) + "\n"
                rospy.loginfo("Catch destination y:%f", destination.x)
                last_catch = False

            else:
                send_data = "p" + \
                    str(int(math.floor(destination.x*1000))+plus) + "\n"
                rospy.loginfo("Pull down destination y:%f", destination.x)
                last_catch = False

        elif destination.z == 2:
            send_data = "c" + \
                str(int(math.floor(destination.x*1000))+plus) + "\n"
            rospy.loginfo("Catch destination y:%f", destination.x)
            last_catch = False

        elif destination.z == 3:
            send_data = "n" + "\n"
            rospy.loginfo("Ignore!")
            last_catch = False

        elif destination.z == 4:
            send_data = "r" + \
                str(int(math.floor(destination.x*1000))+plus+30) + "\n"
            rospy.loginfo("Remove!")
            last_catch = False
            #catched.data = True

        else:
            rospy.loginfo(
                "send error (destination.z:%f is not available", destination.z)
            send_data = "n" + "\n"

        ser.write(send_data.encode())
        print(send_data)


def cb_send(send):
    global omelette_flag
    global has_flushed_recieve_buffer

    pub = rospy.Publisher('omelette', Bool, queue_size=10)

    if not ser.cts:
        has_flushed_recieve_buffer = False
        if ser.in_waiting:
            mode = ser.readline()

            if mode.decode() == "o\n":
                print("Execute Omelette")
                omelette_flag.data = True

            elif mode.decode() == "n\n":
                print("Not Execute Omelette")
                omelette_flag.data = False

            else:
                rospy.logerr("mode error")
                omelette_flag.data = False

            pub.publish(omelette_flag)


def listner():
    rospy.init_node('listner', anonymous=True)

    rospy.Subscriber('destination', Vector3, cb_destination)
    rospy.Subscriber('send', Bool, cb_send)

    rospy.loginfo("start send_data node")

    rospy.spin()


if __name__ == '__main__':
    listner()
