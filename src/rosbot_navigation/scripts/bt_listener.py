#!/usr/bin/python3

# listens to bluetooth hc05 and sends to /initialpose topic
import rospy
import tf
from geometry_msgs.msg import TransformStamped, PoseStamped, PoseWithCovarianceStamped, Point, Quaternion, Pose
import serial
import time
import math

ser = serial.Serial('/dev/rfcomm0', 9600)
# ser.open()
ser.flushInput()
pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
rospy.init_node('bt_listener', anonymous=True)


try:
    while True:
        
        #check connection
        if ser.in_waiting > 0:
            # message example:
            # Psi: 0.00 Delta: 0.500 P: 0.0, 0.0 WP: 136.0, 85.0 D2WP: 160.378 Traction: 0.50 Speed Target: 0.22 Speed: 0.00 Nav_state: 
            # read message
            line = ser.readline()
            # split message
            print(line)
            line = line.decode("utf-8")
            data = line.split(" ")
            # check if message is valid
            if data[0] == "Psi:":
                # get data
                psi = float(data[1])
                delta = float(data[3])
                x = float(data[5][:-1]) / 100
                y = float(data[6][:-1]) / 100
                print("x: ", x, "y: ", y, "angle: ", psi)
                # publish data
                # to rad
                euler_z = math.radians(psi)
                euler_y = 0
                euler_x = 0
                q = tf.transformations.quaternion_from_euler(euler_x, euler_y, euler_z)
                pose = PoseWithCovarianceStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "map"
                pose.pose.pose.position.x = x
                pose.pose.pose.position.y = y
                pose.pose.pose.position.z = 0
                pose.pose.pose.orientation.x = q[0]
                pose.pose.pose.orientation.y = q[1]
                pose.pose.pose.orientation.z = q[2]
                pose.pose.pose.orientation.w = q[3]
                pub.publish(pose)

            else:
                print("Invalid data")
        time.sleep(0.001)
            
except KeyboardInterrupt:
    ser.close()
    print("Connection closed")

finally:
    ser.close()
    print("Connection closed")