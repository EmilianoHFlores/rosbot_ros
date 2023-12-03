#!/usr/bin/python3

# listens to bluetooth hc05 and sends to /initialpose topic
import rospy
import tf
from geometry_msgs.msg import TransformStamped, PoseStamped, PoseWithCovarianceStamped, Point, Quaternion, Pose
import serial
import time
import math

class BTController:
    def __init__(self):
        self.ser = serial.Serial('/dev/rfcomm0', 9600)
        self.ser.flushInput()
        self.pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        # listener for point
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.goal_counter = 0
        rospy.init_node('bt_listener', anonymous=True)

    def goal_callback(self, data):
        # get data
        x = data.pose.position.x
        y = data.pose.position.y
        euler_z = tf.transformations.euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])[2]
        psi = math.degrees(euler_z)
        print("x: ", x, "y: ", y, "angle: ", psi)
        # publish data to bluetooth, 2 uint16_t for x and y and a counter of 1 uint8_t
        x = int(x * 100)
        y = int(y * 100)
        count = int(self.goal_counter)
        # format message to send, x as uint16_t, y as uint16_t, count as uint8_t
        x = x.to_bytes(2, byteorder='little', signed=False)
        y = y.to_bytes(2, byteorder='little', signed=False)
        count = count.to_bytes(1, byteorder='little', signed=False)
        # data to send
        data = x[1], x[0], y[1], y[0], count[0]
        # print message to send
        print("Sending: ", data)
        # send data
        self.ser.write(data)
        self.goal_counter += 1

    def run(self):
        try:
            while not rospy.is_shutdown():
                
                #check connection
                if self.ser.in_waiting > 0:
                    # message example:
                    # Psi: 0.00 Delta: 0.500 P: 0.0, 0.0 WP: 136.0, 85.0 D2WP: 160.378 Traction: 0.50 Speed Target: 0.22 Speed: 0.00 Nav_state: 
                    # read message
                    line = self.ser.readline()
                    # split message
                    print(line)
                    line = line.decode("utf-8")
                    data = line.split(" ")
                    # check if message is valid
                    if data[0] == "Psi:":
                        # get data
                        try:
                            psi = float(data[1])
                            delta = float(data[3])
                            x = float(data[5][:-1]) / 100
                            y = float(data[6][:-1]) / 100
                            print("x: ", x, "y: ", y, "angle: ", psi)
                        except:
                            continue
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
                        self.pub.publish(pose)

                    else:
                        print("Invalid data")
                time.sleep(0.001)
                    
        except KeyboardInterrupt:
            self.ser.close()
            print("Connection closed")

        finally:
            self.ser.close()
            print("Connection closed")

if __name__ == '__main__':
    bt = BTController()
    bt.run()