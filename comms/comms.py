#!/usr/bin/env python

import rospy
from plutodrone.msg import PlutoMsg

class DroneCommander:
    def __init__(self):
        rospy.init_node('pluto_drone_controller', anonymous=True)
        self.pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=10)
        self.cmd = PlutoMsg()
        self.set_neutral()
        rospy.sleep(1)

    def set_neutral(self):
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1000
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1000

    def send_command(self, count=20):
        rate = rospy.Rate(10)
        for _ in range(count):
            self.pub.publish(self.cmd)
            rate.sleep()

    def arm(self):
        self.cmd.rcAUX4 = 1500
        self.send_command()
        rospy.loginfo("Drone armed.")

    def disarm(self):
        self.cmd.rcAUX4 = 1000
        self.send_command()
        rospy.loginfo("Drone disarmed.")

    def takeoff(self):
        self.cmd.rcThrottle = 1700
        self.send_command()
        rospy.loginfo("Takeoff initiated.")

    def land(self):
        self.cmd.rcThrottle = 1100
        self.send_command()
        rospy.loginfo("Landing initiated.")

    def move_left(self):
        self.cmd.rcRoll = 1300
        self.send_command()
        rospy.loginfo("Moving left.")

    def move_right(self):
        self.cmd.rcRoll = 1700
        self.send_command()
        rospy.loginfo("Moving right.")

    def move_forward(self):
        self.cmd.rcPitch = 1700
        self.send_command()
        rospy.loginfo("Moving forward.")

    def move_backward(self):
        self.cmd.rcPitch = 1300
        self.send_command()
        rospy.loginfo("Moving backward.")

    def rotate_left(self):
        self.cmd.rcYaw = 1300
        self.send_command()
        rospy.loginfo("Rotating left.")

    def rotate_right(self):
        self.cmd.rcYaw = 1700
        self.send_command()
        rospy.loginfo("Rotating right.")


if __name__ == "__main__":
    try:
        commander = DroneCommander()
        print("""
Available Commands:
  a  : Arm
  d  : Disarm
  t  : Takeoff
  l  : Land
  fl : Forward
  bl : Backward
  ml : Move Left
  mr : Move Right
  rl : Rotate Left
  rr : Rotate Right
        """)
        while not rospy.is_shutdown():
            choice = raw_input("Enter command: ").strip().lower()

            if choice == 'a':
                commander.arm()
            elif choice == 'd':
                commander.disarm()
            elif choice == 't':
                commander.takeoff()
            elif choice == 'l':
                commander.land()
            elif choice == 'fl':
                commander.move_forward()
            elif choice == 'bl':
                commander.move_backward()
            elif choice == 'ml':
                commander.move_left()
            elif choice == 'mr':
                commander.move_right()
            elif choice == 'rl':
                commander.rotate_left()
            elif choice == 'rr':
                commander.rotate_right()
            else:
                print("Unknown command.")

    except rospy.ROSInterruptException:
        pass
