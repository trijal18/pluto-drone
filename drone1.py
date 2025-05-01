#!/usr/bin/env python

import rospy
from plutodrone.msg import PlutoMsg  # Confirm this is the correct message type

def send_command(aux4_value):
    pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=10)
    rospy.init_node('drone_arm_disarm', anonymous=True)
    cmd = PlutoMsg()

    # Neutral/default values
    cmd.rcRoll = 1500
    cmd.rcPitch = 1500
    cmd.rcYaw = 1500
    cmd.rcThrottle = 1000  # minimum throttle
    cmd.rcAUX1 = 1500
    cmd.rcAUX2 = 1500
    cmd.rcAUX3 = 1500
    cmd.rcAUX4 = aux4_value  # 1800 to arm, 1000 to disarm

    rate = rospy.Rate(10)
    for i in range(20):  # Send 20 times to ensure receipt
        pub.publish(cmd)
        rate.sleep()
    rospy.loginfo("Command sent.")

if __name__ == "__main__":
    try:
        choice = raw_input("Enter 'a' to ARM or 'd' to DISARM the drone: ").strip().lower()
        if choice == 'a':
            rospy.loginfo("Arming the drone...")
            send_command(1500)
        elif choice == 'd':
            rospy.loginfo("Disarming the drone...")
            send_command(1000)
        else:
            print("Invalid input. Use 'a' to arm or 'd' to disarm.")
    except rospy.ROSInterruptException:
        pass
