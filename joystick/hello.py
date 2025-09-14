#!/usr/bin/env python3
import rospy
import pygame
from std_msgs.msg import String

def map_joystick(value, deadzone=0.2):
    return 0 if abs(value) < deadzone else (1 if value > 0 else -1)

def main():
    rospy.init_node('joystick_publisher')
    pub = rospy.Publisher('/joystick_data', String, queue_size=10)

    pygame.init()
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"[INFO] Joystick initialized: {joystick.get_name()}")

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        pygame.event.pump()
        x = map_joystick(joystick.get_axis(0))  # Left stick X
        y = map_joystick(joystick.get_axis(1))  # Left stick Y
        z = map_joystick(joystick.get_axis(3))  # Right stick X

        buttons = [joystick.get_button(i) for i in range(8)]
        msg = f"{x},{y},{z}," + ",".join(map(str, buttons))
        pub.publish(msg)
        print(f"[PUBLISH] {msg}")
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("[EXIT] Joystick publisher stopped.")
