#!/usr/bin/env python
import rclpy
import math
from allegro_hand_controllers.allegro_robot import AllegroRobot
import threading
rclpy.init()

hand = AllegroRobot(hand_topic_prefix='/allegroHand')

threading.Thread(target=rclpy.spin, args=[hand]).start()

init_pos, init_vel = hand.poll_joint_position(wait=True)


# command to the initial position
cycles = 1
hz = 20
ros_rate = hand.create_rate(hz)

for count in range(cycles):
    for j in range(16):
        for t in range(hz * cycles):
            pos = list(init_pos)
            pos[j] = init_pos[j] + math.radians(30) * math.sin(2*math.pi*t/hz)
            hand.command_joint_position(pos)
            ros_rate.sleep()

hand.disconnect()
hand.destroy_node()
rclpy.shutdown()