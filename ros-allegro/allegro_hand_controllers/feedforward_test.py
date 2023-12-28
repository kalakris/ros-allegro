#!/usr/bin/env python
import rclpy
import math
import time
from allegro_hand_controllers.allegro_robot import AllegroRobot
import threading
rclpy.init()

hand = AllegroRobot(hand_topic_prefix='/allegroHand')

threading.Thread(target=rclpy.spin, args=[hand]).start()

init_pos, init_vel = hand.poll_joint_position(wait=True)

pos = list(init_pos)
# Start without torques
print("Freezing initial joint positions")
hand.command_joint_position(pos)
time.sleep(2)

while True:
    pos, vel = hand.poll_joint_position(wait=True)
    torques = [0.0] * 16
    hand.command_joint_positions_and_torques(pos, torques)

input('press any key to turn robot off')
hand.disconnect()
hand.destroy_node()
rclpy.shutdown()