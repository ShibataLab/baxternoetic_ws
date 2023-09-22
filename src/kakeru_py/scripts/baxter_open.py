#!/usr/bin/python33
import rospy
import threading
import baxter_interface
import requests
rospy.init_node('Hello_Open2')


limb1 = baxter_interface.Limb('right')
limb2 = baxter_interface.Limb('left')

angles1 = limb1.joint_angles()
angles2 = limb2.joint_angles()

thread = threading.Thread(limb1.move_to_joint_positions(angles1), limb2.move_to_joint_positions(angles2))

thread.start()

thread.join()

wave_1 = {'right_s0':0.475534044244, 'right_s1':-0.933427309428, 'right_e0':-0.121951472637, 'right_e1':0.449839866047, 'right_w0':0.00690291354548, 'right_w1':1.79514101702, 'right_w2':0.726339903064}

wave_4 = {'left_s0':-0.610524353578, 'left_s1':-1.04080596458, 'left_e0':0.878708761789, 'left_e1':0.625097171063, 'left_w0':-0.258475762759, 'left_w1':1.47568951795, 'left_w2':-0.439868990926}

limb1.move_to_joint_positions(wave_1)
limb2.move_to_joint_positions(wave_4)

rospy.sleep(2)

#r = requests.get("http://192.168.0.222:8000/wheel_moving")

quit()
