#!/usr/bin/python3
# coding: utf-8
import rospy
from baxter_interface import Limb
import math
import time

def set_initial_position(arm):
    # 任意の初期位置を設定
    initial_joint_angles = {
        'right_s0': 1.0,
        'right_s1': -0.55,
        'right_e0': 0.0,
        'right_e1': 1.18,
        'right_w0': 0.0,
        'right_w1': 0.0,
        'right_w2': 0.0
    }
    # 設定した関節角度を適用して、腕を初期位置に移動
    arm.move_to_joint_positions(initial_joint_angles)
    return initial_joint_angles['right_s1']

def main():
    # ROSノードの初期化
    rospy.init_node("baxter_sin_wave_control")

    # 右腕のインターフェースを取得
    right_arm = Limb("right")

    # 腕を初期位置に移動し、初期位置の`right_s1`関節の角度を取得
    initial_angle_s1 = set_initial_position(right_arm)

    rate = rospy.Rate(10)  # 10Hzで制御

    start_time = time.time()

    try:
        while not rospy.is_shutdown():
            # 現在の時間から開始時刻を引いて、経過時間を計算
            elapsed_time = time.time() - start_time

            # サイン波の計算
            angle_change = math.sin(elapsed_time) * 0.2  # 0.5ラジアンの振幅とする
            angle = initial_angle_s1 + angle_change
            
            # 現在の関節角度を取得して、第2関節（'right_s1'）の角度を更新
            joint_angles = right_arm.joint_angles()
            joint_angles['right_s1'] = angle
            
            # 設定した関節角度を適用
            right_arm.set_joint_positions(joint_angles)

            rate.sleep()

    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
