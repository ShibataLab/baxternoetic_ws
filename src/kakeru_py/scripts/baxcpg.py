#!/usr/bin/python3
# coding: utf-8
import rospy
from baxter_interface import Limb
import numpy as np
import time
import sys
import os



class Neurone_RS:
    def __init__(self, toM, toS, Af, sigmaS0, sigmaF, w_inj, V0, q0,Epsilon):
        self.toM = toM
        self.toS = toS
        self.Af = Af
        self.sigmaS0 = sigmaS0
        self.sigmaS = sigmaS0
        self.sigmaF = sigmaF
        self.w_inj = w_inj
        self.V = V0
        self.q = q0
        self.V0 = V0
        self.q0 = q0
        self.Epsilon = Epsilon
        self.Vs = 0
        


    #input signal
    def I_inj(self,input_data,t,delta,fb):
        #input first signal
        if t>=0 and t<=0.5:
        #if t>=0 and t<0.5:
            self.apsignal = 0.0001
            I_return = self.apsignal
        #input additional signal(Debug)
        #elif t>=2 and t<=20:
        #    f=1#Hz
            #Amplitude = 0.1 
        #    self.apsignal = np.sin(2*t*np.pi*f)/15
        #    I_return = input_data.V+self.apsignal
        #FB controll
        else:
            self.apsignal = fb
            I_return = input_data.V + self.apsignal
        return I_return

    def F(self):
        return self.V-self.Af*np.tanh((self.sigmaF/self.Af)*self.V)


    def f_V(self,input_data,t,delta,fb):
        #impuslion dirac to start the system
        return -((self.F()+self.q-self.w_inj*self.I_inj(input_data,t,delta,fb)))*(1/self.toM)

    def f_Q(self):
        return (-self.q + self.sigmaS*self.V)/self.toS

    def f_HLearning(self,input_data,t,delta,fb):
        #finite difference method
        y=(self.V-self.Vs)/delta-self.Epsilon*self.I_inj(input_data, t, delta,fb)
        #print(y)
        return (2*self.Epsilon*self.I_inj(input_data, t, delta,fb)*np.sqrt(self.toM*self.toS)*np.sqrt(1+self.sigmaS-self.sigmaF)*y/np.sqrt(self.V**2+y**2))


RS1 = Neurone_RS(
        toM=0.35,
        toS=3.5,
        Af=0.5,
        sigmaS0=49.0,
        sigmaF=1,
        w_inj=0.8,
        V0=0,
        q0=0,
        Epsilon = 0.08
        )
RS2 = Neurone_RS(
        toM=0.35,
        toS=3.5,
        Af=0.5,
        sigmaS0=49.0,
        sigmaF=1,
        w_inj=0.8,
        V0=0,
        q0=0,
        Epsilon = 0.08
        )

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
    rospy.init_node("baxter_cpg_wave_control")
    right_arm = Limb("right")
    # 腕を初期位置に移動し、初期位置の`right_s1`関節の角度を取得
    initial_angle_s1 = set_initial_position(right_arm)
    # 現在の関節角度を取得して、第2関節（'right_s1'）の角度を更新
    joint_angles = right_arm.joint_angles()
    rospy.sleep(0.5)
    Hz = 200
    rate = rospy.Rate(Hz) # control rate, in Hz
    delta = 0.005
    initial_angle_s1 = set_initial_position(right_arm)
    now=rospy.get_time()#time initialaize
    while not rospy.is_shutdown():
        #time from start
        t = rospy.get_time()-now
        #print(t)
        #diplacement

        fb = np.sin(2*t*np.pi) * 10
        #Connections of each neuron.
        #fast current 
        RS1.V = RS1.V + RS1.f_V(RS2,t,delta,fb)*delta
        RS2.V = RS2.V + RS2.f_V(RS1,t,delta,fb)*delta
        #slow current 
        RS1.q = RS1.q + RS1.f_Q()*delta
        RS2.q = RS2.q + RS2.f_Q()*delta
        #sigmaS Learning
        RS1.sigmaS = RS1.sigmaS+RS1.f_HLearning(RS2,t,delta,fb)*delta
        RS2.sigmaS = RS2.sigmaS+RS2.f_HLearning(RS1,t,delta,fb)*delta
        #to save last data
        RS1.Vs=RS1.V
        RS2.Vs=RS2.V
        #to input
        output = (RS1.V+RS2.V)/2
        print(output)
        angle = initial_angle_s1 + output/1000

        
        joint_angles['right_s1'] = angle
        # 設定した関節角度を適用
        right_arm.set_joint_positions(joint_angles)
        rate.sleep()
    

    

if __name__ == '__main__':
    try:
        #Define Publisher
        main()

    except rospy.ROSInterruptException:
        pass