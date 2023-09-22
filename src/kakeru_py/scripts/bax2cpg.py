#!/usr/bin/python3
# coding: utf-8
import rospy
from baxter_interface import Limb, Gripper  # <-- Baxter用のパッケージをインポート
from baxter_core_msgs.msg import EndpointState # <-- 追加
import numpy as np
import datetime
import time
import os
import csv
#import threading

class Neurone_RS_list:
    def __init__(self):    
        self.sigmaS = []
        self.V = []
        self.q = []
        self.signal = []


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
        self.RS_list = Neurone_RS_list()
        self.apsignal = 0

    #input signal
    def I_inj(self,input_data,t,delta,fb):
        #input first signal
        if t>=0 and t<=4.5:
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

class CPG:
    def __init__(self,jointnumber,initial_angle,joints):    
        self.jointnumber = jointnumber
        self.center = initial_angle[jointnumber]
        self.joint = joints[jointnumber]
        self.fb = 0
        self.output = 0
        self.Inputlist = []
        self.Outputlist = []

    def fast_current(self,t,delta):
        global flag
        if not flag:
            self.RS1.V = 0
            self.RS2.V = 0
        else:
            self.RS1.V = self.RS1.V + self.RS1.f_V(self.RS2,t,delta,self.fb)*delta
            self.RS2.V = self.RS2.V + self.RS2.f_V(self.RS1,t,delta,self.fb)*delta

    def slow_current(self,delta):
        self.RS1.q = self.RS1.q + self.RS1.f_Q()*delta
        self.RS2.q = self.RS2.q + self.RS2.f_Q()*delta

    def sigma_learning(self,t,delta):
        global flag
        if not flag:
            self.RS1.sigmaS = 0.0
            self.RS2.sigmaS = 0.0

        else:
            self.RS1.sigmaS = self.RS1.sigmaS+self.RS1.f_HLearning(self.RS2,t,delta,self.fb)*delta
            self.RS2.sigmaS = self.RS2.sigmaS+self.RS2.f_HLearning(self.RS1,t,delta,self.fb)*delta
    
    def save_V(self):
        self.RS1.Vs=self.RS1.V
        self.RS2.Vs=self.RS2.V


    def save_to_list_CPG(self):
        #to save list
        self.RS1.RS_list.V.append(self.RS1.V)
        self.RS2.RS_list.V.append(self.RS2.V)
        self.RS1.RS_list.sigmaS.append(self.RS1.sigmaS)
        self.RS2.RS_list.sigmaS.append(self.RS2.sigmaS)
        self.RS1.RS_list.signal.append(self.RS1.apsignal)
        self.RS2.RS_list.signal.append(self.RS2.apsignal)
        self.RS1.RS_list.q.append(self.RS1.q)
        self.RS2.RS_list.q.append(self.RS2.q)
        self.Inputlist.append(self.fb)
        self.Outputlist.append(self.output)
        
    def csvsave(self,csvlist):
        csvlist.append(self.Inputlist)
        csvlist.append(self.Outputlist)
        csvlist.append(self.RS1.RS_list.sigmaS)
        csvlist.append(self.RS2.RS_list.sigmaS)
        csvlist.append(self.RS1.RS_list.signal)
        csvlist.append(self.RS2.RS_list.signal)
        csvlist.append(self.RS1.RS_list.V)
        csvlist.append(self.RS2.RS_list.V)
        csvlist.append(self.RS1.RS_list.q)
        csvlist.append(self.RS2.RS_list.q)
 

def save_to_list_robot(t,arm,joints):
    #to save list
    listtime.append(t)
    for joint in joints:
        joint_lists[joint].append(arm.joint_angle(joint))
        torque_lists[joint].append(arm.joint_effort(joint))
        velocity_lists[joint].append(arm.joint_velocity(joint))
    listeepos.append(arm.endpoint_pose()["position"])
    listeeeffort.append(eetorque)

def csv_write(CPG1,CPG2):
    csvlist = []
    
    header1 = ["CPG1 RS1 tau_m", "CPG1 RS1 tau_s", "CPG1 RS1 A_f", "CPG1 RS1 Sigma_s0", "CPG1 RS1 Sigma_f", "CPG1 RS1 w_inj", "CPG1 RS1 V_0", "CPG1 RS1 q_0", "CPG1 RS1 Epsilon", " CPG1 RS1 delta",\
    "CPG1 RS2 tau_m", "CPG1 RS2 tau_s", "CPG1 RS2 A_f", "CPG1 RS2 Sigma_s0", "CPG1 RS2 Sigma_f", "CPG1 RS2 w_inj", "CPG1 RS2 V_0", "CPG1 RS2 q_0", "CPG1 RS2 Epsilon", "CPG1 RS2 delta", \
    "CPG2 RS1 tau_m", "CPG2 RS1 tau_s", "CPG2 RS1 A_f", "CPG2 RS1 Sigma_s0", "CPG2 RS1 Sigma_f", "CPG2 RS1 w_inj", "CPG2 RS1 V_0", "CPG2 RS1 q_0", "CPG2 RS1 Epsilon", "CPG2 RS2 delta",\
    "CPG2 RS2 tau_m", "CPG2 RS2 tau_s", "CPG2 RS2 A_f", "CPG2 RS2 Sigma_s0", "CPG2 RS2 Sigma_f", "CPG2 RS2 w_inj", "CPG2 RS2 V_0", "CPG2 RS2 q_0", "CPG2 RS2 Epsilon", "CPG2 RS2 delta"
    ]
    header2 = [CPG1.RS1.toM,CPG1.RS1.toS,CPG1.RS1.Af,CPG1.RS1.sigmaS0,CPG1.RS1.sigmaF,CPG1.RS1.w_inj,CPG1.RS1.V0,CPG1.RS1.q0,CPG1.RS1.Epsilon,0.01,\
    CPG1.RS2.toM,CPG1.RS2.toS,CPG1.RS2.Af,CPG1.RS2.sigmaS0,CPG1.RS2.sigmaF,CPG1.RS2.w_inj,CPG1.RS2.V0,CPG1.RS2.q0,CPG1.RS2.Epsilon,0.01,\
    CPG2.RS1.toM,CPG2.RS1.toS,CPG2.RS1.Af,CPG2.RS1.sigmaS0,CPG2.RS1.sigmaF,CPG2.RS1.w_inj,CPG2.RS1.V0,CPG2.RS1.q0,CPG2.RS1.Epsilon,0.01,\
    CPG2.RS2.toM,CPG2.RS2.toS,CPG2.RS2.Af,CPG2.RS2.sigmaS0,CPG2.RS2.sigmaF,CPG2.RS2.w_inj,CPG2.RS2.V0,CPG2.RS2.q0,CPG2.RS2.Epsilon,0.01]
    header3 = ['time', 'EEposeX' , 'EEposeY', 'EEposeZ' , 'EEefortX', 'EEefortY', 'EEefortZ',\
    'CPG1Inputlist', 'CPG1Outputlist', 'CPG1RS1sigma' ,'CPG1RS2sigma','CPG1RS1signal' ,'CPG1RS2signal','CPG1RS1V' ,'CPG1RS2V','CPG1RS1q' ,'CPG1RS2q', \
    'CPG2Inputlist', 'CPG2Outputlist', 'CPG2RS1sigma' ,'CPG2RS2sigma','CPG2RS1signal' ,'CPG2RS2signal','CPG2RS1V' ,'CPG2RS2V','CPG2RS1q' ,'CPG2RS2q', \
    'joint1_angle','joint2_angle','joint3_angle','joint4_angle','joint5_angle','joint6_angle','joint7_angle',\
    'joint1_torque','joint2_torque','joint3_torque','joint4_torque','joint5_torque','joint6_torque','joint7_torque',\
    'joint1_vel','joint2_vel','joint3_vel','joint4_vel','joint5_vel','joint6_vel','joint7_vel']
    csvlist.append(listtime)
    csvlist.append([pos[0] for pos in listeepos])
    csvlist.append([pos[1] for pos in listeepos])
    csvlist.append([pos[2] for pos in listeepos])
    csvlist.append([eef[0] for eef in listeeeffort])
    csvlist.append([eef[1] for eef in listeeeffort])
    csvlist.append([eef[2] for eef in listeeeffort])
    CPG1.csvsave(csvlist)
    CPG2.csvsave(csvlist)
    for i in range(7):
        csvlist.append(joint_lists[i])
    for i in range(7):
        csvlist.append(torque_lists[i])    
    for i in range(7):
        csvlist.append(velocity_lists[i])  
    with open(fname+"/output.csv", mode='w') as file:
        writer = csv.writer(file)
        writer.writerow(header1)
        writer.writerow(header2)
        writer.writerow(header3)

        for items in zip(*csvlist):
            writer.writerow(items)


def torque_callback(data):
    global eetorque
    eetorque = data.wrench.torque
 
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
    return initial_joint_angles

def main():
    global CPG1
    global CPG2
    rospy.init_node('franka_CPG_control')
    arm = Limb('right') 
    angle_initial = set_initial_position(arm)

    joints = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
    #arm.move_to_neutral()
    rospy.sleep(0.5)
    #got handshaking position
    rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, torque_callback)
    rospy.sleep(0.5)
    Hz = 100
    rate = rospy.Rate(Hz) # control rate, in Hz
    delta = 0.01
    joint_angles = arm.joint_angles()
    right_gripper = Gripper('right')
    right_gripper.calibrate()  # 必要に応じて
    right_gripper.close()      # グリッパーを閉じる
    
    initial_angle = [angle_initial[joint] for joint in joints]

    CPG1 = CPG(
        jointnumber = 1,
        initial_angle = initial_angle,#center position
        joints = joints
        )
    CPG1.RS1 = Neurone_RS(
        toM=0.35,
        toS=3.5,
        Af=0.5,
        sigmaS0=49,
        sigmaF=1,
        w_inj=0.8,
        V0=0,
        q0=0,
        Epsilon = 0.08,
        )
    CPG1.RS1.sigmaS = CPG1.RS1.sigmaS0
    CPG1.RS2 = Neurone_RS(
        toM=0.35,
        toS=3.5,
        Af=0.5,
        sigmaS0=49,
        sigmaF=1,
        w_inj=0.8,
        V0=0,
        q0=0,
        Epsilon = 0.08,
        )
    CPG1.RS2.sigmaS = CPG1.RS2.sigmaS0
    CPG2 = CPG(
        jointnumber = 3,
        initial_angle = initial_angle,#center position
        joints = joints
        )
    CPG2.RS1 = Neurone_RS(
        toM=0.35,
        toS=3.5,
        Af=0.5,
        sigmaS0=49,
        sigmaF=1,
        w_inj=0.8,
        V0=0,
        q0=0,
        Epsilon = 0.08,
        )
    CPG2.RS1.sigmaS = CPG2.RS1.sigmaS0
    CPG2.RS2 = Neurone_RS(
        toM=0.35,
        toS=3.5,
        Af=0.5,
        sigmaS0=49,
        sigmaF=1,
        w_inj=0.8,
        V0=0,
        q0=0,
        Epsilon = 0.08,
        )
    CPG2.RS2.sigmaS = CPG2.RS2.sigmaS0
    #Start threading
    #thread = threading.Thread(target=input_thread)
    #thread.start()

    rospy.sleep(0.5)
    initial_force = eetorque
    print(("\033[41m"+"STARTTTTTTTTTTTTT"+"\033[0m"))
    now=time.time()#time initialaize

    try: 
        while not rospy.is_shutdown():
            #time from start
            t = time.time()-now
            #print(t)
            #to send initial position

            if not flag:
                CPG1.fb=0
                CPG2.fb=0
            else:
                CPG1.fb = eetorque.x-initial_force.x#parameter to initial effort = 0
                CPG2.fb = eetorque.x-initial_force.x#parameter to initial effort = 0
            
            #fb = 0
            #Connections of each neuron.
            #fast current 
            CPG1.fast_current(t,delta)
            CPG2.fast_current(t,delta)
            #slow current 
            CPG1.slow_current(delta)
            CPG2.slow_current(delta)
            #sigmaS Learning
            CPG1.sigma_learning(t,delta)
            CPG2.sigma_learning(t,delta)
            #to save last data
            CPG1.save_V()
            CPG2.save_V()
            #to input
            CPG1.output = (CPG1.RS1.V+CPG1.RS2.V)/2
            CPG2.output = (CPG2.RS1.V+CPG2.RS2.V)/2

            #joint_angles[joints[CPG1.jointnumber]] = CPG1.center + CPG1.output/90
            joint_angles[joints[CPG2.jointnumber]] = CPG2.center + CPG2.output/90
            print(initial_angle)
            #save_to_list_robot(t,arm,joints)
            #CPG1.save_to_list_CPG()
            #CPG2.save_to_list_CPG()
            #impedance
            arm.set_joint_positions(joint_angles)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

    #finally:
    #    csv_write(CPG1,CPG2)
    

if __name__ == '__main__':
    #(toM, toS, Af, sigmaS0, sigmaF, w_inj, V0, q0)
    #sigmaF<sigmaS+1   
    fname = "./"+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+"cpg"
    #os.mkdir(fname)
    joint_lists = [[] for _ in range(7)] 
    torque_lists = [[] for _ in range(7)] 
    velocity_lists = [[] for _ in range(7)] 
    listtime= []
    listeepos = []
    listeeeffort = []
    eetorque = 0
    flag = True


    main()
    rospy.sleep(1)