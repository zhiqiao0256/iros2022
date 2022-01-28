"""
Test 1
2 segments, 3 ridig bodies
only activate bottom one
"""
import numpy as np
import zmq
import pickle
import zlib
from time import time, sleep
import threading
# import math
class pc_client(object):
    """docstring for pc_client"""
    def __init__(self):
        """ Select use mocap or not"""
        self.flag_use_mocap=1
        """ Initiate ZMQ communication"""
        context = zmq.Context()
        self.socket0 = context.socket(zmq.PUB)
        self.socket0.setsockopt(zmq.CONFLATE,True)
        self.socket0.bind("tcp://10.203.49.209:4444")## PUB pd to Raspi Client

        self.socket1 = context.socket(zmq.PUB)##PUb to Record
        self.socket1.setsockopt(zmq.CONFLATE,True)
        self.socket1.bind("tcp://127.0.0.1:5555")

        self.socket2=context.socket(zmq.SUB) ### sub mocap data
        self.socket2.setsockopt(zmq.SUBSCRIBE,'')
        self.socket2.setsockopt(zmq.CONFLATE,True)

        if self.flag_use_mocap == True:
            self.socket2.connect("tcp://127.0.0.1:3885")
            print "Connected to mocap"

        self.socket3=context.socket(zmq.SUB) ### sub Raspi Client
        self.socket3.setsockopt(zmq.SUBSCRIBE,'')
        self.socket3.setsockopt(zmq.CONFLATE,True)
        # self.socket3.setsockopt(zmq.RCVTIMEO,10000)
        self.socket3.connect("tcp://10.203.54.75:3333")
        print "Connected to Low"

        """ Format recording """
        self.array4setsRecord=np.array([0.]*41)#t pd1 pd2 pd3 + pm1 +pm2 +pm3 + (positon +orintation)*3 + u_t, u_tt,  
        self.pd_pm_array=np.array([0.]*6) #pd1 pd2 pd3 + pm1 +pm2 +pm3 (psi)
        self.array4setswithrotation=np.array([0.]*28)# base(x y z qw qx qy qz)[0-6] top(x1 y1 z1 qw1 qx1 qy1 qz1)[7-13] top1(x y z qw qx qy qz)[14-20] top2(x1 y1 z1 qw1 qx1 qy1 qz1)
        self.u_array=np.array([0.0]*3)
        """ Thearding Setup """
        self.th1_flag=True
        self.th2_flag=True
        self.flag_reset=1
        self.run_event=threading.Event()
        self.run_event.set()
        self.th1=threading.Thread(name='raspi_client',target=self.th_pub_raspi_client_pd)
        self.th2=threading.Thread(name='mocap',target=self.th_sub_pub_mocap)
        """State variable Ini """
        self.t0=0.0 # timer sec
        self.t_old=0.0 #timer sec
        self.t_new=0.0
        self.T=10 #sec duriation
        self.a=-10
        self.freq=0.01
    def th_pub_raspi_client_pd(self):
        try:
            if self.flag_reset==1:
                self.step_response(np.array([0.0,0.0,0.0]),5)
                self.flag_reset=0
            if self.flag_use_mocap == True:
                self.array4setswithrotation=self.recv_cpp_socket2()
            self.t0=time()
            self.closedLoopControl()          
            self.step_response(np.array([0.0,0.0,0.0]),5)
            self.th1_flag=False
            self.th2_flag=False
            exit()
        except KeyboardInterrupt:
            exit()
            self.th1_flag=False
            self.th2_flag=False
            print "Press Ctrl+C to Stop"
        #     print "Press Ctrl+C to Stop"
            
    def th_sub_pub_mocap(self):# thread config of read data from mocap and send packed msg to record file.
        try:
            while self.run_event.is_set() and self.th2_flag:
                if self.flag_use_mocap == True:
                    self.array4setswithrotation=self.recv_cpp_socket2()
                self.pd_pm_array=self.recv_zipped_socket3()
                self.array4setsRecord=np.concatenate((self.pd_pm_array, self.array4setswithrotation, self.u_array), axis=None)
                if self.flag_reset==0:
                    self.send_zipped_socket1(self.array4setsRecord)
                    # print round(self.t_new,2),self.pd_pm_array,self.array4setswithrotation
            exit()
        except KeyboardInterrupt:
            exit()
            self.th1_flag=False
            self.th2_flag=False

    def closedLoopControl(self):
        """ Constant"""

        ### replace of t(i)
        t=0.0
        ###
        while t <self.T:
        # try:
            t=time()-self.t0 # sec, from 0 sec
            num_of_segments=1
            # print t
            ### Vector direction in 3D
            e1=np.array([[1],[0],[0]])# 3 by 1
            e2=np.array([[0],[1],[0]])# 3 by 1
            e3=np.array([[0],[0],[1]])# 3 by 1
            ### Geometric
            L0=0.185*num_of_segments # total length m
            r0=0.075*np.sqrt(2)/2 # radius at the cross-section m

            ### need to double check ##
            da=0.0343*0.9 # distance of each actuator tube
            ######
                ### ri is the actuator i distance vector to backbone
            r1= da*np.cos(np.pi/4)*(e1+e2)# 3 by 1
            r2 = da*np.sqrt(2)/2*(e1-e2)# 3 by 1
            r3= da*np.cos(np.pi/4)*(-e1-e2)# 3 by 1
            r4 = da*np.sqrt(2)/2*(-e1+e2)# 3 by 1
            db= L0/2*e3# 3 by 1, distance of the tip force to the COM m
            A0=np.pi*(r0**2) # cicular cross-section area
            rho=792 #density kg/m^3
                ### momentum of inertia
            Ixx=np.pi/4*(r0**4)
            Iyy=np.pi/4*(r0**4)
            Izz=Ixx+Iyy
            J=np.array([[Ixx, 0, 0],[0,Iyy,0],[0,0,Izz]])# inertia tensor
            M=rho*A0*J # stiffness per unit length
            E= 0.27*10**6 # Yong modulus pa
            G=E/(2*(1+0.4))# shear modulus pa
            Kb=np.array([[E*Ixx,0,0],[0,E*Iyy,0],[0,0,G*Izz]])#Stiffness matrix for bending(d1,d2) and extension(d3) [N.m^2]
            tau=0 #ime period of free-end damping in x-y-z axes [s]
            Bb = np.dot(Kb,tau*np.eye(3)) #Damping matrix corresponding to bending and extension [N.m^2.s]
            Force_tip = 0*e3 #zeros(1,3) Force from interaction with environment (FSR) [N]
            # print Force_tip,db
            Moment_tip=np.cross(np.reshape(db,(1,3)),np.reshape(Force_tip,(1,3))).reshape(3,1)#Moment from interaction with the environment (FSR) [Nm]
            g=np.array([[0],[0],[-9.81]]) #gravity accelaration [m/s^2]
            Force_gravity=rho*A0*g #force per unit length at each cross-section [N]
            u_star = np.array([[0],[0],[0]]) #undeformed state of u [1/m]
            a = self.a #amplitude of bending limited by the design and worksapce
            freq = self.freq #frequency of bending limited by air pressure speed [Hz]
            w = 2*np.pi*freq

            ### replace of u(t(i)) desired curvature
            u_desired=np.array([[0],[a*np.sin(w*t)],[0]])
            ut_desired=np.array([[0],[a*w*np.cos(w*t)],[0]])
            utt_desired=np.array([[0],[-a*(w**2)*np.sin(w*t)],[0]])
            self.u_array[0]=u_desired[0][0]
            self.u_array[1]=ut_desired[1][0]
            self.u_array[2]=utt_desired[2][0]
            ###
            R0=np.eye(3)
            ### Gathering Rotation matirx
            q=self.array4setswithrotation[10:14]# 1 by 4, w x y z
            R=self.optitrack_quat2Matrix(q)
            # ### Gathering tip position for 2nd segment
            xi=self.array4setswithrotation[14]
            zi=self.array4setswithrotation[16]
            x0=self.array4setswithrotation[7]
            z0=self.array4setswithrotation[9]
            # ### Gathering tip position for 1st segment
            # xi=self.array4setswithrotation[7]
            # zi=self.array4setswithrotation[9]
            # ### Gathering base position for 1st segment
            # x0=self.array4setswithrotation[0]
            # z0=self.array4setswithrotation[2]
            u_y=0.0
            u_y=2*(xi-x0)/((xi-x0)**2+(zi-z0)**2)
            u=np.array([[0],[u_y],[0]])
            torq_l=np.dot(Kb,(u-u_star))
            K=Kb
            torq_l_desired = np.dot(K,(u_desired-u_star))
            # print "k",K,"u",u_desired-u_star,"t",torq_l_desired
            ##Gathering rotation matrix
            applied_torque_by_actuator= np.dot(R,(torq_l_desired-torq_l+np.dot(M,utt_desired)-Moment_tip))
            # print torq_l_desired,"\n",-torq_l,"\n",+np.dot(M,utt_desired),"\n",-Moment_tip
            ## why they are the same?
            applied_force_by_actuator_1= applied_torque_by_actuator[1][0]/(1*da*np.cos(np.pi/4))
            # applied_force_by_actuator_2= applied_torque_by_actuator[1]/(2*da*np.cos(pi/4))
            applied_pressure_by_actuator_1= applied_force_by_actuator_1/A0; # Pa
            # print applied_torque_by_actuator
            # applied_pressure_by_actuator_2= applied_force_by_actuator_2/A0; # Pa
            ### convert pd (pa) to pd (psi)
            p2_psi=applied_pressure_by_actuator_1*0.000145038
            # print p2_psi
            # p2_psi=applied_pressure_by_actuator_2*0.000145038
            if p2_psi>=20:
                p2_psi=20
            elif p2_psi<=0.0:
                p2_psi=0.0
            p1_psi=0.0
            # if p2_psi>=20:
            #     p2_psi=20.0
            # elif p2_psi<=0.0:
            #     p2_psi=0.0
            pd_array=np.array([p1_psi,p2_psi,0.0])
            print "t",np.round(t,1),np.round(p2_psi,1),"ud",np.round(u_desired[1][0],1),"u",np.round(u[1][0],1)
            sleep(0.01)
            # print pd_array
            self.send_zipped_socket0(pd_array)
        # except KeyboardInterrupt:
        #     exit()
    def optitrack_quat2Matrix(self,q):
        m=np.array([0.0]*9)
        m[0] = 1-2*q[1]*q[1]-2*q[2]*q[2]; m[1] = 2*q[0]*q[1]-2*q[3]*q[2];   m[2] = 2*q[0]*q[2]+2*q[3]*q[1];
        m[3] = 2*q[0]*q[1]+2*q[3]*q[2];   m[4] = 1-2*q[0]*q[0]-2*q[2]*q[2]; m[5] = 2*q[1]*q[2]-2*q[3]*q[0];
        m[6] = 2*q[0]*q[2]-2*q[3]*q[1];   m[7] = 2*q[1]*q[2]+2*q[3]*q[0];   m[8] = 1-2*q[0]*q[0]-2*q[1]*q[1];
        R=np.reshape(m,(3,3))
        return R

    def step_response(self,pd_array,step_time):
            for i in range(int(step_time/0.005)):
                if self.th1_flag:
                    self.send_zipped_socket0(pd_array)
                    sleep(0.005)

    def ramp_response(self,start_array,end_array,ramp_time):
        for i in range(int(ramp_time/0.005)):
            if self.th1_flag:
                pd_array=start_array+(end_array-start_array)/ramp_time*i*0.005
                self.send_zipped_socket0(pd_array)
                sleep(0.005)

    def sine_response(self,A_array,freq_array,B_array,sine_time):
        for i in range(int(sine_time/0.005)):
            if self.th1_flag:
                pd_array=A_array*np.cos(2.0*np.pi*freq_array*0.005*i)+B_array
                self.send_zipped_socket0(pd_array)
                sleep(0.005)

    def sum_of_sine(self,A,B,f_f,f_0,t_total,p23):
        t0=time()
        t_f=t0+t_total
        while time()<t_f:
            t=time()-t0
            f_t=(f_f-f_0)/t_total*t
            p1=A*np.sin(2*np.pi*f_t*t)+B
            pd_array=np.array([p1,p23,p23])
            self.send_zipped_socket0(pd_array)
            # print p1
            sleep(0.005)

    def sum_of_sine2(self,f_f,f_0,t_total):
        t0=time()
        t_f=t0+t_total
        p1=0.
        numOfSines=10
        ftArray=np.linspace(f_0,f_f,num=numOfSines)
        phasArray=2.0*np.pi*np.random.random_sample((numOfSines,))
        while time()<t_f:
            t=time()-t0
            p1=0.
            i=0
            for f_t in ftArray:
                p1=p1+25./numOfSines*np.sin(2*np.pi*f_t*t+phasArray[i])+12.5/numOfSines
                i=i+1
                # print p1
            # p1=2.5*np.sin(2*np.pi*f_t*t)+3.5
            if p1 <=1.0:
                p1 =1.0
            if p1>=25.0:
                p1=25.0
            pd_array=np.array([p1,1.0,1.0])
            self.send_zipped_socket0(pd_array)
            # print 
            sleep(0.005)
            
    def send_zipped_socket0(self, obj, flags=0, protocol=-1):
        """pack and compress an object with pickle and zlib."""
        pobj = pickle.dumps(obj, protocol)
        zobj = zlib.compress(pobj)
        self.socket0.send(zobj, flags=flags)

    def send_zipped_socket1(self, obj, flags=0, protocol=-1):
        """pack and compress an object with pickle and zlib."""
        pobj = pickle.dumps(obj, protocol)
        zobj = zlib.compress(pobj)
        self.socket1.send(zobj, flags=flags)

    def recv_zipped_socket2(self,flags=0):
        """reconstruct a Python object sent with zipped_pickle"""
        zobj = self.socket2.recv(flags)
        pobj = zlib.decompress(zobj)
        return pickle.loads(pobj)

    def recv_zipped_socket3(self,flags=0):
        """reconstruct a Python object sent with zipped_pickle"""
        zobj = self.socket3.recv(flags)
        pobj = zlib.decompress(zobj)
        return pickle.loads(pobj)

    def recv_cpp_socket2(self):
        strMsg =self.socket2.recv()
        # floatArray=np.fromstring(strMsg)
        return np.fromstring(strMsg, dtype=float, sep=' ')

