"""
#Fix 08-12-2021 
#Cam Stream is Z-up
#Phi is calculated from -pi to pi
#Add self.p1_off_set_angle
#Use np.remainder function to estiamte r0
#Change tau_x, tau_y, tau_theta, tau_phi calculation
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
        self.p1_off_set_angle=np.deg2rad(240) # rad
        self.l_tri= 0.07 #m
        """ Initiate ZMQ communication"""
        context = zmq.Context()
        self.socket0 = context.socket(zmq.PUB)
        self.socket0.setsockopt(zmq.CONFLATE,True)
        self.socket0.bind("tcp://10.203.53.226:4444")## PUB pd to Raspi Client

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
        self.array2setsRecord=np.array([0.]*41)#t pd1 pd2 pd3 + pm1 +pm2 +pm3 + positon +orintation
        self.pd_pm_array=np.array([0.]*6) #pd1 pd2 pd3 + pm1 +pm2 +pm3 (psi)
        self.array2setswithrotation=np.array([0.]*14)# base(x y z qw qx qy qz) top(x1 y1 z1 qw1 qx1 qy1 qz1)
        self.tau_array=np.array([0.]*4)#tau_x, tau_y. tau_ theta, tau_phi
        self.state_variable_array=np.array([0.0]*4)# theta, dtheta, phi, dphi

        """ Thearding Setup """
        self.th1_flag=True
        self.th2_flag=True
        self.flag_reset=1
        self.run_event=threading.Event()
        self.run_event.set()
        self.th1=threading.Thread(name='raspi_client',target=self.th_pub_raspi_client_pd)
        self.th2=threading.Thread(name='mocap',target=self.th_sub_pub_mocap)
        """State variable Ini """
        self.x1_old=0.0#theta rad
        self.x1_t0=0.0 #theta rad
        self.x1_current=0.0
        self.x2_current=0.0 # d_theta/dt rad/s
        self.x3_old=0.0# phi rad
        self.x3_t0=0.0# phi rad
        self.x3_current=0.0
        self.x4_current=0.0 # d_phi/dt rad/s
        self.t0=0.0 # timer sec
        self.t_old=0.0 #timer sec
        self.t_new=0.0
        self.loop_timer=time()


    def th_pub_raspi_client_pd(self):
        try:
            if self.flag_reset==1:
                self.step_response(np.array([1.0,1.0,1.0]),5)
                self.flag_reset=0
            if self.flag_use_mocap == True:
                self.array2setswithrotation=self.recv_cpp_socket2()
            vector_phiTheta=np.array([0., 0.])
            vector_phiTheta=self.getThetaPhiAndr0FromXYZ()
            self.x1_old=vector_phiTheta[1]
            self.x1_t0=vector_phiTheta[1]
            self.x3_old=vector_phiTheta[0]
            self.x3_t0=vector_phiTheta[0]
            self.t0=time()
            self.t_old=time()
            for p1 in range(3,40,2):
                self.loop_timer=time()
                self.openloopStepPressureCtrl(np.array([p1,1.0,1.0]),4)
            for p12 in range(3,40,2):
                self.loop_timer=time()
                self.openloopStepPressureCtrl(np.array([p12,p12,1.0]),4)
            for p13 in range(3,40,2):
                self.loop_timer=time()
                self.openloopStepPressureCtrl(np.array([p13,1.0,p13]),4)
            print "Done"
            
            self.step_response(np.array([1.0,1.0,1.0]),5)
            self.th1_flag=False
            self.th2_flag=False
            exit()
        except KeyboardInterrupt:
            self.th1_flag=False
            self.th2_flag=False
            print "Press Ctrl+C to Stop"
            
    def th_sub_pub_mocap(self):# thread config of read data from mocap and send packed msg to record file.
        try:
            while self.run_event.is_set() and self.th2_flag:
                if self.flag_use_mocap == True:
                    self.array2setswithrotation=self.recv_cpp_socket2()
                self.pd_pm_array=self.recv_zipped_socket3()
                self.array2setsRecord=np.concatenate((self.pd_pm_array, self.array2setswithrotation, self.tau_array, self.state_variable_array), axis=None)
                if self.flag_reset==0:
                    self.send_zipped_socket1(self.array2setsRecord)
                    print round(self.t_new,2),self.pd_pm_array
                # sleep(0.005)
            exit()
        except KeyboardInterrupt:
            exit()
            self.th1_flag=False
            self.th2_flag=False

    def getThetaPhiAndr0FromXYZ(self):
        #Fix 08-12-2021 Cam Stream is Z-up
        #Fix 08-12-2021 Phi is calculated from -pi to pi
        # get raw top(x,y,z) bottom (x,y,z)
        vector_base=np.array([0., 0., 0.])
        vector_top=np.array([0., 0., 0.])
        vector_base=self.array2setswithrotation[0:3]# base(x,y,z)
        vector_top=self.array2setswithrotation[7:10]-vector_base# top(x,y,z)-base(x,y,z)
        # print"v_tip",vector_top
        # Rotate to algorithm frame Rz with -90 deg  Rx=([1.0, 0.0, 0.0],[0.0, 0.0, 1.0],[0.0, -1.0, 0.0])
        tip_camFrame=np.array([0., 0., 0.])
        tip_camFrame[0]=vector_top[0] # camFrame x
        tip_camFrame[1]=vector_top[1] #camFrame  y
        tip_camFrame[2]=vector_top[2] # camFrame z

        # Calculate phi rad in [-pi,pi]
        # phi_rad=0.
        phi_rad=np.arctan(tip_camFrame[1]/tip_camFrame[0])

        #Calculate theta rad using xyz
        theta_rad = 2.0*np.sign(tip_camFrame[2])*np.arccos(tip_camFrame[2]/np.sqrt(tip_camFrame[0]*tip_camFrame[0] + tip_camFrame[0]*tip_camFrame[0] + tip_camFrame[0]*tip_camFrame[0]))      
        return np.array([phi_rad,theta_rad])

    def getr0fromPhi(self,tip_camFrame,phi_rad):
        # par_set.trianlge_length/sqrt(3)*cos(pi/3)./cos(mod((trainSet.phi_rad)+p1_offset,2*pi/3)-pi/3);
        r0=self.l_tri/np.sqrt(3)*np.cos(np.pi/3)/np.cos(np.remainder(phi_rad+self.p1_off_set_angle,2*np.pi/3) - np.pi/3)
        return r0

    def openloopStepPressureCtrl(self,pd_array,step_time):
        while time()-self.loop_timer<= step_time:
            vector_phiTheta=np.array([0., 0.])
            phi=0.
            theta=0.
            dtheta=0.
            dphi=0.
            pm1_MPa=self.pd_pm_array[3]
            pm2_MPa=self.pd_pm_array[4]
            pm3_MPa=self.pd_pm_array[5]
            vector_phiTheta=self.getThetaPhiAndr0FromXYZ()
            phi=vector_phiTheta[0]
            theta=vector_phiTheta[1]
            #Update State variables x1,x2, x1 error,
            self.t_new=time()-self.t0
            self.x1_current=theta
            self.x3_current=phi
            self.x2_current=(self.x1_current-self.x1_old)/(self.t_new-self.t_old)
            self.x4_current=(self.x3_current-self.x3_old)/(self.t_new-self.t_old)

            cphi= np.cos(phi)
            sphi= np.sin(phi)
            stheta=np.sin(theta)
            # when p1_offset_angle is 240
            tau_x=-(pm3_MPa*(0.5*np.sqrt(3))+ pm1_MPa*(-0.5*np.sqrt(3)))
            tau_y=-(pm1_MPa*(-0.5)+pm2_MPa*(1.0)+pm3_MPa*(-0.5))

            tau_theta=-cphi*stheta*tau_x + (-cphi)*tau_y
            tau_phi=-sphi*stheta*tau_x+cphi*tau_y
            self.tau_array=np.array([tau_x,tau_y,tau_theta,tau_phi])
            self.state_variable_array=np.array([self.x1_current,self.x2_current,self.x3_current,self.x4_current])

            self.x1_old=self.x1_current
            self.x3_old=self.x3_current
            self.t_old=self.t_new
            # print pd_array
            self.send_zipped_socket0(pd_array)
            sleep(0.005)


"""Utility Functions"""

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

