"""
This code is the PC Client
normal to compliant to recovery 
recorvery to compliant


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

        self.flag_control_mode=0# 0: baseline smc; 
                                # 1: smcndo;
                                # 2: smart;
        self.flag_contact_mode=0 
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
        self.smc_tracking=np.array([0.]*2)#xd,x1
        self.smcNDOB_tracking=np.array([0.]*9)#p1_ub, u_total, u_eq, u_s, u_n, disturbance_tqr,xd_new, ctrl policy,inner loop dist_tqr
        """ Thearding Setup """
        self.th1_flag=True
        self.th2_flag=True
        self.run_event=threading.Event()
        self.run_event.set()
        self.th1=threading.Thread(name='raspi_client',target=self.th_pub_raspi_client_pd)
        self.th2=threading.Thread(name='mocap',target=self.th_sub_pub_mocap)

        """Initialize SMC Parameter """
        # Actuator geometic parameters
        self.m0=0.35# segment weight kg
        self.g=9.8  # gravity m/s**2
        self.L=0.185# segment length m
        self.triangleEdgeLength= 0.07 # m
        self.actuatorWidth= 0.015 # m
        self.R_f= np.sqrt(3.0)/6*self.triangleEdgeLength+self.actuatorWidth # distribution radius of force
        self.offsetAngle_p1=np.radians(150) # deg2rad(150)
        self.edge_pt1=np.array([self.R_f*np.cos(self.offsetAngle_p1-np.pi/3), self.R_f*np.sin(self.offsetAngle_p1-np.pi/3), 0.])
        self.edge_pt2=np.array([self.R_f*np.cos(self.offsetAngle_p1+np.pi/3), self.R_f*np.sin(self.offsetAngle_p1+np.pi/3), 0.])
        self.edge_pt3=np.array([self.R_f*np.cos(self.offsetAngle_p1-np.pi), self.R_f*np.sin(self.offsetAngle_p1-np.pi), 0.])
        self.r0=0.
        # SMC state variable and time stamps
        self.x1_old=0. # state for last iteration
        self.x1_t0=0.
        self.x2_old=0. # state for last iteration
        self.x1_current=0. # state for current iteration
        self.x2_current=0. # state for current iteration
        self.xd1=0.
        self.xd_new=0.
        self.x1_error_old=0.   # error for last iteration
        self.x1_error_current=0.   # error for current iteration
        self.x1_dot_error=0. # error derivative for current iteration
        self.t0=time()
        self.t_old=0.
        self.t_new=0.

        """ILC Parameters"""
        self.ilc_max_iteration=1
        self.ilc_memory_length=6000
        self.ilc_memory=np.array([0.0]*self.ilc_memory_length)
        self.ilc_iteration_index=0
        self.ilc_kp=1.0
        self.ilc_delta_t=0.002
        self.ilc_exp_delta_t=0.01
        """Input signal selection"""
        self.positionProfile_flag=3#  0: sum of sine waves 1: single sine wave, 2: step 3:ramp
        self.trailDuriation=60.0#sec
        # Input sine wave parameters
        self.Amp=np.radians(5)
        self.Boff=np.radians(-40)
        self.Freq=0.1 # Hz
        # Input sum of sine waves
        self.sum_sin_freq_low=0.001
        self.sum_sin_freq_high=0.1
        self.sum_sin_amp=np.radians(1.)
        self.sum_sin_boff=np.radians(-3.)
        self.numOfSines=10
        self.ftArray=np.linspace(self.sum_sin_freq_low,self.sum_sin_freq_high,num=self.numOfSines)
        self.phasArray=2.0*np.pi*np.random.random_sample((self.numOfSines,))
        # Input MultiStep
        self.numOfSteps=5
        self.timeStampSteps=np.linspace(0.0,self.trailDuriation,num=self.numOfSteps)
        indexStampSteps=np.linspace(0,self.ilc_memory_length,num=self.numOfSteps)
        self.indexStampSteps=indexStampSteps.astype(np.int)
        self.multiStepAmps= np.radians(-25)*np.random.random_sample((self.numOfSteps,))+np.radians(-10)
        # Input Ramp from t0 position
        self.rampRateAbs=np.radians(1.) # 1 deg/sec
        self.rampAmpAbs=np.radians(40) # x1(t0)-rampAmp
        self.rampFlatTime=5.0 # sec
        self.flag_ramp_phase=0
        # SMC model uncertainty
        self.alpha0=1.1055 # scaler of torque
        self.k0=0.4413 # Nm/rad
        self.b0=0.7535 # Nm/(rad/s)
        self.delta_k_max=0.3926 # bonded uncertainty for k
        self.delta_b_max=0.5758 # bonded uncertainty for b
        self.delta_alpha_max=0.8043 # alpha=(1+delta_alpha)*alpha0
        self.input_pressure_limit_psi= 40. # input limit of pressure psi
        self.tau_max=0.0 #N.m
        # SMC control gain selection
        # Baseline with centain alpha
        self.smc_lambda=10. # sliding surface gain
        self.smc_eta=10. # reaching speed of sliding surface
        # Input uncertainty with sat bound
        self.smc_lambda=10. # sliding surface gain
        self.smc_eta=10. # Initial values of eta, will be updated throug algorithm
        self.smc_epsilon=10.# reaching speed
        self.smc_sat_bound=np.radians(1)# saturation bound of sliding surface
        """SMCSPO controller design"""
        """Tuning section"""
        self.smcspo_lambda=50.
        self.smcspo_epsilon_o=0.01
        self.smcspo_epsilon_s=1.0
        self.smcspo_alpha_1= 0.01
        self.smcspo_alpha_2= 0.01
        """Calculated values """
        self.smcspo_k1=3*self.smcspo_lambda*self.smcspo_epsilon_o
        self.smcspo_k2=self.smcspo_lambda*self.smcspo_k1
        self.smcspo_k1_epsilon_o=3*self.smcspo_lambda
        self.smcspo_k2_epsilon_o=3*self.smcspo_lambda*self.smcspo_lambda
        self.smcspo_alpha_3=np.sqrt(self.smcspo_lambda/3.0)
        self.smcspo_c=self.smcspo_lambda
        self.smcspo_eta=self.smcspo_lambda*self.smcspo_epsilon_o
        self.x1_hat=0.
        self.x2_hat=0.
        self.x3_hat=0.
        self.x1_e=0. # x1_e = x1_hat-xd
        self.x2_e=0. # x2_e = x2_hat-dxd
        self.dx1_hat=0. # dx1_hat = x2_hat - smcspo_k1*sat(x1_e)-smcspo_alpha_1*x1_e
        self.dx2_hat=0. # dx2_hat = smcspo_alpha_3*u_bar - smcspo_k2*sat(x1_e)-smcspo_alpha_2*x1_e + smcspo_per_est
        self.dx3_hat=0. # dx3_hat = smcspo_alpha_3**2 * (-x3_hat + smcspo_alpha_3*x2_hat + u_bar)
        """ SMC+NDOB design"""
        self.smcNDOB_dist_est=0.
        self.smcNDOB_dist_est_tqr=0.
        self.smcNDOB_dist_est_inner=0.
        self.smcNDOB_th=1.0
        self.flag_first_contact =0
        self.smcNDOB_lambda= 1.0
        self.smcNDOB_k_sig = 1.0
        self.smcNDOB_eta = 1.0
        self.smcNDOB_kp_sig = 5.0
        self.smcNDOB_z_current=0.
        self.smcNDOB_z_current_inner=0.
        self.smcNDOB_z_old=0.
        self.smcNDOB_z_old_inner=0.
        self.smcNDOB_dz=0.
        self.smcNDOB_dz_inner=0.
        self.smcNDOB_sat_bound=1.0
        self.smcNDOB_p1=0.0
        self.smcNDOB_u_total=0.0
        self.smcNDOB_u_eq=0.0
        self.smcNDOB_u_s=0.0
        self.smcNDOB_u_n=0.0
        self.u_fix_value=0.
        self.x1_error_max_value=np.deg2rad(3.0)
        self.xd1_new=0.
        self.x1_new_error=0.
        self.flag_stage=0
        self.smcNDOB_dist_est_inner_tqr=0.
        self.timer_attampt =2.0 
        self.timer_th=100.0
        self.flag_reset=1
        self.mode_C_to_R_th=0.5
        self.mode_R_to_C_th=0.3
        self.xdc=0.
        # self.step_response(np.array([5.0,1.0,1.0]),5)

    def th_pub_raspi_client_pd(self):
        try:
            if self.flag_reset==1:
                self.step_response(np.array([3.0,1.0,1.0]),5)
                self.flag_reset=0
            if self.flag_use_mocap == True:
                self.array2setswithrotation=self.recv_cpp_socket2()
            vector_phiTheta=np.array([0., 0.])
            vector_phiTheta=self.getThetaPhiAndr0FromXYZ()
            self.x1_old=vector_phiTheta[1]
            self.x1_t0=vector_phiTheta[1]
            # self.pd_pm_array=self.recv_zipped_socket3()
            # print 1
            """Input Signal selection"""
            self.t0=time()
            self.t_old=time()-self.t0
            if self.positionProfile_flag==3: # ramp duriation 
                self.trailDuriation= (self.rampAmpAbs/self.rampRateAbs)*2.0+self.rampFlatTime*2
            if self.flag_control_mode == 0: # SMC with input bound
                self.xd1 =self.positionProfileSelection()
                print self.xd1
                self.x1_error_old=self.xd1-self.x1_old
                while self.t_old<=self.trailDuriation:
                    self.calculateControlInputWithInputBoundAndSaturation()
                    print "time",np.round(self.t_old,4),"xd",np.round(np.rad2deg(self.xd1),2),"error(deg)",np.round(np.rad2deg(self.x1_error_old),2),'pd',np.round(self.pd_pm_array[0],2),'pm',np.round(self.pd_pm_array[3],2)
            elif self.flag_control_mode == 1: # p type ILC with SMC
                self.xd1 =self.positionProfileSelectionDiscreteDomain()
                self.x1_error_old=self.xd1-self.x1_old
                for i in range(self.ilc_max_iteration):
                    vector_phiTheta=np.array([0., 0.])
                    vector_phiTheta=self.getThetaPhiAndr0FromXYZ()
                    self.x1_old=vector_phiTheta[1]
                    """Input Signal selection"""
                    self.t0=time()
                    self.t_old=time()-self.t0
                    self.xd1 =self.positionProfileSelectionDiscreteDomain()
                    self.x1_error_old=self.xd1-self.x1_old
                    for self.ilc_iteration_index in range(self.ilc_memory_length):
                        # print self.ilc_iteration_index                                
                        self.pTypeIlc_smcWithInputBoundAndSatDiscrete()
                        print "time",np.round(self.t_old,4),"Iteration",i,"index",self.ilc_iteration_index,"xd",np.round(np.rad2deg(self.xd1),2),"error(deg)",np.round(np.rad2deg(self.x1_error_old),2),'pd',np.round(self.pd_pm_array[0],2),'pm',np.round(self.pd_pm_array[3],2)
                    print "Reset"
            elif self.flag_control_mode == 2: # SMC SPO 
                self.xd1 =self.positionProfileSelection()
                self.x1_error_old=self.xd1-self.x1_old
                self.x1_hat=self.x1_old
                self.x2_hat=self.x2_old
                while self.t_old<=self.trailDuriation:
                    self.smcspoWithInputBoundAndSat()
                    # print "time",np.round(self.t_old,4),"xd",np.round(np.rad2deg(self.xd1),2),"error(deg)",np.round(np.rad2deg(self.x1_error_old),2),'pd',np.round(self.pd_pm_array[0],2),'pm',np.round(self.pd_pm_array[3],2)
            elif self.flag_control_mode == 3: # SMCNDO baseline
                self.xd1 =self.positionProfileSelection()
                print self.xd1
                self.x1_error_old=self.xd1-self.x1_old
                while self.t_old<=self.trailDuriation:
                    self.smcNDOBwithInputBoundAndSat()
                    print "time",np.round(self.t_old,1),"C?",self.flag_stage,"xdn:",np.round(np.rad2deg(self.xd_new),1),"xdt(deg)",np.round(np.rad2deg(self.xd1),1),"et(deg)",np.round(np.rad2deg(self.x1_error_old),1),"en(deg)",np.round(np.rad2deg(self.x1_new_error),1),'pd_ub',np.round(self.smcNDOB_p1,1),'pm',np.round(self.pd_pm_array[3],1),'d',np.round(self.smcNDOB_dist_est_inner_tqr,2)
                    # print 'u_total',np.round(self.smcNDOB_u_total,2),'u_eq',np.round(self.smcNDOB_u_eq,2),'u_s',np.round(self.smcNDOB_u_s,2),'u_n',np.round(self.smcNDOB_u_n,2)               
            # elif self.flag_control_mode == 4: # SMCNDO constrained contact
            #     self.xd1 =self.positionProfileSelection()
            #     print self.xd1
            #     self.x1_error_old=self.xd1-self.x1_old
            #     while self.t_old<=self.trailDuriation:
            #         self.smcNDOBwithInputBoundAndSat()
            #         print "time",np.round(self.t_old,1),"x1",np.round(np.rad2deg(self.xd1),1),"x1e(deg)",np.round(np.rad2deg(self.x1_error_old),1),'pd_ub',np.round(self.smcNDOB_p1,1),'pm',np.round(self.pd_pm_array[3],1),'d',np.round(self.smcNDOB_dist_est,4)
            # self.step_response(np.array([1.0,1.0,1.0]),5)
            self.th1_flag=False
            self.th2_flag=False
            print "Done"
            exit()
        except KeyboardInterrupt:
            self.th1_flag=False
            self.th2_flag=False
            print "Press Ctrl+C to Stop"
        #     print "Press Ctrl+C to Stop"
            
    def th_sub_pub_mocap(self):# thread config of read data from mocap and send packed msg to record file.
        try:
            while self.run_event.is_set() and self.th2_flag:
                if self.flag_use_mocap == True:
                    self.array2setswithrotation=self.recv_cpp_socket2()
                self.pd_pm_array=self.recv_zipped_socket3()
                self.smc_tracking=np.array([self.xd1,self.x1_current])
                self.smcNDOB_tracking=np.array([self.smcNDOB_p1,self.smcNDOB_u_total,self.smcNDOB_u_eq,self.smcNDOB_u_s,self.smcNDOB_u_n,self.smcNDOB_dist_est_tqr,self.xd_new,self.flag_stage,self.smcNDOB_dist_est_inner_tqr])
                self.array2setsRecord=np.concatenate((self.pd_pm_array, self.array2setswithrotation, self.smc_tracking,self.smcNDOB_tracking), axis=None)
                if self.flag_reset==0:
                    self.send_zipped_socket1(self.array2setsRecord)
                # sleep(0.005)
            exit()
        except KeyboardInterrupt:
            exit()
            self.th1_flag=False
            self.th2_flag=False

    def getThetaPhiAndr0FromXYZ(self):
        # get raw top(x,y,z) bottom (x,y,z)
        vector_base=np.array([0., 0., 0.])
        vector_top=np.array([0., 0., 0.])
        vector_base=self.array2setswithrotation[0:3]# base(x,y,z)
        vector_top=self.array2setswithrotation[7:10]-vector_base# top(x,y,z)-base(x,y,z)
        # print"v_tip",vector_top
        # Rotate to algorithm frame Rz with -90 deg  Rx=([1.0, 0.0, 0.0],[0.0, 0.0, 1.0],[0.0, -1.0, 0.0])
        tip_camFrame=np.array([0., 0., 0.])
        tip_camFrame[0]=vector_top[0] # camFrame x
        tip_camFrame[1]=-vector_top[2] #camFram -z
        tip_camFrame[2]=vector_top[1] # camFrame y

        # calculate phi rad (0, 2pi)
        phi_rad=0.
        if tip_camFrame[0] > 0.:
            phi_rad=np.arctan(tip_camFrame[1]/tip_camFrame[0])
        elif tip_camFrame[0] == 0. and tip_camFrame[1] > 0.:
            phi_rad=np.pi/2
        elif tip_camFrame[0] == 0. and tip_camFrame[1] < 0.:
            phi_rad=-np.pi/2
        elif tip_camFrame[0] == 0. and tip_camFrame[1] == 0.:
            phi_rad=0.
        elif tip_camFrame[0] < 0. and tip_camFrame[1] >= 0.:
            phi_rad=np.pi + np.arctan(tip_camFrame[1]/tip_camFrame[0])
        elif tip_camFrame[0] < 0. and tip_camFrame[1] < 0.:
            phi_rad=-np.pi + np.arctan(tip_camFrame[1]/tip_camFrame[0])
        if phi_rad <0.:
            phi_rad=phi_rad+2.0*np.pi
        # calculate r0
        self.r0=self.getr0fromPhi(tip_camFrame,phi_rad)

        # Rotate from CamFrame to BaseFrame with Rz(phi) and Rz2=[1 0 0;0 0 -1; 0 1 0]
        cphi= np.cos(phi_rad)
        sphi= np.sin(phi_rad)
        tip_baseFrame=np.array([0., 0., 0.])
        tip_baseFrame[0]=cphi*tip_camFrame[0]+sphi*tip_camFrame[1] # x_new=c*x + s*y
        tip_baseFrame[1]=tip_camFrame[2]# y_new= z
        tip_baseFrame[2]=sphi*tip_camFrame[0]-cphi*tip_camFrame[1] # z_new= s*x -c*y
        # print tip_baseFrame
        # calculate theta rad theta=2*sign(x)*arcsin(norm(x)/sqrt(x**2+y**2))
        # print "num and dem",np.absolute(tip_baseFrame[0]),np.sqrt((tip_baseFrame[0])*(tip_baseFrame[0])+(tip_baseFrame[1])*(tip_baseFrame[1]))
        theta_rad=(2.0*-np.sign(tip_baseFrame[0])*np.arcsin(np.absolute(tip_baseFrame[0])/np.sqrt((tip_baseFrame[0])*(tip_baseFrame[0])+(tip_baseFrame[1])*(tip_baseFrame[1]))))
        # theta_rad=-np.sign(tip_baseFrame[0])
        # print theta_rad
        return np.array([phi_rad,theta_rad])

    def getr0fromPhi(self,tip_camFrame,phi_rad):
        # calculate intersection point on each edge
        beta_array=np.array([0., 0., 0.])
        if (tip_camFrame[0]*(self.edge_pt2[1]-self.edge_pt1[1])-tip_camFrame[1]*(self.edge_pt2[0]-self.edge_pt1[0]))==0.:
            beta_array[0]=100
        else:
            beta_array[0]=((self.edge_pt1[0]*self.edge_pt2[1]-self.edge_pt2[0]*self.edge_pt1[1])/
                            (tip_camFrame[0]*(self.edge_pt2[1]-self.edge_pt1[1])-tip_camFrame[1]*(self.edge_pt2[0]-self.edge_pt1[0])))
        if (tip_camFrame[0]*(self.edge_pt3[1]-self.edge_pt1[1])-tip_camFrame[1]*(self.edge_pt3[0]-self.edge_pt1[0]))==0.:
            beta_array[1]=100
        else:
            beta_array[1]=((self.edge_pt1[0]*self.edge_pt3[1]-self.edge_pt3[0]*self.edge_pt1[1])/
                            (tip_camFrame[0]*(self.edge_pt3[1]-self.edge_pt1[1])-tip_camFrame[1]*(self.edge_pt3[0]-self.edge_pt1[0])))
        if (tip_camFrame[0]*(self.edge_pt2[1]-self.edge_pt3[1])-tip_camFrame[1]*(self.edge_pt2[0]-self.edge_pt3[0]))==0.:
            beta_array[2]=100
        else:
            beta_array[2]=((self.edge_pt3[0]*self.edge_pt2[1]-self.edge_pt2[0]*self.edge_pt3[1])/
                            (tip_camFrame[0]*(self.edge_pt2[1]-self.edge_pt3[1])-tip_camFrame[1]*(self.edge_pt2[0]-self.edge_pt3[0])))

        # calculate r0 in base frame
        r_beta_array=np.array([0.,0.,0.])
        r0=0.
        for index_i in range(3): # find r_i= norm(beta_i*xt,beta_i*yt)
            r_beta_array[index_i]= np.sqrt((beta_array[index_i])*(beta_array[index_i])*((tip_camFrame[0])*(tip_camFrame[0])+(tip_camFrame[1])*(tip_camFrame[1])))
            if r_beta_array[index_i] <= self.triangleEdgeLength/np.sqrt(3.0):
                r0_x=(beta_array[index_i])*(tip_camFrame[0])
                r0_y=(beta_array[index_i])*(tip_camFrame[1])
                cphi= np.cos(phi_rad)
                sphi= np.sin(phi_rad)
                r0=cphi*r0_x+sphi*r0_y
        return r0

    def calculateControlInputWithInputBoundAndSaturation(self):
        # self.array2setswithrotation=self.recv_cpp_socket2()
        # variable ini.
        vector_phiTheta=np.array([0., 0.])
        phi=0.
        theta=0.
        dtheta=0.
        pm1_MPa=0.
        pm2_MPa=0.
        pm3_MPa=0.
        uMax=0.
        s=0.
        Izz=0.
        M=0.
        C=0.
        G=0.
        f=0.
        uAlpha=0.
        ddxd1=0.
        pd_array=np.array([0.,0.,0.])
        uMin=0.
        sat=0.
        # Update pm1, pm2, pm3, theta, r0, uMax, Ddot(error)
        vector_phiTheta=self.getThetaPhiAndr0FromXYZ()
        phi=vector_phiTheta[0]
        theta=vector_phiTheta[1]
        pm1_MPa=self.pd_pm_array[3]*0.00689476
        pm2_MPa=self.pd_pm_array[4]*0.00689476
        pm3_MPa=self.pd_pm_array[5]*0.00689476
        uMax=(np.absolute(self.alpha0 * (np.sin(phi) * (0.5*self.input_pressure_limit_psi*0.00689476 + 0.5*pm2_MPa - pm3_MPa)
            -np.sqrt(3.0) * np.cos(phi) * (0.5*self.input_pressure_limit_psi*0.00689476 - 0.5*pm2_MPa)))
        )
        #Update State variables x1,x2, x1 error,
        self.t_new=time()-self.t0
        self.x1_current=theta
        self.xd1=self.positionProfileSelection()
        self.x1_error_current=self.xd1-self.x1_current
        self.x2_current=(self.x1_current-self.x1_old)/(self.t_new-self.t_old)
        dtheta=self.x2_current
        self.x1_dot_error=(self.x1_error_current-self.x1_error_old)/(self.t_new-self.t_old)
        # Update SMC 
        s=self.smc_lambda*self.x1_error_current+self.x1_dot_error
        # Update M,C,G
        Izz=self.m0*self.r0*self.r0
        if theta==0.:
            M=self.m0*(self.L/2)**2
            C=0.
            G=-self.g*self.m0
        else:
            M=(Izz/4 + self.m0*((np.cos(theta/2)*(self.r0 - self.L/theta))/2 +
                (self.L*np.sin(theta/2))/theta**2)**2 + (self.m0*np.sin(theta/2)**2*(self.r0 - self.L/theta)**2)/4
            )
            C=(-(self.L*dtheta*self.m0*(2*np.sin(theta/2) - theta*np.cos(theta/2))*(2*self.L*np.sin(theta/2)
                - self.L*theta*np.cos(theta/2) + self.r0*theta**2*np.cos(theta/2)))/(2*theta**5)
            )
            G=(-(self.g*self.m0*(self.L*np.sin(theta) + self.r0*theta**2*np.cos(theta) - self.L*theta*np.cos(theta)))/(2*theta**2)
            )
        # Updata f(x1,x2)
        f=(1.0/M) * (-self.k0* self.x1_current- (self.b0+C)* self.x2_current- G)
        # Update uAlpha
        ddxd1=self.positionDoubleDerevativeSelection()
        # saturation bound for sliding surface
        if np.absolute(s/self.smc_sat_bound) <=1:
            sat= s/self.smc_sat_bound
        else:
            sat= np.sign(s)
        # Update B_star
        B_star=self.delta_alpha_max  
        # Update eta
        self.smc_eta=1/(1+B_star)*(B_star*np.absolute(self.smc_lambda*self.x1_dot_error+ ddxd1 - f)+self.delta_k_max*np.absolute(self.x1_current/M) + self.delta_b_max*np.absolute(self.x2_current/M) + self.tau_max*np.absolute(1/M))
        # Update uAlpha
        uAlpha=M/self.alpha0*(self.smc_lambda*self.x1_dot_error+ ddxd1 - f +self.smc_eta*sat)
        # uAlpha=(M*(self.smc_lambda*self.x1_dot_error+ ddxd1 - f + self.smc_eta* np.sign(s)
        #             +np.sign(s)*(self.delta_k_max*np.absolute(self.x1_current/M) + self.delta_b_max*np.absolute(self.x2_current/M)))
        #         )
        # Constrain uMin<=uAlpha<= uMax
        cphi= np.cos(phi)
        sphi= np.sin(phi)
        p1_MPa=(uAlpha/self.alpha0-(0.5*sphi+0.5*np.sqrt(3)*cphi)*pm2_MPa+sphi*pm3_MPa)/(0.5*sphi-0.5*np.sqrt(3)*cphi)
        p1_psi=p1_MPa*145.038
        if p1_psi>=self.input_pressure_limit_psi:
            pd_array=np.array([self.input_pressure_limit_psi,1.0,1.0])
        elif p1_psi<=1.0:
            pd_array=np.array([1.0,1.0,1.0])
        else:
            pd_array=np.array([p1_psi,1.0,1.0])
        self.x1_old=self.x1_current
        self.x1_error_old=self.x1_error_current
        self.t_old=self.t_new
        # print pd_array
        self.send_zipped_socket0(pd_array)

    def smcNDOBwithInputBoundAndSat(self):
        # self.array2setswithrotation=self.recv_cpp_socket2()
        # variable ini.
        vector_phiTheta=np.array([0., 0.])
        phi=0.
        theta=0.
        dtheta=0.
        pm1_MPa=0.
        pm2_MPa=0.
        pm3_MPa=0.
        uMax=0.
        s=0.
        Izz=0.
        M=0.
        C=0.
        G=0.
        f=0.
        uAlpha=0.
        u_total=0.
        u_fix_value=0.
        ddxd1=0.
        pd_array=np.array([0.,0.,0.])
        uMin=0.
        sat_s=0.
        # Update pm1, pm2, pm3, theta, r0, uMax, Ddot(error)
        vector_phiTheta=self.getThetaPhiAndr0FromXYZ()
        phi=vector_phiTheta[0]
        theta=vector_phiTheta[1]
        pm1_MPa=self.pd_pm_array[3]*0.00689476
        pm2_MPa=self.pd_pm_array[4]*0.00689476
        pm3_MPa=self.pd_pm_array[5]*0.00689476
        uMax=(np.absolute(self.alpha0 * (np.sin(phi) * (0.5*self.input_pressure_limit_psi*0.00689476 + 0.5*pm2_MPa - pm3_MPa)
            -np.sqrt(3.0) * np.cos(phi) * (0.5*self.input_pressure_limit_psi*0.00689476 - 0.5*pm2_MPa)))
        )
        #Update State variables x1,x2, x1 error,
        self.t_new=time()-self.t0
        self.x1_current=theta
        self.xd1=self.positionProfileSelection()
        self.xd_new=self.xd1
        self.x1_error_current=self.x1_current-self.xd1
        self.x2_current=(self.x1_current-self.x1_old)/(self.t_new-self.t_old)
        dtheta=self.x2_current

        # Update M,C,G
        Izz=self.m0*self.r0*self.r0
        if theta==0.:
            M=self.m0*(self.L/2)**2
            C=0.
            G=-self.g*self.m0
        else:
            M=(Izz/4 + self.m0*((np.cos(theta/2)*(self.r0 - self.L/theta))/2 +
                (self.L*np.sin(theta/2))/theta**2)**2 + (self.m0*np.sin(theta/2)**2*(self.r0 - self.L/theta)**2)/4
            )
            C=(-(self.L*dtheta*self.m0*(2*np.sin(theta/2) - theta*np.cos(theta/2))*(2*self.L*np.sin(theta/2)
                - self.L*theta*np.cos(theta/2) + self.r0*theta**2*np.cos(theta/2)))/(2*theta**5)
            )
            G=(-(self.g*self.m0*(self.L*np.sin(theta) + self.r0*theta**2*np.cos(theta) - self.L*theta*np.cos(theta)))/(2*theta**2)
            )
        # Updata f(x1,x2)
        b_x=(self.alpha0/M) 
        f_x=(self.k0* self.x1_current + (self.b0+C)* self.x2_current + G)/M

        if self.flag_contact_mode==0: # normal smcndo
            # Update double dot of xd
            dxd1=self.positionSingleDerevativeSelection()
            ddxd1=self.positionDoubleDerevativeSelection()
            # Update SMC 
            s=self.smcNDOB_lambda*self.x1_error_current+self.x2_current-dxd1
            # saturation bound for sliding surface
            if np.absolute(s/self.smcNDOB_sat_bound) <=1:
                sat_s= s/self.smcNDOB_sat_bound
            else:
                sat_s= np.sign(s)
            # Update u_eq,u_s,u_n
            u_eq = -1.0/b_x*(f_x+self.smcNDOB_lambda*(self.x2_current-dxd1)-ddxd1 + self.smcNDOB_k_sig*s)
            u_s =  -1.0/b_x*self.smcNDOB_eta*sat_s
            u_n = -1.0/b_x*self.smcNDOB_dist_est
            self.smcNDOB_u_eq=u_eq
            self.smcNDOB_u_s=u_s
            self.smcNDOB_u_n=u_n
            # Update smcNDOB estimation
            self.smcNDOB_dz = -self.smcNDOB_kp_sig*(b_x*(u_n+u_s)+self.smcNDOB_dist_est)
            self.smcNDOB_z_current =self.smcNDOB_z_old + self.smcNDOB_dz*(self.t_new-self.t_old)
            self.smcNDOB_dist_est = self.smcNDOB_z_current+self.smcNDOB_kp_sig*s
            self.smcNDOB_dist_est_tqr=self.smcNDOB_dist_est*M
            # Update uAlpha
            self.smcNDOB_u_total = u_eq + u_s + u_n
            u_total=self.smcNDOB_u_total        

        elif self.flag_contact_mode==1: # fix pressrue
            # Update double dot of xd
            dxd1=self.positionSingleDerevativeSelection()
            ddxd1=self.positionDoubleDerevativeSelection()
            # Update SMC 
            s=self.smcNDOB_lambda*self.x1_error_current+self.x2_current-dxd1
            # saturation bound for sliding surface
            if np.absolute(s/self.smcNDOB_sat_bound) <=1:
                sat_s= s/self.smcNDOB_sat_bound
            else:
                sat_s= np.sign(s)
            # Update u_eq,u_s,u_n
            u_eq = -1.0/b_x*(f_x+self.smcNDOB_lambda*(self.x2_current-dxd1)-ddxd1 + self.smcNDOB_k_sig*s)
            u_s =  -1.0/b_x*self.smcNDOB_eta*sat_s
            u_n = -1.0/b_x*self.smcNDOB_dist_est
            self.smcNDOB_u_eq=u_eq
            self.smcNDOB_u_s=u_s
            self.smcNDOB_u_n=u_n
            # Update smcNDOB estimation
            self.smcNDOB_dz = -self.smcNDOB_kp_sig*(b_x*(u_n+u_s)+self.smcNDOB_dist_est)
            self.smcNDOB_z_current =self.smcNDOB_z_old + self.smcNDOB_dz*(self.t_new-self.t_old)
            self.smcNDOB_dist_est = self.smcNDOB_z_current+self.smcNDOB_kp_sig*s
            self.smcNDOB_dist_est_tqr=self.smcNDOB_dist_est*M
            # Swithcing algorithm based on smcNDOB_th
            if np.absolute(self.smcNDOB_dist_est_tqr)>= self.smcNDOB_th:
                if self.flag_first_contact ==0: # first contact
                    self.u_fix_value=self.smcNDOB_u_total
                    self.flag_first_contact =1
                else:
                    u_total=self.u_fix_value
            else:
                self.smcNDOB_u_total = u_eq + u_s + u_n
                u_total=self.smcNDOB_u_total
                self.flag_first_contact=0  

        elif self.flag_contact_mode ==2: #fix to maximum position errors
            # Update double dot of xd
            dxd1=self.positionSingleDerevativeSelection()
            ddxd1=self.positionDoubleDerevativeSelection()
            # Update SMC 
            s=self.smcNDOB_lambda*self.x1_error_current+self.x2_current-dxd1
            # saturation bound for sliding surface
            if np.absolute(s/self.smcNDOB_sat_bound) <=1:
                sat_s= s/self.smcNDOB_sat_bound
            else:
                sat_s= np.sign(s)
            # Update u_eq,u_s,u_n
            u_eq = -1.0/b_x*(f_x+self.smcNDOB_lambda*(self.x2_current-dxd1)-ddxd1 + self.smcNDOB_k_sig*s)
            u_s =  -1.0/b_x*self.smcNDOB_eta*sat_s
            u_n = -1.0/b_x*self.smcNDOB_dist_est
            self.smcNDOB_u_eq=u_eq
            self.smcNDOB_u_s=u_s
            self.smcNDOB_u_n=u_n
            # Update smcNDOB estimation
            self.smcNDOB_dz = -self.smcNDOB_kp_sig*(b_x*(u_n+u_s)+self.smcNDOB_dist_est)
            self.smcNDOB_z_current =self.smcNDOB_z_old + self.smcNDOB_dz*(self.t_new-self.t_old)
            self.smcNDOB_dist_est = self.smcNDOB_z_current+self.smcNDOB_kp_sig*s
            self.smcNDOB_dist_est_tqr=self.smcNDOB_dist_est*M
            # Swithcing algorithm based on smcNDOB_th
            if np.absolute(self.smcNDOB_dist_est_tqr)>= self.smcNDOB_th: 
                if np.absolute(self.x1_error_current) >= self.x1_error_max_value:
                    if self.flag_first_contact ==0: # first contact
                        self.xd1_new=self.xd1
                        self.smcNDOB_dist_est_inner=self.smcNDOB_dist_est
                        self.smcNDOB_z_old_inner=self.smcNDOB_z_old
                        self.flag_first_contact =1
                        self.x1_new_error=0.
                        self.timer_attampt=np.time()
                    self.x1_new_error=self.x1_current-self.xd1_new
                    if np.absolute(self.x1_new_error) >=0.5*self.x1_error_max_value:
                        dxd1=0.
                        ddxd1=0.
                        xd1=self.xd1_new
                        self.flag_stage =1
                        s =self.smcNDOB_lambda*(self.x1_current-xd1) +self.x2_current-dxd1
                        if np.absolute(s/self.smcNDOB_sat_bound) <=1:
                            sat_s= s/self.smcNDOB_sat_bound
                        else:
                            sat_s= np.sign(s)
                        # Update u_eq,u_s,u_n
                        u_eq = -1.0/b_x*(f_x+self.smcNDOB_lambda*(self.x2_current-dxd1)-ddxd1 + self.smcNDOB_k_sig*s)
                        u_s =  -1.0/b_x*self.smcNDOB_eta*sat_s
                        u_n = -1.0/b_x*self.smcNDOB_dist_est_inner
                        # Update smcNDOB estimation
                        self.smcNDOB_dz_inner = -self.smcNDOB_kp_sig*(b_x*(u_n+u_s)+self.smcNDOB_dist_est_inner)
                        self.smcNDOB_z_current_inner =self.smcNDOB_z_old_inner + self.smcNDOB_dz_inner*(self.t_new-self.t_old)
                        self.smcNDOB_dist_est_inner = self.smcNDOB_z_current_inner+self.smcNDOB_kp_sig*s
                        self.smcNDOB_dist_est_inner_tqr=self.smcNDOB_dist_est_inner*M
                        self.smcNDOB_z_old_inner=self.smcNDOB_z_current_inner
                        u_total = u_eq + u_s + u_n # trying to go back to xd_t
                        self.xd_new=xd1
                    elif np.absolute(self.x1_new_error) <0.5*self.x1_error_max_value: # trying to reach xd
                        self.xd1_new = self.x1_current +np.sign(self.x1_error_current)*self.x1_error_max_value
                        dxd1=0.
                        ddxd1=0.
                        xd1=self.xd1_new
                        self.flag_stage =2
                        s =self.smcNDOB_lambda*(self.x1_current-xd1) +self.x2_current-dxd1
                        if np.absolute(s/self.smcNDOB_sat_bound) <=1:
                            sat_s= s/self.smcNDOB_sat_bound
                        else:
                            sat_s= np.sign(s)
                        # Update u_eq,u_s,u_n
                        u_eq = -1.0/b_x*(f_x+self.smcNDOB_lambda*(self.x2_current-dxd1)-ddxd1 + self.smcNDOB_k_sig*s)
                        u_s =  -1.0/b_x*self.smcNDOB_eta*sat_s
                        u_n = -1.0/b_x*self.smcNDOB_dist_est_inner
                        # Update smcNDOB estimation
                        self.smcNDOB_dz_inner = -self.smcNDOB_kp_sig*(b_x*(u_n+u_s)+self.smcNDOB_dist_est_inner)
                        self.smcNDOB_z_current_inner =self.smcNDOB_z_old_inner + self.smcNDOB_dz_inner*(self.t_new-self.t_old)
                        self.smcNDOB_dist_est_inner = self.smcNDOB_z_current_inner+self.smcNDOB_kp_sig*s
                        self.smcNDOB_z_old_inner=self.smcNDOB_z_current_inner
                        self.smcNDOB_dist_est_inner_tqr=self.smcNDOB_dist_est_inner*M
                        u_total = u_eq + u_s + u_n
                        self.xd_new=xd1
                else:
                    self.smcNDOB_u_total = u_eq + u_s + u_n
                    u_total=self.smcNDOB_u_total
                    self.flag_first_contact=0
                    self.flag_stage =0
                    self.xd_new=self.xd1     
            else:
                self.smcNDOB_u_total = u_eq + u_s + u_n
                u_total=self.smcNDOB_u_total
                self.flag_first_contact=0
                self.flag_stage =0
                self.xd_new=self.xd1 

        elif self.flag_contact_mode ==3: #N-C-R
            if self.flag_stage==0: # normal mode
                self.xd1_new = self.xd1
                dxd1=self.positionSingleDerevativeSelection()
                ddxd1=self.positionDoubleDerevativeSelection()
                xd1=self.xd1_new
                self.x1_new_error=self.x1_current-self.xd1_new
                self.x1_error_current=self.x1_current-self.xd1
                s =self.smcNDOB_lambda*(self.x1_current-xd1) +self.x2_current-dxd1
                if np.absolute(s/self.smcNDOB_sat_bound) <=1:
                    sat_s= s/self.smcNDOB_sat_bound
                else:
                    sat_s= np.sign(s)
                # Update u_eq,u_s,u_n
                u_eq = -1.0/b_x*(f_x+self.smcNDOB_lambda*(self.x2_current-dxd1)-ddxd1 + self.smcNDOB_k_sig*s)
                u_s =  -1.0/b_x*self.smcNDOB_eta*sat_s
                u_n = -1.0/b_x*self.smcNDOB_dist_est_inner
                # Update smcNDOB estimation
                self.smcNDOB_dz_inner = -self.smcNDOB_kp_sig*(b_x*(u_n+u_s)+self.smcNDOB_dist_est_inner)
                self.smcNDOB_z_current_inner =self.smcNDOB_z_old_inner + self.smcNDOB_dz_inner*(self.t_new-self.t_old)
                self.smcNDOB_dist_est_inner = self.smcNDOB_z_current_inner+self.smcNDOB_kp_sig*s
                self.smcNDOB_z_old_inner=self.smcNDOB_z_current_inner
                self.smcNDOB_dist_est_inner_tqr=self.smcNDOB_dist_est_inner*M
                u_total = u_eq + u_s + u_n
                if np.absolute(self.x1_error_current) >= self.x1_error_max_value: # swith to compliant mode
                    if np.absolute(self.smcNDOB_dist_est_inner_tqr)>= self.smcNDOB_th:
                        if self.flag_first_contact ==0: # first contact
                            self.xd1_new=self.x1_current
                            # self.smcNDOB_dist_est_inner=0.
                            # self.smcNDOB_z_old_inner=0.

                            # self.smcNDOB_dist_est_inner=self.smcNDOB_dist_est
                            # self.smcNDOB_z_old_inner=self.smcNDOB_z_old
                            self.flag_first_contact =1
                            # self.x1_new_error=0.
                            # self.timer_attampt=time()
                        self.x1_new_error=self.x1_current-self.xd1_new
                        # if time()-self.timer_attampt <= self.timer_th:
                        self.flag_stage =1
                        dxd1=0.
                        ddxd1=0.
                        xd1=self.xd1_new
                        s =self.smcNDOB_lambda*(self.x1_current-xd1) +self.x2_current-dxd1
                        if np.absolute(s/self.smcNDOB_sat_bound) <=1:
                            sat_s= s/self.smcNDOB_sat_bound
                        else:
                            sat_s= np.sign(s)
                        # Update u_eq,u_s,u_n
                        u_eq = -1.0/b_x*(f_x+self.smcNDOB_lambda*(self.x2_current-dxd1)-ddxd1 + self.smcNDOB_k_sig*s)
                        u_s =  -1.0/b_x*self.smcNDOB_eta*sat_s
                        u_n = -1.0/b_x*self.smcNDOB_dist_est_inner
                        # Update smcNDOB estimation
                        self.smcNDOB_dz_inner = -self.smcNDOB_kp_sig*(b_x*(u_n+u_s)+self.smcNDOB_dist_est_inner)
                        self.smcNDOB_z_current_inner =self.smcNDOB_z_old_inner + self.smcNDOB_dz_inner*(self.t_new-self.t_old)
                        self.smcNDOB_dist_est_inner = self.smcNDOB_z_current_inner+self.smcNDOB_kp_sig*s
                        self.smcNDOB_dist_est_inner_tqr=self.smcNDOB_dist_est_inner*M
                        self.smcNDOB_z_old_inner=self.smcNDOB_z_current_inner
                        self.smcNDOB_dist_est_inner_tqr=self.smcNDOB_dist_est_inner*M
                        u_total = u_eq + u_s + u_n # trying to go back to xd_t
                        self.xd_new=xd1
                    elif np.absolute(self.smcNDOB_dist_est_tqr)< self.smcNDOB_th: # stay in normal 
                        self.xd1_new = self.positionProfileSelection()
                        dxd1=self.positionSingleDerevativeSelection()
                        ddxd1=self.positionDoubleDerevativeSelection()
                        xd1=self.xd1_new
                        self.flag_stage =0
                        s =self.smcNDOB_lambda*(self.x1_current-xd1) +self.x2_current-dxd1
                        if np.absolute(s/self.smcNDOB_sat_bound) <=1:
                            sat_s= s/self.smcNDOB_sat_bound
                        else:
                            sat_s= np.sign(s)
                        # Update u_eq,u_s,u_n
                        u_eq = -1.0/b_x*(f_x+self.smcNDOB_lambda*(self.x2_current-dxd1)-ddxd1 + self.smcNDOB_k_sig*s)
                        u_s =  -1.0/b_x*self.smcNDOB_eta*sat_s
                        u_n = -1.0/b_x*self.smcNDOB_dist_est_inner
                        # Update smcNDOB estimation
                        self.smcNDOB_dz_inner = -self.smcNDOB_kp_sig*(b_x*(u_n+u_s)+self.smcNDOB_dist_est_inner)
                        self.smcNDOB_z_current_inner =self.smcNDOB_z_old_inner + self.smcNDOB_dz_inner*(self.t_new-self.t_old)
                        self.smcNDOB_dist_est_inner = self.smcNDOB_z_current_inner+self.smcNDOB_kp_sig*s
                        self.smcNDOB_z_old_inner=self.smcNDOB_z_current_inner
                        self.smcNDOB_dist_est_inner_tqr=self.smcNDOB_dist_est_inner*M
                        u_total = u_eq + u_s + u_n
                        self.xd_new=xd1
                elif np.absolute(self.x1_error_current) < self.x1_error_max_value:
                    self.xd1_new = self.positionProfileSelection()
                    dxd1=self.positionSingleDerevativeSelection()
                    ddxd1=self.positionDoubleDerevativeSelection()
                    xd1=self.xd1_new
                    self.flag_stage =0
                    s =self.smcNDOB_lambda*(self.x1_current-xd1) +self.x2_current-dxd1
                    if np.absolute(s/self.smcNDOB_sat_bound) <=1:
                        sat_s= s/self.smcNDOB_sat_bound
                    else:
                        sat_s= np.sign(s)
                    # Update u_eq,u_s,u_n
                    u_eq = -1.0/b_x*(f_x+self.smcNDOB_lambda*(self.x2_current-dxd1)-ddxd1 + self.smcNDOB_k_sig*s)
                    u_s =  -1.0/b_x*self.smcNDOB_eta*sat_s
                    u_n = -1.0/b_x*self.smcNDOB_dist_est_inner
                    # Update smcNDOB estimation
                    self.smcNDOB_dz_inner = -self.smcNDOB_kp_sig*(b_x*(u_n+u_s)+self.smcNDOB_dist_est_inner)
                    self.smcNDOB_z_current_inner =self.smcNDOB_z_old_inner + self.smcNDOB_dz_inner*(self.t_new-self.t_old)
                    self.smcNDOB_dist_est_inner = self.smcNDOB_z_current_inner+self.smcNDOB_kp_sig*s
                    self.smcNDOB_z_old_inner=self.smcNDOB_z_current_inner
                    self.smcNDOB_dist_est_inner_tqr=self.smcNDOB_dist_est_inner*M
                    u_total = u_eq + u_s + u_n
                    self.xd_new=xd1  #stay in normal mode
            elif self.flag_stage==1: # compliant mode
                # if np.absolute(self.x1_error_current) >= self.x1_error_max_value: # swith to compliant mode
                if np.absolute(self.smcNDOB_dist_est_inner_tqr)>= self.mode_C_to_R_th: #stay in compliant
                    dxd1=0.
                    ddxd1=0.
                    xd1=self.xd1_new
                    self.flag_stage =1
                    s =self.smcNDOB_lambda*(self.x1_current-xd1) +self.x2_current-dxd1
                    if np.absolute(s/self.smcNDOB_sat_bound) <=1:
                        sat_s= s/self.smcNDOB_sat_bound
                    else:
                        sat_s= np.sign(s)
                    # Update u_eq,u_s,u_n
                    u_eq = -1.0/b_x*(f_x+self.smcNDOB_lambda*(self.x2_current-dxd1)-ddxd1 + self.smcNDOB_k_sig*s)
                    u_s =  -1.0/b_x*self.smcNDOB_eta*sat_s
                    u_n = -1.0/b_x*self.smcNDOB_dist_est_inner
                    # Update smcNDOB estimation
                    self.smcNDOB_dz_inner = -self.smcNDOB_kp_sig*(b_x*(u_n+u_s)+self.smcNDOB_dist_est_inner)
                    self.smcNDOB_z_current_inner =self.smcNDOB_z_old_inner + self.smcNDOB_dz_inner*(self.t_new-self.t_old)
                    self.smcNDOB_dist_est_inner = self.smcNDOB_z_current_inner+self.smcNDOB_kp_sig*s
                    self.smcNDOB_dist_est_inner_tqr=self.smcNDOB_dist_est_inner*M
                    self.smcNDOB_z_old_inner=self.smcNDOB_z_current_inner
                    u_total = u_eq + u_s + u_n # trying to go back to xd_t
                    self.xd_new=xd1 #stay in compliant mode
                elif np.absolute(self.smcNDOB_dist_est_inner_tqr)< self.mode_C_to_R_th: # switch to recovery mode
                    if self.flag_first_contact ==1: # first contact
                        # self.smcNDOB_dz_inner = 0.
                        # self.smcNDOB_z_current_inner =0.
                        # self.smcNDOB_dist_est_inner = 0.
                        # self.smcNDOB_dist_est_inner_tqr=0.
                        # self.smcNDOB_z_old_inner=0.
                        self.flag_first_contact =0
                    self.xd1_new = self.x1_current-np.sign(self.x1_error_current)*self.x1_error_max_value
                    dxd1=0.
                    ddxd1=0.
                    xd1=self.xd1_new
                    self.flag_stage =2
                    s =self.smcNDOB_lambda*(self.x1_current-xd1) +self.x2_current-dxd1
                    if np.absolute(s/self.smcNDOB_sat_bound) <=1:
                        sat_s= s/self.smcNDOB_sat_bound
                    else:
                        sat_s= np.sign(s)
                    # Update u_eq,u_s,u_n
                    u_eq = -1.0/b_x*(f_x+self.smcNDOB_lambda*(self.x2_current-dxd1)-ddxd1 + self.smcNDOB_k_sig*s)
                    u_s =  -1.0/b_x*self.smcNDOB_eta*sat_s
                    u_n = -1.0/b_x*self.smcNDOB_dist_est_inner
                    # Update smcNDOB estimation
                    self.smcNDOB_dz_inner = -self.smcNDOB_kp_sig*(b_x*(u_n+u_s)+self.smcNDOB_dist_est_inner)
                    self.smcNDOB_z_current_inner =self.smcNDOB_z_old_inner + self.smcNDOB_dz_inner*(self.t_new-self.t_old)
                    self.smcNDOB_dist_est_inner = self.smcNDOB_z_current_inner+self.smcNDOB_kp_sig*s
                    self.smcNDOB_z_old_inner=self.smcNDOB_z_current_inner
                    self.smcNDOB_dist_est_inner_tqr=self.smcNDOB_dist_est_inner*M
                    u_total = u_eq + u_s + u_n
                    self.xd_new=xd1
            elif self.flag_stage==2: #recovery mode
                if np.absolute(self.x1_error_current) >= self.x1_error_max_value: # stay in R mode
                    self.flag_stage = 2
                    self.xd1_new = self.x1_current-np.sign(self.x1_error_current)*self.x1_error_max_value
                    dxd1=0.
                    ddxd1=0.
                    xd1=self.xd1_new
                    s =self.smcNDOB_lambda*(self.x1_current-xd1) +self.x2_current-dxd1
                    if np.absolute(s/self.smcNDOB_sat_bound) <=1:
                        sat_s= s/self.smcNDOB_sat_bound
                    else:
                        sat_s= np.sign(s)
                    # Update u_eq,u_s,u_n
                    u_eq = -1.0/b_x*(f_x+self.smcNDOB_lambda*(self.x2_current-dxd1)-ddxd1 + self.smcNDOB_k_sig*s)
                    u_s =  -1.0/b_x*self.smcNDOB_eta*sat_s
                    u_n = -1.0/b_x*self.smcNDOB_dist_est_inner
                    # Update smcNDOB estimation
                    self.smcNDOB_dz_inner = -self.smcNDOB_kp_sig*(b_x*(u_n+u_s)+self.smcNDOB_dist_est_inner)
                    self.smcNDOB_z_current_inner =self.smcNDOB_z_old_inner + self.smcNDOB_dz_inner*(self.t_new-self.t_old)
                    self.smcNDOB_dist_est_inner = self.smcNDOB_z_current_inner+self.smcNDOB_kp_sig*s
                    self.smcNDOB_dist_est_inner_tqr=self.smcNDOB_dist_est_inner*M
                    self.smcNDOB_z_old_inner=self.smcNDOB_z_current_inner
                    self.smcNDOB_dist_est_inner_tqr=self.smcNDOB_dist_est_inner*M
                    u_total = u_eq + u_s + u_n # trying to go back to xd_t
                    self.xd_new=xd1
                elif np.absolute(self.x1_error_current) < self.x1_error_max_value: # switch to normal
                    self.xd1_new = self.xd1
                    dxd1=self.positionSingleDerevativeSelection()
                    ddxd1=self.positionDoubleDerevativeSelection()
                    xd1=self.xd1_new
                    self.flag_stage =0
                    s =self.smcNDOB_lambda*(self.x1_current-xd1) +self.x2_current-dxd1
                    if np.absolute(s/self.smcNDOB_sat_bound) <=1:
                        sat_s= s/self.smcNDOB_sat_bound
                    else:
                        sat_s= np.sign(s)
                    # Update u_eq,u_s,u_n
                    u_eq = -1.0/b_x*(f_x+self.smcNDOB_lambda*(self.x2_current-dxd1)-ddxd1 + self.smcNDOB_k_sig*s)
                    u_s =  -1.0/b_x*self.smcNDOB_eta*sat_s
                    u_n = -1.0/b_x*self.smcNDOB_dist_est_inner
                    # Update smcNDOB estimation
                    self.smcNDOB_dz_inner = -self.smcNDOB_kp_sig*(b_x*(u_n+u_s)+self.smcNDOB_dist_est_inner)
                    self.smcNDOB_z_current_inner =self.smcNDOB_z_old_inner + self.smcNDOB_dz_inner*(self.t_new-self.t_old)
                    self.smcNDOB_dist_est_inner = self.smcNDOB_z_current_inner+self.smcNDOB_kp_sig*s
                    self.smcNDOB_z_old_inner=self.smcNDOB_z_current_inner
                    self.smcNDOB_dist_est_inner_tqr=self.smcNDOB_dist_est_inner*M
                    u_total = u_eq + u_s + u_n
                    self.xd_new=xd1 
                    self.xd_new=xd1 # swith to try mode
        elif self.flag_contact_mode ==4: #N-C-R-C-R-N
            if self.flag_stage==0: # normal mode
                self.xd1_new = self.xd1
                dxd1=self.positionSingleDerevativeSelection()
                ddxd1=self.positionDoubleDerevativeSelection()
                xd1=self.xd1_new
                self.x1_new_error=self.x1_current-self.xd1_new
                self.x1_error_current=self.x1_current-self.xd1
                s =self.smcNDOB_lambda*(self.x1_current-xd1) +self.x2_current-dxd1
                if np.absolute(s/self.smcNDOB_sat_bound) <=1:
                    sat_s= s/self.smcNDOB_sat_bound
                else:
                    sat_s= np.sign(s)
                # Update u_eq,u_s,u_n
                u_eq = -1.0/b_x*(f_x+self.smcNDOB_lambda*(self.x2_current-dxd1)-ddxd1 + self.smcNDOB_k_sig*s)
                u_s =  -1.0/b_x*self.smcNDOB_eta*sat_s
                u_n = -1.0/b_x*self.smcNDOB_dist_est_inner
                # Update smcNDOB estimation
                self.smcNDOB_dz_inner = -self.smcNDOB_kp_sig*(b_x*(u_n+u_s)+self.smcNDOB_dist_est_inner)
                self.smcNDOB_z_current_inner =self.smcNDOB_z_old_inner + self.smcNDOB_dz_inner*(self.t_new-self.t_old)
                self.smcNDOB_dist_est_inner = self.smcNDOB_z_current_inner+self.smcNDOB_kp_sig*s
                self.smcNDOB_z_old_inner=self.smcNDOB_z_current_inner
                self.smcNDOB_dist_est_inner_tqr=self.smcNDOB_dist_est_inner*M
                u_total = u_eq + u_s + u_n
                if np.absolute(self.x1_error_current) >= self.x1_error_max_value: #stay in normal mode OR switch to R mode
                    if np.absolute(self.smcNDOB_dist_est_inner_tqr)>= self.smcNDOB_th:# swith to compliant mode
                        if self.flag_first_contact ==0: # first contact
                            self.xd1_new=self.x1_current
                            self.xdc=self.xd1_new
                            # self.smcNDOB_dist_est_inner=0.
                            # self.smcNDOB_z_old_inner=0.

                            # self.smcNDOB_dist_est_inner=self.smcNDOB_dist_est
                            # self.smcNDOB_z_old_inner=self.smcNDOB_z_old
                            self.flag_first_contact =1
                            # self.x1_new_error=0.
                            # self.timer_attampt=time()
                        self.x1_new_error=self.x1_current-self.xd1_new
                        # if time()-self.timer_attampt <= self.timer_th:
                        self.flag_stage =1
                        dxd1=0.
                        ddxd1=0.
                        xd1=self.xd1_new
                        s =self.smcNDOB_lambda*(self.x1_current-xd1) +self.x2_current-dxd1
                        if np.absolute(s/self.smcNDOB_sat_bound) <=1:
                            sat_s= s/self.smcNDOB_sat_bound
                        else:
                            sat_s= np.sign(s)
                        # Update u_eq,u_s,u_n
                        u_eq = -1.0/b_x*(f_x+self.smcNDOB_lambda*(self.x2_current-dxd1)-ddxd1 + self.smcNDOB_k_sig*s)
                        u_s =  -1.0/b_x*self.smcNDOB_eta*sat_s
                        u_n = -1.0/b_x*self.smcNDOB_dist_est_inner
                        # Update smcNDOB estimation
                        self.smcNDOB_dz_inner = -self.smcNDOB_kp_sig*(b_x*(u_n+u_s)+self.smcNDOB_dist_est_inner)
                        self.smcNDOB_z_current_inner =self.smcNDOB_z_old_inner + self.smcNDOB_dz_inner*(self.t_new-self.t_old)
                        self.smcNDOB_dist_est_inner = self.smcNDOB_z_current_inner+self.smcNDOB_kp_sig*s
                        self.smcNDOB_dist_est_inner_tqr=self.smcNDOB_dist_est_inner*M
                        self.smcNDOB_z_old_inner=self.smcNDOB_z_current_inner
                        self.smcNDOB_dist_est_inner_tqr=self.smcNDOB_dist_est_inner*M
                        u_total = u_eq + u_s + u_n # trying to go back to xd_t
                        self.xd_new=xd1
                    elif np.absolute(self.smcNDOB_dist_est_tqr)< self.smcNDOB_th: # stay in normal 
                        self.xd1_new = self.positionProfileSelection()
                        dxd1=self.positionSingleDerevativeSelection()
                        ddxd1=self.positionDoubleDerevativeSelection()
                        xd1=self.xd1_new
                        self.flag_stage =0
                        s =self.smcNDOB_lambda*(self.x1_current-xd1) +self.x2_current-dxd1
                        if np.absolute(s/self.smcNDOB_sat_bound) <=1:
                            sat_s= s/self.smcNDOB_sat_bound
                        else:
                            sat_s= np.sign(s)
                        # Update u_eq,u_s,u_n
                        u_eq = -1.0/b_x*(f_x+self.smcNDOB_lambda*(self.x2_current-dxd1)-ddxd1 + self.smcNDOB_k_sig*s)
                        u_s =  -1.0/b_x*self.smcNDOB_eta*sat_s
                        u_n = -1.0/b_x*self.smcNDOB_dist_est_inner
                        # Update smcNDOB estimation
                        self.smcNDOB_dz_inner = -self.smcNDOB_kp_sig*(b_x*(u_n+u_s)+self.smcNDOB_dist_est_inner)
                        self.smcNDOB_z_current_inner =self.smcNDOB_z_old_inner + self.smcNDOB_dz_inner*(self.t_new-self.t_old)
                        self.smcNDOB_dist_est_inner = self.smcNDOB_z_current_inner+self.smcNDOB_kp_sig*s
                        self.smcNDOB_z_old_inner=self.smcNDOB_z_current_inner
                        self.smcNDOB_dist_est_inner_tqr=self.smcNDOB_dist_est_inner*M
                        u_total = u_eq + u_s + u_n
                        self.xd_new=xd1
                elif np.absolute(self.x1_error_current) < self.x1_error_max_value: #stay in normal mode
                    self.xd1_new = self.positionProfileSelection()
                    dxd1=self.positionSingleDerevativeSelection()
                    ddxd1=self.positionDoubleDerevativeSelection()
                    xd1=self.xd1_new
                    self.flag_stage =0
                    s =self.smcNDOB_lambda*(self.x1_current-xd1) +self.x2_current-dxd1
                    if np.absolute(s/self.smcNDOB_sat_bound) <=1:
                        sat_s= s/self.smcNDOB_sat_bound
                    else:
                        sat_s= np.sign(s)
                    # Update u_eq,u_s,u_n
                    u_eq = -1.0/b_x*(f_x+self.smcNDOB_lambda*(self.x2_current-dxd1)-ddxd1 + self.smcNDOB_k_sig*s)
                    u_s =  -1.0/b_x*self.smcNDOB_eta*sat_s
                    u_n = -1.0/b_x*self.smcNDOB_dist_est_inner
                    # Update smcNDOB estimation
                    self.smcNDOB_dz_inner = -self.smcNDOB_kp_sig*(b_x*(u_n+u_s)+self.smcNDOB_dist_est_inner)
                    self.smcNDOB_z_current_inner =self.smcNDOB_z_old_inner + self.smcNDOB_dz_inner*(self.t_new-self.t_old)
                    self.smcNDOB_dist_est_inner = self.smcNDOB_z_current_inner+self.smcNDOB_kp_sig*s
                    self.smcNDOB_z_old_inner=self.smcNDOB_z_current_inner
                    self.smcNDOB_dist_est_inner_tqr=self.smcNDOB_dist_est_inner*M
                    u_total = u_eq + u_s + u_n
                    self.xd_new=xd1  
            elif self.flag_stage==1: # compliant mode
                if np.absolute(self.smcNDOB_dist_est_inner_tqr)>= self.mode_C_to_R_th: #stay in compliant
                    dxd1=0.
                    ddxd1=0.
                    xd1=self.xd1_new
                    self.flag_stage =1
                    s =self.smcNDOB_lambda*(self.x1_current-xd1) +self.x2_current-dxd1
                    if np.absolute(s/self.smcNDOB_sat_bound) <=1:
                        sat_s= s/self.smcNDOB_sat_bound
                    else:
                        sat_s= np.sign(s)
                    # Update u_eq,u_s,u_n
                    u_eq = -1.0/b_x*(f_x+self.smcNDOB_lambda*(self.x2_current-dxd1)-ddxd1 + self.smcNDOB_k_sig*s)
                    u_s =  -1.0/b_x*self.smcNDOB_eta*sat_s
                    u_n = -1.0/b_x*self.smcNDOB_dist_est_inner
                    # Update smcNDOB estimation
                    self.smcNDOB_dz_inner = -self.smcNDOB_kp_sig*(b_x*(u_n+u_s)+self.smcNDOB_dist_est_inner)
                    self.smcNDOB_z_current_inner =self.smcNDOB_z_old_inner + self.smcNDOB_dz_inner*(self.t_new-self.t_old)
                    self.smcNDOB_dist_est_inner = self.smcNDOB_z_current_inner+self.smcNDOB_kp_sig*s
                    self.smcNDOB_dist_est_inner_tqr=self.smcNDOB_dist_est_inner*M
                    self.smcNDOB_z_old_inner=self.smcNDOB_z_current_inner
                    u_total = u_eq + u_s + u_n # trying to go back to xd_t
                    self.xd_new=xd1 #stay in compliant mode
                elif np.absolute(self.smcNDOB_dist_est_inner_tqr)< self.mode_C_to_R_th: # switch to recovery mode
                    if self.flag_first_contact ==1: # first contact
                        # self.smcNDOB_dz_inner = 0.
                        # self.smcNDOB_z_current_inner =0.
                        # self.smcNDOB_dist_est_inner = 0.
                        # self.smcNDOB_dist_est_inner_tqr=0.
                        # self.smcNDOB_z_old_inner=0.
                        self.flag_first_contact =0
                    self.xd1_new = self.x1_current-np.sign(self.x1_error_current)*self.x1_error_max_value
                    dxd1=0.
                    ddxd1=0.
                    xd1=self.xd1_new
                    self.flag_stage =2
                    s =self.smcNDOB_lambda*(self.x1_current-xd1) +self.x2_current-dxd1
                    if np.absolute(s/self.smcNDOB_sat_bound) <=1:
                        sat_s= s/self.smcNDOB_sat_bound
                    else:
                        sat_s= np.sign(s)
                    # Update u_eq,u_s,u_n
                    u_eq = -1.0/b_x*(f_x+self.smcNDOB_lambda*(self.x2_current-dxd1)-ddxd1 + self.smcNDOB_k_sig*s)
                    u_s =  -1.0/b_x*self.smcNDOB_eta*sat_s
                    u_n = -1.0/b_x*self.smcNDOB_dist_est_inner
                    # Update smcNDOB estimation
                    self.smcNDOB_dz_inner = -self.smcNDOB_kp_sig*(b_x*(u_n+u_s)+self.smcNDOB_dist_est_inner)
                    self.smcNDOB_z_current_inner =self.smcNDOB_z_old_inner + self.smcNDOB_dz_inner*(self.t_new-self.t_old)
                    self.smcNDOB_dist_est_inner = self.smcNDOB_z_current_inner+self.smcNDOB_kp_sig*s
                    self.smcNDOB_z_old_inner=self.smcNDOB_z_current_inner
                    self.smcNDOB_dist_est_inner_tqr=self.smcNDOB_dist_est_inner*M
                    u_total = u_eq + u_s + u_n
                    self.xd_new=xd1
            elif self.flag_stage==2: #recovery mode
                if np.absolute(self.x1_error_current) >= self.x1_error_max_value: #stay in R mode OR switch to C mode
                    if np.absolute(self.smcNDOB_dist_est_inner_tqr) <= self.mode_R_to_C_th: # stay in R mode
                        self.flag_stage = 2
                        self.xd1_new = self.x1_current-np.sign(self.x1_error_current)*self.x1_error_max_value
                        dxd1=0.
                        ddxd1=0.
                        xd1=self.xd1_new
                        s =self.smcNDOB_lambda*(self.x1_current-xd1) +self.x2_current-dxd1
                        if np.absolute(s/self.smcNDOB_sat_bound) <=1:
                            sat_s= s/self.smcNDOB_sat_bound
                        else:
                            sat_s= np.sign(s)
                        # Update u_eq,u_s,u_n
                        u_eq = -1.0/b_x*(f_x+self.smcNDOB_lambda*(self.x2_current-dxd1)-ddxd1 + self.smcNDOB_k_sig*s)
                        u_s =  -1.0/b_x*self.smcNDOB_eta*sat_s
                        u_n = -1.0/b_x*self.smcNDOB_dist_est_inner
                        # Update smcNDOB estimation
                        self.smcNDOB_dz_inner = -self.smcNDOB_kp_sig*(b_x*(u_n+u_s)+self.smcNDOB_dist_est_inner)
                        self.smcNDOB_z_current_inner =self.smcNDOB_z_old_inner + self.smcNDOB_dz_inner*(self.t_new-self.t_old)
                        self.smcNDOB_dist_est_inner = self.smcNDOB_z_current_inner+self.smcNDOB_kp_sig*s
                        self.smcNDOB_dist_est_inner_tqr=self.smcNDOB_dist_est_inner*M
                        self.smcNDOB_z_old_inner=self.smcNDOB_z_current_inner
                        self.smcNDOB_dist_est_inner_tqr=self.smcNDOB_dist_est_inner*M
                        u_total = u_eq + u_s + u_n # trying to go back to xd_t
                        self.xd_new=xd1
                    elif np.absolute(self.smcNDOB_dist_est_inner_tqr) > self.mode_R_to_C_th: # change to C mode
                        self.flag_stage = 1
                        self.xd1_new = self.xdc
                        dxd1=0.
                        ddxd1=0.
                        xd1=self.xd1_new
                        s =self.smcNDOB_lambda*(self.x1_current-xd1) +self.x2_current-dxd1
                        if np.absolute(s/self.smcNDOB_sat_bound) <=1:
                            sat_s= s/self.smcNDOB_sat_bound
                        else:
                            sat_s= np.sign(s)
                        # Update u_eq,u_s,u_n
                        u_eq = -1.0/b_x*(f_x+self.smcNDOB_lambda*(self.x2_current-dxd1)-ddxd1 + self.smcNDOB_k_sig*s)
                        u_s =  -1.0/b_x*self.smcNDOB_eta*sat_s
                        u_n = -1.0/b_x*self.smcNDOB_dist_est_inner
                        # Update smcNDOB estimation
                        self.smcNDOB_dz_inner = -self.smcNDOB_kp_sig*(b_x*(u_n+u_s)+self.smcNDOB_dist_est_inner)
                        self.smcNDOB_z_current_inner =self.smcNDOB_z_old_inner + self.smcNDOB_dz_inner*(self.t_new-self.t_old)
                        self.smcNDOB_dist_est_inner = self.smcNDOB_z_current_inner+self.smcNDOB_kp_sig*s
                        self.smcNDOB_dist_est_inner_tqr=self.smcNDOB_dist_est_inner*M
                        self.smcNDOB_z_old_inner=self.smcNDOB_z_current_inner
                        self.smcNDOB_dist_est_inner_tqr=self.smcNDOB_dist_est_inner*M
                        u_total = u_eq + u_s + u_n # trying to go back to xd_t
                        self.xd_new=xd1
                elif np.absolute(self.x1_error_current) < self.x1_error_max_value: # switch to normal
                    self.xd1_new = self.xd1
                    dxd1=self.positionSingleDerevativeSelection()
                    ddxd1=self.positionDoubleDerevativeSelection()
                    xd1=self.xd1_new
                    self.flag_stage =0
                    s =self.smcNDOB_lambda*(self.x1_current-xd1) +self.x2_current-dxd1
                    if np.absolute(s/self.smcNDOB_sat_bound) <=1:
                        sat_s= s/self.smcNDOB_sat_bound
                    else:
                        sat_s= np.sign(s)
                    # Update u_eq,u_s,u_n
                    u_eq = -1.0/b_x*(f_x+self.smcNDOB_lambda*(self.x2_current-dxd1)-ddxd1 + self.smcNDOB_k_sig*s)
                    u_s =  -1.0/b_x*self.smcNDOB_eta*sat_s
                    u_n = -1.0/b_x*self.smcNDOB_dist_est_inner
                    # Update smcNDOB estimation
                    self.smcNDOB_dz_inner = -self.smcNDOB_kp_sig*(b_x*(u_n+u_s)+self.smcNDOB_dist_est_inner)
                    self.smcNDOB_z_current_inner =self.smcNDOB_z_old_inner + self.smcNDOB_dz_inner*(self.t_new-self.t_old)
                    self.smcNDOB_dist_est_inner = self.smcNDOB_z_current_inner+self.smcNDOB_kp_sig*s
                    self.smcNDOB_z_old_inner=self.smcNDOB_z_current_inner
                    self.smcNDOB_dist_est_inner_tqr=self.smcNDOB_dist_est_inner*M
                    u_total = u_eq + u_s + u_n
                    self.xd_new=xd1 
                    self.xd_new=xd1 # swith to try mode
        cphi= np.cos(phi)
        sphi= np.sin(phi)
        p1_MPa=(u_total/self.alpha0-(0.5*sphi+0.5*np.sqrt(3)*cphi)*pm2_MPa+sphi*pm3_MPa)/(0.5*sphi-0.5*np.sqrt(3)*cphi)
        p1_psi=p1_MPa*145.038
        self.smcNDOB_p1=p1_psi
        if p1_psi>=self.input_pressure_limit_psi:
            pd_array=np.array([self.input_pressure_limit_psi,1.0,1.0])
        elif p1_psi<=1.0:
            pd_array=np.array([1.0,1.0,1.0])
        else:
            pd_array=np.array([p1_psi,1.0,1.0])
        self.x1_old=self.x1_current
        self.x1_error_old=self.x1_error_current
        self.t_old=self.t_new
        self.smcNDOB_z_old=self.smcNDOB_z_current
        # print pd_array
        self.send_zipped_socket0(pd_array)        

    def calculateControlInput(self):
        # self.array2setswithrotation=self.recv_cpp_socket2()
        # variable ini.
        vector_phiTheta=np.array([0., 0.])
        phi=0.
        theta=0.
        dtheta=0.
        pm1_MPa=0.
        pm2_MPa=0.
        pm3_MPa=0.
        uMax=0.
        s=0.
        Izz=0.
        M=0.
        C=0.
        G=0.
        f=0.
        uAlpha=0.
        ddxd1=0.
        pd_array=np.array([0.,0.,0.])
        uMin=0.
        # Update pm1, pm2, pm3, theta, r0, uMax, Ddot(error)
        vector_phiTheta=self.getThetaPhiAndr0FromXYZ()
        phi=vector_phiTheta[0]
        theta=vector_phiTheta[1]
        pm1_MPa=self.pd_pm_array[3]*0.00689476
        pm2_MPa=self.pd_pm_array[4]*0.00689476
        pm3_MPa=self.pd_pm_array[5]*0.00689476
        uMax=(np.absolute(self.alpha0 * (np.sin(phi) * (0.5*self.input_pressure_limit_psi*0.00689476 + 0.5*pm2_MPa - pm3_MPa)
            -np.sqrt(3.0) * np.cos(phi) * (0.5*self.input_pressure_limit_psi*0.00689476 - 0.5*pm2_MPa)))
        )
        #Update State variables x1,x2, x1 error,
        self.t_new=time()-self.t0
        self.x1_current=theta
        self.xd1=self.positionProfileSelection()
        self.x1_error_current=self.xd1-self.x1_current
        self.x2_current=(self.x1_current-self.x1_old)/(self.t_new-self.t_old)
        dtheta=self.x2_current
        self.x1_dot_error=(self.x1_error_current-self.x1_error_old)/(self.t_new-self.t_old)
        # Update SMC 
        s=self.smc_lambda*self.x1_error_current+self.x1_dot_error
        # Update M,C,G
        Izz=self.m0*self.r0*self.r0
        if theta==0.:
            M=self.m0*(self.L/2)**2
            C=0.
            G=-self.g*self.m0
        else:
            M=(Izz/4 + self.m0*((np.cos(theta/2)*(self.r0 - self.L/theta))/2 +
                (self.L*np.sin(theta/2))/theta**2)**2 + (self.m0*np.sin(theta/2)**2*(self.r0 - self.L/theta)**2)/4
            )
            C=(-(self.L*dtheta*self.m0*(2*np.sin(theta/2) - theta*np.cos(theta/2))*(2*self.L*np.sin(theta/2)
                - self.L*theta*np.cos(theta/2) + self.r0*theta**2*np.cos(theta/2)))/(2*theta**5)
            )
            G=(-(self.g*self.m0*(self.L*np.sin(theta) + self.r0*theta**2*np.cos(theta) - self.L*theta*np.cos(theta)))/(2*theta**2)
            )
        # Updata f(x1,x2)
        f=(1.0/M) * (-self.k0* self.x1_current- (self.b0+C)* self.x2_current- G)
        # Update uAlpha
        ddxd1=self.positionDoubleDerevativeSelection()
        uAlpha=(M*(self.smc_lambda*self.x1_dot_error+ ddxd1 - f + self.smc_eta* np.sign(s)
                    +np.sign(s)*(self.delta_k_max*np.absolute(self.x1_current/M) + self.delta_b_max*np.absolute(self.x2_current/M)))
                )
        # Constrain uMin<=uAlpha<= uMax
        cphi= np.cos(phi)
        sphi= np.sin(phi)
        p1_MPa=(uAlpha/self.alpha0-(0.5*sphi+0.5*np.sqrt(3)*cphi)*pm2_MPa+sphi*pm3_MPa)/(0.5*sphi-0.5*np.sqrt(3)*cphi)
        p1_psi=p1_MPa*145.038

        if p1_psi>=self.input_pressure_limit_psi:
            pd_array=np.array([self.input_pressure_limit_psi,1.0,1.0])
        elif p1_psi<=1.0:
            pd_array=np.array([1.0,1.0,1.0])
        else:
            pd_array=np.array([p1_psi,1.0,1.0])  
        # # print "uAlpha",uAlpha,"uMin",uMin,"den",(0.5*sphi-0.5*np.sqrt(3)*cphi),"p1_psi",p1_MPa*145.038
        # if uAlpha >= uMax:
        #     pd_array=np.array([25.0,1.0,1.0])
        # elif uAlpha <=uMin:
        #     pd_array=np.array([1.0,1.0,1.0])
        # else:
        #     cphi= np.cos(phi)
        #     sphi= np.sin(phi)
        #     p1=0.
        #     print "den",0.5*sphi-0.5*np.sqrt(3)*cphi
        #     if 0.5*sphi-0.5*np.sqrt(3)*cphi == 0.:
        #         pd_array=np.array([1.0,1.0,1.0])
        #     else:
        #         p1_MPa=(uAlpha/self.alpha0-(0.5*sphi+0.5*np.sqrt(3)*cphi)*pm2_MPa+sphi*pm3_MPa)/(0.5*sphi-0.5*np.sqrt(3)*cphi)
        #         pd_array=np.array([p1_MPa*145.038,1.0,1.0])     
        # Iterate x1, x1 error and time stamp
        self.x1_old=self.x1_current
        self.x1_error_old=self.x1_error_current
        self.t_old=self.t_new
        # print pd_array
        self.send_zipped_socket0(pd_array)

    def positionSumOfSine(self,t):
        xd=0
        i=0
        for ft in self.ftArray:
            xd=xd-self.sum_sin_amp*np.sin(2*np.pi*ft*t+self.phasArray[i])+self.sum_sin_boff
            i=i+1
        return xd

    def positionSingleSine(self,t):
        xd=-self.Amp*np.sin(2*np.pi*self.Freq*t)+self.Boff
        return xd

    def positionMultiSteps(self,t):
        xd=0
        i=0
        for t_i in self.timeStampSteps:
            if t >= t_i:
                xd= self.multiStepAmps[i]
            i=i+1
        return xd

    def positionRampFromT0(self,t):
        xd=0.
        xd=self.x1_t0
        if t <= self.rampAmpAbs/self.rampRateAbs:
            xd= self.x1_t0-self.rampRateAbs*t
            self.flag_ramp_phase=0
        elif t>self.rampAmpAbs/self.rampRateAbs and t<= self.rampAmpAbs/self.rampRateAbs+self.rampFlatTime:
            xd= self.x1_t0-self.rampAmpAbs
            self.flag_ramp_phase=1
        elif t>self.rampAmpAbs/self.rampRateAbs+self.rampFlatTime and t<= (self.rampAmpAbs/self.rampRateAbs)*2.0+self.rampFlatTime:
            xd= self.x1_t0-self.rampAmpAbs +  self.rampRateAbs*(t-(self.rampAmpAbs/self.rampRateAbs+self.rampFlatTime))
            self.flag_ramp_phase=2
        else:
            xd=self.x1_t0-np.radians(5)
            xd=self.x1_t0
            self.flag_ramp_phase=3
        return xd

    def positionProfileSelection(self):
        if self.positionProfile_flag==0:
            return self.positionSumOfSine(self.t_old)
        elif self.positionProfile_flag==1:
            return self.positionSingleSine(self.t_old)
        elif self.positionProfile_flag==2:
            return self.positionMultiSteps(self.t_old)
        elif self.positionProfile_flag==3:
            return self.positionRampFromT0(self.t_old)

    def positionSumOfSineDiscrete(self,ilc_iteration_index):
        xd=0
        i=0
        for ft in self.ftArray:
            xd=xd-self.sum_sin_amp*np.sin(2*np.pi*ft*ilc_iteration_index*self.ilc_exp_delta_t+self.phasArray[i])+self.sum_sin_boff
            i=i+1
        # sleep(self.ilc_delta_t)
        return xd

    def positionSingleSineDiscrete(self,ilc_iteration_index):
        xd=-self.Amp*np.sin(2*np.pi*self.Freq*ilc_iteration_index*self.ilc_exp_delta_t)+self.Boff
        # sleep(self.ilc_delta_t)
        return xd

    def positionMultiStepsDiscrete(self,tilc_iteration_index):
        xd=0
        i=0
        for index_i in self.indexStampSteps:
            if ilc_iteration_index >= index_i:
                xd= self.multiStepAmps[i]
            i=i+1
        # sleep(self.ilc_delta_t)
        return xd
        
    def positionProfileSelectionDiscreteDomain(self):
        if self.positionProfile_flag==0:
            return self.positionSumOfSineDiscrete(self.ilc_iteration_index )
        elif self.positionProfile_flag==1:
            return self.positionSingleSineDiscrete(self.ilc_iteration_index )
        elif self.positionProfile_flag==2:
            return self.positionMultiStepsDiscrete(self.ilc_iteration_index )

    def positionDoubleDerevativeSelection(self):
        ddxd1=0.
        ft=0.
        t=self.t_old
        if self.positionProfile_flag==0:
            i=0

            for ft in self.ftArray:
                ddxd1=ddxd1+self.sum_sin_amp*((2*np.pi*ft)**2)*np.sin(2*np.pi*ft*t+self.phasArray[i])
                i=i+1
        elif self.positionProfile_flag==1:
            ddxd1 =+self.Amp*((2*np.pi*ft)**2)*np.sin(2*np.pi*self.Freq*t)
        elif self.positionProfile_flag==2:
            ddxd1=0.
        elif self.positionProfile_flag==3:
            ddxd1=0.
        return ddxd1

    def positionSingleDerevativeSelection(self):
        dxd1=0.
        ft=0.
        t=self.t_old
        if self.positionProfile_flag==0:
            i=0
            for ft in self.ftArray:
                dxd1=dxd1+self.sum_sin_amp*((2*np.pi*ft))*np.cos(2*np.pi*ft*t+self.phasArray[i])
                i=i+1
        elif self.positionProfile_flag==1:
            dxd1 =self.Amp*((2*np.pi*ft))*np.cos(2*np.pi*self.Freq*t)
        elif self.positionProfile_flag==2:
            dxd1=0.
        elif self.positionProfile_flag==3:
            if self.flag_ramp_phase==0:
                dxd1 = -1.0*self.rampRateAbs
            elif self.flag_ramp_phase==1:
                dxd1 = -0.0*self.rampRateAbs
            elif self.flag_ramp_phase==2:
                dxd1 = 1.0*self.rampRateAbs
            elif self.flag_ramp_phase==3:
                dxd1 = 0.0*self.rampRateAbs
        return dxd1

    def positionDoubleDerevativeSelectionDiscrete(self):
        ddxd1=0.
        ft=0.
        if self.positionProfile_flag==0:
            i=0
            for ft in self.ftArray:
                ddxd1=ddxd1+self.sum_sin_amp*((2*np.pi*ft)**2)*np.sin(2*np.pi*ft*self.ilc_iteration_index*self.ilc_exp_delta_t+self.phasArray[i])
                i=i+1
        elif self.positionProfile_flag==1:
            ddxd1 =+self.Amp*((2*np.pi*ft)**2)*np.sin(2*np.pi*self.Freq*self.ilc_iteration_index*self.ilc_exp_delta_t)
        elif self.positionProfile_flag==2:
            ddxd1=0.
        # sleep(self.ilc_delta_t)
        return ddxd1

    def positionIlcReset(self,pd_array,step_time):
        for i in range(int(step_time/0.005)):
            if self.th1_flag:
                self.send_zipped_socket0(pd_array)
                sleep(0.005)

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

