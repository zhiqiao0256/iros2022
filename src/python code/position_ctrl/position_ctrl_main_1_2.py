"""
This code is the PC Client
normal to compliant to recovery 
recorvery to compliant


"""

import smc_main
import smart_main
import smcndo_main
import smart2_main
from time import time,sleep
import numpy as np
def main():
    try:
        # smc=smc_main()
        # smart=smart_main()
        # smcndo=smcndo_main()
        #### Select control method ####
        flag_ctrl_mode=2# 0 SMC
                        # 1 SMCNDO
                        # 2 SMART
        if flag_ctrl_mode==0:
            p_client=smc_main.pc_client()
        elif flag_ctrl_mode==1:
            p_client=smcndo_main.pc_client()
        elif flag_ctrl_mode==2:
            p_client=smart_main.pc_client()
        elif flag_ctrl_mode==3:
            p_client=smart2_main.pc_client()
        p_client.positionProfile_flag=3 
        p_client.flag_use_mocap=1
        p_client.trailDuriation=60.
        p_client.smcNDOB_lambda= 10.0
        p_client.smcNDOB_k_sig = 30.0
        p_client.smcNDOB_eta = 40.0
        p_client.smcNDOB_kp_sig = 10.0
        p_client.smcNDOB_th=0.5# N.m
        p_client.mode_C_to_R_th=0.3
        p_client.tau_max=8.0 #N.m
        p_client.x1_error_max_value=np.deg2rad(1)

        p_client.rampRateAbs=np.radians(1.) # 1 deg/sec
        p_client.rampAmpAbs=np.radians(15) # x1(t0)-rampAmp
        p_client.rampFlatTime=20.0 # sec
        p_client.timer_th=3.0#sec


        p_client.th2.start()
        sleep(0.5)
        p_client.th1.start()
        while 1:
            pass
    except KeyboardInterrupt:
        p_client.th1_flag=False
        p_client.th2_flag=False
        p_client.socket0.unbind("tcp://10.203.53.226:4444")#
        exit()
if __name__ == '__main__':
    main()
