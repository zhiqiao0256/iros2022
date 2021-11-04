"""
This code is the PC Client
normal to compliant to recovery 
recorvery to compliant


"""
import p1_main
import p2_main
import p3_main
import tau_main
import tau_main_xy
import cali_main
from time import time,sleep
import numpy as np
def main():
    try:
        #### Select control method ####
        flag_ctrl_mode=1
        if flag_ctrl_mode==0:
            p_client=cali_main.pc_client()
        elif flag_ctrl_mode==1:
            p_client=p1_main.pc_client()
        elif flag_ctrl_mode==2:
            p_client=p2_main.pc_client()
        elif flag_ctrl_mode==3:
            p_client=p3_main.pc_client()
        elif flag_ctrl_mode==4:
            p_client=tau_main.pc_client()
        elif flag_ctrl_mode==5:
            p_client=tau_main_xy.pc_client()

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
