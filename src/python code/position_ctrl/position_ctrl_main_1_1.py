"""
This code is the PC Client
normal to compliant to recovery 
recorvery to compliant


"""

import test1
from time import time,sleep
import numpy as np
def main():
    try:
        #### Select control method ####
        flag_ctrl_mode=0# 0
        if flag_ctrl_mode==0:
            p_client=test1.pc_client()
            p_client.T=60
            p_client.a=-1
            p_client.freq=0.01
        p_client.th2.start()
        sleep(0.5)
        p_client.th1.start()
        while 1:
            pass
    except KeyboardInterrupt:
        exit()
        p_client.th1_flag=False
        p_client.th2_flag=False
        # p_client.socket0.unbind("tcp://10.203.53.226:4444")#
        exit()
if __name__ == '__main__':
    main()
