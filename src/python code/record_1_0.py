

import zmq
from time import time,sleep,gmtime, strftime
import zlib
import pickle
import numpy as np
class recordFromLocalHost(object):
    """docstring for recordFromLocalHost"""
    def __init__(self):
        context = zmq.Context()
        self.socket2=context.socket(zmq.SUB) ### sub mocap data
        self.socket2.setsockopt(zmq.SUBSCRIBE,'')
        self.socket2.setsockopt(zmq.CONFLATE,True)
        self.socket2.connect("tcp://127.0.0.1:5555")
    def recv_zipped_pickle(self,flags=0):
        """reconstruct a Python object sent with zipped_pickle"""
        zobj = self.socket2.recv(flags)
        pobj = zlib.decompress(zobj)
        return pickle.loads(pobj)


def main():
    try:
        file_name='data_collect_1.txt'
#         msg=np.array([0.]*39)
        lines=""
        print file_name
        raw_input("Press Enter to Start")
        pc_record=recordFromLocalHost()
        with open(file_name,'w+') as data_file:
            msg=pc_record.recv_zipped_pickle()
            t0=time()
            told=t0
            while True:
                msg=pc_record.recv_zipped_pickle()
                lines=str(time()-t0)+"|"+np.array2string(msg,separator='|').replace('[','').replace(']','').replace('\n','')+"|"+"\n"
                data_file.write(lines)
                data_file.flush()
                # sleep(0.01)
                print time()-t0,'at',1.0/(time()-told)
                told=time()
    except KeyboardInterrupt:
        exit()


if __name__ == '__main__':
    main()
