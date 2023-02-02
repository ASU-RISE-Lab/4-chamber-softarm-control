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

        self.flag_control_mode=3# 0: baseline smc; 
                                # 1: smc+ilc;
                                # 2: smc+spo;
        self.flag_reset = 1
        self.trial_start_reset = 1
        """ Initiate ZMQ communication"""
        context = zmq.Context()

        self.addr_pub_pres_to_low_1 = "tcp://10.203.51.138:4444"
        self.addr_pub_pres_to_low_2 = "tcp://10.203.51.138:5555"

        self.addr_sub_wireEnco_from_low_1 = "tcp://10.203.54.76:5555"


        self.addr_sub_pres_from_low_1 = "tcp://10.203.54.76:3333"
        self.addr_sub_pres_from_low_2 = "tcp://10.203.50.11:3333"

        self.socket0 = context.socket(zmq.PUB)
        self.socket0.setsockopt(zmq.CONFLATE,True)
        self.socket0.bind(self.addr_pub_pres_to_low_1)## PUB pd to low pi 1

        self.socket4 = context.socket(zmq.PUB)
        self.socket4.setsockopt(zmq.CONFLATE,True)
        self.socket4.bind(self.addr_pub_pres_to_low_2)## PUB pd to low pi 2

        self.socket1 = context.socket(zmq.PUB)##PUb to Record
        self.socket1.setsockopt(zmq.CONFLATE,True)
        self.socket1.bind("tcp://127.0.0.1:5555")

        self.socket2=context.socket(zmq.SUB) ### sub mocap data
        self.socket2.setsockopt_string(zmq.SUBSCRIBE,'', encoding='utf-8')
        # self.socket2.
        self.socket2.setsockopt(zmq.CONFLATE,True)

        if self.flag_use_mocap == True:
            self.socket2.connect("tcp://127.0.0.1:3885")
            print ("Connected to mocap")

        self.socket3=context.socket(zmq.SUB) ### sub low 1 pres
        self.socket3.setsockopt_string(zmq.SUBSCRIBE,'', encoding='utf-8')
        self.socket3.setsockopt(zmq.CONFLATE,True)
        self.socket3.connect(self.addr_sub_pres_from_low_1)

        self.socket5=context.socket(zmq.SUB) ### sub low 2 pres
        self.socket5.setsockopt_string(zmq.SUBSCRIBE,'', encoding='utf-8')
        self.socket5.setsockopt(zmq.CONFLATE,True)
        self.socket5.connect(self.addr_sub_pres_from_low_2)


        self.socket6=context.socket(zmq.SUB) ### sub low 1 encoder
        self.socket6.setsockopt_string(zmq.SUBSCRIBE,'', encoding='utf-8')
        self.socket6.setsockopt(zmq.CONFLATE,True)
        self.socket6.connect(self.addr_sub_wireEnco_from_low_1)
        print ("Connected to Low")

        """ Format recording """
        self.array3setswithrotation=np.array([0.]*21)# base(x y z qw qx qy qz) top(x1 y1 z1 qw1 qx1 qy1 qz1)
        self.pd_pm_array_1=np.array([0.]*6) #pd1 pd2 pd3 + pm1 +pm2 +pm3 (psi)
        self.pd_pm_array_2 = self.pd_pm_array_1
        self.filt_array_wireEnco = np.array([0.]*4)
        # assmble all to recording
        self.arr_comb_record=np.concatenate((self.pd_pm_array_1, self.pd_pm_array_2, self.filt_array_wireEnco, self.array3setswithrotation), axis=None)
        

        """ Thearding Setup """
        self.th1_flag=True
        self.th2_flag=True
        self.th3_flag = True
        self.run_event=threading.Event()
        self.run_event.set()
        self.th1=threading.Thread(name='raspi_client',target=self.th_pd_gen)
        self.th2=threading.Thread(name='mocap',target=self.th_data_exchange)
        self.th3 = threading.Thread(name='pub2rec',target=self.th_data_exchange_high)

        """ Common variable"""
        self.t0_on_glob = time()
        self.filter_size = 2
        self.rawVel_array_wireEnco_0 = np.array([0.0]*(self.filter_size))
        self.rawVel_array_wireEnco_1 = np.array([0.0]*(self.filter_size))
        self.rawVel_array_wireEnco_2 = np.array([0.0]*(self.filter_size))
        self.rawVel_array_wireEnco_3 = np.array([0.0]*(self.filter_size))

        self.filtVel_array_wireEnco_0 = np.array([0.0]*(self.filter_size))
        self.filtVel_array_wireEnco_1 = np.array([0.0]*(self.filter_size))
        self.filtVel_array_wireEnco_2 = np.array([0.0]*(self.filter_size))
        self.filtVel_array_wireEnco_3 = np.array([0.0]*(self.filter_size))

        self.rawAcc_array_wireEnco_0 = np.array([0.0]*(self.filter_size))
        self.rawAcc_array_wireEnco_1 = np.array([0.0]*(self.filter_size))
        self.rawAcc_array_wireEnco_2 = np.array([0.0]*(self.filter_size))
        self.rawAcc_array_wireEnco_3 = np.array([0.0]*(self.filter_size))

        self.filtAcc_array_wireEnco_0 = np.array([0.0]*(self.filter_size))
        self.filtAcc_array_wireEnco_1 = np.array([0.0]*(self.filter_size))
        self.filtAcc_array_wireEnco_2 = np.array([0.0]*(self.filter_size))
        self.filtAcc_array_wireEnco_3 = np.array([0.0]*(self.filter_size)) 

        """Initialize SMC Parameter """
        # Actuator geometic parameters

        # SMC state variable and time stamps

        """Input signal selection"""
        self.positionProfile_flag=2#  0: sum of sine waves 1: single sine wave, 2: step
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
        self.multiStepAmps= np.radians(-25)*np.random.random_sample((self.numOfSteps,))+np.radians(-10)

        # self.step_response(np.array([5.0,1.0,1.0]),5)

    def th_pd_gen(self):
        try:
            if self.flag_reset==1:
                self.t0_on_trial = time()
                self.pres_single_step_response(np.array([1.0]*6),10)
                self.flag_reset=0
            self.t0_on_glob = time()
            # print(time()-self.t0_on_glob < self.trailDuriation)
            while (time()-self.t0_on_glob < self.trailDuriation):
                # print("here")
                try:
                    if self.flag_use_mocap == True:
                        # print("here")
                        self.array3setswithrotation=self.recv_cpp_socket2()
                    
                    # print(self.pd_pm_array_2)
                    seg1_r= 10.0
                    seg1_l = 1.0
                    seg1_m = 1.0

                    seg2_r = 10.0
                    seg2_l = 1.0
                    seg2_m = 1.0

                    td = 10
                    pd_array=np.array([seg1_r,seg1_l,seg1_m,seg2_r,seg2_l,seg2_m])
                    if self.trial_start_reset == 1:
                        self.t0_on_trial = time()
                        self.trial_start_reset = 0
                    self.pres_single_step_response(pd_array,td)
                    if self.trial_start_reset == 0:
                        self.t0_on_trial = time()
                        self.trial_start_reset = 1
                    self.pres_single_step_response(np.array([1.0]*6),10)
                    if self.trial_start_reset == 1:
                        self.t0_on_trial = time()
                        self.trial_start_reset = 0
                    self.pres_single_step_response(pd_array,td)
                    # self.pres_single_step_response(np.array([1.0]*6),10)    
                except KeyboardInterrupt:
                    break
                    print("E-stop")
                    self.th1_flag=False
                    self.th2_flag=False
            if self.flag_reset==0:
                self.t0_on_trial = time()
                self.pres_single_step_response(np.array([1.0]*6),10)
                self.flag_reset=1
            self.th1_flag=False
            self.th2_flag=False
            print ("Done")
            exit()
        except KeyboardInterrupt:
            self.th1_flag=False
            self.th2_flag=False
            print ("Press Ctrl+C to Stop")
            
    def th_data_exchange(self):# thread config of read data from mocap and send packed msg to record file.
        while self.run_event.is_set() and self.th2_flag:
            try:
                if self.flag_use_mocap == True:
                    self.array3setswithrotation = self.recv_cpp_socket2()

                self.pd_pm_array_1 = self.recv_zipped_socket3()
                self.pd_pm_array_2 = self.recv_zipped_socket5()
                self.filt_array_wireEnco = self.recv_zipped_socket6()
                print(self.pd_pm_array_1[0:3],self.pd_pm_array_2[0:3])
                # if self.flag_reset==0:
                #     self.send_zipped_socket1(self.arr_comb_record)
            except KeyboardInterrupt:
                break
                self.th1_flag=False
                self.th2_flag=False
                exit()


    def th_data_exchange_high(self):# thread config of read data from mocap and send packed msg to record file.
        while self.run_event.is_set() and self.th3_flag:
            try:
                self.arr_comb_record=np.concatenate((self.pd_pm_array_1, self.pd_pm_array_2, self.filt_array_wireEnco, self.array3setswithrotation), axis=None)
                print(self.pd_pm_array_1[0:3],self.pd_pm_array_2[0:3])
                if self.flag_reset==0:
                    self.send_zipped_socket1(self.arr_comb_record)
            except KeyboardInterrupt:
                break
                exit()

    def pres_single_step_response(self,pd_array,step_time):
        t = time() - self.t0_on_trial # range from 0
        while (self.th1_flag and self.th2_flag and (t <= step_time)):
            try:
                t = time() - self.t0_on_trial # range from 0
                self.pd_pm_array_1[0:3] = pd_array[0:3]
                self.pd_pm_array_2[0] = pd_array[3]
                self.pd_pm_array_2[1] = pd_array[4]
                self.pd_pm_array_2[2] = pd_array[5]
                self.send_zipped_socket0(self.pd_pm_array_1[0:3])
                self.send_zipped_socket4(self.pd_pm_array_2[0:3])
            except KeyboardInterrupt:
                break
                self.th1_flag = 0
                self.th2_flag = 0
                

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

    def send_zipped_socket4(self, obj, flags=0, protocol=-1):
        """pack and compress an object with pickle and zlib."""
        pobj = pickle.dumps(obj, protocol)
        zobj = zlib.compress(pobj)
        self.socket4.send(zobj, flags=flags)

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

    def recv_zipped_socket5(self,flags=0):
        """reconstruct a Python object sent with zipped_pickle"""
        zobj = self.socket5.recv(flags)
        pobj = zlib.decompress(zobj)
        return pickle.loads(pobj)

    def recv_zipped_socket6(self,flags=0):
        """reconstruct a Python object sent with zipped_pickle"""
        zobj = self.socket6.recv(flags)
        pobj = zlib.decompress(zobj)
        return pickle.loads(pobj)

    def recv_cpp_socket2(self):
        strMsg =self.socket2.recv()
        floatArray=np.fromstring(strMsg.decode("utf-8"),dtype = float, sep= ',')
        # print("here")
        # print(floatArray)
        # return self.array3setswithrotation
        return floatArray
        # floatArray=np.fromstring(strMsg)
        # return np.fromstring(strMsg, dtype=float, sep=',')

