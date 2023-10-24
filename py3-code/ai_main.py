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
        self.flag_end_test =1
        """ Initiate ZMQ communication"""
        context = zmq.Context()

        self.addr_pub_pres_to_low_1 = "tcp://10.203.55.239:4444"
        self.addr_pub_pres_to_low_2 = "tcp://10.203.55.239:5555"

        self.addr_sub_wireEnco_from_low_1 = "tcp://10.203.54.76:5555"


        self.addr_sub_pres_from_low_1 = "tcp://10.203.54.76:3333"
        self.addr_sub_pres_from_low_2 = "tcp://10.203.54.11:3333"

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
        self.position_d_array = np.array([0.]*8)
        self.position_est_array = np.array([0.]*4)
        self.wireEnco_ini = np.array([1.4692,    1.5103,    1.8416,    1.8475])
        # self.wireEnco_ini = np.array([1.4692,    1.5103,    1.86,    1.86])
        self.d_est_old = np.array([0.]*4)
        # assmble all to recording
        self.arr_comb_record=np.concatenate((self.pd_pm_array_1, self.pd_pm_array_2, self.filt_array_wireEnco, self.array3setswithrotation,self.position_est_array,self.position_d_array,self.d_est_old), axis=None)
        

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
        # Actuator Design parameter
        self.act_r0 = 0.043
        self.t_old = time()
        
        # NDOB parameters
        self.l1 = -5*10**(0)
        self.l2 = -1*10**(0)
        self.l3 = -1*10**(0)
        self.l4 = -5*10**(0)
        self.eta1 = 5*10**2
        self.eta2 = 1*10**4
        self.eta3 = 1*10**2
        self.eta4 = 2*10**4
        self.eta01 = 1*10**2
        self.eta02 = 3*10**2
        self.eta03 = 1*10**2
        self.eta04 = 3*10**2
        # self.step_response(np.array([5.0,1.0,1.0]),5)

        self.a_array = np.array([-np.deg2rad(20),0.01,-np.deg2rad(20),0.01])
        self.b_array = np.array([0.0,0.005,0.0,0.005])
        self.freq = 1/60
    def th_pd_gen(self):
        try:
            if self.flag_reset==1:
                self.t0_on_trial = time()
                seg1_r= 0.0
                seg1_l = 0.0
                seg1_m = 2.0

                seg2_r = 0.0
                seg2_l = 0.0
                seg2_m = 0.0
                self.pres_single_step_response(np.array([seg1_r,seg1_l,seg1_m,seg2_r,seg2_l,seg2_m]),10)
                self.flag_reset=0
            self.t0_on_glob = time()
            try:
                while(self.flag_end_test):
                    td = 60
                    self.seg1and2_position_sin_square_response(td,time())
                    # td = 10
                    # theta1d = np.deg2rad(-20)
                    # l1d = 0.006 # 0-0.019
                    # theta2d = np.deg2rad(-20)
                    # l2d = 0.006 # 0-0.019
                    # self.position_d_array=np.array([theta1d,l1d,theta2d,l2d,0,0,0,0])
                    # self.seg1and2_position_step_response(self.position_d_array,td,time())

                    # td = 10
                    # theta1d = np.deg2rad(-30)
                    # l1d = 0.006 # 0-0.019
                    # theta2d = np.deg2rad(-30)
                    # l2d = 0.006 # 0-0.019
                    # self.position_d_array=np.array([theta1d,l1d,theta2d,l2d,0,0,0,0])
                    # self.seg1and2_position_step_response(self.position_d_array,td,time())
                    self.flag_end_test = 0
            except KeyboardInterrupt:
                    print("E-stop")
                    self.th1_flag=False
                    self.th2_flag=False
            if self.flag_reset==0:
                self.t0_on_trial = time()
                seg1_r= 0.0
                seg1_l = 0.0
                seg1_m = 2.0

                seg2_r = 0.0
                seg2_l = 0.0
                seg2_m = 0.0
                self.pres_single_step_response(np.array([seg1_r,seg1_l,seg1_m,seg2_r,seg2_l,seg2_m]),15)
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
                # print(self.pd_pm_array_1[0:3],self.pd_pm_array_2[0:3])
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
                self.arr_comb_record=np.concatenate((self.pd_pm_array_1, self.pd_pm_array_2, self.filt_array_wireEnco, self.array3setswithrotation,self.position_est_array,self.position_d_array,self.d_est_old), axis=None)
                # print(self.pd_pm_array_1[0:3],self.pd_pm_array_2[0:3])
                if self.flag_reset==0:
                    self.send_zipped_socket1(self.arr_comb_record)
            except KeyboardInterrupt:
                break
                exit()

    def position_sin_square(self,a_array,b_array,freq,t_from_t0_on_glob):
        x1d = a_array[0]*np.sin(2*np.pi*freq*t_from_t0_on_glob)*np.sin(2*np.pi*freq*t_from_t0_on_glob) + b_array[0]
        x2d = a_array[1]*np.sin(2*np.pi*freq*t_from_t0_on_glob)*np.sin(2*np.pi*freq*t_from_t0_on_glob) + b_array[1]
        x3d = a_array[2]*np.sin(2*np.pi*freq*t_from_t0_on_glob)*np.sin(2*np.pi*freq*t_from_t0_on_glob) + b_array[2]
        x4d = a_array[3]*np.sin(2*np.pi*freq*t_from_t0_on_glob)*np.sin(2*np.pi*freq*t_from_t0_on_glob) + b_array[3]

        dtdx1d = a_array[0]*np.cos(2*np.pi*freq*t_from_t0_on_glob)*np.sin(2*np.pi*freq*t_from_t0_on_glob) + b_array[0]
        dtdx2d = a_array[1]*np.cos(2*np.pi*freq*t_from_t0_on_glob)*np.sin(2*np.pi*freq*t_from_t0_on_glob) + b_array[1]
        dtdx3d = a_array[2]*np.cos(2*np.pi*freq*t_from_t0_on_glob)*np.sin(2*np.pi*freq*t_from_t0_on_glob) + b_array[2]
        dtdx4d = a_array[3]*np.cos(2*np.pi*freq*t_from_t0_on_glob)*np.sin(2*np.pi*freq*t_from_t0_on_glob) + b_array[3]
        return np.array([x1d,x2d,x3d,x4d,dtdx1d,dtdx2d,dtdx3d,dtdx4d])

    def position_sin_square_sin(self,a_array,b_array,freq,t_from_t0_on_glob):
        x1d = a_array[0]*np.sin(2*np.pi*freq*t_from_t0_on_glob)+ b_array[0]
        x2d = a_array[1]*np.sin(2*np.pi*freq*t_from_t0_on_glob)*np.sin(2*np.pi*freq*t_from_t0_on_glob) + b_array[1]
        x3d = a_array[2]*np.sin(2*np.pi*freq*t_from_t0_on_glob) + b_array[2]
        x4d = a_array[3]*np.sin(2*np.pi*freq*t_from_t0_on_glob)*np.sin(2*np.pi*freq*t_from_t0_on_glob) + b_array[3]

        dtdx1d = a_array[0]*np.cos(2*np.pi*freq*t_from_t0_on_glob) + b_array[0]
        dtdx2d = a_array[1]*np.cos(2*np.pi*freq*t_from_t0_on_glob)*np.sin(2*np.pi*freq*t_from_t0_on_glob) + b_array[1]
        dtdx3d = a_array[2]*np.cos(2*np.pi*freq*t_from_t0_on_glob) + b_array[2]
        dtdx4d = a_array[3]*np.cos(2*np.pi*freq*t_from_t0_on_glob)*np.sin(2*np.pi*freq*t_from_t0_on_glob) + b_array[3]
        return np.array([x1d,x2d,x3d,x4d,dtdx1d,dtdx2d,dtdx3d,dtdx4d])

    def seg1and2_position_sin_square_response(self,step_time,t0_time):
        t = time() - t0_time # range from 0
        self.t_old =time()
        while (self.th1_flag and self.th2_flag and (t <= step_time)):   
            try:
                t = time() - t0_time # range from 0
                t_new = time()
                # State Parameter update
                # self.position_d_array = self.position_sin_square(self.a_array,self.b_array,self.freq,t)
                self.position_d_array = self.position_sin_square_sin(self.a_array,self.b_array,self.freq,t)
                x1d = self.position_d_array[0]
                x2d =self.position_d_array[1]
                x3d = self.position_d_array[2]
                x4d = self.position_d_array[3] 

                dtdx1d = self.position_d_array[4]
                dtdx2d =self.position_d_array[5]
                dtdx3d = self.position_d_array[6]
                dtdx4d = self.position_d_array[7] 

                s1_l = (self.filt_array_wireEnco[0] - self.wireEnco_ini[0])/5
                s1_r = (self.filt_array_wireEnco[1] - self.wireEnco_ini[1])/5
                s2_l = (self.filt_array_wireEnco[3] - self.wireEnco_ini[3])/5
                s2_r = (self.filt_array_wireEnco[2] - self.wireEnco_ini[2])/5

                x1 = (s1_l-s1_r)/self.act_r0
                x2 = (s1_l+s1_r)/2
                x3 = (s2_l-s2_r)/self.act_r0
                x4 = (s2_l+s2_r)/2
                self.position_est_array = np.array([x1,x2,x3,x4])

                e01 = x1d - x1
                e02 = x2d - x2
                e03 = x3d - x3
                e04 = x4d - x4
                # LPV parameter
                zold1 = self.pd_pm_array_1[4] - self.pd_pm_array_1[3]
                zold2 = self.pd_pm_array_1[4] + self.pd_pm_array_1[3]
                zold3 = self.pd_pm_array_2[4] - self.pd_pm_array_2[3]
                zold4 = self.pd_pm_array_2[4] + self.pd_pm_array_2[3]

                kk1 = -1.767*zold1**2 + 17.55*np.absolute(zold1)+33.471
                kk2 = 10.25*zold2**2 - 325.1*zold2+3299
                kk3 =-1.013*zold3**2+11.55*np.absolute(zold3)+4.419
                kk4 = 15.34*zold4**2 - 474.1*zold4+4475

                d1= 0.9725*zold1**2- 11.2*np.absolute(zold1)+38.471
                d2= -0.9725*zold2**2+ 30.23*zold2+435.471
                d3=  0.1125*zold3**2- 1.2*np.absolute(zold3)+14.471
                d4= 4.34*zold4**2 - 155.21*zold4+2146
                # NDOB estimation
                pxold1 = self.l1*x1
                z_new1 = self.funcRK4_z_update(t_new-self.t_old,self.l1,e01,self.eta1)
                self.d_est_old[0] = z_new1 + pxold1

                pxold2 = self.l2*x2
                z_new2 = self.funcRK4_z_update(t_new-self.t_old,self.l2,e01,self.eta2)
                self.d_est_old[1] = z_new2 + pxold2

                pxold3 = self.l3*x3
                z_new3 = self.funcRK4_z_update(t_new-self.t_old,self.l3,e01,self.eta3)
                self.d_est_old[2] = z_new3 + pxold3

                pxold4 = self.l4*x4
                z_new4 = self.funcRK4_z_update(t_new-self.t_old,self.l4,e01,self.eta4)
                self.d_est_old[3] = z_new4 + pxold4

                # torque calculation
                u1 = d1*(dtdx1d - kk1/d1*x1 +self.eta01*e01+self.eta1*np.sign(e01)-self.d_est_old[0])
                u2 = d2*(dtdx2d - kk2/d2*x2 +self.eta02*e02+self.eta2*np.sign(e02)-self.d_est_old[1])
                u3 = d3*(dtdx3d - kk3/d3*x3 +self.eta03*e03+self.eta3*np.sign(e03)-self.d_est_old[2])
                u4 = d4*(dtdx4d - kk4/d4*x4 +self.eta04*e04+self.eta4*np.sign(e04)-self.d_est_old[3])
                # print(u1,u2,u3,u4)
                pd1_ub = (u2-u1)/2
                pd2_ub = (u2+u1)/2
                pd3_ub = (u4-u3)/2
                pd4_ub = (u4+u3)/2

                pd1 = self.func_input_saturation(pd1_ub)
                pd2 = self.func_input_saturation(pd2_ub)
                pd3 = self.func_input_saturation(pd3_ub)
                pd4 = self.func_input_saturation(pd4_ub)

                self.pd_pm_array_1[0] = pd1
                self.pd_pm_array_1[1] = pd2
                self.pd_pm_array_1[2] = 2.0
                self.pd_pm_array_2[0] = pd3
                self.pd_pm_array_2[1] = pd4
                self.pd_pm_array_2[2] = 0.0
                self.send_zipped_socket0(self.pd_pm_array_1[0:3])
                self.send_zipped_socket4(self.pd_pm_array_2[0:3])
                self.t_old = t_new
                print(self.position_d_array[0:4]-self.position_est_array)
            except KeyboardInterrupt:
                self.th1_flag = 0
                self.th2_flag = 0
        
    def seg1and2_position_step_response(self,position_array,step_time,t0_time):
        t = time() - t0_time # range from 0
        self.t_old =time()
        while (self.th1_flag and self.th2_flag and (t <= step_time)):   
            try:
                t = time() - t0_time # range from 0
                t_new = time()
                # State Parameter update
                x1d = self.position_d_array[0]
                x2d =self.position_d_array[1]
                x3d = self.position_d_array[2]
                x4d = self.position_d_array[3] 

                dtdx1d = self.position_d_array[4]
                dtdx2d =self.position_d_array[5]
                dtdx3d = self.position_d_array[6]
                dtdx4d = self.position_d_array[7] 

                s1_l = (self.filt_array_wireEnco[0] - self.wireEnco_ini[0])/5
                s1_r = (self.filt_array_wireEnco[1] - self.wireEnco_ini[1])/5
                s2_l = (self.filt_array_wireEnco[2] - self.wireEnco_ini[2])/5
                s2_r = (self.filt_array_wireEnco[3] - self.wireEnco_ini[3])/5

                x1 = (s1_l-s1_r)/self.act_r0
                x2 = (s1_l+s1_r)/2
                x3 = (s2_l-s2_r)/self.act_r0
                x4 = (s2_l+s2_r)/2
                self.position_est_array = np.array([x1,x2,x3,x4])

                e01 = x1d - x1
                e02 = x2d - x2
                e03 = x3d - x3
                e04 = x4d - x4
                # LPV parameter
                zold1 = self.pd_pm_array_1[4] - self.pd_pm_array_1[3]
                zold2 = self.pd_pm_array_1[4] + self.pd_pm_array_1[3]
                zold3 = self.pd_pm_array_2[4] - self.pd_pm_array_2[3]
                zold4 = self.pd_pm_array_2[4] + self.pd_pm_array_2[3]

                kk1 = -1.767*zold1**2 + 17.55*np.absolute(zold1)+33.471
                kk2 = 10.25*zold2**2 - 325.1*zold2+3299
                kk3 =-1.013*zold3**2+11.55*np.absolute(zold3)+4.419
                kk4 = 15.34*zold4**2 - 474.1*zold4+4475

                d1= 0.9725*zold1**2- 11.2*np.absolute(zold1)+38.471
                d2= -0.9725*zold2**2+ 30.23*zold2+435.471
                d3=  0.1125*zold3**2- 1.2*np.absolute(zold3)+14.471
                d4= 4.34*zold4**2 - 155.21*zold4+2146
                # NDOB estimation
                pxold1 = self.l1*x1
                z_new1 = self.funcRK4_z_update(t_new-self.t_old,self.l1,e01,self.eta1)
                self.d_est_old[0] = z_new1 + pxold1

                pxold2 = self.l2*x2
                z_new2 = self.funcRK4_z_update(t_new-self.t_old,self.l2,e01,self.eta2)
                self.d_est_old[1] = z_new2 + pxold2

                pxold3 = self.l3*x3
                z_new3 = self.funcRK4_z_update(t_new-self.t_old,self.l3,e01,self.eta3)
                self.d_est_old[2] = z_new3 + pxold3

                pxold4 = self.l4*x4
                z_new4 = self.funcRK4_z_update(t_new-self.t_old,self.l4,e01,self.eta4)
                self.d_est_old[3] = z_new4 + pxold4

                # torque calculation
                u1 = 1/d1*(dtdx1d - kk1*x1 +self.eta01*e01+self.eta1*np.sign(e01)-self.d_est_old[0])
                u2 = 1/d2*(dtdx2d - kk2*x2 +self.eta02*e02+self.eta2*np.sign(e02)-self.d_est_old[1])
                u3 = 1/d3*(dtdx3d - kk3*x3 +self.eta03*e03+self.eta3*np.sign(e03)-self.d_est_old[2])
                u4 = 1/d4*(dtdx4d - kk4*x4 +self.eta04*e04+self.eta4*np.sign(e04)-self.d_est_old[3])
                # print(u1,u2,u3,u4)
                pd1_ub = (u2-u1)/2
                pd2_ub = (u2+u1)/2
                pd3_ub = (u4-u3)/2
                pd4_ub = (u4+u3)/2

                pd1 = self.func_input_saturation(pd1_ub)
                pd2 = self.func_input_saturation(pd2_ub)
                pd3 = self.func_input_saturation(pd3_ub)
                pd4 = self.func_input_saturation(pd4_ub)

                self.pd_pm_array_1[0] = pd1
                self.pd_pm_array_1[1] = pd2
                self.pd_pm_array_1[2] = 2.0
                self.pd_pm_array_2[0] = pd3
                self.pd_pm_array_2[1] = pd4
                self.pd_pm_array_2[2] = 0.0
                self.send_zipped_socket0(self.pd_pm_array_1[0:3])
                self.send_zipped_socket4(self.pd_pm_array_2[0:3])
                self.t_old = t_new
                print(self.position_d_array[0:4]-self.position_est_array)
            except KeyboardInterrupt:
                self.th1_flag = 0
                self.th2_flag = 0


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

    def pres_single_step_response_v2(self,pd_array,step_time,t0_time):
        t = time() - t0_time # range from 0
        while (self.th1_flag and self.th2_flag and (t <= step_time)):
            try:
                t = time() - t0_time # range from 0
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

    def func_input_saturation(self,pdi_ub):
        if pdi_ub <=0:
            pd =0
        elif pdi_ub>=20:
            pd =20
        else:
            pd = pdi_ub
        return pd
        
    def funcRK4_z_update(self,dt,li,e0i,etai):
        xold_k1 = e0i
        dtdintvar = -li*etai*np.sign(xold_k1)
        k1 = dtdintvar

        xold_k2 = e0i + 0.5*dt*k1
        dtdintvar = -li*etai*np.sign(xold_k2)
        k2 = dtdintvar

        xold_k3 = e0i + 0.5*dt*k2
        dtdintvar = -li*etai*np.sign(xold_k3)
        k3 = dtdintvar

        xold_k4 = e0i + 0.5*dt*k3
        dtdintvar = -li*etai*np.sign(xold_k4)
        k4 = dtdintvar

        x_new = e0i + dt/6*(k1 + 2*k2 + 2*k3 +k4);
        return x_new

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

