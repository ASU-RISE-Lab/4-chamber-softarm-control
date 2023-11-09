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
import ADS1263
import RPi.GPIO as GPIO
import threading

class wireEnco(object):
    """docstring for pc_client"""
    def __init__(self):
        context = zmq.Context()
        self.addr_pub_wireEnco_to_high = "tcp://10.203.54.76:5555"
        self.addr_pub_wireEnco_to_local = "tcp://127.0.0.1:5555"
        self.socket0 = context.socket(zmq.PUB)
        self.socket0.setsockopt(zmq.CONFLATE,True)
        self.socket0.bind(self.addr_pub_wireEnco_to_high)## PUB pd to low pi 1
        self.socket1 = context.socket(zmq.PUB)
        self.socket1.setsockopt(zmq.CONFLATE,True)
        self.socket1.bind(self.addr_pub_wireEnco_to_local)## PUB pd to low pi 1

        """ Format recording """
        self.array_wireEnco = np.array([0.]*4)
        self.filt_array_wireEnco = np.array([0.]*4)
        self.filter_size = 10
        self.array_wireEnco_0 = np.array([1.0]*self.filter_size)
        self.array_wireEnco_1 = np.array([1.0]*self.filter_size)
        self.array_wireEnco_2 = np.array([1.0]*self.filter_size)
        self.array_wireEnco_3 = np.array([1.0]*self.filter_size)

        # Modify according to actual voltage
        # external AVDD and AVSS(Default), or internal 2.5V
        self.REF = 5.08

        # The faster the rate, the worse the stability
        # and the need to choose a suitable digital filter(REG_MODE1)
        self.ADC = ADS1263.ADS1263()

        if (self.ADC.ADS1263_init_ADC1('ADS1263_1200SPS') == -1):
            exit()
        self.ADC.ADS1263_SetMode(1)   # 0 is singleChannel, 1 is diffChannel    

        self.th1_flag=True
        self.th2_flag=True
        self.run_event=threading.Event()
        self.run_event.set()
        self.th1=threading.Thread(name='sensor_capture',target=self.th_filter_reading)
        self.th2=threading.Thread(name='data_exchange',target=self.th_data_exchange)
        self.t0 =time()
        self.dt = 0.0
        self.dt2 = 0.0
        self.t_old = time()
        self.t_oldmsg = time()
    def th_filter_reading(self):
        while (1):
            try:
                REF = 5.08 
                channelList = [0, 1, 2, 3]
                ADC_Value = self.ADC.ADS1263_GetAll(channelList)
                for i in channelList:
                    if(ADC_Value[i]>>31 ==1):
                        self.array_wireEnco[i] = (REF*2 - ADC_Value[i] * REF / 0x80000000)
                    else:
                        self.array_wireEnco[i] = (ADC_Value[i] * REF / 0x7fffffff)# 32bit
                # send data to buffer        
                self.array_wireEnco_0[self.filter_size-1] = self.array_wireEnco[0]
                self.array_wireEnco_1[self.filter_size-1] = self.array_wireEnco[1]
                self.array_wireEnco_2[self.filter_size-1] = self.array_wireEnco[2]
                self.array_wireEnco_3[self.filter_size-1] = self.array_wireEnco[3]

                # 3pts mv
                self.filt_array_wireEnco[0] = np.average(self.array_wireEnco_0)
                self.filt_array_wireEnco[1] = np.average(self.array_wireEnco_1)
                self.filt_array_wireEnco[2] = np.average(self.array_wireEnco_2)
                self.filt_array_wireEnco[3] = np.average(self.array_wireEnco_3)

                # roll the oldest data to the last of array and wait for replaced by new reading
                temp_array =  np.roll(self.array_wireEnco_0,-1)
                self.array_wireEnco_0 = temp_array
                temp_array =  np.roll(self.array_wireEnco_1,-1)
                self.array_wireEnco_1 = temp_array
                temp_array =  np.roll(self.array_wireEnco_2,-1)
                self.array_wireEnco_2 = temp_array
                temp_array =  np.roll(self.array_wireEnco_3,-1)
                self.array_wireEnco_3 = temp_array
                self.dt = time() - self.t_old
                self.t_old = time()
                # print('new sample',np.around(self.array_wireEnco,2))
                # print('filt buffer 0',np.around(self.array_wireEnco_0,2))
                # print('data sendout',np.around(self.filt_array_wireEnco,2)) 
            except KeyboardInterrupt:
                break
                self.th2_flag = False        

    def read_4_ch(self):
        REF = 5.08 
        channelList = [0, 1, 2, 3]
        ADC_Value = self.ADC.ADS1263_GetAll(channelList)
        for i in channelList:
            if(ADC_Value[i]>>31 ==1):
                self.array_wireEnco[i] = (REF*2 - ADC_Value[i] * REF / 0x80000000)
            else:
                self.array_wireEnco[i] = (ADC_Value[i] * REF / 0x7fffffff)# 32bit
        self.send_zipped_socket0(self.array_wireEnco)
        print(np.around(self.array_wireEnco,2))

            

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

    def th_data_exchange(self):
        while (self.th2_flag ):
            try:
                if (self.dt >0):
                        
                    self.send_zipped_socket0(self.filt_array_wireEnco)
                    # self.send_zipped_socket1(np.concatenate((self.array_wireEnco,self.filt_array_wireEnco), axis=None))
                    self.dt2 = time()-self.t_oldmsg
                    self.t_oldmsg=time()
                    print('dt',np.round(self.dt,4),'dtmsg',np.round(self.dt2,4),np.around(self.filt_array_wireEnco,2))        
            except KeyboardInterrupt:
                break
                print("E-stop Enco")           


def main():
    try:
        enco_obj = wireEnco()
        enco_obj.th2.start()
        sleep(0.5)
        enco_obj.th1.start()
        while 1:
            pass
    except KeyboardInterrupt:
        print("E-stop")
        enco_obj.th1_flag = False
        enco_obj.th2_flag = False

        exit()
if __name__ == '__main__':
    main()

