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

class wireEnco(object):
    """docstring for pc_client"""
    def __init__(self):
        context = zmq.Context()
        self.addr_pub_wireEnco_to_high = "tcp://10.203.54.76:5555"
        self.socket0 = context.socket(zmq.PUB)
        self.socket0.setsockopt(zmq.CONFLATE,True)
        self.socket0.bind(self.addr_pub_wireEnco_to_high)## PUB pd to low pi 1

        """ Format recording """
        self.array_wireEnco = np.array([0.]*4)

        # Modify according to actual voltage
        # external AVDD and AVSS(Default), or internal 2.5V
        self.REF = 5.08

        # The faster the rate, the worse the stability
        # and the need to choose a suitable digital filter(REG_MODE1)
        self.ADC = ADS1263.ADS1263()

        if (self.ADC.ADS1263_init_ADC1('ADS1263_400SPS') == -1):
            exit()
        self.ADC.ADS1263_SetMode(1)   # 0 is singleChannel, 1 is diffChannel    

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

def main():
    enco_obj = wireEnco()
    while (1):
        try:
            enco_obj.read_4_ch()
        except IOError as e:
            break
            print(e)
           
        except KeyboardInterrupt:
            break
            print("ctrl + c:")
            print("Program end")
            ADC.ADS1263_Exit()
            exit()
if __name__ == '__main__':
    main()

