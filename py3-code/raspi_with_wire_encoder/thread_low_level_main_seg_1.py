"""
This code is for Raspi Client
Hardware list:
    1. I2c Multiplx
    2. 3 DACs
    3. 3 Regulator SMC ITV050
    4. 1 ADC
    6. 1 24V power supply
    7. 1 12V supply for Raspi
"""
import numpy as np
import zmq
import Adafruit_MCP4725 as DAC
from ads1015 import ADS1015 
import pickle
import zlib
import tca9548a
import threading
from time import time, sleep

class raspi_client(object):
    """docstring for ClassName"""
    def __init__(self):

        #ZMQ
        context = zmq.Context()
        self.socket0 = context.socket(zmq.PUB)
        self.socket0.setsockopt(zmq.CONFLATE,True)
        # self.socket0.bind("tcp://127.0.0.1:3333")## PUB pd pm 
        self.socket0.bind("tcp://10.203.54.76:3333")## PUB pd pm 

        self.socket1 = context.socket(zmq.SUB)
        self.socket1.setsockopt(zmq.CONFLATE,1)
        self.socket1.setsockopt_string(zmq.SUBSCRIBE,'', encoding='utf-8')
        self.socket1.connect("tcp://10.203.55.239:4444")## sub pd 
        # DAC + I2Cs
        self.tca_driver = tca9548a.TCA9548A(0x70)
        self.tca_driver.set_channel(0,1)
        self.dac_0=DAC.MCP4725()
        self.dac_0.set_voltage(0)
        self.tca_driver.set_channel(0,0)
        
        self.tca_driver.set_channel(1,1)
        self.dac_1=DAC.MCP4725()
        self.dac_1.set_voltage(0)
        self.tca_driver.set_channel(1,0)
        
        self.tca_driver.set_channel(2,1)
        self.dac_2=DAC.MCP4725()
        self.dac_2.set_voltage(0)
        self.tca_driver.set_channel(2,0)
        # ADC and 
        self.adc=ADS1015()
        self.adc.set_mode('single')
        self.adc.set_programmable_gain(6.144)
        self.adc.set_sample_rate(1600)
        # Other
        self.freq_t_old = time()
        self.pe_max=1.0 #psi 
        self.pm_offset=np.array([0.,0.,0.])
        self.kp=0
        self.pm_array = np.array([0.,0.,0.])
        self.pd_array = np.array([0.,0.,0.])
        self.print_flag = False
        self.th1_flag=True
        self.th2_flag=True
        self.run_event=threading.Event()
        self.run_event.set()
        self.th1=threading.Thread(name='sensor_capture',target=self.th_pd_ctrl)
        self.th2=threading.Thread(name='data_exchange',target=self.th_data_exchange)

    def th_pd_ctrl(self):
        print("Start Pressure Sensor Calibration")
        self.pressure_sensor_cali()
        input("press Enter to continue")
        self.print_flag = True
        while self.th1_flag:
            try:
                self.pd_array=self.recv_zipped_pickle()
                for kk in range(3):
                    self.vout_from_pd(kk,self.pd_array[kk])
                    self.pm_array[kk] = self.read_adc_ch(kk)
            except KeyboardInterrupt:
                pd_array=np.array([0.,0.,0.])
                for kk in range(3):
                    self.vout_from_pd(kk,0.)
                self.th1_flag = 0
                break
                print("E-stop1")
 
    def th_data_exchange(self):
        while self.th2_flag:
            try:
                msg=np.concatenate((self.pd_array, self.pm_array), axis=None)                
                if self.print_flag:
                    self.send_zipped_pickle(msg)
                    if time() - self.freq_t_old <= 0.001:
                        print("freq:",1000)
                    else:
                        print("freq:",np.round(1.0/(time() - self.freq_t_old),0))
                    self.freq_t_old = time()
                    print("pd",self.pd_array)
                    print("pm",np.around(self.pm_array,1))        
            except KeyboardInterrupt:
                pd_array=np.array([0.,0.,0.])
                for kk in range(3):
                    self.vout_from_pd(kk,0.)
                self.th2_flag = 0
                break
                print("E-stop2")           

    def send_zipped_pickle(self, obj, flags=0, protocol=-1):
        """pack and compress an object with pickle and zlib."""
        pobj = pickle.dumps(obj, protocol)
        zobj = zlib.compress(pobj)
        self.socket0.send(zobj, flags=flags)

    def recv_zipped_pickle(self,flags=0):
        """reconstruct a Python object sent with zipped_pickle"""
        zobj = self.socket1.recv(flags)
        pobj = zlib.decompress(zobj)
        return pickle.loads(pobj)

    def read_adc_ch(self,line_id):
        if line_id==0:
            channel='in0/gnd'
        elif line_id==1:
            channel='in1/gnd'
        elif line_id==2:
            channel='in2/gnd'
        else:
            channel=None  
        v_in=self.adc.get_voltage(channel=channel)
        ### exp id regulator pm
        # pm=v_in*31.13-29.33-self.pm_offset[line_id]
        pm=v_in*31.13-29.33
        return pm

    def vout_from_pd(self,line_id,pd):      
        # v_out=abs(pe)/self.pe_max*4095# 0-pe_max maps to 0-5V maps to 0-4095
        if line_id == 0:
            u = pd
        else:
            u=pd
        v_out=(u)/60*4095#regualtor control
        # print u,v_out
        if v_out>=4095:
            v_out=4095
        elif v_out<=0:
            v_out=0
        if line_id==0:
            self.tca_driver.set_channel(line_id,1)
            self.dac_0.set_voltage(int(v_out))
            self.tca_driver.set_channel(line_id,0)
        elif line_id==1:
            self.tca_driver.set_channel(line_id,1)
            self.dac_1.set_voltage(int(v_out))
            self.tca_driver.set_channel(line_id,0)
        elif line_id==2:
            self.tca_driver.set_channel(line_id,1)
            self.dac_2.set_voltage(int(v_out))
            self.tca_driver.set_channel(line_id,0)
        

    def pressure_sensor_cali(self):
        k=100
        pm_off=np.array([0.,0.,0.])
        for i in range(k):
            v_in_0=self.adc.get_voltage(channel='in0/gnd')
            v_in_1=self.adc.get_voltage(channel='in1/gnd')
            v_in_2=self.adc.get_voltage(channel='in2/gnd')
            # print np.array([v_in_0,v_in_1,v_in_2])
            # pm_off=pm_off+((np.array([v_in_0,v_in_1,v_in_2])-0.1*5)*4.0/(0.8*5.0)*14.5038)
            pm_off=pm_off+(np.array([v_in_0,v_in_1,v_in_2])*31.13-29.33)
        pm_off=pm_off/k
        self.pm_offset=pm_off
        print(np.around(self.pm_offset,1))

def main():
    try:
        r_client=raspi_client()
        r_client.th2.start()
        sleep(0.5)
        r_client.th1.start()
        while 1:
            pass
    except KeyboardInterrupt:
        print("E-stop")
        r_client.th1_flag = False
        r_client.th2_flag = False
        for kk in range(3):
            r_client.vout_from_pd(kk,0.)
        exit()

if __name__ == '__main__':
    main()

        

        
