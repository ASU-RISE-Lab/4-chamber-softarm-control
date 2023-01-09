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
from time import time, sleep
class raspi_client(object):
    """docstring for ClassName"""
    def __init__(self):

        #ZMQ
        context = zmq.Context()
        self.socket0 = context.socket(zmq.PUB)
        self.socket0.setsockopt(zmq.CONFLATE,True)
        # self.socket0.bind("tcp://127.0.0.1:3333")## PUB pd pm 
        self.socket0.bind("tcp://10.203.50.11:3333")## PUB pd pm 

        self.socket1 = context.socket(zmq.SUB)
        self.socket1.setsockopt(zmq.CONFLATE,1)
        self.socket1.setsockopt_string(zmq.SUBSCRIBE,'', encoding='utf-8')
        self.socket1.connect("tcp://10.203.52.227:5555")## sub pd 
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
        self.pe_max=1.0 #psi 
        self.pm_offset=np.array([0.,0.,0.])
        self.kp=0
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

    def vout_from_pd(self,line_id,pd):
        if line_id==0:
            channel='in0/gnd'
        elif line_id==1:
            channel='in1/gnd'
        elif line_id==2:
            channel='in2/gnd'
        else:
            channel=None
        v_in=self.adc.get_voltage(channel=channel)
        #   pm_array=(np.array([v0])-0.1*5)*4.0/(0.8*5.0)*14.5038# 5V 58 psi
        # pm=(v_in-0.1*5)*4.0/(0.8*5.0)*14.5038-self.pm_offset[line_id]
        ### exp id regulator pm
        pm=v_in*31.13-29.33-self.pm_offset[line_id]

        #pm=(v_in-0.1*5)*4.0/(0.8*5.0)*14.5038
#         pe=pd-pm
#         v_out=abs(pe)/self.pe_max*4095# 0-pe_max maps to 0-5V maps to 0-4095
        u=pd+self.kp*(pd-pm)
        # v_out=(u-0.3)/60*4095#regualtor control
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
        return pm
    def pressure_sensor_cali(self):
        k=1000
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

def main():
    r_client=raspi_client()
    pd_array=np.array([0.,0.,0.])
    vm_array=np.array([0.,0.,0.])
    pm_array=np.array([0.,0.,0.])
    dt=0.005
    print("Start Pressure Sensor Calibration")
    r_client.pressure_sensor_cali()
    print (r_client.pm_offset)
#     raw_input()
    try:
        pd_array=r_client.recv_zipped_pickle()
        for kk in range(3):
            pm_array[kk]=r_client.vout_from_pd(kk,pd_array[kk])
        while 1:
#         for i in range(1000):
            t_old=time()
            pd_array=r_client.recv_zipped_pickle()
            for kk in range(3):
                pm_array[kk]=r_client.vout_from_pd(kk,pd_array[kk])
            print("pd",pd_array)
            print("pm",np.around(pm_array,1))
            msg=np.concatenate((pd_array, pm_array), axis=None)
            r_client.send_zipped_pickle(msg)
            dt_now=time()-t_old
            # print(msg)
            # print (round(1./dt_now,0),"pe",round(pd_array[0]-pm_array[0],1),round(pd_array[1]-pm_array[1],1),round(pd_array[2]-pm_array[2],1))

            # print 1./dt_now
#             if dt_now <=dt:
#                 sleep(dt-dt_now)
#             else:
#                 pass
# #                 print("loop is too slow")
# #                 print dt_now
        for kk in range(3):
            bahaha=r_client.vout_from_pd(kk,0.)
        sleep(10)
        print("Trail Done")
    except KeyboardInterrupt:
        pd_array=np.array([0.,0.,0.])
        for kk in range(3):
            bahaha=r_client.vout_from_pd(kk,0.)
        sleep(3)
        print("Trail Done")




if __name__ == '__main__':
    main()

        

        
