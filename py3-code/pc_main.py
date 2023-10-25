"""
This code is the PC Client
normal to compliant to recovery 
recorvery to compliant


"""
import arm_main # mode 0high -- low1-pressure + low1-sensor + low2-pressure
import cali_main
import elong_main
import arm2_main
import ai_main
import insmc_main
import basesmc_main
# import sensor_test # mode 1
# import ramp_test # mode 2
# import step_test # mode 3
# from time import time,sleep
import numpy as np
from time import sleep
def main():
    try:
        #### Select control method ####
        flag_ctrl_mode=0
        if flag_ctrl_mode==0:
            p_client=basesmc_main.pc_client()
        elif flag_ctrl_mode==1:
            p_client=insmc_main.pc_client()
        elif flag_ctrl_mode==2:
            p_client=elong_main.pc_client()
        elif flag_ctrl_mode==3:
            p_client=arm2_main.pc_client()

        p_client.positionProfile_flag=3 
        p_client.flag_use_mocap=1
        p_client.trailDuriation= 300

        # p_client.rampRateAbs=np.radians(0.5) # 1 deg/sec
        # p_client.rampAmpAbs=np.radians(15) # x1(t0)-rampAmp
        # p_client.rampFlatTime=5.0 # sec



        p_client.th2.start()
        sleep(0.5)
        p_client.th1.start()
        sleep(0.5)
        p_client.th3.start()
        while 1:
            pass
    except KeyboardInterrupt:
        p_client.th1_flag=False
        p_client.th2_flag=False
        exit()
if __name__ == '__main__':
    main()
