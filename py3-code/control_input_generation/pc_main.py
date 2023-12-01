"""
This code is the PC Client

"""
import ai2_main
import insmc_main
import basesmc2_main
import openloop_main
import numpy as np
from time import sleep
def main():
    try:
        #### Select control method ####
        flag_ctrl_mode=3
        if flag_ctrl_mode==0:
            p_client=basesmc2_main.pc_client()
        elif flag_ctrl_mode==1:
            p_client=insmc_main.pc_client()
        elif flag_ctrl_mode==2:
            p_client=ai2_main.pc_client()
        elif flag_ctrl_mode==3:
            p_client=openloop_main.pc_client()

        p_client.positionProfile_flag=3 
        p_client.flag_use_mocap=1
        p_client.trailDuriation= 300

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
