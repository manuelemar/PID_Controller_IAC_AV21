import pandas as pd
import numpy as np
import io


sheet='Torque_RPM'
df=pd.read_excel(r'Callas Characteristic.xlsx',sheet_name='Torque_RPM')


r_w=0.333
k_gf=50/11
k_g=(35/12, 28/15, 22/16, 20/18, 20/21, 24/27)



def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return idx



def calculate_thr(RPM, force_desired, gear_num):
    data=df

    

    Torque=data[24:41]
    Torque=Torque[Torque.columns[3:14]]
    

    desired_RPM=RPM
    desired_torque=0.1*(force_desired*r_w/k_gf)/k_g[gear_num-1]
    
    #in deca neuton meters

    RPM1=[500,1500,2500,6300,6600,6900,7200,7500,7800,8100,8400,8700,9000,9300,9600,9900,10200]
    RPM1=np.array(RPM1)
    rpm_idx=find_nearest(RPM1, desired_RPM)
    
    Torque1=Torque[rpm_idx:rpm_idx+1]
    Torque_list=Torque1.values.tolist()
    Torque_list=np.array(Torque_list)


    thr_idx=find_nearest(Torque_list,desired_torque)
    return thr_idx/10.0
    
