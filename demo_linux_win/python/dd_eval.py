from fx_kine import Marvin_Kine
import os
import random
import logging


logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)# 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级

current_path=os.getcwd()
kk = Marvin_Kine()

kk.initial_marvin_config(serial=1,config_path=os.path.join(current_path,'MARVINKINE_CONFIG'))  # serial=1为CCS，config_path用相对路径

joints1=[32,-64,72,59,-107,-30,58]
mat1=kk.fk(joints=joints1)
xyzabc1=kk.mat2xyzabc(mat4x4=mat1)
logger.info(f'current_xyzabc:{xyzabc1}')

joints2=[34.005621445796635, -54.91758495359238, 86.43905004673366, 59.000000000000014, -121.23002585693729, -22.31040423857405, 61.8305081865891]
mat2=kk.fk(joints=joints2)
xyzabc2=kk.mat2xyzabc(mat4x4=mat2)
logger.info(f'adjust joints, xyzabc:{xyzabc2}')


target_xyzabc=xyzabc2.copy()
target_xyzabc[2]-=100
logger.info(f'z-100 ,target xyzabc:{target_xyzabc}')
joints3=[-21.515618891988254, -40.354537221724456, 188.5638949223844, 78.00657664472239, -217.63049188477964, -59.90811474277217, 25.856706102699548]
mat3=kk.fk(joints=joints3)
xyzabc3=kk.mat2xyzabc(mat4x4=mat3)
logger.info(f'after optimize, target xyzabc:{xyzabc3}')