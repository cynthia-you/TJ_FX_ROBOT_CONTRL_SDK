from fx_kine import Marvin_Kine
import os
import random
import logging
import time


logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)# 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级

current_path=os.getcwd()
kk = Marvin_Kine()
'''
配置导入
!!! 非常重要！！！
使用前，请一定确认机型，导入正确的配置文件，文件导错，计算会错误啊啊啊,甚至看起来运行正常，但是值错误！！！
'''
ini_result=kk.load_config(config_path='ccs.MvKDCfg')
time.sleep(0.5)
'''
初始化动力学
#一定要确认robot_serial是左臂0 还是右臂1
本DEMO中仅示例了单臂的计算
如果人形，选择右臂， 则为：
initial_kine_tag=kk.initial_kine(robot_serial=1,
                                 robot_type=ini_result['TYPE'][1],
                                 dh=ini_result['DH'][1],
                                 pnva=ini_result['PNVA'][1],
                                 j67=ini_result['BD'][1])
其余有robot_serial参数作为输入的都要确认双臂还是单臂，不再赘述。
'''
initial_kine_tag=kk.initial_kine(robot_serial=0,
                                 robot_type=ini_result['TYPE'][0],
                                 dh=ini_result['DH'][0],
                                 pnva=ini_result['PNVA'][0],
                                 j67=ini_result['BD'][0])
time.sleep(0.5)
print('-'*50)


joints1=[32,-64,72,59,-107,-30,58]
mat1=kk.fk(robot_serial=0,joints=joints1)
xyzabc1=kk.mat4x4_to_xyzabc(pose_mat=mat1)
logger.info(f'---current_xyzabc:{xyzabc1}')

joints2=[24.710327454579385, -34.626590037831534, 129.573376956529, 58.999999819257596, -154.5774252821101, -9.250648598649414, 76.73495447348635]
mat2=kk.fk(robot_serial=0,joints=joints2)
xyzabc2=kk.mat4x4_to_xyzabc(pose_mat=mat2)
logger.info(f'---adjust joints, xyzabc:{xyzabc2}')


target_xyzabc=xyzabc2.copy()
target_xyzabc[2]-=100
logger.info(f'---z-100 ,target xyzabc:{target_xyzabc}')
joints3=[30.441992064349336, -41.66004132484181, 124.19196073906699, 73.1034294701139, -147.2634786470432, 5.414599152769855, 71.29893883226201]
mat3=kk.fk(robot_serial=0,joints=joints3)
xyzabc3=kk.mat4x4_to_xyzabc(pose_mat=mat3)
logger.info(f'---after optimize, target xyzabc:{xyzabc3}')