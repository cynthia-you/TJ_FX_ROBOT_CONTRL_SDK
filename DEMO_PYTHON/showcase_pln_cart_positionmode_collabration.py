import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)
current_file_path = os.path.abspath(__file__)
current_path = os.path.dirname(current_file_path)
from SDK_PYTHON.fx_kine import Marvin_Kine,FX_InvKineSolvePara,convert_to_8x8_matrix


'''#################################################################
该DEMO 为机器人计算SDK 两个手臂同时计算的演示脚本

'''#################################################################

'''实列化计算'''
kk1=Marvin_Kine() # LEFT ARM

kk2=Marvin_Kine() # RIGHT ARM

'''
配置导入
!!! 非常重要！！！
使用前，请一定确认机型，导入正确的配置文件config_path，文件导错，计算会错误啊啊啊,甚至看起来运行正常，但是值错误！！！
一定要确认arm_type是左臂0 还是右臂1
'''
ini_result1=kk1.load_config(arm_type=0,config_path=os.path.join(parent_dir,'CommonConfig/ccs_m6_40.MvKDCfg'))
print(ini_result1)

ini_result2=kk2.load_config(arm_type=1,config_path=os.path.join(parent_dir,'CommonConfig/ccs_m6_40.MvKDCfg'))
print(ini_result2)
print('-'*50)

'''
初始化动力学
'''
initial_kine_tag=kk1.initial_kine(
                                 robot_type=ini_result1['TYPE'][0],
                                 dh=ini_result1['DH'][0],
                                 pnva=ini_result1['PNVA'][0],
                                 j67=ini_result1['BD'][0])

initial_kine_tag=kk2.initial_kine(
                                 robot_type=ini_result1['TYPE'][1],
                                 dh=ini_result1['DH'][1],
                                 pnva=ini_result1['PNVA'][1],
                                 j67=ini_result1['BD'][1])
print('-'*50)

kk1.log_switch(0)
kk2.log_switch(0)


'''===========movla================='''
start0=[509.734, 233.609, 365.948, -169.144, 55.011, -146.752]
end0=[509.734, 233.609, 265.948, -169.144, 55.011, -146.752]
ref0=[19.597, -32.480, 10.050, -58.939, -8.863, -33.821, 4.772]
start1=[509.734, -233.609, 365.948, 169.144, 55.011, 146.752]
end1=[509.734, -233.609, 265.948, 169.144, 55.011, 146.752]
ref1=[-19.597, -32.480, -10.050, -58.939, 8.863, -33.821, -4.772]
vel=100
acc=100
freq=50
points0, pset1 = kk1.movLA(start0, end0, ref0, vel, acc, freq)
if pset1==None:
    print("Error", f"Left arm planning failed.")
points1, pset2 = kk2.movLA(start1, end1, ref1, vel, acc, freq)
if pset2==None:
    print("Error", f"right arm planning failed.")
print(f"MOVLA success: left arm points: {len(points0)}, right arm points: {len(points1)}")


points_arm0 = [
            [509.731, 233.614, 265.949, -169.144, 55.011, -146.752],
            [509.731, 233.614, 65.949, -169.144, 55.011, -146.752],
            [509.731, 33.614, 65.949, -169.144, 55.011, -146.752],
            [509.731, 33.614, 265.949, -169.144, 55.011, -146.752]
        ]
start_joints_arm0=[17.970, -35.197, 11.414, -73.344, -9.154, -17.035, 7.086]

points_arm1 = [
            [509.731, -233.614, 265.949, 169.144, 55.011, 146.752],
            [509.731, -233.614, 65.949, 169.144, 55.011, 146.752],
            [509.731, -33.614, 65.949, 169.144, 55.011, 146.752],
            [509.731, -33.614, 265.949, 169.144, 55.011, 146.752]
        ]
start_joints_arm1=[-17.970, -35.197, -11.414, -73.344, 9.154, -17.035, -7.086]

zsp_type=1
zsp_params=[0,0,-1,0,0,0]
allow_range=5
vel=100
acc=100
freq=50


tag_multi_start = kk1.multi_movL_set_start(start_joints_arm0, points_arm0[0], points_arm0[1],
                                           allow_range,
                                           zsp_type, zsp_params, vel, acc, freq)
if not tag_multi_start:
    print("Error", 'multi-segment planing: set start failed')

for next_one in points_arm0[2:]:
    ret1 = kk1.multi_movL_next_point(next_one, allow_range, zsp_type, zsp_params, vel, acc)
    if not ret1:
        print("Error", f"multi-segment planing: set next failed")

data, pset1 = kk1.multi_movL_get_points()
if pset1 == None:
    print("Error", f"Left arm planning failed.")
print(f'arm0 points: {len(data)}')


tag_multi_start = kk2.multi_movL_set_start(start_joints_arm1, points_arm1[0], points_arm1[1],
                                           allow_range,
                                           zsp_type, zsp_params, vel, acc, freq)
if not tag_multi_start:
    print("Error", 'multi-segment planing: set start failed')

for next_one in points_arm1[2:]:
    ret1 = kk2.multi_movL_next_point(next_one, allow_range, zsp_type, zsp_params, vel, acc)
    if not ret1:
        print("Error", f"multi-segment planing: set next failed")

data1, pset2 = kk2.multi_movL_get_points()
if pset2 == None:
    print("Error", f"right arm planning failed.")
print(f'arm1 points: {len(data1)}')



