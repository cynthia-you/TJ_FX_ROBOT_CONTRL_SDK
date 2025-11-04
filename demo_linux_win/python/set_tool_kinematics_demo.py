from fx_kine import Marvin_Kine
import time

'''#################################################################
该DEMO 为机器人计算全接口 

使用可以全部运行。
    注意 
    实列化计算
    配置导入
    初始化动力学

    是前置操作，其余所有接口调用前必须这三个接口先调用以初始化信息到缓存

'''  #################################################################
'''实列化计算'''
kk = Marvin_Kine()
'''
配置导入
!!! 非常重要！！！
使用前，请一定确认机型，导入正确的配置文件，文件导错，计算会错误啊啊啊,甚至看起来运行正常，但是值错误！！！
'''
ini_result = kk.load_config(config_path='ccs_m6.MvKDCfg')
time.sleep(0.5)
# print(ini_result)
# print(type(ini_result['TYPE'][0]))
# print(type(ini_result['DH'][0]))
# print(type(ini_result['PNVA'][0]))
# print(type(ini_result['BD'][0]))


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
initial_kine_tag = kk.initial_kine(robot_serial=0,
                                   robot_type=ini_result['TYPE'][0],
                                   dh=ini_result['DH'][0],
                                   pnva=ini_result['PNVA'][0],
                                   j67=ini_result['BD'][0])

time.sleep(0.5)
print('-' * 50)


'''加工具前的正解矩阵 关节全为0， 仅Z轴有值方便对比 '''
fk_mat1=kk.fk(robot_serial=0,joints=[0,0,0,0,0,0,0])
print(f'no tool, fk_mat1:{fk_mat1}')


'''
末端工具运动学设置
#一定要确认robot_serial是左臂0 还是右臂1
假设工具和末端法兰的安装无姿态差，工具的中心为末端法兰的Z轴的正方向偏移50毫米
'''
# 设置
tool = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 50], [0, 0, 0, 1]]  # 改成工具真实参数
tag1 = kk.set_tool_kine(robot_serial=0, tool_mat=tool)
time.sleep(0.5)

'''加工具后的正解矩阵，确认是否在fk_mat1基础上z多了50毫米'''
fk_mat2=kk.fk(robot_serial=0,joints=[0,0,0,0,0,0,0])
print(f'add tool, fk_mat2:{fk_mat2}')


'''移除工具'''
tag2 = kk.remove_tool_kine(robot_serial=0)
time.sleep(0.5)

'''移除工具后的正解矩阵，确认是否与fk_mat1一致'''
fk_mat3=kk.fk(robot_serial=0,joints=[0,0,0,0,0,0,0])
print(f'remove tool, fk_mat3:{fk_mat3}')