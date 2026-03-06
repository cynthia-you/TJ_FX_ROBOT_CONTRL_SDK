import time
import numpy as np
import threading
from typing import  Optional
import sys
import os
import logging
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)
current_file_path = os.path.abspath(__file__)
current_path = os.path.dirname(current_file_path)
from SDK_PYTHON.fx_kine import Marvin_Kine, FX_InvKineSolvePara
from SDK_PYTHON.fx_robot import Marvin_Robot, DCSS

# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)  # 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级


# ========== 连接机器人 ==========
robot = Marvin_Robot()

result = robot.connect('192.168.1.190')
if result == 0:
    print('failed:端口占用，连接失败!')
    sys.exit(1)

# 防总线通信异常,先清错
time.sleep(0.5)
robot.clear_set()
robot.clear_error('A')
robot.send_cmd()
time.sleep(0.5)

# 验证连接
dcss = DCSS()
motion_tag = 0
frame_update = None
for i in range(5):
    sub_data = robot.subscribe(dcss)
    print(f"connect frames :{sub_data['outputs'][0]['frame_serial']}")
    if sub_data['outputs'][0]['frame_serial'] != 0 and frame_update != sub_data['outputs'][0]['frame_serial']:
        motion_tag += 1
        frame_update = sub_data['outputs'][0]['frame_serial']
    time.sleep(0.1)
if motion_tag > 0:
    print('success:机器人连接成功!')
else:
    print('failed:机器人连接失败!')
    sys.exit(0)


robot.clear_set()
robot.set_state(arm='A', state=1)
timeout=robot.send_cmd_wait_response(100)
logger.info(f'100ms 内测到的执行的响应延迟是 ：{timeout} ms')
time.sleep(1)

robot.clear_set()
robot.set_state(arm='A', state=0)
timeout=robot.send_cmd_wait_response(100)
logger.info(f'100ms 内测到的执行的响应延迟是 ：{timeout} ms')
time.sleep(1)



robot.release_robot()
