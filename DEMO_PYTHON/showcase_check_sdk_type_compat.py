import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)
current_file_path = os.path.abspath(__file__)
current_path = os.path.dirname(current_file_path)
from SDK_PYTHON.fx_robot import Marvin_Robot, DCSS
import time
import logging
'''#################################################################
该DEMO 为检查SDK类型兼容性案例,检查调用方字节大小是否与SDK定义(FxType.h)一致、#pragma pack(4)对齐是否生效，并检测大小端.

使用逻辑
    初始化订阅数据的结构体
    初始化机器人接口
    查验连接是否成功,失败程序直接退出
    开启日志以便检查
    左臂强制松闸
    调整完毕,左臂强制抱闸
    右臂强制抱闸
    调整完毕,右臂强制松闸
    任务完成,释放内存使别的程序或者用户可以连接机器人
'''#################################################################

# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)# 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级


'''初始化机器人接口'''
robot=Marvin_Robot()


ret, byte_order = robot.check_sdk_type_compat()
if ret < 0:
    logger.error(f"SDK type compatibility check failed, error mask = 0x{-ret:x}")
    '''释放机器人内存'''
    robot.release_robot()
    exit(-1)
else:
    endian = "little-endian" if byte_order == 0 else "big-endian"
    logger.info(f"All checks passed, byte order: {endian}")

'''释放机器人内存'''
robot.release_robot()

