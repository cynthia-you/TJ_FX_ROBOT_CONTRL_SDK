import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)
current_file_path = os.path.abspath(__file__)
current_path = os.path.dirname(current_file_path)
from SDK_PYTHON.fx_robot import Concise_Marvin_Robot, DCSS
from SDK_PYTHON.fx_kine import Marvin_Kine
import time
import logging

# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)# 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级


def jointsTorque2Eef(jacobin:list,joints: list, tau: list):
        if len(joints) != 7:
            raise ValueError("joints must be (7,)")
        if len(tau) != 7:
            raise ValueError("tau must be (7,)")
        J = jacobin

        JJt = [[0.0 for _ in range(6)] for _ in range(6)]
        for i in range(6):
            for j in range(6):
                s = 0.0
                for k in range(7):
                    s += float(J[i][k]) * float(J[j][k])
                JJt[i][j] = s
        rhs = [0.0 for _ in range(6)]
        for i in range(6):
            s = 0.0
            for k in range(7):
                s += float(J[i][k]) * float(tau[k])
            rhs[i] = s

        n = 6
        M = [JJt[i][:] + [rhs[i]] for i in range(n)]  # 增广矩阵 6x7

        for col in range(n):
            pivot_row = col
            max_abs = abs(M[col][col])
            for r in range(col + 1, n):
                v = abs(M[r][col])
                if v > max_abs:
                    max_abs = v
                    pivot_row = r

            if max_abs < 1e-12:
                return False

            if pivot_row != col:
                M[col], M[pivot_row] = M[pivot_row], M[col]

            pivot = M[col][col]
            for c in range(col, n + 1):
                M[col][c] /= pivot
            for r in range(n):
                if r == col:
                    continue
                factor = M[r][col]
                if abs(factor) < 1e-18:
                    continue
                for c in range(col, n + 1):
                    M[r][c] -= factor * M[col][c]
        F = [M[i][n] for i in range(n)]
        return F


if __name__=="__main__":
    '''Case Description: 
        Obtain the current joint positions and joint torques of the left arm to calculate the six-dimensional force at the end effector
    '''

    arm='A'
    idx=0
    calculate_config_file='ccs_m6.MvKDCfg'
    if arm=='B':
        idx=1

    ''' load kinematics'''
    kk=Marvin_Kine()
    ini_result=kk.load_config(arm_type=idx,config_path=os.path.join(current_path,calculate_config_file))
    if not ini_result:
        logger.error(f'load {calculate_config_file} error, please check file or path')
        exit(-1)
    initial_kine_tag=kk.initial_kine(
                                     robot_type=ini_result['TYPE'][idx],
                                     dh=ini_result['DH'][idx],
                                     pnva=ini_result['PNVA'][idx],
                                     j67=ini_result['BD'][idx])

    ''' initialize subscribe structure '''
    dcss = DCSS()
    ''' initialize robot class '''
    robot = Concise_Marvin_Robot()
    ''' connection '''
    if not robot.connect(robot_ip='192.168.1.190', log_switch=1):  # log ON
        logger.error("--- connect robot failed ---")
        exit(-1)

    '''loop: calculate eef torque'''
    for i in range(50):

        sub_data=robot.subscribe(dcss)
        current_joints=sub_data['outputs'][idx]['fb_joint_pos']
        current_torque = sub_data['outputs'][idx]['est_joint_force']

        jacobin_mat=kk.joints2JacobMatrix(joints=current_joints)
        if not jacobin_mat:
            logger.error('current joints can not transfer to jacobin matrix')
            exit(-1)

        eef_torque = jointsTorque2Eef(
            jacobin=jacobin_mat,
            joints=current_joints,
            tau=current_torque
        )
        fx, fy, fz, nx, ny, nz = eef_torque
        print(
            f"[Eef torque] "
            f"Fx={fx:8.3f}  Fy={fy:8.3f}  Fz={fz:8.3f}  |  "
            f"Nx={nx:8.3f}  Ny={ny:8.3f}  Nz={nz:8.3f}"
        )



    '''release robot
    After release, other programs or computers can connect to the robot. 
    After release, if you want to control the robot again, you must reconnect.
    '''
    time.sleep(1)
    robot.release_robot()


