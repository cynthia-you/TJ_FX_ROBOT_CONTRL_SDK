import os
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)
current_file_path = os.path.abspath(__file__)
current_path = os.path.dirname(current_file_path)
from SDK_PYTHON.fx_kine import Marvin_Kine, FX_InvKineSolvePara, convert_to_8x8_matrix
from SDK_PYTHON.fx_robot import Concise_Marvin_Robot, DCSS
import time
import logging
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)
logger.setLevel(logging.DEBUG)

def check_joints_reached(joint1, joint2, tolerance=0.01):
    if not (joint1 and joint2 and len(joint1) == 7 and len(joint2) == 7):
        return False
    return all(abs(j1 - j2) < tolerance for j1, j2 in zip(joint1, joint2))

def case1_link_robot():

    ''' initialize robot class '''
    robot = Concise_Marvin_Robot()

    ''' connection '''
    if not robot.connect(  robot_ip='192.168.1.190',log_switch=1):# log ON
        logger.error("--- connect failed ---")

    ''' release robot
    After release, other programs or computers can connect to the robot. 
    After release, if you want to control the robot again, you must reconnect.
    '''
    time.sleep(1)
    robot.release_robot()

def case2_position():
    '''define parameters'''
    arm='A'
    vel=50
    acc=50
    idx =0
    if arm=='B':
        idx=1
    joints1=[0]*7
    joints2=[9.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35]

    ''' initialize subscribe structure '''
    dcss = DCSS()

    ''' initialize robot class '''
    robot = Concise_Marvin_Robot()

    ''' connection '''
    if not robot.connect(robot_ip='192.168.1.190', log_switch=1):  # log ON
        logger.error("--- connect failed ---")

    '''switch to position state'''
    if not robot.set_position_state(arm=arm,velRatio=vel,AccRatio=acc):
        logger.error("--- switch to position failed ---")
    time.sleep(0.5) #reserve time for switch to position state

    '''go to joint1'''
    if not robot.set_joint_position_cmd(arm=arm,joint=joints1):
        logger.error("--- set joint cmd failed ---")
    '''judge reached or los speed'''
    time.sleep(0.05)  # waite robot accelerate then check low speed
    while 1:
        sub_data=robot.subscribe(dcss)
        fb_joints=sub_data['outputs'][idx]['fb_joint_pos']
        if sub_data['outputs'][idx]['low_speed_flag'][0]==1 or check_joints_reached(fb_joints,joints1):
            break
        time.sleep(0.001)

    '''go to joint2'''
    if not robot.set_joint_position_cmd(arm=arm,joint=joints2):
        logger.error("--- set joint cmd failed ---")
    '''judge reached or los speed'''
    time.sleep(0.05)  # waite robot accelerate then check low speed
    while 1:
        sub_data=robot.subscribe(dcss)
        fb_joints=sub_data['outputs'][idx]['fb_joint_pos']
        if sub_data['outputs'][idx]['low_speed_flag'][0]==1 or check_joints_reached(fb_joints,joints2):
            break
        time.sleep(0.001)

    ''' disable and release robot
    After release, other programs or computers can connect to the robot. 
    After release, if you want to control the robot again, you must reconnect.
    '''
    time.sleep(1)
    robot.disable(arm=arm)
    robot.release_robot()

def case3_position_two_arms():
    '''define parameters'''
    vel=50
    acc=50
    joints1=[0]*7
    joints2=[9.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35]

    ''' initialize subscribe structure '''
    dcss = DCSS()

    ''' initialize robot class '''
    robot = Concise_Marvin_Robot()

    ''' connection '''
    if not robot.connect(robot_ip='192.168.1.190', log_switch=1):  # log ON
        logger.error("--- connect failed ---")

    '''switch to position state'''
    re=robot.set_position_state(arm='A',velRatio=vel,AccRatio=acc)
    re1=robot.set_position_state(arm='B',velRatio=vel,AccRatio=acc)
    if not re or not re1:
        logger.error("--- switch to position failed ---")
    time.sleep(0.5) #reserve time for switch to position state

    '''go to joint1'''
    re=robot.set_joint_position_cmd(arm='A',joint=joints1)
    re1=robot.set_joint_position_cmd(arm='B',joint=joints1)
    if not re or not re1:
        logger.error("--- set joints cmd failed ---")
    '''judge reached or los speed'''
    time.sleep(0.05) #waite robot accelerate then check low speed
    while 1:
        sub_data=robot.subscribe(dcss)
        fb_joints1=sub_data['outputs'][0]['fb_joint_pos']
        fb_joints2 = sub_data['outputs'][1]['fb_joint_pos']
        if ((sub_data['outputs'][0]['low_speed_flag'][0]==1 or check_joints_reached(fb_joints1,joints1)) and
                (sub_data['outputs'][1]['low_speed_flag'][0]==1 or check_joints_reached(fb_joints2,joints1))):
            break
        time.sleep(0.001)

    '''go to joint2'''
    re=robot.set_joint_position_cmd(arm='A',joint=joints2)
    re1=robot.set_joint_position_cmd(arm='B',joint=joints2)
    if not re or not re1:
        logger.error("--- set joints cmd failed ---")
    '''judge reached or los speed'''
    time.sleep(0.05) #waite robot accelerate then check low speed
    while 1:
        sub_data=robot.subscribe(dcss)
        fb_joints1=sub_data['outputs'][0]['fb_joint_pos']
        fb_joints2 = sub_data['outputs'][1]['fb_joint_pos']
        if ((sub_data['outputs'][0]['low_speed_flag'][0]==1 or check_joints_reached(fb_joints1,joints2)) and
                (sub_data['outputs'][1]['low_speed_flag'][0]==1 or check_joints_reached(fb_joints2,joints2))):
            break
        time.sleep(0.001)

    ''' disable and release robot
    After release, other programs or computers can connect to the robot. 
    After release, if you want to control the robot again, you must reconnect.
    '''
    time.sleep(1)
    robot.disable(arm='A')
    robot.disable(arm='B')
    robot.release_robot()

def case4_impedance_joint():
    '''define parameters'''
    arm='A'
    vel=50
    acc=50
    k= [10, 10, 10, 1.6, 1, 1, 1]
    d = [0.8, 0.8, 0.8, 0.4, 0.4, 0.4, 0.4]
    joints1=[0]*7
    joints2=[9.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35]
    idx =0
    if arm=='B':
        idx=1

    ''' initialize subscribe structure '''
    dcss = DCSS()

    ''' initialize robot class '''
    robot = Concise_Marvin_Robot()

    ''' connection '''
    if not robot.connect(robot_ip='192.168.1.190', log_switch=1):  # log ON
        logger.error("--- connect failed ---")

    '''switch to joint impedance state'''
    if not robot.set_imp_joint_state(arm=arm,velRatio=vel,AccRatio=acc, K=k, D=d):
        logger.error("--- switch to joint impedance failed ---")
    time.sleep(0.5) #reserve time for switch to joint impedance state

    '''go to joint1'''
    if not robot.set_joint_position_cmd(arm=arm,joint=joints1):
        logger.error("--- set joint cmd failed ---")
    '''judge reached or los speed'''
    time.sleep(0.05)  # waite robot accelerate then check low speed
    while 1:
        sub_data=robot.subscribe(dcss)
        fb_joints=sub_data['outputs'][idx]['fb_joint_pos']
        if sub_data['outputs'][idx]['low_speed_flag'][0]==1 or check_joints_reached(fb_joints,joints1):
            break
        time.sleep(0.001)

    '''go to joint2'''
    if not robot.set_joint_position_cmd(arm=arm,joint=joints2):
        logger.error("--- set joint cmd failed ---")
    '''judge reached or los speed'''
    time.sleep(0.05)  # waite robot accelarate then check low speed
    while 1:
        sub_data=robot.subscribe(dcss)
        fb_joints=sub_data['outputs'][idx]['fb_joint_pos']
        if sub_data['outputs'][idx]['low_speed_flag'][0]==1 or check_joints_reached(fb_joints,joints2):
            break
        time.sleep(0.001)

    ''' disable and release robot
    After release, other programs or computers can connect to the robot. 
    After release, if you want to control the robot again, you must reconnect.
    '''
    time.sleep(1)
    robot.disable(arm=arm)
    robot.release_robot()

def case5_impedance_cart():
    '''define parameters'''
    arm='A'
    vel=20
    acc=20
    k= [3000, 3000, 3000, 100, 100, 100, 1]
    d = [0.8, 0.8, 0.8, 0.4, 0.4, 0.4, 0.4]
    joints1=[0]*7
    joints2=[9.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35]
    idx =0
    if arm=='B':
        idx=1

    ''' initialize subscribe structure '''
    dcss = DCSS()

    ''' initialize robot class '''
    robot = Concise_Marvin_Robot()

    ''' connection '''
    if not robot.connect(robot_ip='192.168.1.190', log_switch=0):  # log ON
        logger.error("--- connect failed ---")

    '''switch to cartesian impedance state'''
    if not robot.set_imp_cart_state(arm=arm,velRatio=vel,AccRatio=acc, K=k, D=d):
        logger.error("--- switch to joint impedance failed ---")
    time.sleep(0.5) #reserve time for switch to cartesian impedance state

    '''go to joint1'''
    if not robot.set_joint_position_cmd(arm=arm,joint=joints1):
        logger.error("--- set joint cmd failed ---")
    '''judge reached or los speed'''
    time.sleep(0.05)  # waite robot accelerate then check low speed
    while 1:
        sub_data=robot.subscribe(dcss)
        fb_joints=sub_data['outputs'][idx]['fb_joint_pos']
        if sub_data['outputs'][idx]['low_speed_flag'][0]==1 or check_joints_reached(fb_joints,joints1):
            break
        time.sleep(0.001)

    '''go to joint2'''
    if not robot.set_joint_position_cmd(arm=arm,joint=joints2):
        logger.error("--- set joint cmd failed ---")
    '''judge reached or los speed'''
    time.sleep(0.05)  # waite robot accelarate then check low speed
    while 1:
        sub_data=robot.subscribe(dcss)
        fb_joints=sub_data['outputs'][idx]['fb_joint_pos']
        if sub_data['outputs'][idx]['low_speed_flag'][0]==1 or check_joints_reached(fb_joints,joints2):
            break
        time.sleep(0.001)

    ''' disable and release robot
    After release, other programs or computers can connect to the robot. 
    After release, if you want to control the robot again, you must reconnect.
    '''
    time.sleep(1)
    robot.disable(arm=arm)
    robot.release_robot()

def case6_impedance_force():
    '''define parameters'''
    arm='A'
    vel=20
    acc=20
    k= [10, 10, 10, 1.6, 1, 1, 1]
    d = [0.8, 0.8, 0.8, 0.4, 0.4, 0.4, 0.4]
    joints1=[0]*7
    joints2=[69.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35]
    fxDir= [0, 1, 0, 0, 0, 0]
    fcAdjLmt = 50
    force = 50
    idx =0
    if arm=='B':
        idx=1

    ''' initialize subscribe structure '''
    dcss = DCSS()

    ''' initialize robot class '''
    robot = Concise_Marvin_Robot()

    ''' connection '''
    if not robot.connect(robot_ip='192.168.1.190', log_switch=0):  # log ON
        logger.error("--- connect failed ---")

    '''switch to joint impedance state'''
    if not robot.set_imp_joint_state(arm=arm, velRatio=vel, AccRatio=acc, K=k, D=d):
        logger.error("--- switch to joint impedance failed ---")
    time.sleep(0.5)  # reserve time for switch to joint impedance state

    '''go to joint1'''
    if not robot.set_joint_position_cmd(arm=arm,joint=joints1):
        logger.error("--- set joint cmd failed ---")
    '''judge reached or los speed'''
    time.sleep(0.05)  # waite robot accelerate then check low speed
    while 1:
        sub_data=robot.subscribe(dcss)
        fb_joints=sub_data['outputs'][idx]['fb_joint_pos']
        if sub_data['outputs'][idx]['low_speed_flag'][0]==1 or check_joints_reached(fb_joints,joints1):
            break
        time.sleep(0.001)

    '''go to joint2'''
    if not robot.set_joint_position_cmd(arm=arm,joint=joints2):
        logger.error("--- set joint cmd failed ---")
    '''judge reached or los speed'''
    time.sleep(0.05)  # waite robot accelarate then check low speed
    while 1:
        sub_data=robot.subscribe(dcss)
        fb_joints=sub_data['outputs'][idx]['fb_joint_pos']
        if sub_data['outputs'][idx]['low_speed_flag'][0]==1 or check_joints_reached(fb_joints,joints2):
            break
        time.sleep(0.001)
    ''' stay for 5 seconds'''
    time.sleep(5)
    if not robot.set_imp_force_state(arm=arm,fx_dir=fxDir,fc_adj_lmt=fcAdjLmt):
        logger.error("--- switch to force state failed ---")
    time.sleep(0.2)
    if not robot.set_force_cmd(arm=arm,force=force):
        logger.error("--- set force cmd failed ---")
    '''hold on for 5 seconds'''
    time.sleep(5)

    ''' disable and release robot
    After release, other programs or computers can connect to the robot. 
    After release, if you want to control the robot again, you must reconnect.
    '''
    time.sleep(1)
    robot.disable(arm=arm)
    robot.release_robot()

def case7_joint_drag():
    '''define parameters'''
    arm='A'
    idx =0
    if arm=='B':
        idx=1

    ''' initialize subscribe structure '''
    dcss = DCSS()

    ''' initialize robot class '''
    robot = Concise_Marvin_Robot()

    ''' connection '''
    if not robot.connect(robot_ip='192.168.1.190', log_switch=0):  # log ON
        logger.error("--- connect failed ---")


    '''switch to joint drag mode'''
    if not robot.set_joint_drag(arm=arm):
        logger.error("--- switch to joint drag mode failed ---")
    time.sleep(0.5)  # reserve time for switch to joint impedance state
    logger.info("---press and hold the end button to start dragging")

    stage=0
    while 1:
        sub_data = robot.subscribe(dcss)
        if sub_data['outputs'][idx]['tip_di'][0] == 1:
            stage=1
            break
        time.sleep(0.001)

    while stage==1:
        sub_data = robot.subscribe(dcss)
        if sub_data['outputs'][idx]['tip_di'][0]== 0:
            time.sleep(5)
            break
        time.sleep(0.001)

    '''exit drag'''
    if not robot.exit_drag(arm=arm):
        logger.error("--- exit drag failed ---")


    ''' disable and release robot
    After release, other programs or computers can connect to the robot. 
    After release, if you want to control the robot again, you must reconnect.
    '''
    time.sleep(1)
    robot.disable(arm=arm)
    robot.release_robot()

def case8_cart_drag_z_and_save_data(save_path='drag_z.txt'):
    '''define parameters'''
    arm='A'
    type = 'Z'
    cols = 7
    features = [0, 1, 2, 3, 4, 5, 6,
           0, 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0, 0]
    rows = 1000000
    idx =0
    if arm=='B':
        idx=1

    ''' initialize subscribe structure '''
    dcss = DCSS()

    ''' initialize robot class '''
    robot = Concise_Marvin_Robot()

    ''' connection '''
    if not robot.connect(robot_ip='192.168.1.190', log_switch=0):  # log ON
        logger.error("--- connect failed ---")


    '''switch to cartesian drag mode'''
    if not robot.set_cart_drag(arm=arm,type_=type):
        logger.error("--- switch to cartesian drag mode failed ---")
    time.sleep(0.5)  # reserve time for switch to joint impedance state
    logger.info("---press and hold the end button to start dragging")

    stage=0
    '''if button be pressed, start gather data'''
    while 1:
        sub_data = robot.subscribe(dcss)
        if sub_data['outputs'][idx]['tip_di'][0] == 1:
            robot.start_collect_data(target_num=cols,target_id=features,record_num=rows)
            stage=1
            break
        time.sleep(0.001)

    ''' once detect button br released, stop gather data'''
    while stage==1:
        sub_data = robot.subscribe(dcss)
        if sub_data['outputs'][idx]['tip_di'][0]== 0:
            robot.stop_collect_data()
            break
        time.sleep(0.001)

    '''save data to specific path'''
    robot.save_gather_data(path=save_path)

    '''exit drag'''
    if not robot.exit_drag(arm=arm):
        logger.error("--- exit drag failed ---")

    ''' disable and release robot
    After release, other programs or computers can connect to the robot. 
    After release, if you want to control the robot again, you must reconnect.
    '''
    time.sleep(1)
    robot.disable(arm=arm)
    robot.release_robot()

def case9_run_pln_joint_space(config_path="ccs_m6_40.MvKDCfg"):
    '''define parameters'''
    arm='A'
    pln_vel = 0.5
    pln_acc = 0.5
    start_joint= [69.02, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35]
    target_joint = [9.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35]
    idx =0
    if arm=='B':
        idx=1

    ''' initialize subscribe structure '''
    dcss = DCSS()

    ''' initialize robot class '''
    robot = Concise_Marvin_Robot()

    ''' connection '''
    if not robot.connect(robot_ip='192.168.1.190', log_switch=0):  # log ON
        logger.error("--- connect failed ---")

    ''' joint space PLN initialize'''
    if not robot.pln_init(path=config_path):
        logger.error("--- initialize pln failed ---")

    ''' if robot not at start joints planing to start joints '''
    sub_data=robot.subscribe(dcss)
    current_joints = sub_data['outputs'][idx]['fb_joint_pos']
    if not check_joints_reached(current_joints,start_joint):
        while 1:
            sub_data = robot.subscribe(dcss)
            if sub_data['outputs'][idx]['traj_state'][0]==0:
                break
            time.sleep(0.001)
        if not robot.run_pln_joint(arm=arm, start_joints=current_joints, stop_joints=start_joint,
                            vel_ratio=pln_vel,acc_ratio=pln_acc):
            logger.error("--- pln failed ---")
        while 1:
            sub_data = robot.subscribe(dcss)
            fb_joints = sub_data['outputs'][idx]['fb_joint_pos']
            if sub_data['outputs'][idx]['low_speed_flag'][0] == 1 or check_joints_reached(fb_joints, start_joint):
                break
            time.sleep(0.001)

    '''Perform multiple loops, plan motion from the starting point to the end point, and interrupt the planned motion during the movement.'''
    for i in range(5):
        '''waite for no running pln'''
        while True:
            data = robot.subscribe(dcss)
            time.sleep(0.001)
            if data['outputs'][idx]['traj_state'][0] == 0:
                break

        if not robot.run_pln_joint(arm=arm, start_joints=start_joint, stop_joints=target_joint,
                                     vel_ratio=pln_vel,acc_ratio=pln_acc):
            logger.error("--- pln failed ---")


        '''break pln'''
        time.sleep(0.5)
        if not robot.stop_pln(arm=arm):
            logger.error("--- stop pln failed ---")
        data = robot.subscribe(dcss)
        logger.info(f'break at:{data["outputs"][idx]["fb_joint_pos"]}')


    ''' disable and release robot
    After release, other programs or computers can connect to the robot. 
    After release, if you want to control the robot again, you must reconnect.
    '''
    time.sleep(1)
    robot.disable(arm=arm)
    robot.release_robot()

def case10_run_pln_joint_space(config_path="ccs_m6_40.MvKDCfg"):
    '''define parameters'''
    arm='A'
    pln_vel = 0.5
    pln_acc = 0.5
    start_joint= [44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.1]
    idx =0
    if arm=='B':
        idx=1

    ''' initialize subscribe structure '''
    dcss = DCSS()

    ''' initialize robot class '''
    robot = Concise_Marvin_Robot()

    ''' connection '''
    if not robot.connect(robot_ip='192.168.1.190', log_switch=0):  # log ON
        logger.error("--- connect failed ---")

    ''' if robot not at start joints planing to start joints '''
    sub_data=robot.subscribe(dcss)
    current_joints = sub_data['outputs'][idx]['fb_joint_pos']
    if not check_joints_reached(current_joints,start_joint):
        ''' joint space PLN initialize'''
        if not robot.pln_init(path=config_path):
            logger.error("--- initialize pln failed ---")

        while 1:
            sub_data = robot.subscribe(dcss)
            if sub_data['outputs'][idx]['traj_state'][0]==0:
                break
            time.sleep(0.001)
        if not robot.run_pln_joint(arm=arm, start_joints=current_joints, stop_joints=start_joint,
                            vel_ratio=pln_vel,acc_ratio=pln_acc):
            logger.error("--- pln failed ---")
        while 1:
            sub_data = robot.subscribe(dcss)
            fb_joints = sub_data['outputs'][idx]['fb_joint_pos']
            if sub_data['outputs'][idx]['low_speed_flag'][0] == 1 or check_joints_reached(fb_joints, start_joint):
                break
            time.sleep(0.001)

    '''kinematics sdk'''
    kk = Marvin_Kine()
    kk.log_switch(0)  # 0 off, 1 on
    ini_result = kk.load_config(arm_type=0, config_path=config_path)
    kk.initial_kine(
        robot_type=ini_result['TYPE'][0],
        dh=ini_result['DH'][0],
        pnva=ini_result['PNVA'][0],
        j67=ini_result['BD'][0])
    fk_mat = kk.fk(joints=start_joint)
    if not fk_mat:
        logger.info("kk.fk failes")
    pose_6d_start = kk.mat4x4_to_xyzabc(pose_mat=fk_mat)

    '''矩形框yz平面'''
    '''z+200'''
    pose_6d_end = pose_6d_start.copy()
    pose_6d_end[2] += 200
    '''MOVLA'''
    sub_data=robot.subscribe(dcss)
    current_joints = sub_data['outputs'][idx]['fb_joint_pos']
    points, pset = kk.movLA(start_xyzabc=pose_6d_start, end_xyzabc=pose_6d_end,
                            ref_joints=current_joints, vel=1000, acc=1000, freq_hz=50)
    if pset:
        '''run cart pln'''
        if not robot.run_pln_cart(arm=arm, pset=pset):
            logger.error("--- pln failed ---")

    '''waite for no running pln'''
    while True:
        data = robot.subscribe(dcss)
        if data['outputs'][idx]['traj_state'][0] == 0:
            break
        time.sleep(0.001)

    '''y-200'''
    pose_6d_start = pose_6d_end.copy()
    pose_6d_end = pose_6d_start.copy()
    pose_6d_end[1] -= 200  # Y方向移动200mm
    '''直线规划（MOVLA）'''
    sub_data=robot.subscribe(dcss)
    current_joints = sub_data['outputs'][idx]['fb_joint_pos']
    points, pset = kk.movLA(start_xyzabc=pose_6d_start, end_xyzabc=pose_6d_end,
                            ref_joints=current_joints, vel=1000, acc=1000, freq_hz=50)
    if pset:
        '''run cart pln'''
        if not robot.run_pln_cart(arm=arm, pset=pset):
            logger.error("--- pln failed ---")

    '''waite for no running pln'''
    while True:
        data = robot.subscribe(dcss)
        if data['outputs'][idx]['traj_state'][0] == 0:
            break
        time.sleep(0.001)

    '''z-200'''
    pose_6d_start = pose_6d_end.copy()
    pose_6d_end = pose_6d_start.copy()
    pose_6d_end[2] -= 200  # z方向移动200mm
    '''直线规划（MOVLA）'''
    sub_data=robot.subscribe(dcss)
    current_joints = sub_data['outputs'][idx]['fb_joint_pos']
    points, pset = kk.movLA(start_xyzabc=pose_6d_start, end_xyzabc=pose_6d_end,
                            ref_joints=current_joints, vel=1000, acc=1000, freq_hz=50)
    if pset:
        '''run cart pln'''
        if not robot.run_pln_cart(arm=arm, pset=pset):
            logger.error("--- pln failed ---")

    '''waite for no running pln'''
    while True:
        data = robot.subscribe(dcss)
        if data['outputs'][idx]['traj_state'][0] == 0:
            break
        time.sleep(0.001)

    '''y+200'''
    pose_6d_start = pose_6d_end.copy()
    pose_6d_end = pose_6d_start.copy()
    pose_6d_end[1] += 200  # Y方向移动200mm
    '''直线规划（MOVLA）'''
    sub_data=robot.subscribe(dcss)
    current_joints = sub_data['outputs'][idx]['fb_joint_pos']
    points, pset = kk.movLA(start_xyzabc=pose_6d_start, end_xyzabc=pose_6d_end,
                            ref_joints=current_joints, vel=1000, acc=1000, freq_hz=50)
    if pset:
        '''run cart pln'''
        if not robot.run_pln_cart(arm=arm, pset=pset):
            logger.error("--- pln failed ---")

    '''waite for no running pln'''
    while True:
        data = robot.subscribe(dcss)
        if data['outputs'][idx]['traj_state'][0] == 0:
            break
        time.sleep(0.001)


    ''' disable and release robot
    After release, other programs or computers can connect to the robot. 
    After release, if you want to control the robot again, you must reconnect.
    '''
    time.sleep(1)
    robot.disable(arm=arm)
    robot.release_robot()

def case11_set_eef_tools():
    '''define parameters'''
    '''The motion parameters are the offset and rotation of the tool relative to the center of the flange'''
    kine_a= [0.01, 0, 0, 0, 0, 0]
    kine_b= [0.02, 0, 0, 0, 0, 0]
    '''The tool's dynamic parameters include mass center coordinates and inertia'''
    dyn_a= [0.011, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    dyn_b= [0.012, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    ''' initialize subscribe structure '''
    dcss = DCSS()

    ''' initialize robot class '''
    robot = Concise_Marvin_Robot()

    ''' connection '''
    if not robot.connect(robot_ip='192.168.1.190', log_switch=1):  # log ON
        logger.error("--- connect failed ---")

    '''set tools for two arms'''
    re = robot.set_tool(arm='A',kine_para=kine_a,dyn_para=dyn_a)
    re1 = robot.set_tool(arm='B', kine_para=kine_b, dyn_para=dyn_b)
    if not re and not re1:
        logger.error("--- set tools failed ---")

    time.sleep(0.5)
    sub_data=robot.subscribe(dcss)
    logger.info('the tool of arm A:')
    logger.info(f"kinematics:{sub_data['inputs'][0]['tool_kine']}")
    logger.info(f"dynamics:{sub_data['inputs'][0]['tool_dyn']}")

    logger.info('\nthe tool of arm B:')
    logger.info(f"kinematics:{sub_data['inputs'][1]['tool_kine']}")
    logger.info(f"dynamics:{sub_data['inputs'][1]['tool_dyn']}")

    ''' release robot
    After release, other programs or computers can connect to the robot. 
    After release, if you want to control the robot again, you must reconnect.
    '''
    time.sleep(1)
    robot.release_robot()

def case12_eef_communication():
    import queue
    '''创建队列'''
    data_queue = queue.Queue()

    '''define parameters'''
    arm='A'
    channel=2

    def read_data():
        while True:
            try:
                tag, receive_hex_data = robot.get_ch_data(arm=arm,channel=channel )
                if tag >= 1:
                    logger.info(f"receive HEX:{receive_hex_data}")
                    data_queue.put(receive_hex_data)
                else:
                    time.sleep(0.001)
            except Exception as e:
                logger.error(f"read data error:{e}")
                time.sleep(0.001)

    def get_received_data():
        received_count = 0
        received_data_list = []
        while True:
            try:
                data = data_queue.get_nowait()
                received_count += 1
                received_data_list.append(data)
            except queue.Empty:
                break
        return received_count, received_data_list


    ''' initialize robot class '''
    robot = Concise_Marvin_Robot()

    ''' connection '''
    if not robot.connect(robot_ip='192.168.1.190', log_switch=1):  # log ON
        logger.error("--- connect failed ---")

    '''clear cache'''
    robot.clear_ch_data()

    '''send HEX to com1'''
    hex_data = "01 06 00 00 00 01 48 0A"
    success, sdk_return = robot.set_ch_data(arm=arm, data=hex_data, size_int=len(hex_data), set_ch=channel)
    logger.info(f"result {'success' if success else 'fail'}")

    '''read data from com1'''
    received_count, received_data = get_received_data()
    if received_count > 0:
        print(f'receiving info, frames：{received_count},  received data:\n{received_data}')

    ''' release robot
    After release, other programs or computers can connect to the robot. 
    After release, if you want to control the robot again, you must reconnect.
    '''
    time.sleep(1)
    robot.release_robot()

if __name__=="__main__":
    logger.info("----------------------------------------------------")
    logger.info("------------Concise SDK API showcases--------------")
    logger.info("Concise SDK APIe coexists compatibly with the old interface.\nPlease uncomment the showcases below one by one and compile and run them.")

    '''showcase1: connect'''
    # logger.info("--------------------------")
    # logger.info("showcase: connect to robot:")
    # case1_link_robot()

    '''showcase1: set position state and run from {0,0,0,0,0,0,0} to {9.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35}'''
    # logger.info("--------------------------")
    # logger.info("showcase: set position state and run from {0,0,0,0,0,0,0} to {9.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35}")
    # case2_position()

    '''showcase3: arm A&B set position state and run from {0,0,0,0,0,0,0} to {9.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35}'''
    # logger.info("--------------------------")
    # logger.info("showcase: arm A&B set position state and run from {0,0,0,0,0,0,0} to {9.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35}")
    # case3_position_two_arms()

    '''showcase4: set joint impedance state and run from {0,0,0,0,0,0,0} to {9.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35}'''
    # logger.info("--------------------------")
    # logger.info("showcase: set joint impedance state and run from {0,0,0,0,0,0,0} to {9.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35}")
    # case4_impedance_joint()

    '''showcase5: set cartesian impedance state and run from {0,0,0,0,0,0,0} to {9.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35}'''
    # logger.info("--------------------------")
    # logger.info("showcase: set cartesian impedance state and run from {0,0,0,0,0,0,0} to {9.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35}")
    # case5_impedance_cart()


    '''showcase6: set force state and run to {69.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35}, Apply a force of 50N in the Y direction, over a range of 50 millimeters'''
    # logger.info("--------------------------")
    # logger.info("showcase: set force state and run to {69.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35}, Apply a force of 50N in the Y direction, over a range of 50 millimeters")
    # case6_impedance_force()


    '''showcase7: set joint drag state '''
    # logger.info("--------------------------")
    # logger.info("showcase: set joint drag state ")
    # logger.info("USAGE:After entering drag mode, press and hold the end button to start dragging, release the button, and disable after 5 seconds")
    # case7_joint_drag()

    '''showcase8: set cart drag state :direction: Z .then save drag data(max collect time:100 seconds '''
    # logger.info("--------------------------")
    # logger.info("showcase: set cart drag state :direction: Z .then save drag data(max collect time:100 seconds")
    # logger.info("USAGE:After entering drag mode, press and hold the end button to start dragging, release the button, and disable after 5 seconds")
    # case8_cart_drag_z_and_save_data(save_path='drag_z.txt')


    '''showcase9: In joint space: Perform multiple loops, plan motion from the starting point to the end point, and interrupt the planned motion during the movement.'''
    # logger.info("--------------------------")
    # logger.info("showcase: In joint space: Perform multiple loops, plan motion from the starting point to the end point, and interrupt the planned motion during the movement.")
    # case9_run_pln_joint_space(config_path="ccs_m6_40.MvKDCfg")

    '''showcase10: In joint space: Perform multiple loops, plan motion from the starting point to the end point, and interrupt the planned motion during the movement.'''
    # logger.info("--------------------------")
    # logger.info("showcase: In joint space: Perform multiple loops, plan motion from the starting point to the end point, and interrupt the planned motion during the movement.")
    # case10_run_pln_joint_space(config_path="ccs_m6_40.MvKDCfg")

    '''showcase11: set End-effector tools info :kinematics and dynamics'''
    # logger.info("--------------------------")
    # logger.info("showcase: set End-effector tools info :kinematics and dynamics")
    # case11_set_eef_tools()
    # logger.info("!!!After the controller restarts, the tool information must be rewritten.!!!")
    #

    '''showcase12: End-effector tool serial communication. Please note to replace the communication commands with the ones used by your own tool.'''
    # logger.info("--------------------------")
    # logger.info("showcase: End-effector tool serial communication. Please note to replace the communication commands with the ones used by your own tool.")
    # case12_eef_communication()


    logger.info("--------------------------")






