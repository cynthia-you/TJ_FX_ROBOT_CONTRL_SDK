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
该DEMO 为笛卡尔阻抗控制案列

使用逻辑
    1 初始化订阅数据的结构体
    2 初始化机器人接口
    3 查验连接是否成功,失败程序直接退出
    4 开启日志以便检查
    5 为了防止伺服有错，先清错
    6 设置扭矩模式,关节阻抗模式,速度加速度百分比
    7 设置阻抗参数
    8 订阅数据查看是否设置
    9 切换为末端笛卡尔阻抗
    10 订阅查看是否运动到位
    11 任务完成,下使能,释放内存使别的程序或者用户可以连接机器人
'''#################################################################


# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)# 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级


def auto_eefCart(robot,dcss,calculate_cfg_path:str,which_arm:str):
    '''
    robot:instance
    dcss: instance sub_data
    calculate_cfg_path:相对路径
    which_arm:
    '''
    idx = 0
    if which_arm=='B':
        idx=1
    from SDK_PYTHON.fx_kine import Marvin_Kine
    kk = Marvin_Kine()
    kk.log_switch(0)  # 0 off, 1 on
    ini_result = kk.load_config(arm_type=idx, config_path=os.path.join(current_path, calculate_cfg_path))
    initial_kine_tag = kk.initial_kine(
        robot_type=ini_result['TYPE'][idx],
        dh=ini_result['DH'][idx],
        pnva=ini_result['PNVA'][idx],
        j67=ini_result['BD'][idx])
    if not initial_kine_tag:
        print('initial calculation cfg error, pls check file')
        exit(1)

    sub_data = robot.subscribe(dcss)
    cur_jv=sub_data["outputs"][idx]["fb_joint_pos"]
    (f'cur jv:{cur_jv}')
    fk_mat = kk.fk(joints=cur_jv)
    xyzabc=kk.mat4x4_to_xyzabc(pose_mat=fk_mat)
    cart_dir=[xyzabc[3],xyzabc[4],xyzabc[5],0,0,0,0]
    # cart_dir =[0]*7
    robot.clear_set()
    robot.set_EefCart_control_params(arm=which_arm, fcType=1,CartCtrlPara=cart_dir)
    robot.send_cmd()
    time.sleep(0.5)


if __name__=="__main__":
    '''初始化订阅数据的结构体'''
    dcss=DCSS()

    '''初始化机器人接口'''
    robot=Marvin_Robot()


    '''查验连接是否成功'''
    init = robot.connect('192.168.1.190')
    if init==0:
        logger.error('failed:端口占用，连接失败!')
        exit(0)
    else:
        '''防总线通信异常,先清错'''
        time.sleep(0.5)
        robot.clear_set()
        robot.clear_error('A')
        robot.clear_error('B')
        robot.send_cmd()
        time.sleep(0.5)

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
            logger.info('success:机器人连接成功!')
        else:
            logger.error('failed:机器人连接失败!')
            exit(0)


    '''开启日志以便检查'''
    robot.log_switch('1') #全局日志开关
    robot.local_log_switch('1') # 主要日志


    '''阻抗参数'''
    robot.clear_set()
    robot.set_cart_kd_params(arm='A', K=[10, 5000, 5000,600, 600, 600, 20], D=[0.1, 0.1, 0.1, 0.3, 0.3, 0.3, 1],
                             type=2)  # 预设参考。
    robot.send_cmd()
    time.sleep(0.5)


    '''设置扭矩模式,关节阻抗模式,速度加速度百分比'''
    robot.clear_set()
    robot.set_state(arm='A',state=3)#state=3扭矩模式
    robot.set_impedance_type(arm='A',type=2) #type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
    robot.set_vel_acc(arm='A',velRatio=50, AccRatio=50)
    robot.send_cmd()
    time.sleep(0.5)


    '''订阅数据查看是否设置'''
    sub_data=robot.subscribe(dcss)
    logger.info(f"current state{sub_data['states'][0]['cur_state']}")
    logger.info(f"cmd state:{sub_data['states'][0]['cmd_state']}")
    logger.info(f"arm error code:{sub_data['states'][0]['err_code']}")
    logger.info(f'set vel={sub_data["inputs"][0]["joint_vel_ratio"]}, acc={sub_data["inputs"][0]["joint_acc_ratio"]}')
    logger.info(f'set card k={sub_data["inputs"][0]["cart_k"][:]}, d={sub_data["inputs"][0]["cart_d"][:]}')
    logger.info(f'set impedance type={sub_data["inputs"][0]["imp_type"]}')


    '''点位1'''
    robot.clear_set()
    joint_cmd_1=[-94, 80, 86, -104, -6, -30, 1]
    robot.set_joint_cmd_pose(arm='A',joints=joint_cmd_1)
    robot.send_cmd()
    time.sleep(3)


    # '''判断低速标志'''
    # sub_data = robot.subscribe(dcss)
    # while sub_data['outputs'][0]['low_speed_flag'] != b'\x01':
    #     sub_data = robot.subscribe(dcss)
    #     print({sub_data['outputs'][0]['low_speed_flag']})
    #     time.sleep(0.1)

    sub_data=robot.subscribe(dcss)
    time.sleep(0.002)
    logger.info(f'cur jv:{sub_data["outputs"][0]["fb_joint_pos"]}')
    '''末端笛卡尔阻抗'''
    auto_eefCart(robot,dcss,'ccs_m6_40.MvKDCfg','A')

    '''订阅数据查看是否到位'''
    sub_data=robot.subscribe(dcss)
    logger.info(f'1   set eef directions={sub_data["inputs"][0]["force_pidul"]}')
    time.sleep(30)


    '''点位1'''
    robot.clear_set()
    joint_cmd_1 = [0,0,0,-90,0,0,0]
    robot.set_joint_cmd_pose(arm='A',joints=joint_cmd_1)
    robot.send_cmd()
    time.sleep(3)


    # '''判断低速标志'''
    # sub_data = robot.subscribe(dcss)
    # while sub_data['outputs'][0]['low_speed_flag'] != b'\x01':
    #     sub_data = robot.subscribe(dcss)
    #     print({sub_data['outputs'][0]['low_speed_flag']})
    #     time.sleep(0.1)

    sub_data=robot.subscribe(dcss)
    time.sleep(0.002)
    logger.info(f'cur jv:{sub_data["outputs"][0]["fb_joint_pos"]}')
    '''末端笛卡尔阻抗'''
    auto_eefCart(robot,dcss,'ccs_m6_40.MvKDCfg','A')


    '''订阅数据查看是否到位'''
    sub_data=robot.subscribe(dcss)
    logger.info(f'2  set eef directions={sub_data["inputs"][0]["force_pidul"]}')


    # '''下使能'''
    # robot.clear_set()
    # robot.set_state(arm='A',state=0)
    # robot.send_cmd()

    '''释放机器人内存'''
    robot.release_robot()

    '''
    cur jv:[-94.3674, 86.9552, 92.5672, -105.0294, 4.998, -28.7622, 6.345]
    user platform: win32
    Load config successful
    Initial kinematics successful
    fk result, matrix:[[-0.7463492802256253, 0.06205375411391407, 0.6626553277749028, 404.6201455015258], [-0.6603026611006182, 0.055793587332562164, -0.7489242093012265, 126.82863193271486], [-0.08344547664181905, -0.9965121209560144, -0.000667244771649006, 171.701495761779], [0.0, 0.0, 0.0, 1.0]]
    Pose mat to xyzabc Success
    xyzabc:[404.6201455015258, 126.82863193271486, 171.701495761779, -90.03836411243607, 4.786639618316381, -138.50049100816594]
    1   set eef directions=[-90.0384, 4.7866, -138.5005, 0.0, 0.0, 0.0, 0.0]
    cur jv:[0.3318, -0.1194, -0.172, -89.8932, 0.0336, 0.1098, 0.214]
    user platform: win32
    Load config successful
    Initial kinematics successful
    fk result, matrix:[[0.001696588787233129, 0.0009460848231199761, 0.999998113252371, 427.6004849078508], [0.0005849078025145947, 0.9999993804625184, -0.0009470783719068734, 0.839574777544625], [-0.9999983897325317, 0.0005865135014875822, 0.0016960343637378993, 479.55448786665613], [0.0, 0.0, 0.0, 1.0]]
    Pose mat to xyzabc Success
    xyzabc:[427.6004849078508, 0.839574777544625, 479.55448786665613, 19.076134449770166, 89.89717787306407, 19.021899260334617]
    2  set eef directions=[19.0761, 89.8972, 19.0219, 0.0, 0.0, 0.0, 0.0]
    
    
    
    cur jv:[-94.3738, 87.0449, 92.7199, -105.0234, 5.193, -28.7578, 6.4951]
    user platform: win32
    Load config successful
    Initial kinematics successful
    fk result, matrix:[[-0.7464798777458679, 0.062158322052249734, 0.6624984038030074, 404.599601938697], [-0.6601543910078416, 0.05568301599806902, -0.7490631359797635, 126.85199912941047], [-0.0834504168677565, -0.9965117885546084, -0.0005321617317321503, 170.49934473013715], [0.0, 0.0, 0.0, 1.0]]
    Pose mat to xyzabc Success
    xyzabc:[404.599601938697, 126.85199912941047, 170.49934473013715, -90.03059734789862, 4.7869236635684835, -138.51185051092975]
    1   set eef directions=[-90.0306, 4.7869, -138.5119, 0.0, 0.0, 0.0, 0.0]
    cur jv:[0.3255, -0.0972, -0.1844, -89.914, 0.0346, 0.1091, 0.1906]
    user platform: win32
    Load config successful
    Initial kinematics successful
    fk result, matrix:[[0.001708953402674929, 0.0008675364671757715, 0.9999981634270022, 427.48915255724506], [0.0006026273765369993, 0.9999994412152681, -0.0008685674397061875, 0.736801458399299], [-0.9999983581572245, 0.0006041106110493628, 0.001708429646512595, 479.5705222915146], [0.0, 0.0, 0.0, 1.0]]
    Pose mat to xyzabc Success
    xyzabc:[427.48915255724506, 0.736801458399299, 479.5705222915146, 19.47388929044398, 89.89617465534909, 19.4241535230102]

    问题不会是在你和我的，这个笛卡尔的 应该不在我这里吧 这里用啥了 我一点不知道 我感觉不在咱俩这里 但我不敢奶 奶吧 我担着 你这边涉及到什么： 接口传递数据给控制器 是不是那个矩阵转ABC 用的是计算库的旧接口
    '''









