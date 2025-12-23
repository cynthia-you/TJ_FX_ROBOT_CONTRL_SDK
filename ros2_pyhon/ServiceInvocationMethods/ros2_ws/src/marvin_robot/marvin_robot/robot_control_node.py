import rclpy
from rclpy.node import Node
# from rclpy.callback_groups import ReentrantCallbackGroup
from marvin_interfaces.srv import ArmClearErr, ArmSoftStop, CollectData, Connect, DownloadLog, GetPara,SetPara,GetServoErrCode,  Release, SaveCollectDataAsCsv, SaveCollectData, SdkVersion, SetForceCmd, SetForceCtrlPara,SendFile, SendPvt, SetArmState, SetArmVelAcc, SetCardKD, SetImpedanceType, SetJointCmdPos, SetJointKD, SetPvtId,SetToolPara, StopCollectData, UpdateSdk,InitialKine,FK,IK,IkNsp,IkRangeCross67,JointsToJacobMatrix,ReceiveFile,SaveParam,SetDragSpace,LogSwitch,RobotData
from marvin_interfaces.msg import State, Output, Input,MarvinResponse
import ctypes
from ctypes import *
import os
import inspect
from typing import Union

current_path = os.getcwd()


class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')

        # self.callback_group = ReentrantCallbackGroup()
        self.data=None
        # 创建服务
        self.srv = self.create_service(
            RobotData,
            'get_robot_data',
            self.handle_request,
            # callback_group=self.callback_group
        )

        self.clear_srv = self.create_service(
            ArmClearErr, 'clear_arm_error', self.clear_arm_error_callback)

        self.soft_stop_srv = self.create_service(
            ArmSoftStop, 'arm_soft_stop', self.arm_soft_stop_callback)

        self.collect_srv = self.create_service(
            CollectData, 'collect_data', self.collect_data_callback)

        self.connect_srv = self.create_service(
            Connect, 'connect_robot', self.connect_callback)

        self.download_log_srv = self.create_service(
            DownloadLog, 'download_log', self.download_sdk_log_callback)

        self.get_para_srv = self.create_service(
            GetPara, 'get_param', self.get_param_callback)

        self.log_switch_srv=self.create_service(
            LogSwitch,'log_switch',self.log_switch_callback)

        self.set_para_srv = self.create_service(
            SetPara, 'set_param', self.set_param_callback)

        self.servo_error_code_srv = self.create_service(
            GetServoErrCode, 'get_servo_error_code', self.get_servo_error_code_callback)

        self.release_srv = self.create_service(
            Release, 'release_robot', self.release_robot_callback)

        self.receive_file_srv = self.create_service(
            ReceiveFile, 'receive_file', self.receive_file_callback)

        self.save_collected_data_as_srv = self.create_service(
            SaveCollectData, 'save_collected_data',self.save_collected_data_callback)

        self.save_collected_data_as_csv_srv = self.create_service(
            SaveCollectDataAsCsv, 'save_collected_data_as_csv',self.save_collected_data_as_csv_callback)

        self.save_param_srv = self.create_service(
            SaveParam, 'save_param', self.save_param_callback)

        self.sdk_version_srv = self.create_service(
            SdkVersion, 'sdk_version', self.get_sdk_version_callback)

        self.set_force_cmd_srv=self.create_service(
            SetForceCmd,'set_force_cmd',self.set_force_cmd_callback)

        self.set_force_ctrl_para_srv=self.create_service(
            SetForceCtrlPara,'set_force_ctrl_para',self.set_force_ctrl_para_callback)

        self.send_file_srv = self.create_service(
            SendFile, 'send_local_file_to_remote', self.send_file_callback)

        self.receive_file_srv = self.create_service(
            ReceiveFile, 'receive_file_from_robot', self.receive_file_callback)

        self.send_pvt_srv = self.create_service(
            SendPvt, 'send_pvt_file', self.send_pvt_callback)

        self.set_arm_state_srv = self.create_service(
            SetArmState, 'set_arm_state', self.set_arm_state_callback)

        self.set_arm_vel_acc_srv = self.create_service(
            SetArmVelAcc, 'set_arm_vel_acc', self.set_arm_vel_acc_callback)

        self.set_card_kd_srv = self.create_service(
            SetCardKD, 'set_card_kd', self.set_card_kd_callback)

        self.set_drag_space_srv=self.create_service(
            SetDragSpace,'set_drag_space',self.set_drag_space_callback)

        self.set_impedance_type_srv = self.create_service(
            SetImpedanceType, 'set_impedance_type', self.set_impedance_type_callback)

        self.set_joint_params_srv = self.create_service(
            SetJointCmdPos, 'set_joint_cmd_pos', self.set_joint_cmd_pos_callback)

        self.set_joint_kd_srv = self.create_service(
            SetJointKD, 'set_joint_kd', self.set_joint_kd_callback)

        self.set_pvt_id_srv = self.create_service(
            SetPvtId, 'set_pvt_id', self.set_pvt_id_callback)

        self.set_tool_para_srv = self.create_service(
            SetToolPara, 'set_tool_params', self.set_tool_para_callback)

        self.stop_srv = self.create_service(
            StopCollectData, 'stop_collect_data', self.stop_collect_data_callback)

        self.update_sdk_srv = self.create_service(
            UpdateSdk, 'update_sdk', self.update_sdk_callback)

        # ''' ########  kine ###'''
        #
        # self.initial_marvin_kine_srv = self.create_service(
        #     InitialKine, 'initial_kine_config', self.initial_kine_config_callback)
        #
        # self.fk_srv = self.create_service(
        #     FK, 'forward_kinematics', self.fk_callback)
        #
        # self.ik_srv = self.create_service(
        #     IK, 'inverse_kinematics', self.ik_callback)
        #
        # self.ik_nsp_srv = self.create_service(
        #     IkNsp, 'ik_nsp', self.ik_nsp_callback)
        #
        # self.ik_range_cross_67_srv = self.create_service(
        #     IkRangeCross67, 'ik_range_cross_67', self.ik_range_cross_67_callback)
        #
        # self.joints2jacob_srv = self.create_service(
        #     JointsToJacobMatrix, 'joints_to_jacob_matrix', self.joints2jacob_callback)

        self.robot = Marvin_Robot()
        # self.kine=Marvin_Kine()

        self.connected = False
        self.collecting = False
        # self.initial_kine=False

        self.get_logger().info("Marvin Robot ready")

        # 发布器，用于发布机器人状态
        self.publisher_ = self.create_publisher(MarvinResponse, 'robot_data', 10)

        # 订阅器，用于接收控制指令
        self.subscription = self.create_subscription(
            Input,  # 这里需要根据你的控制指令定义消息类型，这里用Input消息类型作为例子
            'robot_control_input',
            self.control_callback,
            10)

        # 定时器，定期读取机器人状态
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100Hz

    def timer_callback(self):
        # 读取机器人状态
        try:
            dscc = DCSS()
            self.data = self.robot.subscribe(dscc)
            # 将self.data转换为MarvinResponse消息
            msg = MarvinResponse()
            # ... 填充msg，类似于handle_request方法中的代码
            # 发布消息
            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error in timer_callback: {str(e)}")

    def control_callback(self, msg):
        # 根据接收到的控制指令，直接调用机器人的方法
        # 例如，如果消息中包含关节指令，我们调用set_joint_cmd_pose
        # 注意：这里需要根据你的消息定义来解析
        try:
            # 示例：假设msg中包含arm_id和joints
            self.robot.clear_set()
            self.robot.set_joint_cmd_pose(arm=msg.arm_id, joints=msg.joints)
            success = self.robot.send_cmd()
            if not success:
                self.get_logger().error("Failed to send control command")
        except Exception as e:
            self.get_logger().error(f"Error in control_callback: {str(e)}")


    def handle_request(self, request, response):
        try:

            dscc=DCSS()
            self.data=self.robot.subscribe(dscc)
            print(f'sub data:{self.data["para_name"]}')
            # print(f'sub data:{self.data["states"]}')
            # print(f'sub data:{self.data["outputs"]}')
            # print(f'sub data:{self.data["inputs"]}')

            # 转换数据到ROS消息
            response.data = MarvinResponse()
            # 填充响应数据
            response.data.robot_name = self.data["para_name"]
            # 填充状态数据
            response.data.states = []
            for state in self.data['states']:
                state_msg = State()
                state_msg.cur_state = state['cur_state']
                state_msg.cmd_state = state['cmd_state']
                state_msg.err_code = state['err_code']
                response.data.states.append(state_msg)

            # 填充输出数据
            response.data.outputs = []
            for output in self.data['outputs']:
                output_msg = Output()
                output_msg.out_frame_serial = output['frame_serial']
                # output_msg.tip_di = output['tip_di']
                # output_msg.low_speed_flag =output['low_speed_flag']
                output_msg.fb_joint_pos = output['fb_joint_pos']
                output_msg.fb_joint_vel = output['fb_joint_vel']
                # output_msg.fb_joint_pos_e = output['fb_joint_posE']
                output_msg.fb_joint_cmd = output['fb_joint_cmd']
                output_msg.fb_joint_c_toq = output['fb_joint_cToq']
                output_msg.fb_joint_s_toq = output['fb_joint_sToq']
                output_msg.fb_joint_them = output['fb_joint_them']
                output_msg.est_joint_firc = output['est_joint_firc']
                output_msg.est_joint_firc_dot = output['est_joint_firc_dot']
                output_msg.est_joint_force = output['est_joint_force']
                output_msg.est_cart_fn = output['est_cart_fn']
                response.data.outputs.append(output_msg)

            # 填充输入数据
            response.data.inputs = []
            for input_data in self.data['inputs']:
                input_msg = Input()
                input_msg.rt_in_switch = input_data['rt_in_switch']
                input_msg.imp_type = input_data['imp_type']
                input_msg.in_frame_serial = input_data['in_frame_serial']
                # input_msg.frame_miss_cnt = input_data['frame_miss_cnt']
                # input_msg.max_frame_miss_cnt = input_data['max_frame_miss_cnt']
                # input_msg.sys_cyc = input_data['sys_cyc']
                # input_msg.sys_cyc_miss_cnt = input_data['sys_cyc_miss_cnt']
                # input_msg.max_sys_cyc_miss_cnt = input_data['max_sys_cyc_miss_cnt']
                input_msg.tool_kine = input_data['tool_kine']
                input_msg.tool_dyn = input_data['tool_dyn']
                input_msg.joint_cmd_pos = input_data['joint_cmd_pos']
                input_msg.joint_vel_ratio = input_data['joint_vel_ratio']
                input_msg.joint_acc_ratio = input_data['joint_acc_ratio']
                input_msg.joint_k = input_data['joint_k']
                input_msg.joint_d = input_data['joint_d']
                input_msg.drag_sp_type = input_data['drag_sp_type']
                input_msg.drag_sp_para = input_data['drag_sp_para']
                input_msg.cart_kd_type = input_data['cart_kd_type']
                input_msg.cart_k = input_data['cart_k']
                input_msg.cart_d = input_data['cart_d']
                input_msg.cart_kn = input_data['cart_kn']
                input_msg.cart_dn = input_data['cart_dn']
                input_msg.force_fb_type = input_data['force_fb_type']
                input_msg.force_type = input_data['force_type']
                input_msg.force_dir = input_data['force_dir']
                input_msg.force_pidul = input_data['force_pidul']
                input_msg.force_adj_lmt = input_data['force_adj_lmt']
                input_msg.force_cmd = input_data['force_cmd']
                input_msg.set_tags = input_data['set_tags']
                input_msg.update_tags = input_data['update_tags']
                input_msg.pvt_id = input_data['pvt_id']
                input_msg.pvt_id_update = input_data['pvt_id_update']
                input_msg.pvt_run_id = input_data['pvt_run_id']
                input_msg.pvt_run_state = input_data['pvt_run_state']
                response.data.inputs.append(input_msg)

            # # 填充其他字段
            # response.data.para_name = list(self.data['ParaName'])
            # response.data.para_type = list(self.data['ParaType'])
            # response.data.para_ins = list(self.data['ParaIns'])
            # response.data.para_value_i = list(self.data['ParaValueI'])
            # response.data.para_value_f = list(self.data['ParaValueF'])
            # response.data.para_cmd_serial = list(self.data['ParaCmdSerial'])
            # response.data.para_ret_serial = list(self.data['ParaRetSerial'])



        except Exception as e:
            self.get_logger().error(f"Error processing request: {str(e)}")
            # 返回空响应表示错误
            return RobotData.Response()

        return response



    def connect_callback(self, request, response):
        if self.connected:
            response.success = False
            response.message = "Already connected to a robot"
            return response
        try:
            success = self.robot.connect(request.robot_ip)
            if success:
                self.connected = True
                response.success = True
                response.message = f"Connected to {request.robot_ip}"
            else:
                response.success = False
                response.message = "Connection failed"
        except Exception as e:
            response.success = False
            response.message = f"Connection error: {str(e)}"
        return response

    def release_robot_callback(self, request, response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            return response
        try:
            self.robot.clear_set()
            self.robot.release_robot()
            success =self.robot.send_cmd()
            if success:
                self.connected = False
                response.success = True
                response.message = f"Robot released"
            else:
                response.success = False
                response.message = "Robot release failed"
        except Exception as e:
            response.success = False
            response.message = f"release_robot_callback error: {str(e)}"
        return response

    def get_sdk_version_callback(self, request, response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            return response
        try:
            version = self.robot.SDK_version()
            if version:
                response.success = True
                response.message = f"SDK version: {version}"
            else:
                response.success = False
                response.message = f"Get SDK version failed"
        except Exception as e:
            response.success = False
            response.message = f"SDK_version error: {str(e)}"
        return response

    def save_param_callback(self, request, response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            return response
        try:
            id = self.robot.save_para_file()
            if id==-1 or id==2:
                response.success = False
                response.message = f"Save parameter file failed"
            else:
                response.success = True
                response.message = f"Parameter file saved, save id {id}"
                response.id=id
        except Exception as e:
            response.success = False
            response.message = f"save_param_callback error: {str(e)}"
        return response

    def download_sdk_log_callback(self, request, response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            return response
        try:
            success = self.robot.download_sdk_log(log_path=request.log_path)
            if success:
                response.success = True
                response.message = f"SDK log downloaded to {request.log_path}"
            else:
                response.success = False
                response.message = f"SDK log download failed"
        except Exception as e:
            response.success = False
            response.message = f"download_sdk_log_callback error: {str(e)}"
        return response

    def update_sdk_callback(self, request, response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            return response
        try:
            success = self.robot.update_SDK(sdk_path=request.sdk_path)
            if success:
                response.success = True
                response.message = f"SDK file {request.sdk_path} send to controller"
            else:
                response.success = False
                response.message = f"SDK file {request.sdk_path} send to controller failed"
        except Exception as e:
            response.success = False
            response.message = f"update_sdk_callback error: {str(e)}"
        return response

    def get_param_callback(self, request, response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            response.value = 0.0
            return response

        try:
            # 获取参数
            result = self.robot.get_param(type=request.type, paraName=request.para_name)

            # 直接提取第二个元素（假设总是(0, value)格式）
            if isinstance(result, tuple) and len(result) >= 2:
                actual_value = result[1]  # 提取实际值
            else:
                actual_value = result  # 如果不是元组，直接使用

            # 根据类型转换
            if request.type == 'int':
                response.value = float(int(actual_value))
            elif request.type == 'float':
                response.value = float(actual_value)
            else:
                response.success = False
                response.message = f"Unsupported parameter type: {request.type}"
                return response

            response.success = True
            response.message = f"Parameter {request.para_name} = {actual_value}"

        except Exception as e:
            response.success = False
            response.message = f"Error getting parameter: {str(e)}"
            response.value = 0.0

        return response


    def log_switch_callback(self, request, response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            return response
        try:
            if request.global_or_local=='global':
                success= self.robot.log_switch(flag=request.flag)
                if success:
                    response.success = True
                    response.message = f"Set {request.global_or_local} log switch:{request.flag}"
            elif request.global_or_local=='local':
                success = self.robot.local_log_switch(flag=request.flag)
                if success:
                    response.success = True
                    response.message = f"Set {request.global_or_local} log switch:{request.flag}"
        except Exception as e:
            response.success = False
            response.message = f"log_switch_callback error: {str(e)}"
        return response


    def set_param_callback(self, request, response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            return response
        try:
            self.robot.clear_set()
            self.robot.set_param(type=request.type,paraName=request.para_name,value=request.value)
            success =self.robot.send_cmd()
            if success:
                response.success = True
                response.message = f"Set {request.type} parameter{request.para_name}, value={request.value}"
            else:
                response.success = False
                response.message = f"Set {request.type} param filed"
        except Exception as e:
            response.success = False
            response.message = f"set_param_callback error: {str(e)}"
        return response

    def collect_data_callback(self, request, response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            return response
        if self.collecting:
            response.success = False
            response.message = "Robot collect already in progress"
            return response
        try:
            success = self.robot.collect_data(
                request.target_num,
                request.target_ids,
                request.record_num
            )
            if success:
                self.collecting = True
                response.success = True
                response.message = f"Collecting {request.record_num} records"
            else:
                response.success = False
                response.message = "Failed to start collection"
        except Exception as e:
            response.success = False
            response.message = f"Collection error: {str(e)}"
        return response

    def stop_collect_data_callback(self, request, response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            return response
        if not self.collecting:
            response.success = False
            response.message = "No active data collection"
            return response
        try:
            self.robot.clear_set()
            self.robot.stop_collect_data()
            success =self.robot.send_cmd()
            if success:
                self.collecting = False
                response.success = True
                response.message = "Data collection stopped"
            else:
                response.success = False
                response.message = "Failed to stop collection"
        except Exception as e:
            response.success = False
            response.message = f"stop_collect_data_callback error: {str(e)}"
        return response

    def save_collected_data_as_csv_callback(self, request, response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            return response
        try:
            success = self.robot.save_collected_data_as_csv_to_path(path=request.local_save_path)
            if success:
                self.collecting = False
                response.success = True
                response.message = f"Save collected data to {request.local_save_path}"
            else:
                response.success = False
                response.message = "Save collected data failed"
        except Exception as e:
            response.success = False
            response.message = f"save_collected_data_as_csv_callback error: {str(e)}"
        return response

    def save_collected_data_callback(self, request, response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            return response
        try:
            success = self.robot.save_collected_data_to_path(path=request.local_save_path)
            if success:
                self.collecting = False
                response.success = True
                response.message = f"Save collected data to {request.local_save_path}"
            else:
                response.success = False
                response.message = "Save collected data failed"
        except Exception as e:
            response.success = False
            response.message = f"save_collected_data_callback error: {str(e)}"
        return response

    def receive_file_callback(self, request, response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            return response
        try:
            success = self.robot.receive_file(local_path=request.local_path,remote_path=request.remote_path)
            if success:
                self.collecting = False
                response.success = True
                response.message = f"Receive file from {request.remote_path} to local {request.local_path}"
            else:
                response.success = False
                response.message = "Receive file failed"
        except Exception as e:
            response.success = False
            response.message = f"receive_file_callback error: {str(e)}"
        return response

    def clear_arm_error_callback(self, request, response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            return response
        try:
            self.robot.clear_set()
            self.robot.clear_error(arm=request.arm_id)
            success = self.robot.send_cmd()
            if success:
                response.success = True
                response.message = f"Arm {request.arm_id} error cleared"
            else:
                response.success = False
                response.message = f"Arm {request.arm_id} error clear failed"
        except Exception as e:
            response.success = False
            response.message = f"clear_arm_error_callback error: {str(e)}"
        return response

    def get_servo_error_code_callback(self, request, response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            return response
        try:
            err_code = self.robot.get_servo_error_code(arm=request.arm_id)
            if err_code:
                response.success = True
                response.message = f"Get arm {request.arm_id} servo error code:{err_code}"
                response.error_code=err_code
            else:
                response.success = False
                response.message = f"Get arm {request.arm_id} servo error code failed or no error"
        except Exception as e:
            response.success = False
            response.message = f"clear_arm_error_callback error: {str(e)}"
        return response

    def arm_soft_stop_callback(self, request, response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            return response
        try:
            success = self.robot.soft_stop(arm=request.arm_id)
            if success:
                response.success = True
                response.message = f"Arm {request.arm_id} soft stopped"
            else:
                response.success = False
                response.message = f"Arm {request.arm_id} soft stop failed"
        except Exception as e:
            response.success = False
            response.message = f"arm_soft_stop_callback error: {str(e)}"
        return response

    def send_pvt_callback(self, request, response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            return response
        try:
            success=self.robot.send_pvt_file(arm=request.arm_id,pvt_path=request.pvt_path, id=request.id)
            if success:
                response.success = True
                response.message = f"Send pvt file{request.pvt_path} to {request.id} under arm {request.arm_id}"
            else:
                response.success = False
                response.message = f"pvt file{request.pvt_path} to {request.id} under arm {request.arm_id} send failed"
        except Exception as e:
            response.success = False
            response.message = f"send_pvt_callback error: {str(e)}"
        return response

    def set_pvt_id_callback(self, request, response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            return response
        try:
            self.robot.clear_set()
            success = self.robot.set_pvt_id(arm=request.arm_id,id=request.pvt_id)
            self.robot.send_cmd()
            if success:
                response.success = True
                response.message = f"Set pvt_id={request.pvt_id} to arm {request.arm_id}"
            else:
                response.success = False
                response.message = f"set pvt id failed"
        except Exception as e:
            response.success = False
            response.message = f"set_pvt_id_callback error: {str(e)}"
        return response

    def send_file_callback(self, request, response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            return response
        try:
            success = self.robot.send_file(local_path=request.local_path, remote_path=request.remote_path)
            if success:
                response.success = True
                response.message = f"Send local file {request.local_path} to arm remote path {request.remote_path}"
            else:
                response.success = False
                response.message = f"send local file {request.local_path} to arm remote path {request.remote_path} failed"
        except Exception as e:
            response.success = False
            response.message = f"send_file_callback error: {str(e)}"
        return response

    def receive_file_callback(self, request, response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            return response
        try:
            success = self.robot.receive_file(local_path=request.local_path, remote_path=request.remote_path)
            if success:
                response.success = True
                response.message = f"Receive file {request.local_path} from arm remote path {request.remote_path}"
            else:
                response.success = False
                response.message = f"send local file {request.local_path} to arm remote path {request.remote_path} failed"
        except Exception as e:
            response.success = False
            response.message = f"receive_file_callback error: {str(e)}"
        return response

    def set_joint_cmd_pos_callback(self, request, response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            return response
        try:
            self.robot.clear_set()
            self.robot.set_joint_cmd_pose(arm=request.arm_id,joints=request.joints)
            success = self.robot.send_cmd()
            if success:
                response.success = True
                response.message = f"Send joint={request.joints} to arm {request.arm_id}"
            else:
                response.success = False
                response.message = f"Send joint={request.joints} to arm {request.arm_id} failed"
        except Exception as e:
            response.success = False
            response.message = f"set_joint_cmd_pos_callback error: {str(e)}"
        return response


    def set_force_cmd_callback(self,request,response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            return response
        try:
            self.robot.clear_set()
            self.robot.set_force_cmd(arm=request.arm_id,f=request.f)
            success = self.robot.send_cmd()
            if success:
                response.success = True
                response.message = f"Set arm {request.arm_id} force={request.f}"
            else:
                response.success = False
                response.message = f"Set arm {request.arm_id} force={request.f} failed"
        except Exception as e:
            response.success = False
            response.message = f"set_force_cmd_callback error: {str(e)}"
        return response

    def set_force_ctrl_para_callback(self,request,response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            return response
        try:
            self.robot.clear_set()
            self.robot.set_force_control_params(arm=request.arm_id,fcType=request.fc_type,fxDirection=request.directions, fcCtrlpara=request.fc_ctrl_para, fcAdjLmt=request.adjustment)
            success = self.robot.send_cmd()
            if success:
                response.success = True
                response.message = f"Set arm {request.arm_id} force control parameters as: fcType={request.fc_type}, fxDirection={request.directions}, fcCtrlpara={request.fc_ctrl_para}, fcAdjLmt={request.adjustment}"
            else:
                response.success = False
                response.message = f"Set arm {request.arm_id} force control parameters failed"
        except Exception as e:
            response.success = False
            response.message = f"set_force_ctrl_para_callback error: {str(e)}"
        return response

    def set_arm_state_callback(self,request,response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            return response
        try:
            self.robot.clear_set()
            self.robot.set_state(arm=request.arm_id,state=request.state)
            success = self.robot.send_cmd()
            if success:
                response.success = True
                response.message = f"Set arm {request.arm_id} state={request.state}"
            else:
                response.success = False
                response.message = f"Set arm {request.arm_id} state failed"
        except Exception as e:
            response.success = False
            response.message = f"set_arm_state_callback error: {str(e)}"
        return response

    def set_arm_vel_acc_callback(self,request,response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            return response
        try:
            self.robot.clear_set()
            self.robot.set_vel_acc(arm=request.arm_id,velRatio=request.vel,AccRatio=request.acc)
            success = self.robot.send_cmd()
            if success:
                response.success = True
                response.message = f"Set arm {request.arm_id} velocity ratio={request.vel}, acceleration ratio={request.acc}"
            else:
                response.success = False
                response.message = f"Set arm {request.arm_id} vel and acc failed"
        except Exception as e:
            response.success = False
            response.message = f"sset_arm_vel_acc_callback error: {str(e)}"
        return response

    def set_card_kd_callback(self,request,response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            return response
        try:
            self.robot.clear_set()
            self.robot.set_card_kd_params(arm=request.arm_id,K=request.k,D=request.d,type=request.type)
            success = self.robot.send_cmd()
            if success:
                response.success = True
                response.message = f"Set arm {request.arm_id} cartesian parameters k={request.k}, d={request.d}, type={request.type}"
            else:
                response.success = False
                response.message = f"Set arm {request.arm_id} cartesian parameters failed"
        except Exception as e:
            response.success = False
            response.message = f"set_card_kd_callback error: {str(e)}"
        return response

    def set_impedance_type_callback(self,request,response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            return response
        try:
            self.robot.clear_set()
            self.robot.set_impedance_type(arm=request.arm_id,type=request.type)
            success = self.robot.send_cmd()
            if success:
                response.success = True
                response.message = f"Set arm {request.arm_id} impedance type={request.type}"
            else:
                response.success = False
                response.message = f"Set arm {request.arm_id} impedance type failed"
        except Exception as e:
            response.success = False
            response.message = f"set_impedance_type_callback error: {str(e)}"
        return response

    def set_drag_space_callback(self,request,response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            return response
        try:
            self.robot.clear_set()
            self.robot.set_drag_space(arm=request.arm_id,dgType=request.dg_type)
            success = self.robot.send_cmd()
            if success:
                response.success = True
                response.message = f"Set arm {request.arm_id} drag type={request.dg_type}"
            else:
                response.success = False
                response.message = f"Set arm {request.arm_id} drag type failed"
        except Exception as e:
            response.success = False
            response.message = f"set_drag_space_callback error: {str(e)}"
        return response

    def set_joint_kd_callback(self,request,response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            return response
        try:
            self.robot.clear_set()
            self.robot.set_joint_kd_params(arm=request.arm_id,K=request.k,D=request.d)
            success = self.robot.send_cmd()
            if success:
                response.success = True
                response.message = f"Set arm {request.arm_id} joint parameters k={request.k}, d={request.d}"
            else:
                response.success = False
                response.message = f"Set arm {request.arm_id} joint parameters failed"
        except Exception as e:
            response.success = False
            response.message = f"set_joint_kd_callback error: {str(e)}"
        return response


    def set_tool_para_callback(self,request,response):
        if not self.connected:
            response.success = False
            response.message = "Not connected to any robot"
            return response
        try:
            self.robot.clear_set()
            self.robot.set_tool(arm=request.arm_id,kineParams=request.kine_para,dynamicParams=request.dynamic_para)
            success = self.robot.send_cmd()
            if success:
                response.success = True
                response.message = f"Set arm {request.arm_id} kineParams={request.kine_para},dynamicParams={request.dynamic_para}"
            else:
                response.success = False
                response.message = f"Set arm {request.arm_id} tool parameters failed"
        except Exception as e:
            response.success = False
            response.message = f"set_tool_para_callback error: {str(e)}"
        return response


    '''##########################################  kine  ########################################################'''
    def initial_kine_config_callback(self,request,response):
        try:
            success=self.kine.initial_marvin_config(serial=request.serial, config_path=request.marvin_config)
            if success:
                self.initial_kine = True
                response.success = True
                response.message = f"Initial kinematics of Marvin serial={request.serial}, config path={request.marvin_config}"
            else:
                response.success = False
                response.message = f"Initial kinematics of Marvin failed"

        except Exception as e:
            response.success = False
            response.message = f"initial_kine_config_callback error: {str(e)}"
        return response

    def fk_callback(self,request,response):
        if not self.initial_kine:
            response.success = False
            response.message = "Robot kinematic not initial please check MARVIN_CONFIG file and path"
            return response
        try:
            fk_mat=self.kine.fk(joints=request.joints)
            if fk_mat:
                response.success = True
                response.message = f"FK joints={request.joints}, output fk matrix={fk_mat}"
                flattened = []
                for sublist in fk_mat:
                    for element in sublist:
                        flattened.append(element)
                response.matrix=flattened
                # self.get_logger().info(
                #     f"Computed matrix with {len(response.matrix)} elements"
                # )
            else:
                response.success = False
                response.message = f"fk failed"
        except Exception as e:
            response.success = False
            response.message = f"fk_callback error: {str(e)}"
        return response

    def ik_callback(self,request,response):
        if not self.initial_kine:
            response.success = False
            response.message = "Robot kinematic not initial please check MARVIN_CONFIG file and path"
            return response
        try:
            ik_joints=self.kine.ik(mat4x4=request.mat4x4,ref_joints=request.ref_joints,isOutRange=request.out_range, Is123Deg=request.deg_123,
                      Is567Deg=request.deg_123)
            if ik_joints:
                response.success = True
                response.message = f"IK matrix={request.mat4x4}, ref_joints={request.ref_joints}, output ik_joints={ik_joints}"
                response.ik_joints=ik_joints
            else:
                response.success = False
                response.message = f"IK failed"
        except Exception as e:
            response.success = False
            response.message = f"ik_callback error: {str(e)}"
        return response

    def ik_nsp_callback(self,request,response):
        if not self.initial_kine:
            response.success = False
            response.message = "Robot kinematic not initial please check MARVIN_CONFIG file and path"
            return response
        try:
            ik_joints=self.kine.ik_nsp(nsp_angle=request.nsp_angle,ref_joints=request.ref_joints,isOutRange=request.out_range, Is123Deg=request.deg_123,
                      Is567Deg=request.deg_123)
            if ik_joints:
                response.success = True
                response.message = f"IK_nsp  nsp_angle={request.nsp_angle}, ref_joints={request.ref_joints}, output ik_joints={ik_joints}"
                response.ik_joints=ik_joints
            else:
                response.success = False
                response.message = f"IK_nsp failed"
        except Exception as e:
            response.success = False
            response.message = f"ik_nsp_callback error: {str(e)}"
        return response


    def ik_range_cross_67_callback(self,request,response):
        if not self.initial_kine:
            response.success = False
            response.message = "Robot kinematic not initial please check MARVIN_CONFIG file and path"
            return response
        try:
            ik_joints=self.kine.ik_range_crosss67(joints67=request.joints67)
            if ik_joints:
                response.success = True
                response.message = f"Ik_range_crosss67 input joints={request.joints67}, output ik_joints67={ik_joints}"
                response.ik_joints67=ik_joints
            else:
                response.success = False
                response.message = f"Ik_range_crosss67 failed"
        except Exception as e:
            response.success = False
            response.message = f"ik_range_cross_67_callback error: {str(e)}"
        return response

    def joints2jacob_callback(self,request,response):
        if not self.initial_kine:
            response.success = False
            response.message = "Robot kinematic not initial please check MARVIN_CONFIG file and path"
            return response
        try:
            mat6x7=self.kine.joints2JacobMatrix(joints=request.joints)
            if mat6x7:
                response.success = True
                response.message = f"Joints2JacobMatrix input joints={request.joints}, output jacob_matrix={mat6x7}"
                flattened = []
                for sublist in mat6x7:
                    for element in sublist:
                        flattened.append(element)
                response.mat6x7=flattened
            else:
                response.success = False
                response.message = f"Joints2JacobMatrix failed"
        except Exception as e:
            response.success = False
            response.message = f"joints2jacob_callback error: {str(e)}"
        return response




def main(args=None):
    rclpy.init(args=args)
    node = RobotControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def update_text_file_simple(mode, data_list, filename):
    """
    简化版的文件更新函数
    """
    if mode not in ['A', 'B'] or len(data_list) != 16:
        return False
    try:
        # 如果文件存在，读取内容；否则创建默认内容
        if os.path.exists(filename):
            with open(filename, 'r', encoding='utf-8') as file:
                lines = file.readlines()
        # 更新对应行
        line_index = 0 if mode == 'A' else 1
        lines[line_index] = ','.join(str(x) for x in data_list) + '\n'

        # 写回文件
        with open(filename, 'w', encoding='utf-8') as file:
            file.writelines(lines)
        return True
    except Exception as e:
        print(f"更新文件时出错: {e}")
        return False

def read_csv_file_to_float_strict(filename, expected_columns=16):
    """
    读取CSV格式的文件内容并转换为float，严格验证每列数量

    参数:
        filename: 文件名
        expected_columns: 期望的列数（默认16）

    返回:
        如果文件为空: 返回0
        如果文件有一行: 返回 [float1, float2, ...]
        如果文件有两行: 返回 [[float1, float2, ...], [float1, float2, ...]]
        如果文件不存在或转换失败: 返回-1
    """

    if os.path.getsize(filename) == 0:
        return 0

    try:
        with open(filename, 'r', encoding='utf-8') as file:
            lines = file.readlines()

        non_empty_lines = [line.strip() for line in lines if line.strip()]

        if len(non_empty_lines) == 0:
            return 0

        all_float_data = []
        for line_num, line in enumerate(non_empty_lines, 1):
            values = line.split(',')
            # 过滤空值并去除空格
            cleaned_values = [v.strip() for v in values if v.strip()]

            # 验证列数
            if len(cleaned_values) != expected_columns:
                print(f"第{line_num}行: 期望{expected_columns}列，实际找到{len(cleaned_values)}列")
                return -1

            float_values = []
            for value in cleaned_values:
                try:
                    float_value = float(value)
                    float_values.append(float_value)
                except ValueError:
                    print(f"第{line_num}行: 无法将内容 '{value}' 转换为float")
                    return -1

            all_float_data.append(float_values)

        # 根据行数返回不同格式
        if len(all_float_data) == 1:
            return all_float_data[0]
        elif len(all_float_data) == 2:
            return all_float_data
        else:
            print(f"文件包含{len(all_float_data)}行，只支持1-2行")
            return -1

    except Exception as e:
        print(f"读取文件时出错: {e}")
        return -1
def decimal_to_hex(number, prefix=False, upper=True, float_precision=8):
    """
    将十进制数转换为十六进制表示

    参数:
        number: 要转换的十进制数，可以是整数或浮点数
        prefix: 是否添加"0x"前缀，默认为False
        upper: 是否使用大写字母，默认为True
        float_precision: 浮点数转换时的精度（小数位数），默认为8

    返回:
        str: 十六进制表示的字符串

    异常:
        TypeError: 当输入不是数字时抛出
    """
    # 检查输入是否为数字
    if not isinstance(number, (int, float)):
        raise TypeError("输入必须是整数或浮点数")

    # 处理整数
    if isinstance(number, int):
        hex_str = hex(number)
    # 处理浮点数
    else:
        # 使用float.hex()方法获取浮点数的十六进制表示
        hex_str = float(number).hex()

        # 如果需要，可以限制小数部分的精度
        if float_precision is not None:
            parts = hex_str.split('.')
            if len(parts) > 1:
                exponent_part = parts[1].split('p')
                if len(exponent_part) > 1:
                    hex_str = f"{parts[0]}.{exponent_part[0][:float_precision]}p{exponent_part[1]}"

    # 移除或保留前缀
    if not prefix and hex_str.startswith('0x'):
        hex_str = hex_str[2:]
    elif prefix and not hex_str.startswith('0x'):
        hex_str = '0x' + hex_str

    # 处理大小写
    if upper:
        hex_str = hex_str.upper()
    else:
        hex_str = hex_str.lower()

    return hex_str

def identify_and_calculate_length(input_data: Union[str, bytes]) -> dict:
    result = {
        "input": input_data,
        "type": None,
        "length_bytes": None,
        "bytes_representation": None
    }

    # 处理字节串输入
    if isinstance(input_data, bytes):
        result["type"] = "bytes"
        result["length_bytes"] = len(input_data)
        result["bytes_representation"] = input_data
        return result

    # 处理字符串输入
    if isinstance(input_data, str):
        # 检查是否是十六进制字符串（可能包含空格和0x前缀）
        # 移除所有空格和0x前缀
        clean_input = re.sub(r'\s+', '', input_data.lower())

        if clean_input.startswith('0x'):
            clean_input = clean_input[2:]

        # 检查是否为有效的十六进制字符串
        hex_pattern = re.compile(r'^[0-9a-f]+$')
        if hex_pattern.match(clean_input):
            # 确保长度为偶数
            if len(clean_input) % 2 != 0:
                clean_input = '0' + clean_input

            try:
                bytes_rep = bytes.fromhex(clean_input)
                result["type"] = "hex string"
                result["length_bytes"] = len(bytes_rep)
                result["bytes_representation"] = bytes_rep
                return result
            except ValueError:
                pass  # 如果不是有效的十六进制，继续尝试其他解释

        # 检查是否已经是字节串表示形式（如b"\x06\x01\xe3\x08"）
        if input_data.startswith('b"') and input_data.endswith('"'):
            try:
                # 使用eval安全地转换（注意：在实际应用中可能需要更安全的方法）
                bytes_rep = eval(input_data)
                if isinstance(bytes_rep, bytes):
                    result["type"] = "bytes representation string"
                    result["length_bytes"] = len(bytes_rep)
                    result["bytes_representation"] = bytes_rep
                    return result
            except:
                pass

        # 如果不是上述任何类型，将其视为普通字符串
        try:
            bytes_rep = input_data.encode('utf-8')
            result["type"] = "regular string"
            result["length_bytes"] = len(bytes_rep)
            result["bytes_representation"] = bytes_rep
            return result
        except UnicodeEncodeError:
            raise ValueError("输入不是有效的十六进制字符串，也无法编码为UTF-8字节串")

    # 如果既不是字符串也不是字节串，抛出异常
    raise TypeError("输入必须是字符串或字节串")

def structure2dict(dcss):
    result = {
        "para_name": ['Marvin_sub_data'],
        "states": [
            {
                "cur_state": dcss.m_State[0].m_CurState,
                "cmd_state": dcss.m_State[0].m_CmdState,
                "err_code": dcss.m_State[0].m_ERRCode
            },
            {
                "cur_state": dcss.m_State[1].m_CurState,
                "cmd_state": dcss.m_State[1].m_CmdState,
                "err_code": dcss.m_State[1].m_ERRCode
            }
        ]
    }
    # 3. 处理实时输出数组
    result["outputs"] = [
        {
            "frame_serial": rt_out.m_OutFrameSerial,
            "tip_di": rt_out.m_TipDI,
            "low_speed_flag": rt_out.m_LowSpdFlag,
            "fb_joint_pos": [round(rt_out.m_FB_Joint_Pos[j], 4) for j in range(7)],
            "fb_joint_vel": [round(rt_out.m_FB_Joint_Vel[j], 4) for j in range(7)],
            "fb_joint_posE": [round(rt_out.m_FB_Joint_PosE[j], 4) for j in range(7)],
            "fb_joint_cmd": [round(rt_out.m_FB_Joint_Cmd[j], 4) for j in range(7)],
            "fb_joint_cToq": [round(rt_out.m_FB_Joint_CToq[j], 4) for j in range(7)],
            "fb_joint_sToq": [round(rt_out.m_FB_Joint_SToq[j], 4) for j in range(7)],
            "fb_joint_them": [round(rt_out.m_FB_Joint_Them[j], 4) for j in range(7)],
            "est_joint_firc": [round(rt_out.m_EST_Joint_Firc[j], 4) for j in range(7)],
            "est_joint_firc_dot": [round(rt_out.m_EST_Joint_Firc_Dot[j], 4) for j in range(7)],
            "est_joint_force": [round(rt_out.m_EST_Joint_Force[j], 4) for j in range(7)],
            "est_cart_fn": [round(rt_out.m_EST_Cart_FN[j], 4) for j in range(6)]
        } for rt_out in dcss.m_Out
    ]

    # 4. 处理实时输入数组 (RT_IN)
    result["inputs"] = [
        {
            "rt_in_switch": rt_in.m_RtInSwitch,
            "imp_type": rt_in.m_ImpType,
            "in_frame_serial": rt_in.m_InFrameSerial,
            "frame_miss_cnt": rt_in.m_FrameMissCnt,
            "max_frame_miss_cnt": rt_in.m_MaxFrameMissCnt,
            "sys_cyc": rt_in.m_SysCyc,
            "sys_cyc_miss_cnt": rt_in.m_SysCycMissCnt,
            "max_sys_cyc_miss_cnt": rt_in.m_MaxSysCycMissCnt,
            "tool_kine": [round(rt_in.m_ToolKine[j], 4) for j in range(6)],
            "tool_dyn": [round(rt_in.m_ToolDyn[j], 4) for j in range(10)],
            "joint_cmd_pos": [round(rt_in.m_Joint_CMD_Pos[j], 4) for j in range(7)],
            "joint_vel_ratio": rt_in.m_Joint_Vel_Ratio,
            "joint_acc_ratio": rt_in.m_Joint_Acc_Ratio,
            "joint_k": [round(rt_in.m_Joint_K[j], 4) for j in range(7)],
            "joint_d": [round(rt_in.m_Joint_D[j], 4) for j in range(7)],
            "drag_sp_type": rt_in.m_DragSpType,
            "drag_sp_para": [round(rt_in.m_DragSpPara[j], 4) for j in range(6)],
            "cart_kd_type": rt_in.m_Cart_KD_Type,
            "cart_k": [round(rt_in.m_Cart_K[j], 4) for j in range(6)],
            "cart_d": [round(rt_in.m_Cart_D[j], 4) for j in range(6)],
            "cart_kn": round(rt_in.m_Cart_KN, 4),
            "cart_dn": round(rt_in.m_Cart_DN, 4),
            "force_fb_type": rt_in.m_Force_FB_Type,
            "force_type": rt_in.m_Force_Type,
            "force_dir": [round(rt_in.m_Force_Dir[j], 4) for j in range(6)],
            "force_pidul": [round(rt_in.m_Force_PIDUL[j], 4) for j in range(7)],
            "force_adj_lmt": round(rt_in.m_Force_AdjLmt, 4),
            "force_cmd": round(rt_in.m_Force_Cmd, 4),
            "set_tags": list(rt_in.m_SET_Tags),
            "update_tags": list(rt_in.m_Update_Tags),
            "pvt_id": rt_in.m_PvtID,
            "pvt_id_update": rt_in.m_PvtID_Update,
            "pvt_run_id": rt_in.m_Pvt_RunID,
            "pvt_run_state": rt_in.m_Pvt_RunState
        } for rt_in in dcss.m_In
    ]

    result["ParaName"]=[list(dcss.m_ParaName)]
    result["ParaType"]=[dcss.m_ParaType]
    result["ParaIns"]=[dcss.m_ParaIns]
    result["ParaValueI"]=[dcss.m_ParaValueI]
    result["ParaValueF"]=[dcss.m_ParaValueF]
    result["ParaCmdSerial"]=[dcss.m_ParaCmdSerial]
    result["ParaRetSerial"]=[dcss.m_ParaRetSerial]

    return result

class Marvin_Robot:
    def __init__(self):
        """初始化机器人控制类"""
        self.robot =  ctypes.CDLL(current_path + '/src/marvin_robot/marvin_robot/MarvinLib/libMarvinSDK.so')
        self.ErrorCode = None
        self.a_pvt_path=None
        self.b_pvt_path = None
        self.local_file_path=None
        self.remote_file_path=None
        self.save_csv_path=None
        self.save_data_path=None

    def _convert_ip(self, ip_str):
        """将IP字符串转换为ctypes数组"""
        ip1, ip2, ip3, ip4 = ip_str.split('.')
        ip_uchar = ctypes.c_ubyte
        return ip_uchar(int(ip1)), ip_uchar(int(ip2)), ip_uchar(int(ip3)), ip_uchar(int(ip4))

    def connect(self, robot_ip: str):
        '''连接机器人
        :param robot_ip: 器人IP地址,确保网线连接可以ping通。
        :return:
            int: 连接状态码 1: True; 0: Flase

        eg:
            connect(robot_ip='192.168.1.190')
        '''
        ip1, ip2, ip3, ip4 = self._convert_ip(robot_ip)
        return self.robot.OnLinkTo(ip1, ip2, ip3, ip4)


    def subscribe(self,dcss):
        '''订阅机器人状态数据
        :param dcss:  结构体，见structure_data.py
        :return:
            嵌套字典
        '''
        self.robot.OnGetBuf(ctypes.byref(dcss))
        result=structure2dict(dcss)
        return result

    def release_robot(self):
        ''' 断开机器人连接
        :return:
            int: 断开状态码 1: True; 0: Flase
        '''
        return self.robot.OnRelease()

    def SDK_version(self):
        '''查看SDK版本
        :return:
            long: SDK version
        '''
        return self.robot.OnGetSDKVersion()

    def update_SDK(self, sdk_path: str):
        '''更新系统SDK版本
        :param sdk_path: 本机存放SDK的绝对路径的SDK文件更新到控制柜上
        :return:
        '''
        sdk_char = ctypes.c_char_p(sdk_path.encode('utf-8'))
        self.robot.OnUpdateSystem(sdk_char)

    def download_sdk_log(self, log_path:str):
        '''下载SDK日志到本机
        :param log_path: 日志下载到本机的绝对路
        :return:
        '''
        log_char = ctypes.c_char_p(log_path.encode('utf-8'))
        return self.robot.OnDownloadLog(log_char)


    def get_param(self,type:str,paraName:str):
        '''获取参数信息
        :param type: float or int .参数类型
        :param paraName:  参数名见robot.ini
        :return:参数值
        eg:
         robot,ini:
            [R.A0.BASIC]
            BDRange=1.5
            BDToqR=1
            Dof=7
            GravityX=0
            GravityY=9.81
            GravityZ=0
            LoadOffsetSwitch=0
            TerminalPolar=1
            TerminalType=1
            Type=1007
            [R.A0.CTRL]
            CartJNTDampJ1=0.6
            ....
            #浮点类型参数获取：
            我想获取[R.A0.CTRL]这个参数组里CartJNTDampJ1的值:
            para=get_float_params('float','R.A0.CTRL.CartJNTDampJ1')

            #整数类型参数获取：
            我想获取[R.A0.BASIC]这个参数组里Type的值
            para=get_int_params('int','R.A0.BASIC.Type')
        '''
        try:
            param_buf = (ctypes.c_char * 30)(*paraName.encode('ascii'), 0)  # 显式添加终止符
            if type=='float':
                result = ctypes.c_double(0)
                self.robot.OnGetFloatPara.restype = ctypes.c_long
                re_flag=self.robot.OnGetFloatPara(param_buf, ctypes.byref(result))
                # print(f"parameter:{paraName}, float parameters={result.value}")
                return re_flag,result.value
            elif type=='int':
                result = ctypes.c_int(0)
                self.robot.OnGetIntPara.restype = ctypes.c_long
                re_flag=self.robot.OnGetIntPara(param_buf, ctypes.byref(result))
                # print(f"parameter:{paraName}, int parameters={result.value}")
                return re_flag, result.value
        except Exception as e:
            print("ERROR:",e)

    def save_para_file(self):
        '''保存配置文件
        :return:
        '''
        self.robot.OnSavePara.restype = ctypes.c_long
        return self.robot.OnSavePara()


    def set_param(self,type:str,paraName:str,value:float):
        '''设置参数信息
        :param type: float or int .参数类型
        :param paraName:  参数名见robot.ini
        :param value:
        :return:
        eg:
         robot,ini:
            [R.A0.BASIC]
            BDRange=1.5
            BDToqR=1
            Dof=7
            GravityX=0
            GravityY=9.81
            GravityZ=0
            LoadOffsetSwitch=0
            TerminalPolar=1
            TerminalType=1
            Type=1007
            [R.A0.CTRL]
            CartJNTDampJ1=0.6
            ....
            #设置浮点类型参数获取：
            我想设置[R.A0.CTRL]这个参数组里CartJNTDampJ1的值为0.0
            set_params('float','R.A0.CTRL.CartJNTDampJ1,0.0)

            #设置整数类型参数获取：
            我想设置[R.A0.BASIC]这个参数组里Type的值为0
            set_params('int','R.A0.BASIC.Type',0)
        '''

        try:
            param_buf = (ctypes.c_char * 30)(*paraName.encode('ascii'), 0)  # 显式添加终止符
            if type=='float':
                result = ctypes.c_double(value)
                self.robot.OnSetFloatPara.restype = ctypes.c_long
                return self.robot.OnSetFloatPara(param_buf, result)
            elif type=='int':
                result = ctypes.c_int(int(value))
                self.robot.OnSetIntPara.restype = ctypes.c_long
                return self.robot.OnSetIntPara(param_buf, result)
        except Exception as e:
            print("ERROR:",e)

    def clear_set(self):
        '''指令发送前清除
        :return:
            int: 1: True; 0: Flase
        '''
        return self.robot.OnClearSet()

    def send_cmd(self):
        '''发送指令
        :return:
            int: 1: True; 0: Flase
        '''
        return self.robot.OnSetSend()

    def collect_data(self,targetNum:int,targetID:list[int],recordNum:int):
        '''采集数据
        :param targetNum:targetNum采集列数 值最大35， 因为一次最多采集35个特征。
        :param targetID: list(35,1) 对应采集数据ID序号(见下)
        :param recordNum: 采集行数，小于1000会采集1000行，设置大于一百万行会采集一百万行。
        :return:
                    采集数据ID序号
                    左臂
                        0-6  	左臂关节位置
                        10-16 	左臂关节速度
                        20-26   左臂外编位置
                        30-36   左臂关节指令位置
                        40-46	左臂关节电流（千分比）
                        50-56   左臂关节传感器扭矩NM
                        60-66	左臂摩擦力估计值
                        70-76	左臂摩檫力速度估计值
                        80-85   左臂关节外力估计值
                        90-95	左臂末端点外力估计值
                    右臂对应 + 100

                    eg1: 采集左臂和右臂的关节位置，一共14列， 采集1000行：
                        cols=14
                        idx=[0,1,2,3,4,5,6,
                             100,101,102,103,104,105,106,
                             0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0]
                        rows=1000
                        robot.collect_date(targetNum=cols,targetID=idx,recordNum=rows)

                    eg2: 采集左臂第二关节的速度和电流一共2列， 采集500行：
                        cols=2
                        idx=[11,31,0,0,0,0,0,
                             0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0]
                        rows=500
                        robot.collect_date(targetNum=cols,targetID=idx,recordNum=rows)
        '''
        targetNum_int=ctypes.c_int(targetNum)
        targetID_int=(ctypes.c_long * len(targetID))(*targetID)
        recordNum_int=ctypes.c_int(recordNum)
        return self.robot.OnStartGather(targetNum_int,targetID_int,recordNum_int)

    def stop_collect_data(self):
        '''停止采集数据
        注： 在行数采集满后会自动停止采集,若需要中途停止采集调用本函数并等待1ms之后会停止采集。
        :return:
            int: 1: True; 0: Flase
        '''
        return self.robot.OnStopGather()

    def save_collected_data_to_path(self,path:str):
        '''将采集的数据保存到指定的绝对路径
        :param path:本机绝对路径
        :return:
        '''
        self.save_data_path=path.encode('utf-8')
        path_char=ctypes.c_char_p(self.save_data_path)
        return self.robot.OnSaveGatherData(path_char)

    def save_collected_data_as_csv_to_path(self,path:str):
        '''以csv格式将采集的数据保存到指定的绝对路径
        :param path:本机绝对路径
        :return:
        '''
        path1='tmp.txt'
        self.save_data_path = path1.encode('utf-8')
        path_char = ctypes.c_char_p(self.save_data_path)
        self.robot.OnSaveGatherData(path_char)

        time.sleep(0.2)
        with open(path1, 'r') as file:
            lines = file.readlines()
        processed_data=[]
        lines = lines[1:]
        for i, line in enumerate(lines):
            parts = line.strip().split('$')
            numbers = []
            for part in parts:
                if part:
                    num_str = part.split()[-1]
                    numbers.append(num_str)
            if len(numbers) >= 2:
                numbers = numbers[2:]
            processed_data.append(numbers)

        try:
            with open(path, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerows(processed_data)
            print(f"数据已成功保存到: {path}")
            if os.path.exists(path1):
                os.remove(path1)
            return True
        except Exception as e:
            print(f"保存失败: {e}")
            if os.path.exists(path1):
                os.remove(path1)
            return False


    def soft_stop(self, arm:str):
        '''机械臂急停
        :param arm: ‘A’, 'B', 'AB', 可以让一条臂软急停，或者两条臂都软急停。
        :return:
        '''
        try:
            if arm=='A':
                return self.robot.OnEMG_A()
            elif arm=='B':
                return self.robot.OnEMG_B()
            elif arm=='AB':
                return self.robot.OnEMG_AB()
        except Exception as e:
            print("ERROR:", e)


    def get_servo_error_code(self, arm:str):
       '''获取机械臂伺服错误码
       :param self:
       :param arm:
       :return: (7,1)错误列表， 16进制
       '''
       try:
           err_code_value = (ctypes.c_long * 7)()
           if arm=='A':
               self.robot.OnGetServoErr_A.argtypes = [ctypes.POINTER(ctypes.c_long * 7)]
               self.robot.OnGetServoErr_A(ctypes.byref(err_code_value))
               # print('err_code_value',err_code_value[-1])
               err_code = [0] * 7
               for i in range(7):
                   err_code[i] = decimal_to_hex(err_code_value[i], prefix=True)
               return err_code
           elif arm=='B':
               self.robot.OnGetServoErr_B.argtypes = [ctypes.POINTER(ctypes.c_long * 7)]
               self.robot.OnGetServoErr_B(ctypes.byref(err_code_value))
               err_code = [0] * 7
               for i in range(7):
                   err_code[i] = decimal_to_hex(err_code_value[i], prefix=True)
               return err_code

       except Exception as e:
           print("ERROR:", e)


    def clear_error(self,arm:str):
        '''清错
        :return:无
        '''
        try:
            if arm=='A':
                return self.robot.OnClearErr_A()
            elif arm=='B':
                return self.robot.OnClearErr_B()
        except Exception as e:
            print(f'ERROR:{e}')


    def set_state(self,arm:str,state:int):
        '''设置状态
        :param state:
                   ARM_STATE_IDLE = 0,            //////// 下伺服
                   ARM_STATE_POSITION = 1,		//////// 位置跟随
                   ARM_STATE_PVT = 2,			//////// PVT
                   ARM_STATE_TORQ = 3,			//////// 扭矩
                   ARM_STATE_RELEASE = 4,		//////// 协作释放

        :return:
        '''
        try:
            state_int = ctypes.c_int(state)
            if arm=="A":
                return self.robot.OnSetTargetState_A(state_int)
            elif arm=='B':
                return self.robot.OnSetTargetState_B(state_int)
        except Exception as e:
            print(f'ERROR:{e}')

    def set_impedance_type(self, arm:str,type: int):
        '''设置阻抗类型
        :param type:
            Type = 1 关节阻抗
            Type = 2 坐标阻抗
            Type = 3 力控
            注：需要在ARM_STATE_TORQ状态: set_state(arm='A',state=3)  才能以阻抗模式控制!!!
        :return:
            int : 1: True,  2: False
        '''
        try:
            type_int = ctypes.c_int(type)
            if arm=='A':
                return self.robot.OnSetImpType_A(type_int)
            elif arm == 'B':
                return self.robot.OnSetImpType_B(type_int)
        except Exception as e:
            print(f'ERROR:{e}')


    def set_vel_acc(self, arm:str, velRatio: int, AccRatio: int):
        '''设置速度和加速度百分比
        :param velRatio: 速度百分比
        :param AccRatio: 加速度百分比
        :return:
            int： 1: True; 0:Flase
        '''
        try:
            velRatio_int = ctypes.c_int(velRatio)
            AccRatio_int = ctypes.c_int(AccRatio)
            if arm=='A':
                return self.robot.OnSetJointLmt_A(velRatio_int, AccRatio_int)
            elif arm=='B':
                return self.robot.OnSetJointLmt_B(velRatio_int, AccRatio_int)
        except Exception as e:
            print(f'ERROR:{e}')

    def set_tool(self,arm:str, kineParams: list, dynamicParams: list):
        '''设置工具信息
        :param kineParams: list(6,1). 运动学参数 XYZABC 单位毫米和度
        :param dynamicParams: list(10,1). 动力学参数分别为 质量M  质心[3]:mx,my,mz 惯量I[6]:XX,XY,XZ,YY,YZ,ZZ
        :return:
            int : 1: True,  2: False
        '''
        try:
            k0, k1, k2, k3, k4, k5 = kineParams
            d0, d1, d2, d3, d4, d5, d6, d7, d8, d9 = dynamicParams
            kp_double = ctypes.c_double * 6
            kineParams_value = kp_double(k0, k1, k2, k3, k4, k5)
            dp_double = ctypes.c_double * 10
            dynamicParams_value = dp_double(d0, d1, d2, d3, d4, d5, d6, d7, d8, d9)
            if arm=='A':
                return self.robot.OnSetTool_A(kineParams_value, dynamicParams_value)
            if arm=='B':
                return self.robot.OnSetTool_B(kineParams_value, dynamicParams_value)
        except Exception as e:
            print(f'ERROR:{e}')

    def set_joint_kd_params(self,arm:str, K: list, D: list):
        '''设置关节阻抗参数

        #关节阻抗时，需更低刚度避免震动，且希望机械臂有顺从性，因此采用低刚度配低阻尼。
        1-7关节刚度不超过2
        1-7关节阻尼0-1之间
        :param K: list(7,1). 刚度 牛米 / 度 。 设置每个轴的的力为刚度系数。 如K=[2，2,2,1,1,1,1]，第1到3轴有2N作为刚度系数参与控制计算，第4到7轴有1N作为刚度系数参与控制计算。
        :param D: list(7,1). 阻尼 牛米 / (度 / 秒)。 设置每个轴的的阻尼系数。
        :return:
            int : 1: True,  2: False
        '''
        try:
            k0, k1, k2, k3, k4, k5, k6 = K
            d0, d1, d2, d3, d4, d5, d6 = D

            k_double = ctypes.c_double * 7
            k_value = k_double(k0, k1, k2, k3, k4, k5, k6)
            d_double = ctypes.c_double * 7
            d_value = d_double(d0, d1, d2, d3, d4, d5, d6)
            if arm=="A":
                return self.robot.OnSetJointKD_A(k_value, d_value)
            elif arm == "B":
                return self.robot.OnSetJointKD_B(k_value, d_value)
        except Exception as e:
            print(f'ERROR:{e}')

    def set_cart_kd_params(self, arm:str, K: list, D: list, type: int):
        '''设置笛卡阻抗尔参数
            # 在笛卡尔阻抗模式下：
            刚度系数： 1-3平移方向刚度系数不超过3000, 4-6旋转方向不超过100。 零空间刚度系数不超过20
            阻尼系数： 平移和旋转阻尼系数0-1之间。 零空间阻尼系数不超过1
            零空间控制是保持末端固定不动，手臂角度运动的控制方式。接口未开放

        :param K: list(7,1). K[0]-k[2] N*m，x,y,z 平移方向每米的控制力; K[3]-k[5] N*m/rad, rx,ry,rz旋转弧度的控制力;K[6]N*m/rad,零空间总和刚度系数
        :param D: list(7,1). D[0]-D[5]  阻尼比例系数, D[6] 零空间总和阻尼比例系数
        :param type:int. set_A_arm_impedance_type设置的阻抗类型
        :return:
            int : 1: True,  2: False
        '''
        try:
            k0, k1, k2, k3, k4, k5, k6 = K
            d0, d1, d2, d3, d4, d5, d6 = D
            k_double = ctypes.c_double * 7
            k_value = k_double(k0, k1, k2, k3, k4, k5, k6)
            d_double = ctypes.c_double * 7
            d_value = d_double(d0, d1, d2, d3, d4, d5, d6)
            type_int = ctypes.c_int(type)
            if arm=="A":
                return self.robot.OnSetCartKD_A(k_value, d_value, type_int)
            if arm == "B":
                return self.robot.OnSetCartKD_B(k_value, d_value, type_int)
        except Exception as e:
            print(f'ERROR:{e}')


    def set_force_control_params(self,arm:str, fcType: int, fxDirection: list, fcCtrlpara: list, fcAdjLmt: float):
        '''设置力控参数
        :param fcType: 力控类型 0:坐标空间力控;1:工具空间力控(暂未实现)
        :param fxDirection: list(6,1). 力控方向 需要控制方向设1，目前只支持 X,Y,Z控制方向.如力控方向为z,fxDirection=[0,0,1,0,0,0]
        :param fcCtrlpara: list(7,1). 控制参数 目前全0
        :param fcAdjLmt:毫米，允许的调节范围
        :return:
            int : 1: True,  2: False
        '''
        try:
            fc_int=ctypes.c_int(fcType)
            k0, k1, k2, k3, k4, k5 = fxDirection
            d0, d1, d2, d3, d4, d5, d6 = fcCtrlpara
            fxDir_arr = (ctypes.c_double * 6)( k0, k1, k2, k3, k4, k5 )
            fcCtrlPara_arr = (ctypes.c_double * 7)(d0, d1, d2, d3, d4, d5, d6 )
            adj_double=ctypes.c_double(fcAdjLmt)
            if arm=='A':
                return self.robot.OnSetForceCtrPara_A(
                    fc_int,
                    fxDir_arr,
                    fcCtrlPara_arr,
                    adj_double)
            elif arm=='B':
                return self.robot.OnSetForceCtrPara_B(
                    fc_int,
                    fxDir_arr,
                    fcCtrlPara_arr,
                    adj_double)
        except Exception as e:
            print(f'ERROR:{e}')

    def set_joint_cmd_pose(self,arm:str, joints:list):
        '''设置关节跟踪指令值
        :param joints: list(7,1). 角度，非弧度，在位置跟随和扭矩模式下均有效
        :return:
            int : 1: True,  2: False
        '''
        try:
            j0, j1, j2, j3, j4, j5, j6= joints
            joints_double = ctypes.c_double * 7
            joints_value = joints_double(j0, j1, j2, j3, j4, j5, j6)
            if arm=='A':
                return self.robot.OnSetJointCmdPos_A(joints_value )
            elif arm == 'B':
                return self.robot.OnSetJointCmdPos_B(joints_value)
        except Exception as e:
            print(f'ERROR:{e}')

    def set_force_cmd(self,arm:str, f:float):
        '''设置力控参数
        :param f: 目标力 单位牛或者牛米
        :return:
            int : 1: True,  2: False
        '''
        try:
            f_double=ctypes.c_double(f)
            if arm=='A':
                return self.robot.OnSetForceCmd_A(f_double)
            elif arm == 'B':
                return self.robot.OnSetForceCmd_B(f_double)
        except Exception as e:
            print(f'ERROR:{e}')

    def set_pvt_id(self,arm:str,id:int):
        '''设置指定id号的pvt路径并运行
        :param id: 范围1-99. 需要在 ARM_STATE_PVT 状态，即： set_arm_state(arm='A',state=2)
        :return:
            int : 1: True,  2: False
        '''
        try:
            if arm=='B':
                id_int = ctypes.c_int(id)
                return self.robot.OnSetPVT_B(id_int)
            elif arm=='A':
                id_int = ctypes.c_int(id)
                return self.robot.OnSetPVT_A(id_int)
        except Exception as e:
            print(f'ERROR:{e}')


    def send_pvt_file(self,arm:str, pvt_path: str, id: int):
        '''上传PVT文件给指定ID
        :param pvt_path: 本地pvt文件的绝对/相对路径
        :param id:
        :return:


            PVT文件格式见：DEMO_SRS_Left.fmv
            数据首行为行数和列数信息，“PoinType=9@9341 ”表示该PVT文件含9列数据，一共9341个点位。
            数据为什么是9列？ 首先前八列为关节角度， 为什么是8？ 我们预留了8关节，人形臂为7自由度，前7个有效值，第八列都填充0，
            好的，第九列，第九列是个标记列，全填0即可。
        '''
        try :
            if arm=='A':
                self.a_pvt_path = pvt_path.encode('utf-8')
                pvt_char = ctypes.c_char_p(self.a_pvt_path)
                id_int = ctypes.c_int(id)
                # print(f'send local pvt file:{pvt_path} to robot')
                return  self.robot.OnSendPVT_A(pvt_char, id_int)
            elif arm=='B':
                self.b_pvt_path = pvt_path.encode('utf-8')
                pvt_char = ctypes.c_char_p(self.b_pvt_path)
                id_int = ctypes.c_int(id)
                # print(f'send local pvt file:{pvt_path} to robot')
                return self.robot.OnSendPVT_B(pvt_char, id_int)
        except Exception as e:
            print(f'ERROR:{e}')


    def set_drag_space(self,arm:str, dgType: int):
        '''设置拖动空间
        :param dgType:
                0 退出拖动模式
                1 关节空间拖动
                2 笛卡尔空间x方向拖动
                3 笛卡尔空间y方向拖动
                4 笛卡尔空间z方向拖动
                5 笛卡尔空间旋转方向拖动
        :return:
        '''
        try:
            type_int = ctypes.c_int(dgType)
            if arm=='A':
                return self.robot.OnSetDragSpace_A(type_int)
            elif arm=='B':
                return self.robot.OnSetDragSpace_B(type_int)
        except Exception as e:
            print(f'ERROR:{e}')

    def receive_file(self, local_path: str, remote_path: str):
        '''将机械臂控制器下载到上位机文件
        :param local_path: 本地绝对路径
        :param remote_path: 机械臂控制器绝对路径
        :return:
        '''
        self.local_file_path = local_path.encode('utf-8')
        local_char = ctypes.c_char_p(self.local_file_path)
        self.remote_file_path = remote_path.encode('utf-8')
        remote_char = ctypes.c_char_p(self.remote_file_path)
        return self.robot.OnRecvFile(local_char, remote_char)


    def send_file(self, local_path: str, remote_path: str):
        '''将上位机文件上传到机械臂控制器
        :param local_path: 本地绝对路径
        :param remote_path: 机械臂控制器绝对路径
        :return:
        '''
        self.local_file_path = local_path.encode('utf-8')
        local_char = ctypes.c_char_p(self.local_file_path)
        self.remote_file_path = remote_path.encode('utf-8')
        remote_char = ctypes.c_char_p(self.remote_file_path)
        return self.robot.OnSendFile(local_char, remote_char)


    def log_switch(self,flag:str):
        try:
            if flag=='1':
                return self.robot.OnLogOn()
            elif flag=='0':
                return self.robot.OnLogOff()
        except Exception as e:
            print(f'ERROR:{e}')


    def local_log_switch(self,flag:str):
        try:
            if flag=='1':
                return self.robot.OnLocalLogOn()
            elif flag=='0':
                return self.robot.OnLocalLogOff()
        except Exception as e:
            print(f'ERROR:{e}')

    def clear_485_cache(self,arm:str):
        '''清空发送缓存

        :param arm: 机械手臂ID “A” OR “B”
        :return: bool
        '''
        try:
            if arm == 'A':
                return self.robot.OnClearChDataA()
            elif arm == 'B':
                return self.robot.OnClearChDataB()
        except Exception as e:
            print(f'ERROR:{e}')

    def set_485_data(self, arm: str, data:bytes, size_int:int,com:int):
        '''发送数据到485的指定来源， 每次长度不超过256字节，超过就切成多个包发。

        :param arm: 机械手臂ID “A” OR “B”
        :param data: 要传递的字节数据 (长度不超过2256)
        :param size_int: int, 发送的字节长度，不能超过256
        :param com: 信息来源， 1：‘C’端; 2：com1; 3:com2
        :return: bool
        '''

        try:
            # 定义函数原型
            self.robot.OnSetChDataA.argtypes = [ctypes.POINTER(ctypes.c_ubyte), ctypes.c_long, ctypes.c_long]
            self.robot.OnSetChDataA.restype = ctypes.c_bool

            # 定义函数原型
            self.robot.OnSetChDataB.argtypes = [ctypes.POINTER(ctypes.c_ubyte), ctypes.c_long, ctypes.c_long]
            self.robot.OnSetChDataB.restype = ctypes.c_long

            # 验证参数
            if len(data) >= 257:
                raise ValueError(f"数据长度({len(data)})超过256字节限制")
            if size_int >= 257:
                print(f"size_int({size_int})超过256，将被截断")
                size_int = 256

            result = identify_and_calculate_length(data)
            if result['type'] == "hex string" or result['type'] == 'bytes' or result[
                'type'] == "bytes representation string":
                print("-" * 50)
                print(f"输入: {data}")
                print(f"类型: {result['type']}")
                print(f"字节长度: {result['length_bytes']}")
                print(f"字节表示: {result['bytes_representation']}")
                print("-" * 50)
            else:
                print(f"ERROR: set_485_data input must be hex string of bytes string")
                return False, False

            size_int_long = ctypes.c_long(result['length_bytes'])
            com_long = ctypes.c_long(com)

            data_buffer = (ctypes.c_ubyte * 256)()
            # 复制数据到缓冲区
            data_length = min(len(result['bytes_representation']), size_int)
            for i in range(data_length):
                data_buffer[i] = result['bytes_representation'][i]
            if arm == 'A':
                return True, self.robot.OnSetChDataA(data_buffer, size_int_long, com_long)
            elif arm == 'B':
                return True, self.robot.OnSetChDataB(data_buffer, size_int_long, com_long)
        except Exception as e:
            print(f'ERROR:{e}')


    def get_485_data(self, arm: str,com:int):
        '''收指定来源的485数据
        :param arm: 机械手臂ID “A” OR “B”
        :param com: 信息来源， 1：‘C’端; 2：com1; 3:com2
        :return: int, 长度size
        '''
        try:
            # 创建 256 字节缓冲区
            data_buffer = (ctypes.c_ubyte * 256)()
            ret_ch = ctypes.c_long(com)
            if arm == 'A':
                result = self.robot.OnGetChDataA(data_buffer, ctypes.byref(ret_ch))
                # 提取字节数据
                byte_data = bytes(data_buffer)  # 或 bytearray(data_buffer)
                print(f'arm receive byte_data :{byte_data}')
                hex_list = []
                for byte in byte_data:
                    # 将每个字节转换为两位十六进制
                    hex_value = hex(byte)[2:].upper().zfill(2)
                    hex_list.append(hex_value)

                return result, ' '.join(hex_list)

            elif arm == 'B':
                result = self.robot.OnGetChDataB(data_buffer, ctypes.byref(ret_ch))
                # 提取字节数据
                byte_data = bytes(data_buffer)  # 或 bytearray(data_buffer)
                # print(f'B arm receive byte_data :{byte_data }')
                hex_list = []
                for byte in byte_data:
                    # 将每个字节转换为两位十六进制
                    hex_value = hex(byte)[2:].upper().zfill(2)
                    hex_list.append(hex_value)

                return result, ' '.join(hex_list)

        except Exception as e:
            print(f'ERROR:{e}')

    def identify_tool_dyn(self, robot_type: int, ipath: str):
        '''工具动力学参数辨识
        FX_BOOL  FX_Robot_Iden_LoadDyn(FX_INT32L Type,FX_CHAR* path,FX_DOUBLE mass, Vect3 mr, Vect6 I);
        :param robot_type: int ,机型，从CONFIG导入
        :param ipath: sting, 相对路径导入工具辨识轨迹数据。
        :return:
            m,mcp,i
        '''
        if type(robot_type) != int:
            raise ValueError("robot_type must be int type")

        if not os.path.exists(ipath):
            raise ValueError(f"no {ipath}, pls check!")

        robot_type_ = ctypes.c_int(robot_type)
        iden_path = ipath.encode('utf-8')
        path_char = ctypes.c_char_p(iden_path)

        # 创建指针变量而不是数组
        mm_ptr = ctypes.pointer(ctypes.c_double(0))
        mcp_ptr = (ctypes.c_double * 3)()
        ii_ptr = (ctypes.c_double * 6)()

        # 设置函数原型
        self.robot.FX_Robot_Iden_LoadDyn.argtypes = [
            ctypes.c_long,
            ctypes.c_char_p,
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double * 3),
            ctypes.POINTER(ctypes.c_double * 6)
        ]
        self.robot.FX_Robot_Iden_LoadDyn.restype = ctypes.c_bool

        # 调用函数
        success1 = self.robot.FX_Robot_Iden_LoadDyn(
            robot_type_,
            path_char,
            mm_ptr,
            mcp_ptr,
            ii_ptr
        )

        if success1:
            print('Identify tool dynamics successful')

            # 提取结果
            dyn_para = []
            m_val = mm_ptr.contents.value
            mcp_list = [mcp_ptr[i] for i in range(3)]
            ii_list = [ii_ptr[i] for i in range(6)]

            dyn_para.append(m_val)
            for i in mcp_list:
                dyn_para.append(i)
            for j in ii_list:
                dyn_para.append(j)

            print(f'tool dynamics: {dyn_para}')
            return dyn_para
        else:
            print('****error: Identify tool dynamics failed!')
            return False


    def get_tool_info(self,):
        '''检查控制器是否已经保存工具信息
        :return:
           m,mcp,i
        '''

        '''tool '''
        local_path='tool_dyn_kine.txt'
        remote_path='/home/fusion/tool_dyn_kine.txt'
        self.local_file_path = local_path.encode('utf-8')
        local_char = ctypes.c_char_p(self.local_file_path)
        self.remote_file_path = remote_path.encode('utf-8')
        remote_char = ctypes.c_char_p(self.remote_file_path)
        self.robot.OnRecvFile(local_char, remote_char)
        time.sleep(1)
        tool_result = read_csv_file_to_float_strict(local_path, expected_columns=16)
        return tool_result



    def help(self, method_name: str = None) -> None:
        """
        显示帮助信息

        参数:
            method_name (str): 可选的方法名，显示特定方法的帮助信息
        """
        print(f"\n{' API 帮助 ':=^50}\n")

        # 获取所有公共方法
        methods = [
            (name, func)
            for name, func in inspect.getmembers(self, inspect.ismethod)
            if not name.startswith('_') and name != 'help'
        ]

        # 如果没有指定方法名，显示所有方法列表
        if method_name is None:
            print("可用方法:")
            for name, func in methods:
                # 获取函数签名
                signature = inspect.signature(func)
                # 获取参数列表
                params = []
                for param in signature.parameters.values():
                    param_str = param.name
                    if param.default is not param.empty:
                        param_str += f"={param.default!r}"
                    if param.annotation is not param.empty:
                        param_str += f": {param.annotation.__name__}"
                    if param.kind == param.VAR_POSITIONAL:
                        param_str = "*" + param_str
                    elif param.kind == param.VAR_KEYWORD:
                        param_str = "**" + param_str
                    elif param.kind == param.KEYWORD_ONLY:
                        param_str = "[kw] " + param_str
                    params.append(param_str)

                param_list = ", ".join(params)
                print(f"  - {name}({param_list})")

            print("\n使用 help('方法名') 获取详细帮助信息")
            print(f"{'=' * 50}")
            return

        # 显示特定方法的帮助
        method_dict = dict(methods)
        if method_name in method_dict:
            func = method_dict[method_name]
            doc = inspect.getdoc(func) or "没有文档说明"

            # 获取函数签名
            signature = inspect.signature(func)

            print(f"方法: {method_name}{signature}")
            print("\n" + dedent(doc))

            # 显示参数详细信息
            print("\n参数详情:")
            for param in signature.parameters.values():
                param_info = f"  {param.name}: "
                if param.annotation is not param.empty:
                    param_info += f"类型: {param.annotation.__name__}, "
                if param.default is not param.empty:
                    param_info += f"默认值: {param.default!r}"
                # param_info += f"类型: {_param_kind_to_str(param.kind)}"
                print(param_info)
        else:
            print(f"错误: 没有找到方法 '{method_name}'")

        print(f"{'=' * 50}")


def _param_kind_to_str(kind):
    """将参数类型转换为可读字符串"""
    mapping = {
        inspect.Parameter.POSITIONAL_ONLY: "位置参数",
        inspect.Parameter.POSITIONAL_OR_KEYWORD: "位置或关键字参数",
        inspect.Parameter.VAR_POSITIONAL: "可变位置参数(*args)",
        inspect.Parameter.KEYWORD_ONLY: "仅关键字参数",
        inspect.Parameter.VAR_KEYWORD: "可变关键字参数(**kwargs)"
    }
    return mapping.get(kind, "未知参数类型")

# 定义StateCtr结构体
class StateCtr(Structure):
    _fields_ = [
        ("m_CurState", c_int),  # * 当前状态 */ ArmState
        ("m_CmdState", c_int),  # * 指令状态 */ DCSSCmdType 0
        ("m_ERRCode", c_int)    # * 机械臂错误码*/
    ]


# 定义RT_IN结构体
class RT_IN(Structure):
    _fields_ = [
        ("m_RtInSwitch", c_int),  # * 实时输入开关 用户实时数据 进行开关设置 0 -  close rt_in ;1- open rt_in*
        ("m_ImpType", c_int),  #阻抗类型
        ("m_InFrameSerial", c_int),  # short 输入帧序号   0 -  1000000 取模
        ("m_FrameMissCnt", c_short),  # short 丢帧计数
        ("m_MaxFrameMissCnt", c_short),  # short 开 启 后 最 大 丢 帧 计 数

        ("m_SysCyc", c_int),  # 0 -  1000000
        ("m_SysCycMissCnt", c_short),  # short 实 时 性  Miss 计 数
        ("m_MaxSysCycMissCnt", c_short),  # short开 启 后 最 大 实 时 性Miss 计 数

        ("m_ToolKine", c_float * 6),  # 工 具 运 动 学 参 数 1
        ("m_ToolDyn", c_float * 10),  # 工 具 动 力 学 参 数 1

        ("m_Joint_CMD_Pos", c_float * 7),  # 关 节 位 置 指 令
        ("m_Joint_Vel_Ratio", c_short),  # short 关 节 速 度 限 制 百分比 2
        ("m_Joint_Acc_Ratio", c_short),  # short 关 节 加 速 度 限 制  百分比 2

        ("m_Joint_K", c_float * 7),  # 关节阻抗刚度K指令 3
        ("m_Joint_D", c_float * 7),  # 关节阻抗刚度D指令 4

        ("m_DragSpType", c_int),  # 零空间类型 5
        ("m_DragSpPara", c_float * 6),  # 零空间参数类型 5

        ("m_Cart_KD_Type", c_int),  # 坐标阻抗类型
        ("m_Cart_K", c_float*6),  # 坐标阻抗刚度K指令 4
        ("m_Cart_D", c_float*6),  # 坐标阻抗阻尼D指令 4
        ("m_Cart_KN", c_float),  # 4
        ("m_Cart_DN", c_float),  # 4

        ("m_Force_FB_Type", c_int),  # 力控反馈源类型
        ("m_Force_Type", c_int),  # 力控类型 6
        ("m_Force_Dir", c_float * 6),  # 力控方向6维空间方向 6
        ("m_Force_PIDUL", c_float * 7),  # 力控pid 6
        ("m_Force_AdjLmt", c_float),  # 允许调节最大范围 6

        ("m_Force_Cmd", c_float),  # 力控指令 8

        ("m_SET_Tags", c_ubyte * 16),  # 零空间类型 5
        ("m_Update_Tags", c_ubyte * 16),  # 零空间类型 5

        ("m_PvtID", c_ubyte),  #设置的PVT号
        ("m_PvtID_Update", c_ubyte),  #PVT号更新情况
        ("m_Pvt_RunID", c_ubyte), #0: no pvt file; 1~99: 用户上传的PVT
        ("m_Pvt_RunState", c_ubyte),  #0: idle空闲; 1: loading正在加载 ; 2: running正在运行; 3: error出错啦

    ]


# 定义RT_OUT结构体
class RT_OUT(Structure):
    _fields_ = [
        ("m_OutFrameSerial", c_int),  # 输出帧序号   0 -  1000000 取模
        ("m_FB_Joint_Pos", c_float * 7),  # 关节位置反馈
        ("m_FB_Joint_Vel", c_float * 7),  # 关节速度反馈
        ("m_FB_Joint_PosE", c_float * 7),  # 关节位置(外编)
        ("m_FB_Joint_Cmd", c_float * 7),  # 位置关节指令
        ("m_FB_Joint_CToq", c_float * 7),  # 关节指令扭矩
        ("m_FB_Joint_SToq", c_float * 7),  # 关节实际扭矩
        ("m_FB_Joint_Them", c_float * 7),  # 关节温度
        ("m_EST_Joint_Firc", c_float * 7),  # 关节摩擦估计
        ("m_EST_Joint_Firc_Dot", c_float * 7),  # 关节力扰动估计值微分
        ("m_EST_Joint_Force", c_float * 7),  # 关节力扰动估计值
        ("m_EST_Cart_FN", c_float * 6),  # 末端笛卡尔空间力扰动估计值
        ("m_TipDI", c_char),  # 末端数字输入
        ("m_LowSpdFlag", c_char),  # 低速标志
        # ("m_pad", c_char * 2)  # 填充字节
    ]


# 定义DCSS结构体
class DCSS(Structure):
    _fields_ = [
        ("m_State", StateCtr * 2),  # 状态控制器数组
        ("m_In", RT_IN * 2),  # 输出数据数组
        ("m_Out", RT_OUT * 2),  # 输出数据数组

        ("m_ParaName", c_char * 30),  # 参数名称，结合配置机器人参数相关
        ("m_ParaType", c_ubyte),  # 0: FX_INT32; 1: FX_DOUBLE; 2: FX_STRING
        ("m_ParaIns", c_ubyte),  # DCSSCfgOperationType
        ("m_ParaValueI", c_int),  # FX_INT32 value
        ("m_ParaValueF", c_float),  # FX_FLOAT value
        ("m_ParaCmdSerial", c_short),  # short from PC
        ("m_ParaRetSerial", c_short),  # short working: 0; finish: cmd serial; error cmd_serial + 100
    ]



'''#################################### KINE CLASS #################################################################'''
class Marvin_Kine:
    def __init__(self):
        """初始化机器人控制类"""
        self.kine = ctypes.CDLL(current_path + '/src/marvin_robot/marvin_robot/MarvinLib/libMarvinKine.so')

    def initial_marvin_config(self, serial: int, config_path: str):
        ''' 初始化机械臂配置信息，解算运动学
        :param serial: 机器人构型：0：SRS，1：CCS
        :param config_path: 本地机械臂配置文件MARVINKINE_CONFIG,绝对路径.
                MARVINKINE_CONFIG如下：
                1007代表SRS， 1017代表CCS。
                以1007配置为例：
                前7行为各个关节的：alpha, a,d,theta, 关节负角度上限， 关节正角度上限， 速度最大值（度/秒）,加速度最大值（度/秒）
                第八行为末端法兰的信息：alpha, a,d,theta
                第九到12行为67关节自干涉的曲线信息，由于SRS构型无6和7关节自干涉，4x3的值都为零；CCS 不为零
                1007
                0.000000,0.000000,185.000000,0.000000,-170.000000,170.000000,180.000000,1800.000000,
                90.000000,0.000000,0.000000,0.000000,-120.000000,120.000000,180.000000,1800.000000,
                -90.000000,0.000000,290.000000,0.000000,-170.000000,170.000000,180.000000,1800.000000,
                90.000000,18.000000,0.000000,0.000000,-160.000000,160.000000,180.000000,1800.000000,
                -90.000000,-18.000000,280.000000,0.000000,-170.000000,170.000000,180.000000,1800.000000,
                90.000000,0.000000,0.000000,0.000000,-120.000000,120.000000,180.000000,1800.000000,
                -90.000000,0.000000,0.000000,0.000000,-170.000000,170.000000,180.000000,1800.000000,
                0.000000,0.000000,160.000000,0.000000,
                0,0,0,
                0,0,0,
                0,0,0,
                0,0,0,
                1017
                0.000000,0.000000,174.5,0.000000,-170.000000,170.000000,180.000000,1800.000000,
                90.000000,0.000000,0.000000,0.000000,-120.000000,120.000000,180.000000,1800.000000,
                -90.000000,0.000000,287.0000,0.000000,-170.000000,170.000000,180.000000,1800.000000,
                90.000000,18.000000,0.000000,180.000000,-160.000000,160.000000,180.000000,1800.000000,
                90.000000,18.000000,314.000000,-180.000000,-170.000000,170.000000,180.000000,1800.000000,
                90.000000,0.000000,0.000000,90.000000,-120.000000,120.000000,180.000000,1800.000000,
                -90.000000,0.000000,0.000000,90.000000,-170.000000,170.000000,180.000000,1800.000000,
                90.000000,0.000000,88.000000,90.000000,
                0.018004,-2.3205,108.4409,
                0.021823,2.5292,107.6665,
                -0.0084307,-1.3321,-100.2068,
                -0.014684,1.8496,-100.247,
        :return:
            bool True False
        '''
        if not os.path.exists(config_path):
            raise ValueError('NO CORRECT CONFIG PATH!')
        serial_long = ctypes.c_int(serial)
        self.marvin_config_path = config_path.encode('utf-8')
        path_char = ctypes.c_char_p(self.marvin_config_path)
        return self.kine.OnInitKine_MARVINKINE(serial_long, path_char);


    def fk(self, joints: list):
        '''关节角度正解到末端TCP位置和姿态XYZABC
        :param joints: list(7,1). 角度值
        :return:
            4x4的位姿矩阵，list(4,4)
        '''
        j0, j1, j2, j3, j4, j5, j6 = joints
        joints_double = (ctypes.c_double * 7)(j0, j1, j2, j3, j4, j5, j6)
        Matrix4x4 = ((ctypes.c_double * 4) * 4)
        pg = Matrix4x4()
        for i in range(4):
            for j in range(4):
                pg[i][j] = 1.0 if i == j else 0.0

        self.kine.OnKine.argtypes = [ctypes.POINTER(ctypes.c_double * 7),
                                     ctypes.POINTER((ctypes.c_double * 4) * 4)]
        self.kine.OnKine(ctypes.byref(joints_double), ctypes.byref(pg))
        fk_mat = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
        for i in range(4):
            for j in range(4):
                fk_mat[i][j] = pg[i][j]
        print('fk matrix:', fk_mat)
        return fk_mat

    def ik(self, mat4x4: list, ref_joints: list, isOutRange: int, Is123Deg: int, Is567Deg: int):
        '''末端位置和姿态XYZABC逆解到关节值
        :param mat4x4: list(4,4), 位置姿态4x4list.
        :param ref_joints: list(7,1),参考输入角度，约束构想接近参考解读，防止解出来的构型跳变。
        :param isOutRange:输入位姿是否超过可达空间范围。  0 or 1
        :param Is123Deg:123关节是否奇异。 0 or 1
        :param Is567Deg:567关节是否奇异。 0 or 1
        :return:
            joints(7,1)
        '''

        joints_double = ctypes.c_double * 7
        j0_, j1_, j2_, j3_, j4_, j5_, j6_ = ref_joints
        ref_joints_value = joints_double(j0_, j1_, j2_, j3_, j4_, j5_, j6_)

        target_joints_value = joints_double()

        isOutRange_char = ctypes.c_ubyte(isOutRange)
        Is123Deg_char = ctypes.c_ubyte(Is123Deg)
        Is567Deg_char = ctypes.c_ubyte(Is567Deg)

        # 创建输入矩阵 (4x4)
        pg = ((ctypes.c_double * 4) * 4)()

        mat16_4x4= [mat4x4[i:i+4] for i in range(0, len(mat4x4), 4)]
        for i in range(4):
            for j in range(4):
                pg[i][j] = mat16_4x4[i][j]

        self.kine.OnInvKine.argtypes = [ctypes.POINTER((ctypes.c_double * 4) * 4),
                                        ctypes.POINTER(ctypes.c_double * 7),
                                        ctypes.POINTER(ctypes.c_double * 7),
                                        ctypes.POINTER(ctypes.c_ubyte),
                                        ctypes.POINTER(ctypes.c_ubyte),
                                        ctypes.POINTER(ctypes.c_ubyte), ]

        self.kine.OnInvKine(ctypes.byref(pg),
                            ctypes.byref(ref_joints_value),
                            ctypes.byref(target_joints_value),
                            ctypes.byref(isOutRange_char),
                            ctypes.byref(Is123Deg_char),
                            ctypes.byref(Is567Deg_char))
        ik_joints = [0.] * 7
        for i in range(7):
            ik_joints[i] = target_joints_value[i]
        print(f'ik joints:', ik_joints)
        return ik_joints

    def ik_nsp(self, nsp_angle: float, ref_joints: list, isOutRange: int, Is123Deg: int, Is567Deg: int):
        '''逆解优化
        当IK得到的关节值未到预期，可以调用该接口调整臂角。
        :param nsp_angle: 臂角平面旋转的角度, 值范围0到360度。类似人手臂，手固定不动，大臂和小臂随意转动以避障或达到理想位姿
        :param ref_joints: list(7,1),参考输入角度，约束构想接近参考解读，防止解出来的构型跳变。
        :param isOutRange:输入位姿是否超过可达空间范围。  0 or 1
        :param Is123Deg:123关节是否奇异。 0 or 1
        :param Is567Deg:567关节是否奇异。 0 or 1
        :return:
            joints(7,1)
        '''
        nsp_angle_double = ctypes.c_double(nsp_angle)
        if len(ref_joints) != 7:
            raise ValueError("ref_joints must have exactly 7 elements")
        ref_joints_array = (ctypes.c_double * 7)(*ref_joints)

        target_joints_array = (ctypes.c_double * 7)()

        isOutRange_char = ctypes.c_ubyte(isOutRange)
        Is123Deg_char = ctypes.c_ubyte(Is123Deg)
        Is567Deg_char = ctypes.c_ubyte(Is567Deg)

        self.kine.OnInvKine_NSP.argtypes = [
            ctypes.c_double,  # nsp_angle (值传递)
            ctypes.POINTER(ctypes.c_double),  # ref_joints (指针传递)
            ctypes.POINTER(ctypes.c_double),  # return_joints (指针传递)
            ctypes.POINTER(ctypes.c_ubyte),  # IsOutRange (指针传递)
            ctypes.POINTER(ctypes.c_ubyte),  # Is123Deg (指针传递)
            ctypes.POINTER(ctypes.c_ubyte)  # Is567Deg (指针传递)
        ]

        self.kine.OnInvKine_NSP(
            nsp_angle_double,  # 值传递 (直接传对象)
            ref_joints_array,  # 数组自动退化为指针
            target_joints_array,  # 数组自动退化为指针
            ctypes.byref(isOutRange_char),
            ctypes.byref(Is123Deg_char),
            ctypes.byref(Is567Deg_char)
        )

        # 将结果转换为Python列表
        ik_joints = [target_joints_array[i] for i in range(7)]
        print(f'nsp_ik_result: {ik_joints}')
        return ik_joints

    def ik_range_crosss67(self, joints67: list):
        '''防止67关节超限碰撞和计算阻尼
        CSS十字交叉构型机械臂，六七关节运动时，可能发生碰撞，因此逆解出的关节，将最后两个关节算一下干涉边界关节值。
        :param joints67: list(2,1). 当前六七关节值.
        :return: 当前六七关节边界关节值.
        '''
        double2 = ctypes.c_double * 2
        j0, j1 = joints67
        joints67_double = double2(j0, j1)
        RetBound67 = double2()
        RetBound67[0] = 0.
        RetBound67[1] = 0.
        self.kine.OnInvKineRange_Cross67.argtypes = [ctypes.POINTER(ctypes.c_double * 2),
                                                     ctypes.POINTER(ctypes.c_double * 2)]
        self.kine.OnInvKineRange_Cross67(ctypes.byref(joints67_double),
                                         ctypes.byref(RetBound67))
        bound67 = [RetBound67[0], RetBound67[1]]
        print(f'result bound67:', bound67)
        return bound67

    def joints2JacobMatrix(self, joints: list):
        '''当前角度转成雅可比矩阵
        :param joints: list(7,1)
        :return: 雅可比矩阵6*7矩阵
        '''

        joints_double = ctypes.c_double * 7
        j0, j1, j2, j3, j4, j5, j6 = joints
        joints_value = joints_double(j0, j1, j2, j3, j4, j5, j6)

        jacob_mat =  ((ctypes.c_double * 7) * 6)()
        for ii in range(6):
            for jj in range(7):
                jacob_mat[ii][jj] = 0

        self.kine.OnJacob.argtypes = [ctypes.POINTER(ctypes.c_double * 7),
                                      ctypes.POINTER((ctypes.c_double * 7) * 6)]

        self.kine.OnJacob(ctypes.byref(joints_value),
                                         ctypes.byref(jacob_mat))

        result_jacob_mat = [[0, 0, 0, 0, 0, 0,0], [0, 0, 0, 0, 0, 0,0], [0, 0, 0, 0, 0, 0,0], [0, 0, 0, 0, 0, 0,0],
                            [0, 0, 0, 0, 0, 0,0], [0, 0, 0, 0, 0, 0,0]]
        for i in range(6):
            for j in range(7):
                result_jacob_mat[i][j] = jacob_mat[i][j]
        print('result_jacob_mat:', result_jacob_mat)
        return result_jacob_mat

if __name__ == '__main__':
    main()
