import tkinter as tk
from tkinter import messagebox, ttk, scrolledtext, filedialog, simpledialog
import threading
import time
import ast  # 用于将字符串转换为列表
from python.fx_robot import Marvin_Robot
from python.structure_data import DCSS
from python.fx_kine import Marvin_Kine
# from python.robot_structures import *
import os
import glob
import math
import sys
import queue

crr_pth = os.getcwd()
dcss = DCSS()
robot = Marvin_Robot()

kk1 = Marvin_Kine()
kk2 = Marvin_Kine()
ini_result = kk1.load_config(config_path=glob.glob('config/*.MvKDCfg')[0])
print(f'ini_results:{ini_result}')
initial_kine_tag = kk1.initial_kine(robot_serial=0,
                                    robot_type=ini_result['TYPE'][0],
                                    dh=ini_result['DH'][0],
                                    pnva=ini_result['PNVA'][0],
                                    j67=ini_result['BD'][0])

ini_result = kk2.load_config(config_path=glob.glob('config/*.MvKDCfg')[0])
initial_kine_tag = kk2.initial_kine(robot_serial=1,
                                    robot_type=ini_result['TYPE'][0],
                                    dh=ini_result['DH'][0],
                                    pnva=ini_result['PNVA'][0],
                                    j67=ini_result['BD'][0])

button_w = 10

# 定义常量
DBL_EPSILON = sys.float_info.epsilon

# 创建队列
data_queue = queue.Queue()
def read_data(robot_id,com):
    '''接收CAN的HEX数据'''
    while True:
        try:
            tag, receive_hex_data = robot.get_485_data(robot_id, com)
            if tag >= 1:
                print(f"接收的HEX数据：{receive_hex_data}")
                data_queue.put(receive_hex_data)
            else:
                time.sleep(0.001)
        except Exception as e:
            # print(f"读取数据错误: {e}")
            time.sleep(0.001)


def get_received_data():
    '''获取接收到的数据并计数'''
    received_count = 0
    received_data_list = []

    while True:
        try:
            data = data_queue.get_nowait()
            received_count += 1
            received_data_list.append(data)
            print(f'received_data_list:{received_data_list}')
        except queue.Empty:
            break

    return received_count, received_data_list


def Matrix2ABC(m, abc):
    """将3x3矩阵转换为ABC角度"""
    r = math.sqrt(m[0][0] * m[0][0] + m[1][0] * m[1][0])
    abc[1] = math.atan2(-m[2][0], r) * 57.295779513082320876798154814105

    if abs(r) <= DBL_EPSILON:
        abc[2] = 0
        if abc[1] > 0:
            abc[0] = math.atan2(m[0][1], m[1][1]) * 57.295779513082320876798154814105
        else:
            abc[0] = -math.atan2(m[0][1], m[1][1]) * 57.295779513082320876798154814105
    else:
        abc[2] = math.atan2(m[1][0], m[0][0]) * 57.295779513082320876798154814105
        abc[0] = math.atan2(m[2][1], m[2][2]) * 57.295779513082320876798154814105

    return True


def FX_VectCross(a, b):
    """向量叉积"""
    result = [0.0] * 3
    result[0] = a[1] * b[2] - a[2] * b[1]
    result[1] = a[2] * b[0] - a[0] * b[2]
    result[2] = a[0] * b[1] - a[1] * b[0]
    return result


def NormVect(a):
    """向量模长"""
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def main_function(vx, vy):
    """主函数 - vx和vy是列向量"""
    m_S = ""

    if NormVect(vx) < 0.01 or NormVect(vy) < 0.01:
        return m_S, [0, 0, 0]

    vz = FX_VectCross(vx, vy)

    vz_norm = NormVect(vz)
    if vz_norm < 0.99 or vz_norm > 1.01:
        return m_S, [0, 0, 0]

    # 构建3x3矩阵 - vx, vy, vz 作为列向量
    m_mat = [
        [vx[0], vy[0], vz[0]],  # 第一列: vx, 第二列: vy, 第三列: vz
        [vx[1], vy[1], vz[1]],
        [vx[2], vy[2], vz[2]]
    ]

    # 矩阵形式显示
    m_S += "矩阵形式（列向量为坐标方向向量）：\n"
    m_S += f"{m_mat[0][0]:.2f}\t{m_mat[0][1]:.2f}\t{m_mat[0][2]:.2f}\n"
    m_S += f"{m_mat[1][0]:.2f}\t{m_mat[1][1]:.2f}\t{m_mat[1][2]:.2f}\n"
    m_S += f"{m_mat[2][0]:.2f}\t{m_mat[2][1]:.2f}\t{m_mat[2][2]:.2f}\n\n"

    # 计算ABC角度
    m_abc = [0.0] * 3
    Matrix2ABC(m_mat, m_abc)

    m_S += f"ABC角度：[{m_abc[0]:.5f}, {m_abc[1]:.5f}, {m_abc[2]:.5f}]\n"

    return m_S


# 格式化数据
def format_vector(vector):
    return ", ".join([f"{v:.2f}" for v in vector])


def preview_text_file():
    """在新窗口中预览文本文件"""
    file_path = os.path.join(crr_pth, "config/python_doc_contrl.md")

    # 创建新窗口
    preview_window = tk.Toplevel(root)
    preview_window.title(f"预览文档: {file_path.split('/')[-1]}")
    preview_window.geometry("600x400")

    # 创建带滚动条的文本框
    scroll_frame = tk.Frame(preview_window)
    scroll_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

    scrollbar = tk.Scrollbar(scroll_frame)
    scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

    text_area = scrolledtext.ScrolledText(
        scroll_frame,
        wrap=tk.WORD,
        yscrollcommand=scrollbar.set
    )
    text_area.pack(fill=tk.BOTH, expand=True)
    scrollbar.config(command=text_area.yview)

    # 添加关闭按钮
    close_btn = tk.Button(
        preview_window,
        text="关闭预览",
        command=preview_window.destroy
    )
    close_btn.pack(pady=10)

    # 读取并显示文件内容
    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            content = file.read()
            text_area.insert(tk.INSERT, content)
            text_area.config(state=tk.DISABLED)  # 设置为只读
    except Exception as e:
        messagebox.showerror("错误", f"无法读取文件:\n{str(e)}")


def preview_text_file_1():
    """在新窗口中预览文本文件"""
    messagebox.showinfo("采集idx说明", f"采集数据ID序号:\n"
                                       "左臂\n"
                                       "0-6:左臂关节位置\n"
                                       "10-16:左臂关节速度\n"
                                       "20-26:左臂外编位置\n"
                                       "30-36:左臂关节指令位置\n"
                                       "40-46:左臂关节电流（千分比）\n"
                                       "50-56:左臂关节传感器扭矩NM\n"
                                       "60-66:左臂摩擦力估计值\n"
                                       "70-76:左臂摩檫力速度估计值\n"
                                       "80-85:左臂关节外力估计值\n"
                                       "90-95:左臂末端点外力估计值\n\n"
                                       "特别注意 76 为左臂动力学辨识标记列\n"
                                       "\n右臂\n"
                                       "100-106:右臂关节位置\n"
                                       "110-116:右臂关节速度\n"
                                       "120-126:右臂外编位置\n"
                                       "130-136:右臂关节指令位置\n"
                                       "140-146:右臂关节电流（千分比）\n"
                                       "150-156:右臂关节传感器扭矩NM\n"
                                       "160-166:右臂摩擦力估计值\n"
                                       "170-176:右臂摩檫力速度估计值\n"
                                       "180-185:右臂关节外力估计值\n"
                                       "190-195:右臂末端点外力估计值\n\n"
                                       "特别注意 176 为右臂动力学辨识标记列\n"
                        )


class DataSubscriber:
    """数据订阅器，定期更新数据"""

    def __init__(self, callback):
        self.callback = callback
        self.running = True
        self.thread = threading.Thread(target=self.generate_data, daemon=True)
        self.thread.start()

    def generate_data(self):
        """订阅数据"""
        while self.running:
            result = robot.subscribe(dcss)
            # 回调更新UI
            self.callback(result)
            time.sleep(0.5)  # 每0.5秒更新一次

    def stop(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)


class App:
    def __init__(self, root):
        self.root = root
        root.title("MARVIN_APP")
        root.geometry("1300x800")
        root.configure(bg="#f0f0f0")

        self.tools_txt = 'tool_dyn_kine.txt'
        self.tool_result = None

        self.save_tool_data_path = None

        # 初始化数据
        self.result = {
            'states': [
                {'cur_state': 0, 'cmd_state': 0, 'err_code': 0}, {'cur_state': 0, 'cmd_state': 0, 'err_code': 0},
                {'cur_state': 0, 'cmd_state': 0, 'err_code': 0}, {'cur_state': 0, 'cmd_state': 0, 'err_code': 0}
            ],
            'outputs':
                [
                    {'frame_serial': 0,
                     'tip_di': b'\x00',
                     'low_speed_flag': b'\x00',
                     'fb_joint_pos': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 反馈关节位置
                     'fb_joint_vel': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 反馈关节速度
                     'fb_joint_posE': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 反馈关节位置(外编)
                     'fb_joint_cmd': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 位置关节指令
                     'fb_joint_cToq': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 反馈关节电流
                     'fb_joint_sToq': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 传感器
                     'fb_joint_them': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 反馈关节温度
                     'est_joint_firc': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                     'est_joint_firc_dot': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                     'est_joint_force': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 轴外力
                     'est_cart_fn': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},

                    {'frame_serial': 0,
                     'tip_di': b'\x00',
                     'low_speed_flag': b'\x00',
                     'fb_joint_pos': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                     'fb_joint_vel': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                     'fb_joint_posE': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                     'fb_joint_cmd': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                     'fb_joint_cToq': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                     'fb_joint_sToq': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                     'fb_joint_them': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                     'est_joint_firc': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                     'est_joint_firc_dot': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                     'est_joint_force': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                     'est_cart_fn': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
                ]
        }

        # 初始化两个点的列表
        self.points1 = []
        self.points2 = []

        self.command1=[]
        self.command2=[]

        # 初始化参数列表
        self.params = []

        self.period_file_path_1 = tk.StringVar()
        self.period_file_path_2 = tk.StringVar()

        self.file_path_50 = tk.StringVar()

        self.file_path_tool = tk.StringVar()

        self.processed_data = []

        self.m_sq_offset_1 = 0.0
        self.m_sq_offset_2 = 0.0

        # 当前显示模式: 0=位置, 1=传感器, 2=电流
        self.display_mode = 0
        self.mode_names = ["位置数据", "速度数据", "传感器数据", "电流数据", "温度数据", "外编位置数据", "指令位置数据",
                           "轴外力数据"]
        self.data_keys = [('fb_joint_pos'), ('fb_joint_vel'), ('fb_joint_sToq'), ('fb_joint_cToq'), ('fb_joint_them'),
                          ('fb_joint_posE'), ('fb_joint_cmd'), ('est_joint_force')]

        # 存储主界面组件的引用
        self.main_interface_widgets = []
        # 创建顶部控制面板
        self.create_control_panel()

        # 创建主内容区域
        self.create_main_content()

        # 创建底部状态栏
        self.create_status_bar()

        # 初始未连接
        self.connected = False
        self.data_subscriber = None

        # 绑定窗口关闭事件
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        # 密码设置
        self.correct_password = "1"

    def create_control_panel(self):
        """创建顶部控制面板"""
        self.control_frame = tk.Frame(self.root, bg="#e0e0e0", padx=10, pady=10)
        self.control_frame.pack(fill="x")

        # 连接按钮
        self.connect_btn = tk.Button(
            self.control_frame,
            text="连接机器人",
            width=15,
            command=self.toggle_connection,
            bg="#4CAF50",
            fg="white",
            font=("Arial", 10, "bold"))
        self.connect_btn.pack(side="left", padx=10)

        self.arm_ip_entry = tk.Entry(self.control_frame, )
        self.arm_ip_entry.insert(0, "192.168.1.190")
        self.arm_ip_entry.pack(side="left", padx=10)

        # 更多功能菜单按钮
        self.more_features_btn = tk.Button(
            self.control_frame,
            text="更多功能",
            width=15,
            command=self.show_more_features,
            bg="#9C27B0",
            fg="white",
            font=("Arial", 10, "bold")
        )
        self.more_features_btn.pack(side="right", padx=5)

        # # 查看文档
        # self.readme_button = tk.Button(self.control_frame, text="查看文档", width=15, command=preview_text_file,
        #                                font=("Arial", 10, "bold"))
        # self.readme_button.pack(side="right", padx=10)

        '''###########################mor more more############################'''
        # 隐藏功能按钮
        self.hidden_features_btn = tk.Button(self.control_frame, text="系统升级", width=15, bg="#F5FC34",
                                             command=self.authenticate_and_show_hidden, font=("Arial", 10, "bold"))
        self.hidden_features_btn.pack(side="right", padx=5, pady=10)

        # 模式切换按钮
        self.mode_btn = tk.Button(
            self.control_frame,
            text="位置数据",
            width=15,
            command=self.toggle_display_mode,

            state="disabled",
            bg="#2196F3",
            fg="#fffef9",
            font=("Arial", 10, "bold"))
        self.mode_btn.pack(side="right", padx=10)

        # 急停
        self.stop_btn = tk.Button(
            self.control_frame,
            text="急停",
            width=15,
            command=self.stop_command,
            bg="#ef4136",
            fg="#fffef9",
            font=("Arial", 10, "bold"))
        self.stop_btn.pack(side="right", padx=10)

        # 状态指示灯
        status_frame = tk.Frame(self.control_frame, bg="#e0e0e0")
        status_frame.pack(side="right", padx=10)

        tk.Label(status_frame, text="连接状态:", bg="#e0e0e0", font=("Arial", 9)).pack(side="left")
        self.status_light = tk.Label(status_frame, text="●", font=("Arial", 16), fg="red")
        self.status_light.pack(side="left", padx=5)
        self.status_label = tk.Label(status_frame, text="未连接", bg="#e0e0e0", font=("Arial", 9))
        self.status_label.pack(side="left")

    '''###############################################################################################################'''

    def show_more_features(self):
        """显示更多功能菜单"""
        # 创建菜单
        menu = tk.Menu(self.root, tearoff=0)

        # 添加菜单项
        menu.add_command(label="附加功能", command=self.additional_settings)
        # menu.add_command(label="数据管理", command=self.open_data_management)
        # menu.add_command(label="日志查看", command=self.open_log_viewer)
        # menu.add_separator()
        # menu.add_command(label="校准工具", command=self.open_calibration_tool)
        # menu.add_command(label="诊断工具", command=self.open_diagnostic_tool)
        menu.add_separator()
        menu.add_command(label="查看文档", command=self.open_doc)
        # menu.add_command(label="关于软件", command=self.open_about)

        # 显示菜单
        try:
            menu.tk_popup(
                self.more_features_btn.winfo_rootx(),
                self.more_features_btn.winfo_rooty() + self.more_features_btn.winfo_height()
            )
        finally:
            menu.grab_release()

    def open_doc(self):
        return preview_text_file()

    def additional_settings(self):
        """打开系统设置窗口"""
        settings_window = tk.Toplevel(self.root)
        settings_window.title("附加功能")
        settings_window.geometry("800x600")
        settings_window.configure(bg="#f0f0f0")
        settings_window.transient(self.root)
        settings_window.grab_set()

        # 创建选项卡
        notebook = ttk.Notebook(settings_window)
        notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # 浮动基座设置选项卡
        floating_base_frame = ttk.Frame(notebook, padding="10")
        notebook.add(floating_base_frame, text="浮动基座参数计算")

        # 网络设置选项卡
        network_frame = ttk.Frame(notebook, padding="10")
        notebook.add(network_frame, text="网络设置")

        # # 界面设置选项卡
        # interface_frame = ttk.Frame(notebook, padding="10")
        # notebook.add(interface_frame, text="界面设置")

        # 填充浮动基座设置选项卡
        self.create_floating_base_tab(floating_base_frame)

        # 填充网络设置选项卡
        self.create_network_settings_tab(network_frame)

        # # 填充界面设置选项卡
        # self.create_interface_settings_tab(interface_frame)

        # 按钮
        button_frame = ttk.Frame(settings_window)
        button_frame.pack(pady=10)

        ttk.Button(button_frame, text="保存设置",
                   command=lambda: self.save_all_settings(notebook)).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="关闭",
                   command=settings_window.destroy).pack(side=tk.LEFT, padx=5)

    def create_network_settings_tab(self, parent):
        """创建网络设置选项卡内容"""
        ttk.Label(parent, text="网络设置", font=("Arial", 14, "bold")).pack(pady=10)

        network_frame = ttk.LabelFrame(parent, text="网络配置", padding="15")
        network_frame.pack(fill=tk.X, pady=10)

        ttk.Label(network_frame, text="默认IP地址:").grid(row=0, column=0, sticky="w", pady=5)
        self.default_ip_entry = ttk.Entry(network_frame, width=20)
        self.default_ip_entry.insert(0, "192.168.1.190")
        self.default_ip_entry.grid(row=0, column=1, pady=5, padx=10)

        ttk.Label(network_frame, text="端口号:").grid(row=1, column=0, sticky="w", pady=5)
        self.port_entry = ttk.Entry(network_frame, width=20)
        self.port_entry.insert(0, "502")
        self.port_entry.grid(row=1, column=1, pady=5, padx=10)

        ttk.Label(network_frame, text="超时时间(秒):").grid(row=2, column=0, sticky="w", pady=5)
        self.timeout_entry = ttk.Entry(network_frame, width=20)
        self.timeout_entry.insert(0, "10")
        self.timeout_entry.grid(row=2, column=1, pady=5, padx=10)

    def create_floating_base_tab(self, parent):
        """创建浮动基座参数设置选项卡"""
        # 存储选择结果的列表
        self.row2_selection = [0, 0, 0]  # 对应X,Y,Z
        self.row3_selection = [0, 0, 0]  # 对应X,Y,Z

        # 存储单选按钮变量
        self.row2_var = tk.StringVar()
        self.row3_var = tk.StringVar()

        # 添加变量追踪，确保选择变化时及时更新
        self.row2_var.trace('w', lambda *args: self.on_selection_change(2))
        self.row3_var.trace('w', lambda *args: self.on_selection_change(3))

        ttk.Label(parent, text="浮动基座参数计算", font=("Arial", 14, "bold")).pack(pady=10)

        # 第一行
        row1_frame = ttk.Frame(parent)
        row1_frame.pack(fill="x", pady=5)

        ttk.Label(row1_frame, text="基座的坐标方向(x轴和y轴)").pack(side="left", padx=5)
        ttk.Label(row1_frame, text="UMI的坐标方向(基座与UMI坐标方向重合选项)").pack(side="right", padx=5)

        # 第二行和第三行容器
        axis_frame = ttk.Frame(parent)
        axis_frame.pack(fill="x", pady=10)

        # 左侧标签
        ttk.Label(axis_frame, text="X轴").grid(row=0, column=0, padx=10, pady=10, sticky="w")
        ttk.Label(axis_frame, text="Y轴").grid(row=1, column=0, padx=10, pady=10, sticky="w")

        # 第二行和第三行的选项
        options = ["x", "-x", "y", "-y", "z", "-z"]

        # 第二行的单选按钮
        self.row2_buttons = []
        for i, option in enumerate(options):
            btn = ttk.Radiobutton(axis_frame, text=option, value=option,
                                  variable=self.row2_var,
                                  command=lambda: self.on_selection_change(2))
            btn.grid(row=0, column=i + 1, padx=5, pady=5)
            self.row2_buttons.append(btn)

        # 第三行的单选按钮
        self.row3_buttons = []
        for i, option in enumerate(options):
            btn = ttk.Radiobutton(axis_frame, text=option, value=option,
                                  variable=self.row3_var,
                                  command=lambda: self.on_selection_change(3))
            btn.grid(row=1, column=i + 1, padx=5, pady=5)
            self.row3_buttons.append(btn)

        # 结果显示区域
        self.result_frame = ttk.LabelFrame(parent, text="计算结果")
        self.result_frame.pack(fill="both", expand=True, pady=10)

        self.result_text = tk.Text(self.result_frame, height=8, wrap=tk.WORD)
        scrollbar = ttk.Scrollbar(self.result_frame, orient=tk.VERTICAL, command=self.result_text.yview)
        self.result_text.config(yscrollcommand=scrollbar.set)

        self.result_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y, pady=5)

    def create_interface_settings_tab(self, parent):
        """创建界面设置选项卡内容"""
        ttk.Label(parent, text="界面设置", font=("Arial", 14, "bold")).pack(pady=10)

        interface_frame = ttk.LabelFrame(parent, text="界面配置", padding="15")
        interface_frame.pack(fill=tk.X, pady=10)

        self.theme_var = tk.StringVar(value="浅色")
        ttk.Label(interface_frame, text="主题:").grid(row=0, column=0, sticky="w", pady=5)
        ttk.Combobox(interface_frame, textvariable=self.theme_var,
                     values=["浅色", "深色", "自动"], state="readonly", width=15).grid(row=0, column=1, pady=5, padx=10)

        self.language_var = tk.StringVar(value="中文")
        ttk.Label(interface_frame, text="语言:").grid(row=1, column=0, sticky="w", pady=5)
        ttk.Combobox(interface_frame, textvariable=self.language_var,
                     values=["中文", "英文", "日文"], state="readonly", width=15).grid(row=1, column=1, pady=5, padx=10)

        self.auto_connect_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(interface_frame, text="启动时自动连接",
                        variable=self.auto_connect_var).grid(row=2, column=0, columnspan=2, sticky="w", pady=5)

        self.auto_save_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(interface_frame, text="自动保存设置",
                        variable=self.auto_save_var).grid(row=3, column=0, columnspan=2, sticky="w", pady=5)

    def on_selection_change(self, changed_row):
        """当选择改变时调用，处理互锁逻辑并更新结果"""
        # 更新选择列表
        self.update_selection_lists()
        # 应用互锁逻辑
        self.apply_mutual_exclusion(changed_row)
        # 如果两行都有选择，则计算并显示结果
        if any(self.row2_selection) and any(self.row3_selection):
            result = self.get_abc_calculation()
            self.display_result(result)
        else:
            self.result_text.delete(1.0, tk.END)
            self.result_text.insert(tk.END, "请完成两行的选择以查看计算结果")

    def update_selection_lists(self):
        """根据当前选择更新选择列表"""
        # 重置选择列表
        self.row2_selection = [0, 0, 0]
        self.row3_selection = [0, 0, 0]

        # 更新第二行选择
        row2_val = self.row2_var.get()
        if row2_val=="x":
            self.row2_selection[0] = 1
        if row2_val=="-x":
            self.row2_selection[0] = -1
        if row2_val=="y":
            self.row2_selection[1] = 1
        if row2_val=="-y":
            self.row2_selection[1] = -1
        if row2_val=="z":
            self.row2_selection[2] = 1
        if row2_val=="-z":
            self.row2_selection[2] = -1


        # 更新第三行选择
        row3_val = self.row3_var.get()
        if row3_val =="x":
            self.row3_selection[0] = 1
        if row3_val =="-x":
            self.row3_selection[0] = -1
        if row3_val =="y":
            self.row3_selection[1] = 1
        if row3_val =="-y":
            self.row3_selection[1] = -1
        if row3_val =="z":
            self.row3_selection[2] = 1
        if row3_val =="-z":
            self.row3_selection[2] = -1

    def apply_mutual_exclusion(self, changed_row):
        """应用互锁逻辑，禁用冲突的选项"""
        row2_val = self.row2_var.get()
        row3_val = self.row3_var.get()

        # 重置所有按钮状态
        for btn in self.row2_buttons + self.row3_buttons:
            btn.state(["!disabled"])

        # 如果第二行有选择，禁用第三行对应的轴
        if row2_val:
            if row2_val in ["x", "-x"]:
                self.disable_axis_options(self.row3_buttons, ["x", "-x"])
            elif row2_val in ["y", "-y"]:
                self.disable_axis_options(self.row3_buttons, ["y", "-y"])
            elif row2_val in ["z", "-z"]:
                self.disable_axis_options(self.row3_buttons, ["z", "-z"])

        # 如果第三行有选择，禁用第二行对应的轴
        if row3_val:
            if row3_val in ["x", "-x"]:
                self.disable_axis_options(self.row2_buttons, ["x", "-x"])
            elif row3_val in ["y", "-y"]:
                self.disable_axis_options(self.row2_buttons, ["y", "-y"])
            elif row3_val in ["z", "-z"]:
                self.disable_axis_options(self.row2_buttons, ["z", "-z"])

    def disable_axis_options(self, buttons, options_to_disable):
        """禁用指定的选项"""
        for btn in buttons:
            if btn['value'] in options_to_disable:
                btn.state(["disabled"])

    def get_abc_calculation(self):
        """计算函数，返回多行结果"""
        result = f"基座坐标方向在陀螺仪imu的旋转为\n"
        result += "=" * 20 + "\n"

        try:
            # 这里调用您的计算函数
            # print(f'*********{self.row2_selection},{self.row2_selection}')
            abc = main_function(self.row2_selection, self.row3_selection)
            result += abc
            result += "\n"
        except Exception as e:
            result += f"计算错误: {str(e)}\n"

        result += "=" * 20 + "\n\n"
        result += ("请将ABC三个角度分别更新到robot.ini [R.A0.BASIC]组下的:\n"
                   "              GYROSETA、GYROSETB、GYROSETC\n"
                   "请注意左右臂请依次计算，[R.A0.BASIC]为左臂，[R.A1.BASIC]为右臂。")
        return result

    def main_function(self, selection1, selection2):
        """模拟计算函数，您需要替换为实际的 umi2abc 函数"""
        # 这里应该是您从 umi2abc 导入的 main_function
        # 暂时返回模拟结果
        return f"A: 45.0°, B: 30.0°, C: 15.0°"

    def display_result(self, result):
        """在文本框中显示结果"""
        self.result_text.delete(1.0, tk.END)
        self.result_text.insert(tk.END, result)

    def save_all_settings(self, notebook):
        """保存所有设置"""
        try:
            # 获取网络设置
            ip = self.default_ip_entry.get()
            port = self.port_entry.get()
            timeout = self.timeout_entry.get()

            # 获取界面设置
            theme = self.theme_var.get()
            language = self.language_var.get()
            auto_connect = self.auto_connect_var.get()
            auto_save = self.auto_save_var.get()

            messagebox.showinfo("保存成功",
                                f"设置已保存:\n"
                                f"IP: {ip}\n"
                                f"端口: {port}\n"
                                f"主题: {theme}\n"
                                f"语言: {language}")
        except Exception as e:
            messagebox.showerror("保存错误", f"保存设置时出错: {str(e)}")

    def create_hidden_features_interface(self):

        self.hidden_features_frame = tk.Frame(self.root, bg="#f0f0f0")

        # 隐藏功能标题
        hidden_title = tk.Label(self.hidden_features_frame, text="系统更新升级",
                                font=("Arial", 16, "bold"), bg="#f0f0f0")
        hidden_title.pack(pady=20)

    def authenticate_and_show_hidden(self):
        """验证密码并显示隐藏功能选择窗口"""
        self.show_update_system_menu()
        # password = simpledialog.askstring("密码验证", "请输入密码:", show='*')
        # if password == self.correct_password:
        #     self.show_update_system_menu()
        # elif password is not None:
        #     messagebox.showerror("错误", "密码错误!")

    def show_update_system_menu(self):
        """显示隐藏功能选择窗口"""
        hidden_window = tk.Toplevel(self.root)
        hidden_window.title("系统升级")
        hidden_window.geometry("600x400")
        hidden_window.configure(bg="#f0f0f0")
        hidden_window.transient(self.root)  # 设置为主窗口的子窗口
        hidden_window.grab_set()  # 模态窗口

        # 标题
        title_label = tk.Label(hidden_window, text="系统升级",
                               font=("Arial", 16, "bold"), bg="#f0f0f0")
        title_label.pack(pady=20)

        # 功能按钮框架
        button_frame111 = tk.Frame(hidden_window, bg="white")
        button_frame111.pack(fill="x", pady=5)

        # 版本
        vervion_btn = tk.Button(button_frame111, text="当前版本", width=15, command=self.get_verion,
                                bg="#2196F3",
                                fg="#fffef9",
                                font=("Arial", 10, "bold"))
        vervion_btn.pack(side="left", padx=5, pady=5)

        self.entry_var1 = tk.StringVar(value="1003")
        self.vv_entry = tk.Entry(button_frame111, textvariable=self.entry_var1, width=10)
        self.vv_entry.pack(side="right", padx=5, pady=5)

        '''第二行'''
        state_a_frame = tk.Frame(hidden_window, bg="white")
        state_a_frame.pack(fill="x", pady=5)

        # 复位
        reset_a_button = tk.Button(state_a_frame, text="获取机器人参数文件", width=button_w + 10,
                                   command=self.get_ini)
        reset_a_button.pack(side="left", padx=5, pady=5)

        # PVT
        pvt_a_button = tk.Button(state_a_frame, text="更新机器人参数文件", width=button_w + 10,
                                 command=self.update_ini)
        pvt_a_button.pack(side="right", padx=5, pady=5)

        state_a_frame1 = tk.Frame(hidden_window, bg="white")
        state_a_frame1.pack(fill="x", pady=5)

        reset_a_button = tk.Button(state_a_frame1, text="更新系统", width=button_w + 30,
                                   command=self.update_sys, bg="#F6FC39",
                                   fg="#151513",
                                   font=("Arial", 10, "bold"))
        reset_a_button.pack(side="left", padx=5, pady=5)

        state_a_frame2 = tk.Frame(hidden_window, bg="white")
        state_a_frame2.pack(fill="x", pady=5)
        label = tk.Label(state_a_frame2,
                         text='首次用软件更新系统后，方可查看到小版本，否则仅显示大版本1003;\n 后续在该机器上再用软件，可直接查看小版本，再更新需要的版本。\n\n '
                              '参数配置文件robot.ini如果有更新，先获取参数文件，\n在存到本地的文件上面对比修改，再更新机器人参数文件.\n\n '
                              '更新系统选择更新包*.MV_SYS_UPDATE')
        label.pack(padx=5, pady=10)

        '''第二行'''
        # 关闭按钮
        close_btn = tk.Button(hidden_window, text="关闭",
                              command=hidden_window.destroy,
                              bg="orange", width=15)
        close_btn.pack(pady=5)

    def get_verion(self):
        if self.connected:
            # robot.receive_file('version.txt', "/home/fusion/version.txt")
            # time.sleep(0.5)
            # with open('version.txt', 'r') as f:
            #     version = f.readline()
            # f.close()
            re_flag, version = robot.get_param('int', 'VERSION')
            self.vv_entry.delete(0, tk.END)
            self.vv_entry.insert(0, version)
        else:
            messagebox.showerror('error', '请先连接机器人')

    def on_close(self):
        """关闭窗口时清理资源"""
        if messagebox.askokcancel("退出", "确定要退出应用程序吗?"):
            '''save tools txt'''
            robot.send_file(self.tools_txt, os.path.join('/home/fusion/', self.tools_txt))
            time.sleep(0.2)
            if os.path.exists(self.tools_txt):
                os.remove(self.tools_txt)
            if os.path.exists('version.txt'):
                os.remove('version.txt')
            if self.data_subscriber:
                self.data_subscriber.stop()
            self.root.destroy()
            robot.release_robot()

    def get_ini(self):
        if self.connected:
            file_path = filedialog.asksaveasfilename(
                defaultextension=".ini",
                filetypes=[("ini files", "*.ini"), ("All files", "*.*")],
                title="保存机器人配置参数文件"
            )

            if file_path:
                tag = robot.receive_file(file_path, "/home/FUSION/Config/cfg/robot.ini")
                # tag = robot.receive_file(file_path, "/home/fusion/1.txt")
                time.sleep(1)
                if tag:
                    messagebox.showinfo('success', '参数已保存')
        else:
            messagebox.showerror('error', '请先连接机器人')

    def update_ini(self):
        if self.connected:
            file_path = filedialog.askopenfilename(
                defaultextension=".ini",
                filetypes=[("ini files", "*.ini"), ("All files", "*.*")],
                title="选择机器人参数文件"
            )
            if file_path:
                tag = robot.send_file(file_path, "/home/FUSION/Config/cfg/robot.ini")
                # tag = robot.send_file(file_path, "/home/fusion/1.txt")
                time.sleep(1)
                if tag:
                    messagebox.showinfo('success', '参数已保存')
        else:
            messagebox.showerror('error', '请先连接机器人')

    def update_sys(self):
        if self.connected:
            file_path = filedialog.askopenfilename(
                filetypes=[("All files", "*.*")],
                title="选择系统更新文件"
            )
            if file_path:
                tag1 = robot.update_SDK(file_path)
                if tag1:
                    messagebox.showinfo('success', '系统文件已上传，请重启控制器自动更新。')
                # # tag1 = robot.send_file(file_path, "/home/FUSION/Tmp/ctrl_package.tar")# 代码写的是这个名字
                # print(f"file path:{file_path}")
                # a = file_path.split('/')[-1].split('.')[0].split('_')
                # b = a[2] + '-' + a[3]
                # print(b)
                # with open('version.txt', 'w') as f:
                #     f.write(b)
                # f.close()
                #
                # tag = robot.send_file('version.txt', "/home/fusion/version.txt")
                # time.sleep(1)
                # if tag1 and tag:
                #     messagebox.showinfo('success', '系统文件已上传，请重启控制器自动更新。')
                #     os.remove('version.txt')
        else:
            messagebox.showerror('error', '请先连接机器人')

    def create_main_content(self):
        """创建主内容区域 - 居中布局"""
        # 创建容器框架，用于居中内容
        center_container = tk.Frame(self.root, bg="#f0f0f0")
        center_container.pack(fill="both", expand=True, padx=10, pady=10)

        # 创建带滚动区域的画布
        self.canvas = tk.Canvas(center_container, bg="white", highlightthickness=0)
        self.scrollbar = ttk.Scrollbar(center_container, orient="vertical", command=self.canvas.yview)

        # 可滚动的框架
        self.scrollable_frame = tk.Frame(self.canvas, bg="white")

        # 绑定滚动区域
        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        )

        self.canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)

        # 布局 - 居中
        self.canvas.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")

        # 鼠标滚轮支持
        self.canvas.bind_all("<MouseWheel>", self.on_mousewheel)

        # 创建容器框架用于居中内容
        content_container = tk.Frame(self.scrollable_frame, bg="white")
        content_container.pack(expand=True, fill="both", padx=20)

        state_a_frame = tk.Frame(content_container, bg="white")
        state_a_frame.pack(fill="x", pady=10)

        # # 添加列权重使组件扩展
        # for i in range(7):
        #     state_a_frame.columnconfigure(i, weight=1)
        '''###### 基础功能 ######'''
        # 0复位 1PVT 2关节跟随 3关节阻抗 4笛卡尔阻抗 5力控阻抗
        # 0拖动  1关节拖动 2X拖动 3Y拖动 4Z拖动 5旋转拖动 6退出拖动 7 保存拖动数据
        # 0状态 1  2错误码 3  4 错误码说明 5清错 6

        a_label = tk.Label(state_a_frame, text="#1", width=10, bg="#2196F3",
                           fg="white", font=("Arial", 10, "bold"))
        a_label.grid(row=0, column=0, padx=5, pady=5, sticky="ew")

        # 复位
        reset_a_button = tk.Button(state_a_frame, text="复位", width=button_w,
                                   command=lambda: self.reset_robot('A'))
        reset_a_button.grid(row=0, column=1, padx=5, pady=5)

        # PVT
        pvt_a_button = tk.Button(state_a_frame, text="PVT", width=button_w,
                                 command=lambda: self.pvt_mode('A'))
        pvt_a_button.grid(row=0, column=2, padx=5, pady=5)

        # 关节跟随
        pos_a_button = tk.Button(state_a_frame, text="关节跟随", width=button_w,
                                 command=lambda: self.position_mode('A'))
        pos_a_button.grid(row=0, column=3, padx=5, pady=5)

        # 关节阻抗
        imped_j_a_button = tk.Button(state_a_frame, text="关节阻抗", width=button_w,
                                     command=lambda: self.imded_j_mode('A'))
        imped_j_a_button.grid(row=0, column=4, padx=5, pady=5)

        # 笛卡尔阻抗
        imped_c_a_button = tk.Button(state_a_frame, text="笛卡尔阻抗", width=button_w,
                                     command=lambda: self.imded_c_mode('A'))
        imped_c_a_button.grid(row=0, column=5, padx=5, pady=5)

        b_label_ = tk.Label(state_a_frame, text="", width=3, bg="white")
        b_label_.grid(row=0, column=6, padx=5, pady=5, sticky="ew")

        # 力控
        f_a_button = tk.Button(state_a_frame, text="力控", width=button_w,
                               command=lambda: self.imded_f_mode('A'))
        f_a_button.grid(row=0, column=7, padx=5, pady=5)

        f_label = tk.Label(state_a_frame, text="力N", width=3, bg="white")
        f_label.grid(row=0, column=8, padx=3, pady=5)

        self.f_a_entry = tk.Entry(state_a_frame, width=3)
        self.f_a_entry.insert(0, "0")
        self.f_a_entry.grid(row=0, column=9, padx=3, pady=5)

        f_adj_label = tk.Label(state_a_frame, text="调节量mm", width=8, bg="white")
        f_adj_label.grid(row=0, column=10, padx=3, pady=5)

        self.f_a_adj_entry = tk.Entry(state_a_frame, width=3)
        self.f_a_adj_entry.insert(0, "0")
        self.f_a_adj_entry.grid(row=0, column=11, padx=3, pady=5)

        # 下拉框（XYZ）
        self.direction_label = tk.Label(state_a_frame, text="方向", bg="white")
        self.direction_label.grid(row=0, column=12, padx=3, pady=5)
        self.axis_combobox_a = ttk.Combobox(
            state_a_frame,
            values=["X", "Y", "Z"],
            width=3,
            state="readonly"  # 禁止直接输入
        )
        self.axis_combobox_a.current(0)  # 默认选中第一个选项（X）
        self.axis_combobox_a.grid(row=0, column=13, padx=3, pady=5)

        # 0拖动  1关节拖动 2X拖动 3Y拖动 4Z拖动 5旋转拖动 6退出拖动
        row1_label = tk.Label(state_a_frame, text=" ", width=10, bg='white')
        row1_label.grid(row=1, column=0, padx=5, pady=5, sticky="ew")

        # 关节拖动
        drag_j_a_button = tk.Button(state_a_frame, text="关节拖动", width=button_w,
                                    command=lambda: self.drag_j('A'))
        drag_j_a_button.grid(row=1, column=1, padx=5, pady=5)

        # X拖动
        drag_x_a_button = tk.Button(state_a_frame, text="X拖动", width=button_w,
                                    command=lambda: self.drag_x('A'))
        drag_x_a_button.grid(row=1, column=2, padx=5, pady=5)

        # Y拖动
        drag_y_a_button = tk.Button(state_a_frame, text="Y拖动", width=button_w,
                                    command=lambda: self.drag_y('A'))
        drag_y_a_button.grid(row=1, column=3, padx=5, pady=5)

        # Z拖动
        drag_z_a_button = tk.Button(state_a_frame, text="Z拖动", width=button_w,
                                    command=lambda: self.drag_z('A'))
        drag_z_a_button.grid(row=1, column=4, padx=5, pady=5)

        # R拖动
        drag_r_a_button = tk.Button(state_a_frame, text="R拖动", width=button_w,
                                    command=lambda: self.drag_r('A'))
        drag_r_a_button.grid(row=1, column=5, padx=5, pady=5)

        # 退出拖动
        drag_exit_a_button = tk.Button(state_a_frame, text="退出拖动", width=button_w,
                                       command=lambda: self.drag_exit('A'))
        drag_exit_a_button.grid(row=1, column=6, padx=5, pady=5)

        # 拖动保存数据
        drag_save_a_button = tk.Button(state_a_frame, text="拖动数据保存", width=button_w,
                                       command=lambda: self.thread_drag_save('A'))
        drag_save_a_button.grid(row=1, column=7, padx=5, pady=5)

        # 0blank  1pvt运行 2选择PVT号 3PVT id 4上传PVT 5运行PVT
        row2_label = tk.Label(state_a_frame, text=" ", width=10, bg='white')
        row2_label.grid(row=2, column=0, padx=5, sticky="ew")
        # 1pvt运行
        row2_text_label = tk.Label(state_a_frame, text="PVT运行", width=10, bg='#d9d6c3')
        row2_text_label.grid(row=2, column=1, padx=5, sticky="ew")
        # 2选择PVT号
        pvt_a_text_label = tk.Label(state_a_frame, text="选择PVT号1~99", width=10, bg='white')
        pvt_a_text_label.grid(row=2, column=2, padx=5, sticky="ew")
        # 3PVT id
        self.pvt_a_entry = tk.Entry(state_a_frame, width=10)
        self.pvt_a_entry.insert(0, "1")
        self.pvt_a_entry.grid(row=2, column=3, padx=5)
        # 4上传PVT
        send_pvt_a_button = tk.Button(state_a_frame, text="上传PVT", width=button_w,
                                      command=lambda: self.send_pvt('A'))
        send_pvt_a_button.grid(row=2, column=4, padx=5)

        # 5运行PVT
        run_pvt_a_button = tk.Button(state_a_frame, text="运行PVT", width=button_w,
                                     command=lambda: self.run_pvt('A'))
        run_pvt_a_button.grid(row=2, column=5, padx=5)

        # row 4
        row3_label = tk.Label(state_a_frame, text=" ", width=10, bg='white')
        row3_label.grid(row=2, column=6, padx=5, )

        # 0状态 1  2错误码 3  4 错误码说明 5清错 6
        # 获取错误码
        error_a_button = tk.Button(state_a_frame, text="获取错误码", width=button_w,
                                   command=lambda: self.error_get('A'))
        error_a_button.grid(row=2, column=7, padx=5, pady=5)

        # 清错
        clear_error_a_button = tk.Button(state_a_frame, text="清错", width=button_w,
                                         command=lambda: self.error_clear('A'))
        clear_error_a_button.grid(row=2, column=8, padx=5, pady=5)

        brak_a_button = tk.Button(state_a_frame, text="强制抱闸", width=button_w,
                                  command=lambda: self.brake('A'))
        brak_a_button.grid(row=2, column=9, padx=5, pady=5)

        release_brak_a_button = tk.Button(state_a_frame, text="强制松闸", width=button_w,
                                          command=lambda: self.release_brake('A'))
        release_brak_a_button.grid(row=2, column=10, padx=5, pady=5)

        # 协作释放
        cr_a_button = tk.Button(state_a_frame, text="协作释放", width=button_w,
                                command=lambda: self.cr_state('A'))
        cr_a_button.grid(row=2, column=11, padx=5, pady=5)

        # 添加更多内容区域
        self.add_more_content(content_container)

        # add parameters settings
        self.add_parameter_settings(content_container)

        # add joints cmd
        self.joints_cmd_settings(content_container)

        # add  data collect content
        self.data_collect_content(content_container)

        # add tool dynamic identy
        self.tool_identy_content(content_container)

        # add sensor(torque) rectify function
        self.sensor_rectify_content(content_container)

        # add motor cler as zero and clear error
        self.motor_content(content_container)

        # add 485
        self.eef_content(content_container)

    def add_more_content(self, parent):
        """添加更多内容到主区域"""
        # 添加第二个设备控制区域
        state_b_frame = tk.Frame(parent, bg="white")
        state_b_frame.pack(fill="x", pady=5)

        # # 添加列权重
        # for i in range(7):
        #     state_b_frame.columnconfigure(i, weight=1)

        b_label = tk.Label(state_b_frame, text="#2", width=10, bg="#2196F3",
                           fg="white", font=("Arial", 10, "bold"))
        b_label.grid(row=0, column=0, padx=5, pady=5, sticky="ew")

        # 复位
        reset_b_button = tk.Button(state_b_frame, text="复位", width=10,
                                   command=lambda: self.reset_robot('B'))
        reset_b_button.grid(row=0, column=1, padx=5, pady=5)

        # PVT
        pvt_b_button = tk.Button(state_b_frame, text="PVT", width=10,
                                 command=lambda: self.pvt_mode('B'))
        pvt_b_button.grid(row=0, column=2, padx=5, pady=5)

        # 关节跟随
        pos_b_button = tk.Button(state_b_frame, text="关节跟随", width=10,
                                 command=lambda: self.position_mode('B'))
        pos_b_button.grid(row=0, column=3, padx=5, pady=5)

        # 关节阻抗
        imped_j_b_button = tk.Button(state_b_frame, text="关节阻抗", width=10,
                                     command=lambda: self.imded_j_mode('B'))
        imped_j_b_button.grid(row=0, column=4, padx=5, pady=5)

        # 笛卡尔阻抗
        imped_c_b_button = tk.Button(state_b_frame, text="笛卡尔阻抗", width=10,
                                     command=lambda: self.imded_c_mode('B'))
        imped_c_b_button.grid(row=0, column=5, padx=5, pady=5)

        b_label_ = tk.Label(state_b_frame, text="", width=3, bg="white")
        b_label_.grid(row=0, column=6, padx=5, pady=5, sticky="ew")

        # 力控
        f_b_button = tk.Button(state_b_frame, text="力控", width=10,
                               command=lambda: self.imded_f_mode('B'))
        f_b_button.grid(row=0, column=7, padx=5, pady=5)

        f_label = tk.Label(state_b_frame, text="力N", width=3, bg="white")
        f_label.grid(row=0, column=8, padx=3, pady=5)

        self.f_b_entry = tk.Entry(state_b_frame, width=3)
        self.f_b_entry.insert(0, "0")
        self.f_b_entry.grid(row=0, column=9, padx=3, pady=5)

        f_adj_b_label = tk.Label(state_b_frame, text="调节量mm", width=8, bg="white")
        f_adj_b_label.grid(row=0, column=10, padx=3, pady=5)

        self.f_b_adj_entry = tk.Entry(state_b_frame, width=3)
        self.f_b_adj_entry.insert(0, "0")
        self.f_b_adj_entry.grid(row=0, column=11, padx=3, pady=5)

        # 下拉框（XYZ）
        self.direction_label = tk.Label(state_b_frame, text="方向", bg="white")
        self.direction_label.grid(row=0, column=12, padx=3, pady=5)
        self.axis_combobox_b = ttk.Combobox(
            state_b_frame,
            values=["X", "Y", "Z"],
            width=3,
            state="readonly"  # 禁止直接输入
        )
        self.axis_combobox_b.current(0)  # 默认选中第一个选项（X）
        self.axis_combobox_b.grid(row=0, column=14, padx=3, pady=5)

        # 0拖动  1关节拖动 2X拖动 3Y拖动 4Z拖动 5旋转拖动 6退出拖动
        row1_label_b = tk.Label(state_b_frame, text=" ", width=10, bg='white')
        row1_label_b.grid(row=1, column=0, padx=5, pady=5, sticky="ew")

        # 关节拖动
        drag_j_b_button = tk.Button(state_b_frame, text="关节拖动", width=button_w,
                                    command=lambda: self.drag_j('B'))
        drag_j_b_button.grid(row=1, column=1, padx=5, pady=5)

        # X拖动
        drag_x_b_button = tk.Button(state_b_frame, text="X拖动", width=button_w,
                                    command=lambda: self.drag_x('B'))
        drag_x_b_button.grid(row=1, column=2, padx=5, pady=5)

        # Y拖动
        drag_y_b_button = tk.Button(state_b_frame, text="Y拖动", width=button_w,
                                    command=lambda: self.drag_y('B'))
        drag_y_b_button.grid(row=1, column=3, padx=5, pady=5)

        # Z拖动
        drag_z_b_button = tk.Button(state_b_frame, text="Z拖动", width=button_w,
                                    command=lambda: self.drag_z('B'))
        drag_z_b_button.grid(row=1, column=4, padx=5, pady=5)

        # R拖动
        drag_r_b_button = tk.Button(state_b_frame, text="R拖动", width=button_w,
                                    command=lambda: self.drag_r('B'))
        drag_r_b_button.grid(row=1, column=5, padx=5, pady=5)

        # 退出拖动
        drag_exit_b_button = tk.Button(state_b_frame, text="退出拖动", width=button_w,
                                       command=lambda: self.drag_exit('B'))
        drag_exit_b_button.grid(row=1, column=6, padx=5, pady=5)

        # 拖动保存数据
        drag_save_b_button = tk.Button(state_b_frame, text="拖动数据保存", width=button_w,
                                       command=lambda: self.thread_drag_save('B'))
        drag_save_b_button.grid(row=1, column=7, padx=5, pady=5)

        # 0blank  1pvt运行 2选择PVT号 3PVT id 4上传PVT 5运行PVT
        row2_label_ = tk.Label(state_b_frame, text=" ", width=10, bg='white')
        row2_label_.grid(row=2, column=0, padx=5, sticky="ew")
        # 1pvt运行
        row2_text_label_ = tk.Label(state_b_frame, text="PVT运行", width=10, bg='#d9d6c3')
        row2_text_label_.grid(row=2, column=1, padx=5, sticky="ew")
        # 2选择PVT号
        pvt_b_text_label = tk.Label(state_b_frame, text="选择PVT号1~99", width=10, bg='white')
        pvt_b_text_label.grid(row=2, column=2, padx=5, sticky="ew")
        # 3PVT id
        self.pvt_b_entry = tk.Entry(state_b_frame, width=10)
        self.pvt_b_entry.insert(0, "1")
        self.pvt_b_entry.grid(row=2, column=3, padx=5)
        # 4上传PVT
        send_pvt_b_button = tk.Button(state_b_frame, text="上传PVT", width=button_w,
                                      command=lambda: self.send_pvt('B'))
        send_pvt_b_button.grid(row=2, column=4, padx=5)

        # 5运行PVT
        run_pvt_b_button = tk.Button(state_b_frame, text="运行PVT", width=button_w,
                                     command=lambda: self.run_pvt('B'))
        run_pvt_b_button.grid(row=2, column=5, padx=5)

        # row 4
        row3_label_ = tk.Label(state_b_frame, text=" ", width=10, bg='white')
        row3_label_.grid(row=2, column=6, padx=5, )
        # 获取错误码
        error_b_button = tk.Button(state_b_frame, text="获取错误码", width=button_w,
                                   command=lambda: self.error_get('B'))
        error_b_button.grid(row=2, column=7, padx=5, pady=5)
        # 清错
        clear_error_b_button = tk.Button(state_b_frame, text="清错", width=button_w,
                                         command=lambda: self.error_clear('B'))
        clear_error_b_button.grid(row=2, column=8, padx=5, pady=5)

        brak_b_button = tk.Button(state_b_frame, text="强制抱闸", width=button_w,
                                  command=lambda: self.brake('B'))
        brak_b_button.grid(row=2, column=9, padx=5, pady=5)

        release_brak_b_button = tk.Button(state_b_frame, text="强制松闸", width=button_w,
                                          command=lambda: self.release_brake('B'))
        release_brak_b_button.grid(row=2, column=10, padx=5, pady=5)
        # 协作释放
        cr_b_button = tk.Button(state_b_frame, text="协作释放", width=button_w,
                                command=lambda: self.cr_state('B'))
        cr_b_button.grid(row=2, column=11, padx=5, pady=5)

        # 添加横线
        horizontal_line1 = tk.Frame(parent, height=2, bg="#%02x%02x%02x" % (50, 150, 200))
        horizontal_line1.pack(fill="x", expand=True)

        # 添加状态显示区域
        status_display_frame0 = tk.Frame(parent, bg="white", padx=10, pady=5)
        status_display_frame0.pack(fill="x", pady=5)

    def add_parameter_settings(self, parent):
        setting_frame = tk.Frame(parent, bg="white")
        setting_frame.pack(fill="x")
        setting_frame_1 = tk.Frame(parent, bg="white")
        setting_frame_1.pack(fill="x")

        # 0#1/2  1设置工具参数M~I_zz   2 entry  3 设置速度和加速度 4speed entry  5acc entry
        a_label = tk.Label(setting_frame, text="#1", width=10, bg="#2196F3",
                           fg="white", font=("Arial", 10, "bold"))
        a_label.grid(row=0, column=0, padx=5, pady=3)

        # 1设置工具参数
        tool_a_button = tk.Button(setting_frame, text="设置工具参数", width=10, command=lambda: self.tool_set('A'))
        tool_a_button.grid(row=0, column=1, padx=5)

        tool_a_label_1 = tk.Label(setting_frame, text="设置工具动力学参数(M~I_zz)", width=25, bg='white')
        tool_a_label_1.grid(row=0, column=2, padx=5)

        # 2tool entry
        self.tool_a_entry = tk.Entry(setting_frame, width=50)
        self.tool_a_entry.insert(0, "[0,0,0,0,0,0,0,0,0,0]")
        self.tool_a_entry.grid(row=0, column=3, padx=5, sticky="ew")

        # 1设置工具运动学参数
        tool_a_label_2 = tk.Label(setting_frame, text="设置工具运动学参数", width=20, bg='white')
        tool_a_label_2.grid(row=0, column=4)

        # 2tool entry
        self.tool_a1_entry = tk.Entry(setting_frame, width=30)
        self.tool_a1_entry.insert(0, "[0,0,0,0,0,0]")
        self.tool_a1_entry.grid(row=0, column=5, padx=5)

        # row 1 0保存参数   1设置关节阻抗参数   2 K  3 K entry  4 D  5 D entry
        # SAVE PARA
        save_param_a_button = tk.Button(setting_frame_1, text="保存参数", width=6, command=lambda: self.save_param('A'))
        save_param_a_button.grid(row=0, column=0, padx=5, pady=3)

        # set joint kd
        joint_kd_a_button = tk.Button(setting_frame_1, text="设置关节阻抗参数", width=20,
                                      command=lambda: self.joint_kd_set('A'))
        joint_kd_a_button.grid(row=0, column=1, padx=5)

        # k laebl
        k_a_label = tk.Label(setting_frame_1, text='K', width=5, bg="white")
        k_a_label.grid(row=0, column=2)

        # k entry
        self.k_a_entry = tk.Entry(setting_frame_1, width=50)
        self.k_a_entry.insert(0, "[2,2,2,1.6,1,1,1]")
        self.k_a_entry.grid(row=0, column=3, sticky="ew")

        # d laebl
        d_a_label = tk.Label(setting_frame_1, text='D', width=5, bg="white")
        d_a_label.grid(row=0, column=4)

        # d entry
        self.d_a_entry = tk.Entry(setting_frame_1, width=30)
        self.d_a_entry.insert(0, "[0.4,0.4,0.4,0.4,0.4,0.4,0.4]")
        self.d_a_entry.grid(row=0, column=5, )

        # 3 spped
        vel_a_button = tk.Button(setting_frame_1, text="设置速度和加速度(百分比)", width=20,
                                 command=lambda: self.vel_acc_set('A'))
        vel_a_button.grid(row=0, column=6)

        # 4 vel entry
        self.vel_a_entry = tk.Entry(setting_frame_1, width=3)
        self.vel_a_entry.insert(0, "10")
        self.vel_a_entry.grid(row=0, column=7)

        # 5 acc entry
        self.acc_a_entry = tk.Entry(setting_frame_1, width=3)
        self.acc_a_entry.insert(0, "10")
        self.acc_a_entry.grid(row=0, column=8)

        # row 2  0导入参数   1设置笛卡尔阻抗参数   2 K  3 K entry  4 D  5 D entry
        # SAVE PARA
        load_param_a_button = tk.Button(setting_frame_1, text="导入参数", width=6, command=lambda: self.load_param('A'))
        load_param_a_button.grid(row=1, column=0, padx=5, pady=3)

        # set joint kd
        cart_kd_a_button = tk.Button(setting_frame_1, text="设置笛卡尔阻抗参数", width=20,
                                     command=lambda: self.cart_kd_set('A'))
        cart_kd_a_button.grid(row=1, column=1, padx=5)

        # k laebl
        k_a_label_ = tk.Label(setting_frame_1, text='K', width=5, bg="white")
        k_a_label_.grid(row=1, column=2)

        # k entry
        self.cart_k_a_entry = tk.Entry(setting_frame_1, width=50)
        self.cart_k_a_entry.insert(0, "[2000,2000,2000,60,60,60,20]")
        self.cart_k_a_entry.grid(row=1, column=3, sticky="ew")

        # d laebl
        d_a_label_ = tk.Label(setting_frame_1, text='D', width=5, bg="white")
        d_a_label_.grid(row=1, column=4)

        # d entry
        self.cart_d_a_entry = tk.Entry(setting_frame_1, width=30)
        self.cart_d_a_entry.insert(0, "[0.4,0.4,0.4,0.4,0.4,0.4,0.4]")
        self.cart_d_a_entry.grid(row=1, column=5)

        # 阻抗类型
        type_a_label = tk.Label(setting_frame_1, text='阻抗类型:1关节 2笛卡尔 3力控', width=30, bg="white")
        type_a_label.grid(row=1, column=6)

        # impedance entry
        self.imped_a_entry = tk.Entry(setting_frame_1, width=5)
        self.imped_a_entry.insert(0, "2")
        self.imped_a_entry.grid(row=1, column=7)

        blank_a_label = tk.Label(setting_frame_1, text='', width=30, bg="white")
        blank_a_label.grid(row=2, column=6)

        '''#2'''
        setting_frame_ = tk.Frame(parent, bg="white")
        setting_frame_.pack(fill="x")
        setting_frame_11 = tk.Frame(parent, bg="white")
        setting_frame_11.pack(fill="x")
        # 0#1/2  1设置工具参数M~I_zz   2 entry  3 设置速度和加速度 4speed entry  5acc entry
        b_label = tk.Label(setting_frame_, text="#2", width=10, bg="#2196F3",
                           fg="white", font=("Arial", 10, "bold"))
        b_label.grid(row=0, column=0, padx=5, pady=3)

        # 1设置工具参数
        tool_b_button = tk.Button(setting_frame_, text="设置工具参数", width=10, command=lambda: self.tool_set('B'))
        tool_b_button.grid(row=0, column=1, padx=5)

        tool_b_label_1 = tk.Label(setting_frame_, text="设置工具动力学参数(M~I_zz)", width=25, bg='white')
        tool_b_label_1.grid(row=0, column=2, padx=5)

        # 2tool entry
        self.tool_b_entry = tk.Entry(setting_frame_, width=50)
        self.tool_b_entry.insert(0, "[0,0,0,0,0,0,0,0,0,0]")
        self.tool_b_entry.grid(row=0, column=3, padx=5, sticky="ew")

        # 1设置工具运动学参数
        tool_b_label_2 = tk.Label(setting_frame_, text="设置工具运动学参数", width=20, bg='white')
        tool_b_label_2.grid(row=0, column=4)

        # 2tool entry
        self.tool_b1_entry = tk.Entry(setting_frame_, width=30)
        self.tool_b1_entry.insert(0, "[0,0,0,0,0,0]")
        self.tool_b1_entry.grid(row=0, column=5, padx=5)

        # row 1 0保存参数   1设置关节阻抗参数   2 K  3 K entry  4 D  5 D entry
        # SAVE PARA
        save_param_b_button = tk.Button(setting_frame_11, text="保存参数", width=6,
                                        command=lambda: self.save_param('B'))  # todo, command=self.save_param
        save_param_b_button.grid(row=0, column=0, padx=5, pady=3)

        # set joint kd
        joint_kd_b_button = tk.Button(setting_frame_11, text="设置关节阻抗参数", width=20,
                                      command=lambda: self.joint_kd_set('B'))
        joint_kd_b_button.grid(row=0, column=1, padx=5)

        # k laebl
        k_b_label = tk.Label(setting_frame_11, text='K', width=5, bg="white")
        k_b_label.grid(row=0, column=2)

        # k entry
        self.k_b_entry = tk.Entry(setting_frame_11, width=50)
        self.k_b_entry.insert(0, "[2,2,2,1.6,1,1,1]")
        self.k_b_entry.grid(row=0, column=3, sticky="ew")

        # d laebl
        d_b_label = tk.Label(setting_frame_11, text='D', width=5, bg="white")
        d_b_label.grid(row=0, column=4)

        # d entry
        self.d_b_entry = tk.Entry(setting_frame_11, width=30)
        self.d_b_entry.insert(0, "[0.4,0.4,0.4,0.4,0.4,0.4,0.4]")
        self.d_b_entry.grid(row=0, column=5, )

        # 3 spped
        vel_b_button = tk.Button(setting_frame_11, text="设置速度和加速度(百分比)", width=20,
                                 command=lambda: self.vel_acc_set('B'))
        vel_b_button.grid(row=0, column=6)

        # 4 vel entry
        self.vel_b_entry = tk.Entry(setting_frame_11, width=3)
        self.vel_b_entry.insert(0, "10")
        self.vel_b_entry.grid(row=0, column=7)

        # 5 acc entry
        self.acc_b_entry = tk.Entry(setting_frame_11, width=3)
        self.acc_b_entry.insert(0, "10")
        self.acc_b_entry.grid(row=0, column=8)

        # row 2  0导入参数   1设置笛卡尔阻抗参数   2 K  3 K entry  4 D  5 D entry
        # SAVE PARA
        load_param_b_button = tk.Button(setting_frame_11, text="导入参数", width=6,
                                        command=lambda: self.load_param('B'))
        load_param_b_button.grid(row=1, column=0, padx=5, pady=3)

        # set joint kd
        cart_kd_b_button = tk.Button(setting_frame_11, text="设置笛卡尔阻抗参数", width=20,
                                     command=lambda: self.cart_kd_set('B'))
        cart_kd_b_button.grid(row=1, column=1, padx=5)

        # k laebl
        k_b_label_ = tk.Label(setting_frame_11, text='K', width=5, bg="white")
        k_b_label_.grid(row=1, column=2)

        # k entry
        self.cart_k_b_entry = tk.Entry(setting_frame_11, width=50)
        self.cart_k_b_entry.insert(0, "[2000,2000,2000,60,60,60,20]")
        self.cart_k_b_entry.grid(row=1, column=3, sticky="ew")

        # d laebl
        d_b_label_ = tk.Label(setting_frame_11, text='D', width=5, bg="white")
        d_b_label_.grid(row=1, column=4)

        # d entry
        self.cart_d_b_entry = tk.Entry(setting_frame_11, width=30)
        self.cart_d_b_entry.insert(0, "[0.4,0.4,0.4,0.4,0.4,0.4,0.4]")
        self.cart_d_b_entry.grid(row=1, column=5)

        # 阻抗类型
        type_b_label = tk.Label(setting_frame_11, text='阻抗类型:1关节 2笛卡尔 3力控', width=30, bg="white")
        type_b_label.grid(row=1, column=6)

        # impedance entry
        self.imped_b_entry = tk.Entry(setting_frame_11, width=5)
        self.imped_b_entry.insert(0, "2")
        self.imped_b_entry.grid(row=1, column=7)

        # 添加横线
        horizontal_line2 = tk.Frame(parent, height=2, bg="#%02x%02x%02x" % (50, 150, 200))
        horizontal_line2.pack(fill="x", expand=True)

        # 添加状态显示区域
        status_display_frame_1 = tk.Frame(parent, bg="white", padx=10, pady=5)
        status_display_frame_1.pack(fill="x", pady=5)

    def joints_cmd_settings(self, parent):
        self.frame1 = tk.Frame(parent, bg="white")
        self.frame1.pack(fill="x")
        # 第一列：1#加点按钮
        self.btn_add1 = tk.Button(self.frame1, text="1#加点", command=self.add_point1)
        self.btn_add1.grid(row=0, column=0, padx=5)

        # 第二列：输入文本框
        self.entry_var = tk.StringVar(value="[0,0,0,0,0,0,0]")
        self.entry = tk.Entry(self.frame1, textvariable=self.entry_var, width=60)
        self.entry.grid(row=0, column=1, padx=5, sticky="ew")

        # 第三列：2#加点按钮
        self.btn_add2 = tk.Button(self.frame1, text="2#加点", command=self.add_point2)
        self.btn_add2.grid(row=0, column=2, padx=5)

        # 第四列：1#
        self.btn_add3 = tk.Button(self.frame1, text="1#获取当前关节数据", command=lambda: self.add_current_joints('A'))
        self.btn_add3.grid(row=0, column=3, padx=5)

        # 第五列：2#
        self.btn_add4 = tk.Button(self.frame1, text="2#获取当前关节数据", command=lambda: self.add_current_joints('B'))
        self.btn_add4.grid(row=0, column=4, padx=5)

        self.frame2 = tk.Frame(parent, bg="white")
        self.frame2.pack(fill="x")

        # 第一列：1#删除点按钮
        self.btn_del1 = tk.Button(self.frame2, text="1#删除点", command=self.delete_point1)
        self.btn_del1.grid(row=0, column=1, padx=5)

        # 第二列：1#下拉文本框
        self.combo1 = ttk.Combobox(self.frame2, state="readonly", width=50)
        self.combo1.grid(row=0, column=2, padx=5)

        # 第三列：1#运行按钮
        self.btn_run1 = tk.Button(self.frame2, text="1#运行", command=self.run1)
        self.btn_run1.grid(row=0, column=3, padx=5)

        # 第四列：1#保存按钮
        self.btn_save1 = tk.Button(self.frame2, text="1#保存", command=self.save_points1)
        self.btn_save1.grid(row=0, column=4, padx=5)

        # 第五列：1#导入按钮
        self.btn_load1 = tk.Button(self.frame2, text="1#导入", command=self.load_points1)
        self.btn_load1.grid(row=0, column=5, padx=5)

        text_blank = tk.Label(self.frame2, text='', width=2, bg='white')
        text_blank.grid(row=0, column=6, padx=5)

        self.text_1_load_file = tk.Label(self.frame2, text='周期运行', bg='#afdfe4')
        self.text_1_load_file.grid(row=0, column=7, padx=3)

        self.btn_load_file1 = tk.Button(self.frame2, text="1#选择文件", command=lambda: self.select_period_file('A'))
        self.btn_load_file1.grid(row=0, column=8, padx=5)

        self.period_path_entry_1 = tk.Entry(self.frame2, textvariable=self.period_file_path_1, width=45,
                                            font=("Arial", 7), state="readonly")
        self.period_path_entry_1.grid(row=0, column=9, padx=5, sticky="ew")

        self.run_period_1 = tk.Button(self.frame2, text="1#运行", command=lambda: self.run_period_file('A'))
        self.run_period_1.grid(row=0, column=10, padx=5)

        self.frame3 = tk.Frame(parent, bg="white")
        self.frame3.pack(fill="x")

        # 第四列：2#删除点按钮
        self.btn_del2 = tk.Button(self.frame3, text="2#删除点", command=self.delete_point2)
        self.btn_del2.grid(row=0, column=0, padx=5, pady=3)

        # 第五列：2#下拉文本框
        self.combo2 = ttk.Combobox(self.frame3, state="readonly", width=50)
        self.combo2.grid(row=0, column=1, padx=5)

        # 第六列：2#运行按钮
        self.btn_run2 = tk.Button(self.frame3, text="2#运行", command=self.run2)
        self.btn_run2.grid(row=0, column=2, padx=5)

        self.btn_save2 = tk.Button(self.frame3, text="2#保存", command=self.save_points2)
        self.btn_save2.grid(row=0, column=3, padx=5)

        # 第五列：2#导入按钮
        self.btn_load2 = tk.Button(self.frame3, text="2#导入", command=self.load_points2)
        self.btn_load2.grid(row=0, column=4, padx=5)

        text_blank_ = tk.Label(self.frame3, text='', width=2, bg='white')
        text_blank_.grid(row=0, column=6, padx=5)

        self.text_2_load_file = tk.Label(self.frame3, text='周期运行', bg='#afdfe4')
        self.text_2_load_file.grid(row=0, column=7, padx=3)

        self.btn_load_file2 = tk.Button(self.frame3, text="1#选择文件", command=lambda: self.select_period_file('B'))
        self.btn_load_file2.grid(row=0, column=8, padx=5)

        self.period_path_entry_2 = tk.Entry(self.frame3, textvariable=self.period_file_path_2, width=45,
                                            font=("Arial", 7), state="readonly")
        self.period_path_entry_2.grid(row=0, column=9, padx=5, sticky="ew")

        self.run_period_2 = tk.Button(self.frame3, text="1#运行", command=lambda: self.run_period_file('B'))
        self.run_period_2.grid(row=0, column=10, padx=5)

        # 初始化下拉框
        self.update_comboboxes()

        # 添加横线
        horizontal_line3 = tk.Frame(parent, height=2, bg="#%02x%02x%02x" % (50, 150, 200))
        horizontal_line3.pack(fill="x", expand=True)

        # 添加状态显示区域
        status_display_frame_2 = tk.Frame(parent, bg="white", padx=10, pady=5)
        status_display_frame_2.pack(fill="x", pady=5)

    def tool_identy_content(self, parent):
        self.identy_tool_frame = tk.Frame(parent, bg="white")
        self.identy_tool_frame.pack(fill="x")

        self.robot_type_choose = tk.Label(self.identy_tool_frame, text="工具动力学辨识", width=15, bg="#9b95c9",
                                          fg="white", font=("Arial", 10, "bold"))
        self.robot_type_choose.grid(row=0, column=0, padx=5)

        self.robot_type_choose = tk.Label(self.identy_tool_frame, text="选择机型", bg='white', width=8)
        self.robot_type_choose.grid(row=0, column=1, padx=5)

        # robot select
        self.type_select_combobox_1 = ttk.Combobox(
            self.identy_tool_frame,
            values=["CCS", "SRS"],
            width=5,
            state="readonly"  # 禁止直接输入
        )
        self.type_select_combobox_1.current(0)  # 默认选中第一个选项
        self.type_select_combobox_1.grid(row=0, column=2, padx=5)

        # choose file
        self.tool_trajectory_file = tk.Button(self.identy_tool_frame, text="选择轨迹文件",
                                              command=self.tool_trajectory)
        self.tool_trajectory_file.grid(row=0, column=3, padx=5)

        # file visual
        self.path_tool = tk.Entry(self.identy_tool_frame, textvariable=self.file_path_tool, width=100,
                                  font=("Arial", 8), state="readonly")
        self.path_tool.grid(row=0, column=4, padx=5, sticky="ew")

        self.identy_tool_frame2 = tk.Frame(parent, bg="white")
        self.identy_tool_frame2.pack(fill="x")

        self.tool_blank = tk.Label(self.identy_tool_frame2, text=" ", width=15, bg="white")
        self.tool_blank.grid(row=0, column=0, padx=5)
        # left
        self.collect_tool_btn = tk.Button(self.identy_tool_frame2, text="左臂空载数据采集",
                                          command=lambda: self.thread_collect_tool_data_no_load('A'))
        self.collect_tool_btn.grid(row=0, column=1, padx=5)

        self.collect_tool_btn2 = tk.Button(self.identy_tool_frame2, text="左臂带载数据采集",
                                           command=lambda: self.thread_collect_tool_data_with_load('A'))
        self.collect_tool_btn2.grid(row=0, column=2, padx=5)

        self.tool_blank1 = tk.Label(self.identy_tool_frame2, text=" ", width=5, bg="white")
        self.tool_blank1.grid(row=0, column=3, padx=5)

        # 工具辨识
        self.tool_dyn_identy_btn = tk.Button(self.identy_tool_frame2, text="工具动力学辨识", bg='#afb4db',
                                             command=self.tool_dyn_identy)
        self.tool_dyn_identy_btn.grid(row=0, column=4, padx=5)

        self.tool_blank3 = tk.Label(self.identy_tool_frame2, text=" ", width=5, bg="white")
        self.tool_blank3.grid(row=0, column=5, padx=5)
        # right
        self.collect_tool_btn1 = tk.Button(self.identy_tool_frame2, text="右臂空载数据采集",
                                           command=lambda: self.thread_collect_tool_data_no_load('B'))
        self.collect_tool_btn1.grid(row=0, column=6, padx=5)

        self.collect_tool_btn22 = tk.Button(self.identy_tool_frame2, text="右臂带载数据采集",
                                            command=lambda: self.thread_collect_tool_data_with_load('B'))
        self.collect_tool_btn22.grid(row=0, column=7, padx=5)

        self.identy_tool_frame1 = tk.Frame(parent, bg="white")
        self.identy_tool_frame1.pack(fill="x")

        self.tool_blank1 = tk.Label(self.identy_tool_frame1, text=" ", width=5, bg="white")
        self.tool_blank1.grid(row=0, column=0, padx=5)

        self.robot_type_choose1 = tk.Label(self.identy_tool_frame1,
                                           text="工具动力学参数[m,mx,my,mz,ixx,ixy,ixz,iyy,iyz,izz]", bg='white',
                                           width=40)
        self.robot_type_choose1.grid(row=0, column=1, padx=5, pady=5)

        self.entry_tool_dyn = tk.StringVar(
            value="[0,0,0,0,0,0,0,0,0,0]")
        self.tool_dyn_entry = tk.Entry(self.identy_tool_frame1, textvariable=self.entry_tool_dyn, width=100)
        self.tool_dyn_entry.grid(row=0, column=2, padx=5, sticky="ew")

        # 添加横线
        horizontal_line_4 = tk.Frame(parent, height=2, bg="#%02x%02x%02x" % (50, 150, 200))
        horizontal_line_4.pack(fill="x", expand=True)

        # 添加状态显示区域
        status_display_frame_3 = tk.Frame(parent, bg="white", padx=10, pady=5)
        status_display_frame_3.pack(fill="x", pady=5)

    def data_collect_content(self, parent):
        self.frame_data_1 = tk.Frame(parent, bg="white")
        self.frame_data_1.pack(fill="x")
        # 第一列：collect 2 arms' data
        self.collect_both_btn = tk.Button(self.frame_data_1, text="位置同步采集", command=self.collect_data_both)
        self.collect_both_btn.grid(row=0, column=0, padx=5)

        # 第2列：stop collect
        self.stop_collect_both_btn = tk.Button(self.frame_data_1, text="停止", command=self.stop_collect_data_both)
        self.stop_collect_both_btn.grid(row=0, column=1, padx=5)

        # 第3列：save collect
        self.save_collect_both_btn = tk.Button(self.frame_data_1, text="保存", command=self.save_collect_data_both)
        self.save_collect_both_btn.grid(row=0, column=2, padx=5)

        # # 第4列：BLANK
        self.blankkkkkk = tk.Label(self.frame_data_1, text=" ", bg='white', width=5)
        self.blankkkkkk.grid(row=0, column=3, padx=5)

        self.text_50_load_file = tk.Label(self.frame_data_1, text='数据下采样50HZ', bg='#cde6c7')
        self.text_50_load_file.grid(row=0, column=4, padx=3)

        self.btn_load_file_50 = tk.Button(self.frame_data_1, text="选择文件", command=self.select_50_file)
        self.btn_load_file_50.grid(row=0, column=5, padx=5)

        self.path_50 = tk.Entry(self.frame_data_1, textvariable=self.file_path_50, width=75,
                                font=("Arial", 7), state="readonly")
        self.path_50.grid(row=0, column=6, padx=5, sticky="ew")

        self.run_generate_50 = tk.Button(self.frame_data_1, text="生成50点位", command=self.generate_50_file)
        self.run_generate_50.grid(row=0, column=7, padx=5)
        # 查看文档
        self.read_file_button = tk.Button(self.frame_data_1, text="采集ID说明", width=15, command=preview_text_file_1,
                                          font=("Arial", 10, "bold"))
        self.read_file_button.grid(row=0, column=8, padx=5)

        self.frame_data_2 = tk.Frame(parent, bg="white")
        self.frame_data_2.pack(fill="x")
        # 第一列：collect 1 arm' data
        self.collect_btn_1 = tk.Button(self.frame_data_2, text="1# 数采", command=lambda: self.collect_data('A'))
        self.collect_btn_1.grid(row=0, column=0, padx=5)

        # 第2列：特征个数
        self.feature_1 = tk.Label(self.frame_data_2, text="特征个数", bg='white')
        self.feature_1.grid(row=0, column=1, padx=5)

        # 第3列：特征个数
        self.features_entry_1 = tk.Entry(self.frame_data_2, width=3)
        self.features_entry_1.insert(0, '7')
        self.features_entry_1.grid(row=0, column=2, padx=5)

        # 第4列：特征
        self.feature_idx_1 = tk.Label(self.frame_data_2, text="特征IDX", bg='white')
        self.feature_idx_1.grid(row=0, column=3, padx=5)

        # 第5列：特征
        self.entry_var_raw_1 = tk.StringVar(
            value="[0,1,2,3,4,5,6,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]")
        self.feature_idx_entry_1 = tk.Entry(self.frame_data_2, textvariable=self.entry_var_raw_1, width=100)
        self.feature_idx_entry_1.grid(row=0, column=4, padx=5, sticky="ew")

        # 第6列：行数文本
        self.lines_1 = tk.Label(self.frame_data_2, text="行数", bg='white')
        self.lines_1.grid(row=0, column=6, padx=5)

        # 第7列：行数
        self.lines_entry_1 = tk.Entry(self.frame_data_2, width=5)
        self.lines_entry_1.insert(0, '1000')
        self.lines_entry_1.grid(row=0, column=7, padx=5)

        # 第8列：stop collect
        self.stop_collect_btn_1 = tk.Button(self.frame_data_2, text="停止", command=self.stop_collect_data_both)
        self.stop_collect_btn_1.grid(row=0, column=8, padx=5)

        # 第3列：save collect
        self.save_collect_btn_1 = tk.Button(self.frame_data_2, text="保存", command=self.save_collect_data_both)
        self.save_collect_btn_1.grid(row=0, column=9, padx=5)

        self.frame_data_3 = tk.Frame(parent, bg="white")
        self.frame_data_3.pack(fill="x")
        # 第一列：collect 1 arm' data
        self.collect_btn_2 = tk.Button(self.frame_data_3, text="2# 数采", command=lambda: self.collect_data('B'))
        self.collect_btn_2.grid(row=0, column=0, padx=5)

        # 第2列：特征个数
        self.feature_2 = tk.Label(self.frame_data_3, text="特征个数", bg='white')
        self.feature_2.grid(row=0, column=1, padx=5)

        # 第3列：特征个数
        self.features_entry_2 = tk.Entry(self.frame_data_3, width=3)
        self.features_entry_2.insert(0, '7')
        self.features_entry_2.grid(row=0, column=2, padx=5)

        # 第4列：特征
        self.feature_idx_2 = tk.Label(self.frame_data_3, text="特征IDX", bg='white')
        self.feature_idx_2.grid(row=0, column=3, padx=5)

        # 第5列：特征
        self.entry_var_raw_2 = tk.StringVar(
            value="[100,101,102,103,104,105,106,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]")
        self.feature_idx_entry_2 = tk.Entry(self.frame_data_3, textvariable=self.entry_var_raw_2, width=100)
        self.feature_idx_entry_2.grid(row=0, column=4, padx=5, sticky="ew")

        # 第6列：行数文本
        self.lines_2 = tk.Label(self.frame_data_3, text="行数", bg='white')
        self.lines_2.grid(row=0, column=6, padx=5)

        # 第7列：行数
        self.lines_entry_2 = tk.Entry(self.frame_data_3, width=5)
        self.lines_entry_2.insert(0, '1000')
        self.lines_entry_2.grid(row=0, column=7, padx=5)

        # 第8列：stop collect
        self.stop_collect_btn_2 = tk.Button(self.frame_data_3, text="停止", command=self.stop_collect_data_both)
        self.stop_collect_btn_2.grid(row=0, column=8, padx=5)

        # 第3列：save collect
        self.save_collect_btn_2 = tk.Button(self.frame_data_3, text="保存", command=self.save_collect_data_both)
        self.save_collect_btn_2.grid(row=0, column=9, padx=5, pady=5)

        # 添加横线
        horizontal_line_5 = tk.Frame(parent, height=2, bg="#%02x%02x%02x" % (50, 150, 200))
        horizontal_line_5.pack(fill="x", expand=True)

        # 添加状态显示区域
        status_display_frame_4 = tk.Frame(parent, bg="white", padx=10, pady=5)
        status_display_frame_4.pack(fill="x", pady=5)

    def sensor_rectify_content(self, parent):
        self.sensor_frame_1 = tk.Frame(parent, bg="white")
        self.sensor_frame_1.pack(fill="x")
        # 第1 :text
        self.sensor_text_1 = tk.Label(self.sensor_frame_1, text="1#传感器偏置", bg="#2196F3",
                                      fg="white", font=("Arial", 10, "bold"))
        self.sensor_text_1.grid(row=0, column=0, padx=5, pady=5)

        # 第2列：sensor select
        self.axis_text_1 = tk.Label(self.sensor_frame_1, text="轴", bg="white")
        self.axis_text_1.grid(row=0, column=1, padx=5)

        # 第3列：axis select
        self.axis_select_combobox_1 = ttk.Combobox(
            self.sensor_frame_1,
            values=["0", "1", "2", "3", "4", "5", "6"],
            width=3,
            state="readonly"  # 禁止直接输入
        )
        self.axis_select_combobox_1.current(0)  # 默认选中第一个选项
        self.axis_select_combobox_1.grid(row=0, column=2, padx=5)

        # 第4列：get offset
        self.get_offset_btn_1 = tk.Button(self.sensor_frame_1, text="获取偏置",
                                          command=lambda: self.get_sensor_offset('A'))
        self.get_offset_btn_1.grid(row=0, column=3, padx=5)

        # 第5列：get offset value

        self.get_offset_entry_1 = tk.Entry(self.sensor_frame_1, width=5)
        self.get_offset_entry_1.insert(0, '0.0')
        self.get_offset_entry_1.grid(row=0, column=4, padx=5)

        # 第6列：set offset
        self.set_offset_btn_1 = tk.Button(self.sensor_frame_1, text="设置偏置",
                                          command=lambda: self.set_sensor_offset('A'))
        self.set_offset_btn_1.grid(row=0, column=5, padx=5)

        # # 第4列：BLANK
        self.blankkkkkk1 = tk.Label(self.sensor_frame_1, text=" ", bg='white', width=5)
        self.blankkkkkk1.grid(row=0, column=6, padx=5)

        # 第1 :text
        self.sensor_text_2 = tk.Label(self.sensor_frame_1, text="2#传感器偏置", bg="#2196F3",
                                      fg="white", font=("Arial", 10, "bold"))
        self.sensor_text_2.grid(row=0, column=7, padx=5)

        # 第2列：sensor select
        self.axis_text_2 = tk.Label(self.sensor_frame_1, text="轴", bg="white")
        self.axis_text_2.grid(row=0, column=8, padx=5)

        # 第3列：axis select
        self.axis_select_combobox_2 = ttk.Combobox(
            self.sensor_frame_1,
            values=["0", "1", "2", "3", "4", "5", "6"],
            width=3,
            state="readonly"  # 禁止直接输入
        )
        self.axis_select_combobox_2.current(0)  # 默认选中第一个选项
        self.axis_select_combobox_2.grid(row=0, column=9, padx=5)

        # 第4列：get offset
        self.get_offset_btn_2 = tk.Button(self.sensor_frame_1, text="获取偏置",
                                          command=lambda: self.get_sensor_offset('B'))
        self.get_offset_btn_2.grid(row=0, column=10, padx=5)

        # 第5列：get offset value
        self.get_offset_entry_2 = tk.Entry(self.sensor_frame_1, width=5)
        self.get_offset_entry_2.insert(0, '0.0')
        self.get_offset_entry_2.grid(row=0, column=11, padx=5)

        # 第6列：set offset
        self.set_offset_btn_2 = tk.Button(self.sensor_frame_1, text="设置偏置",
                                          command=lambda: self.set_sensor_offset('B'))
        self.set_offset_btn_2.grid(row=0, column=12, padx=5, pady=5)

        # 添加横线
        horizontal_line_6 = tk.Frame(parent, height=2, bg="#%02x%02x%02x" % (50, 150, 200))
        horizontal_line_6.pack(fill="x", expand=True)

        # 添加状态显示区域
        status_display_frame_5 = tk.Frame(parent, bg="white", padx=10, pady=5)
        status_display_frame_5.pack(fill="x", pady=5)

    def motor_content(self, parent):
        self.motor_frame_1 = tk.Frame(parent, bg="white")
        self.motor_frame_1.pack(fill="x")
        # 第1 :text
        self.motor_text_1 = tk.Label(self.motor_frame_1, text="1#电机编码器清零", bg="#036073",
                                     fg="white", font=("Arial", 10, "bold"))
        self.motor_text_1.grid(row=0, column=0, padx=5, pady=5)

        # 第2列：axis select
        self.motor_axis_text_1 = tk.Label(self.motor_frame_1, text="轴", bg="white")
        self.motor_axis_text_1.grid(row=0, column=1, padx=5)

        # 第3列：axis select
        self.motor_axis_select_combobox_1 = ttk.Combobox(
            self.motor_frame_1,
            values=["0", "1", "2", "3", "4", "5", "6"],
            width=3,
            state="readonly"  # 禁止直接输入
        )
        self.motor_axis_select_combobox_1.current(0)  # 默认选中第一个选项
        self.motor_axis_select_combobox_1.grid(row=0, column=2, padx=5)

        # 第4列：电机内编
        self.motor_btn_1 = tk.Button(self.motor_frame_1, text="电机内编",
                                     command=lambda: self.clear_motor_as_zero('A'))
        self.motor_btn_1.grid(row=0, column=3, padx=5, pady=5)

        # 第5列：电机外编
        self.motor_btn_2 = tk.Button(self.motor_frame_1, text="电机外编",
                                     command=lambda: self.clear_motorE_as_zero('A'))
        self.motor_btn_2.grid(row=0, column=4, padx=5)
        # # 第6列：空列
        # self.moter_blank_1 = tk.Label(self.motor_frame_1, text=" ", bg='white', width=1)
        # self.moter_blank_1.grid(row=0, column=5, padx=5)

        # 第7列：编码器清错
        self.motor_btn_3 = tk.Button(self.motor_frame_1, text="编码器清错", bg="#D0EBF0",
                                     command=lambda: self.clear_motor_error('A'))
        self.motor_btn_3.grid(row=0, column=5, padx=5)

        # 8：BLANK
        self.blankkkkkk1 = tk.Label(self.motor_frame_1, text=" ", bg='white', width=5)
        self.blankkkkkk1.grid(row=0, column=7, padx=5)

        # 1 :text
        self.motor_text_11 = tk.Label(self.motor_frame_1, text="2#电机编码器清零", bg="#036073",
                                      fg="white", font=("Arial", 10, "bold"))
        self.motor_text_11.grid(row=0, column=8, padx=5, pady=5)

        # 第2列：axis select
        self.motor_axis_text_11 = tk.Label(self.motor_frame_1, text="轴", bg="white")
        self.motor_axis_text_11.grid(row=0, column=9, padx=5)

        # 第3列：axis select
        self.motor_axis_select_combobox_11 = ttk.Combobox(
            self.motor_frame_1,
            values=["0", "1", "2", "3", "4", "5", "6"],
            width=3,
            state="readonly"  # 禁止直接输入
        )
        self.motor_axis_select_combobox_11.current(0)  # 默认选中第一个选项
        self.motor_axis_select_combobox_11.grid(row=0, column=10, padx=5)

        # 第4列：电机内编
        self.motor_btn_11 = tk.Button(self.motor_frame_1, text="电机内编",
                                      command=lambda: self.clear_motor_as_zero('B'))
        self.motor_btn_11.grid(row=0, column=11, padx=5)

        # 第5列：电机外编
        self.motor_btn_21 = tk.Button(self.motor_frame_1, text="电机外编",
                                      command=lambda: self.clear_motorE_as_zero('B'))
        self.motor_btn_21.grid(row=0, column=12, padx=5)
        # # 第6列：空列
        # self.moter_blank_11 = tk.Label(self.motor_frame_1, text=" ", bg='white', width=1)
        # self.moter_blank_11.grid(row=0, column=13, padx=5)

        # 第7列：编码器清错
        self.motor_btn_31 = tk.Button(self.motor_frame_1, text="编码器清错", bg="#D0EBF0",
                                      command=lambda: self.clear_motor_error('B'))
        self.motor_btn_31.grid(row=0, column=14, padx=5)

        # 添加横线
        horizontal_line_7 = tk.Frame(parent, height=2, bg="#%02x%02x%02x" % (50, 150, 200))
        horizontal_line_7.pack(fill="x", expand=True)

        # 添加状态显示区域
        status_display_frame_6 = tk.Frame(parent, bg="white", padx=10, pady=5)
        status_display_frame_6.pack(fill="x", pady=5)

    def eef_content(self, parent):
        self.eef_frame_1 = tk.Frame(parent, bg="white")
        self.eef_frame_1.pack(fill="x")
        # 第1 :text
        self.eef_text_1 = tk.Button(self.eef_frame_1, text="1#末端发送", command=lambda: self.send_data_eef('A'))
        self.eef_text_1.grid(row=0, column=0, padx=5, pady=5)

        # 第2列：sensor select
        self.com_text_1 = tk.Label(self.eef_frame_1, text="端口", bg="white", width=5)
        self.com_text_1.grid(row=0, column=1, padx=5)

        # 第3列：axis select
        self.com_select_combobox_1 = ttk.Combobox(
            self.eef_frame_1,
            values=["CAN", "COM1", "COM2"],
            width=5,
            state="readonly"  # 禁止直接输入
        )
        self.com_select_combobox_1.current(0)  # 默认选中第一个选项
        self.com_select_combobox_1.grid(row=0, column=2, padx=5)

        # self.com_entry_1 = tk.Entry(self.eef_frame_1, width=120)
        # self.com_entry_1.insert(0, "01 06 00 00 00 01 48 0A")
        # self.com_entry_1.grid(row=0, column=4, padx=5, sticky="ew")


        self.eef_delet_1=tk.Button(self.eef_frame_1, text="删除选中", command=lambda: self.delete_eef_command('A'))
        self.eef_delet_1.grid(row=0, column=3, padx=5, pady=5)


        self.eef_combo1 = ttk.Combobox(self.eef_frame_1, state="readonly",width=120)
        self.eef_combo1.grid(row=0, column=4, padx=5)

        self.eef_bt_1 = tk.Button(self.eef_frame_1, text="1#末端接收", command=lambda: self.receive_data_eef('A'))
        self.eef_bt_1.grid(row=0, column=5, padx=5)

        self.eef_frame_1_2 = tk.Frame(parent, bg="white")
        self.eef_frame_1_2.pack(fill="x")

        self.eef1_2_b1= tk.Label(self.eef_frame_1_2, text="", bg="white", width=7)
        self.eef1_2_b1.grid(row=0, column=0, padx=5)

        self.eef1_2_b2= tk.Label(self.eef_frame_1_2, text="", bg="white", width=7)
        self.eef1_2_b2.grid(row=0, column=1, padx=5)

        self.eef1_2_b3= tk.Label(self.eef_frame_1_2, text="", bg="white", width=8)
        self.eef1_2_b3.grid(row=0, column=2, padx=5)

        self.eef_add_1=tk.Button(self.eef_frame_1_2,text='1#加指令',command=lambda :self.add_eef_command('A'))
        self.eef_add_1.grid(row=0, column=3, padx=5)

        self.eef_entry = tk.Entry(self.eef_frame_1_2, width=120)
        self.eef_entry.insert(0, "01 06 00 00 00 01 48 0A")
        self.eef_entry.grid(row=0, column=4, padx=5, sticky="ew")

        self.eef_add_2=tk.Button(self.eef_frame_1_2,text='2#加指令',command=lambda :self.add_eef_command('B'))
        self.eef_add_2.grid(row=0, column=5, padx=5)


        self.eef_frame_2 = tk.Frame(parent, bg="white")
        self.eef_frame_2.pack(fill="x")
        # 第1 :text
        self.eef_bt_2 = tk.Button(self.eef_frame_2, text="2#末端发送", command=lambda: self.send_data_eef('B'))
        self.eef_bt_2.grid(row=0, column=0, padx=5)

        # 第2列：sensor select
        self.com_text_2 = tk.Label(self.eef_frame_2, text="端口", bg="white", width=5)
        self.com_text_2.grid(row=0, column=1, padx=5)

        # 第3列：axis select
        self.com_select_combobox_2 = ttk.Combobox(
            self.eef_frame_2,
            values=["CAN", "COM1", "COM2"],
            width=5,
            state="readonly"  # 禁止直接输入
        )
        self.com_select_combobox_2.current(0)  # 默认选中第一个选项
        self.com_select_combobox_2.grid(row=0, column=2, padx=5)

        # self.com_entry_2 = tk.Entry(self.eef_frame_2, width=120)
        # self.com_entry_2.insert(0, "01 06 00 00 00 01 48 0A")
        # self.com_entry_2.grid(row=0, column=4, padx=5, sticky="ew")

        self.eef_delet_2=tk.Button(self.eef_frame_2, text="删除选中", command=lambda: self.delete_eef_command('B'))
        self.eef_delet_2.grid(row=0, column=3, padx=5, pady=5)


        self.eef_combo2 = ttk.Combobox(self.eef_frame_2, state="readonly",width=120)
        self.eef_combo2.grid(row=0, column=4, padx=5)

        self.eef_bt_4 = tk.Button(self.eef_frame_2, text="2#末端接收", command=lambda: self.receive_data_eef('B'))
        self.eef_bt_4.grid(row=0, column=5, padx=5, pady=5)


        self.eef_frame_3 = tk.Frame(parent, bg="white")
        self.eef_frame_3.pack(fill="x")

        # 接收内容文本框
        recv_label1 = tk.Label(self.eef_frame_3, text="1#接收内容:")
        recv_label1.grid(row=0, column=0, padx=5)


        # 间隔
        spacer = tk.Label(self.eef_frame_3, text="   ", bg='white')
        spacer.grid(row=0, column=1, padx=5)

        self.recv_text1 = scrolledtext.ScrolledText(self.eef_frame_3, width=70, height=8, wrap=tk.WORD)
        self.recv_text1.grid(row=1, column=0, padx=5)
        self.recv_text1.insert(tk.END, '使用提示：\n请先选择端口：CAN/COM1/COM2, \n点击 1#末端接收 按钮， \n输入发送数据，点击 1#末端接收按钮, \n接收到的末端信息以1khz频率刷新显示')

        # 间隔
        spacer1 = tk.Label(self.eef_frame_3, text="   ", bg='white')
        spacer1.grid(row=1, column=1, padx=5)

        # 接收内容文本框
        recv_label2 = tk.Label(self.eef_frame_3, text="2#接收内容:")
        recv_label2.grid(row=0, column=2, padx=5)

        self.recv_text2 = scrolledtext.ScrolledText(self.eef_frame_3, width=70, height=8, wrap=tk.WORD)
        self.recv_text2.grid(row=1, column=2, padx=5)
        self.recv_text2.insert(tk.END, '使用提示：\n请先选择端口：CAN/COM1/COM2, \n点击 2#末端接收 按钮， \n输入发送数据，点击 2#末端接收按钮, \n接收到的末端信息以1khz频率刷新显示')

        # 添加状态显示区域
        status_display_frame_7 = tk.Frame(parent, bg="white", padx=10, pady=5)
        status_display_frame_7.pack(fill="x", pady=5)

    def on_mousewheel(self, event):
        """处理鼠标滚轮事件"""
        self.canvas.yview_scroll(int(-1 * (event.delta / 130)), "units")

    def create_status_bar(self):
        """创建底部状态栏"""

        self.status_frame1 = tk.Frame(self.root, bd=1, relief=tk.SUNKEN, bg="#f0f0f0")
        self.status_frame1.pack(side=tk.BOTTOM, fill=tk.X)

        # 右侧设备状态
        self.right_frame = tk.Frame(self.status_frame1, bg="#f0f0f0")
        self.right_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(20, 10), pady=5)

        self.status_frame = tk.Frame(self.root, bd=1, relief=tk.SUNKEN, bg="#f0f0f0")
        self.status_frame.pack(side=tk.BOTTOM, fill=tk.X)

        # 左侧设备状态
        self.left_frame = tk.Frame(self.status_frame, bg="#f0f0f0")
        self.left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(20, 10), pady=5)

        # # 分隔线
        # separator = ttk.Separator(self.status_frame, orient=tk.VERTICAL)
        # separator.pack(side=tk.LEFT, fill=tk.Y, padx=5)

        # 初始化状态标签
        self.init_status_labels()

    def init_status_labels(self):
        """初始化状态标签"""
        # 左侧设备状态标签
        tk.Label(
            self.left_frame,
            text="#1",
            bg="#f0f0f0",
            font=("Arial", 9, "bold"),
            width=3
        ).pack(side=tk.LEFT, padx=(0, 3))  #

        self.left_state_main = tk.Label(
            self.left_frame,
            text="下始能",
            bg="#fcf16e",
            fg="black",
            font=("Arial", 9),
            padx=2,
            pady=2,
            width=10
        )
        self.left_state_main.pack(side=tk.LEFT, padx=5)

        self.left_state_1 = tk.Label(
            self.left_frame,
            text="拖动按钮: 0",
            bg="#e0e0e0",
            font=("Arial", 9),
            padx=2,
            pady=2,
            width=10
        )
        self.left_state_1.pack(side=tk.LEFT, padx=5)

        self.left_state_2 = tk.Label(
            self.left_frame,
            text="低速标志:1",
            bg="#e0e0e0",
            font=("Arial", 9),
            padx=2,
            pady=2,
            width=10
        )
        self.left_state_2.pack(side=tk.LEFT, padx=2)

        self.left_state_3 = tk.Label(
            self.left_frame,
            text="错误码:0",
            bg="#e0e0e0",
            font=("Arial", 9),
            padx=2,
            pady=2,
            width=10
        )
        self.left_state_3.pack(side=tk.LEFT, padx=2)

        self.left_data = tk.Label(
            self.left_frame,
            text="J1-J7: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
            bg="#e0e0e0",
            font=("Courier New", 9),
            padx=2,
            pady=2
        )
        self.left_data.pack(side=tk.LEFT, fill=tk.X, expand=True)

        self.left_data_6d = tk.Label(
            self.left_frame,
            text="XYZABC:[0.,0.,0.,0.,0.,0.]",
            bg="#e0e0e0",
            font=("Courier New", 9),
            padx=2,
            pady=2
        )
        self.left_data_6d.pack(side=tk.RIGHT, fill=tk.X, expand=True)

        # 右侧设备状态标签
        tk.Label(
            self.right_frame,
            text="#2",
            bg="#f0f0f0",
            font=("Arial", 9, "bold"),
            width=3
        ).pack(side=tk.LEFT, padx=(0, 3))

        self.right_state_main = tk.Label(
            self.right_frame,
            text="下始能",
            bg="#fcf16e",
            fg="black",
            font=("Arial", 9),
            padx=2,
            pady=2,
            width=10
        )
        self.right_state_main.pack(side=tk.LEFT, padx=5)

        self.right_state_1 = tk.Label(
            self.right_frame,
            text="拖动按钮: 0",
            bg="#e0e0e0",
            font=("Arial", 9),
            padx=2,
            pady=2,
            width=10
        )
        self.right_state_1.pack(side=tk.LEFT, padx=5)

        self.right_state_2 = tk.Label(
            self.right_frame,
            text="低速标志:1",
            bg="#e0e0e0",
            font=("Arial", 9),
            padx=2,
            pady=2,
            width=10
        )
        self.right_state_2.pack(side=tk.LEFT, padx=2)

        self.right_state_3 = tk.Label(
            self.right_frame,
            text="错误码:0",
            bg="#e0e0e0",
            font=("Arial", 9),
            padx=2,
            pady=2,
            width=10
        )
        self.right_state_3.pack(side=tk.LEFT, padx=2)

        self.right_data = tk.Label(
            self.right_frame,
            text="J1-J7: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
            bg="#e0e0e0",
            font=("Courier New", 9),
            padx=2,
            pady=2
        )
        self.right_data.pack(side=tk.LEFT, fill=tk.X, expand=True)

        self.right_data_6d = tk.Label(
            self.right_frame,
            text="XYZABC:[0.,0.,0.,0.,0.,0.]",
            bg="#e0e0e0",
            font=("Courier New", 9),
            padx=2,
            pady=2
        )
        self.right_data_6d.pack(side=tk.RIGHT, fill=tk.X, expand=True)

    def toggle_connection(self):
        global_robot_ip = self.arm_ip_entry.get()
        if global_robot_ip:
            init = robot.connect(global_robot_ip)
            print(f'\nrobot connect ({global_robot_ip}), return:{init}')
            # if init==0:
            #     messagebox.showerror('failed','端口占用，连接失败')
            # else:
            '''清错'''
            robot.clear_set()
            robot.clear_error('A')
            robot.clear_error('B')
            robot.send_cmd()
            time.sleep(0.1)

            """切换设备连接状态"""
            self.connected = not self.connected

        if self.connected:
            # 连接设备
            self.connect_btn.config(text="断开连接", bg="#F44336")
            self.status_label.config(text="已连接")
            self.status_light.config(fg="green")
            self.mode_btn.config(state="normal")
            '''judge '''
            time.sleep(0.2)
            motion_tag = 0
            frame_update = None
            for i in range(5):
                sub_data = robot.subscribe(dcss)
                print(f"connect frames :{sub_data['outputs'][0]['frame_serial']}")
                if sub_data['outputs'][0]['frame_serial'] != 0 and frame_update != sub_data['outputs'][0][
                    'frame_serial']:
                    motion_tag += 1
                    frame_update = sub_data['outputs'][0]['frame_serial']
                time.sleep(0.01)
            if motion_tag > 0:
                '''启动读485数据'''

                # 启动数据订阅
                self.data_subscriber = DataSubscriber(self.update_data)

                '''tool '''
                robot.receive_file(self.tools_txt, '/home/fusion/tool_dyn_kine.txt')
                time.sleep(1)
                from python.fx_robot import read_csv_file_to_float_strict
                self.tool_result = read_csv_file_to_float_strict(self.tools_txt, expected_columns=16)
                if self.tool_result==0:
                    messagebox.showinfo('success', '机器人连接成功. 机器人未设置工具信息，如果带工具，请设置工具信息')
                else:
                    messagebox.showinfo('success', '机器人连接成功.  机器人已设置工具信息.')
                    # print(f"成功读取数据: {self.tool_result}")
                    if isinstance(self.tool_result[0], list):
                        # print(f"第一行: {self.tool_result[0]}")
                        # print(f"第二行: {self.tool_result[1]}")

                        self.tool_a_entry.delete(0, tk.END)
                        self.tool_a_entry.insert(0, str(self.tool_result[0][:10]))

                        self.tool_a1_entry.delete(0, tk.END)
                        self.tool_a1_entry.insert(0, str(self.tool_result[0][10:]))

                        self.tool_b_entry.delete(0, tk.END)
                        self.tool_b_entry.insert(0, str(self.tool_result[1][:10]))

                        self.tool_b1_entry.delete(0, tk.END)
                        self.tool_b1_entry.insert(0, str(self.tool_result[1][10:]))

                        tool_mat = kk1.xyzabc_to_mat4x4(self.tool_result[0][10:])
                        tool_mat1 = kk2.xyzabc_to_mat4x4(self.tool_result[1][10:])
                        kk1.set_tool_kine(robot_serial=0, tool_mat=tool_mat)
                        kk2.set_tool_kine(robot_serial=1, tool_mat=tool_mat1)

            if motion_tag == 0:
                messagebox.showerror('failed!', "机器人连接不成功，请重连")

        else:
            # # 断开连接 夹在这就不能读到订阅了，加到关闭窗口里面
            # robot.release_robot()
            self.connect_btn.config(text="连接机器人", bg="#4CAF50")
            self.status_label.config(text="未连接")
            self.status_light.config(fg="red")
            self.mode_btn.config(state="disabled")

            # 停止数据订阅
            if self.data_subscriber:
                self.data_subscriber.stop()
                self.data_subscriber = None

            # 重置数据
            self.result = {
                'states': [
                    {'cur_state': 0, 'cmd_state': 0, 'err_code': 0}, {'cur_state': 0, 'cmd_state': 0, 'err_code': 0},
                    {'cur_state': 0, 'cmd_state': 0, 'err_code': 0}, {'cur_state': 0, 'cmd_state': 0, 'err_code': 0}
                ],
                'outputs':
                    [
                        {'frame_serial': 0,
                         'tip_di': b'\x00',
                         'low_speed_flag': b'\x00',
                         'fb_joint_pos': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 反馈关节位置
                         'fb_joint_vel': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 反馈关节速度
                         'fb_joint_posE': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 反馈关节位置(外编)
                         'fb_joint_cmd': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 位置关节指令
                         'fb_joint_cToq': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 反馈关节电流
                         'fb_joint_sToq': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 传感器
                         'fb_joint_them': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 反馈关节温度
                         'est_joint_firc': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                         'est_joint_firc_dot': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                         'est_joint_force': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 轴外力
                         'est_cart_fn': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},

                        {'frame_serial': 0,
                         'tip_di': b'\x00',
                         'low_speed_flag': b'\x00',
                         'fb_joint_pos': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                         'fb_joint_vel': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                         'fb_joint_posE': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                         'fb_joint_cmd': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                         'fb_joint_cToq': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                         'fb_joint_sToq': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                         'fb_joint_them': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                         'est_joint_firc': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                         'est_joint_firc_dot': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                         'est_joint_force': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                         'est_cart_fn': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
                    ]
            }
            self.update_ui()

    def toggle_display_mode(self):
        """切换数据显示模式"""
        self.display_mode = (self.display_mode + 1) % 8
        self.mode_btn.config(text=self.mode_names[self.display_mode])
        self.update_ui()

    def update_data(self, result):
        """更新订阅的数据"""
        self.result = result
        self.root.after(0, self.update_ui)
        self.root.after(0, self.update_6d)

    def update_6d(self):
        """更新UI显示"""
        data11 = self.result['outputs'][0]['fb_joint_pos']
        data22 = self.result['outputs'][1]['fb_joint_pos']
        list_joints_a = []
        for iii in data11:
            list_joints_a.append(float(iii))

        list_joints_b = []
        for jjj in data22:
            list_joints_b.append(float(jjj))

        if list_joints_a[:] != 0.0:
            fk_mat_1 = kk1.fk(robot_serial=0, joints=list_joints_a)
            # print(f'-----joints a:{list_joints_a}, fk_mat:{type(fk_mat_1)}')
            pose_6d_1 = kk1.mat4x4_to_xyzabc(pose_mat=fk_mat_1)  # 用关节正解的姿态转XYZABC
        else:
            pose_6d_1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        if list_joints_b[:] != 0:
            fk_mat_2 = kk2.fk(robot_serial=1, joints=list_joints_b)
            time.sleep(0.1)
            # print(f'-----jointsb:{list_joints_b}, fk_mat:{type(fk_mat_2)}')
            pose_6d_2 = kk2.mat4x4_to_xyzabc(pose_mat=fk_mat_2)  # 用关节正解的姿态转XYZABC
        else:
            pose_6d_2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # 更新数据显示
        self.left_data_6d.config(text=f"XYZABC: [{format_vector(pose_6d_1)}]")
        self.right_data_6d.config(text=f"XYZABC: [{format_vector(pose_6d_2)}]")

    def update_ui(self):
        """更新UI显示"""
        # 更新状态值
        self.left_state_main.config(text=f"状态:{self.result['states'][0]['cur_state']}")
        self.left_state_1.config(text=f"拖动按钮:{self.result['outputs'][0]['tip_di'][0]}")
        self.left_state_2.config(text=f"低速标志:{self.result['outputs'][0]['low_speed_flag'][0]}")
        self.left_state_3.config(text=f"错误码:{self.result['states'][0]['err_code']}")
        self.right_state_main.config(text=f"状态:{self.result['states'][1]['cur_state']}")
        self.right_state_1.config(text=f"拖动按钮:{self.result['outputs'][1]['tip_di'][0]}")
        self.right_state_2.config(text=f"低速标志:{self.result['outputs'][1]['low_speed_flag'][0]}")
        self.right_state_3.config(text=f"错误码:{self.result['states'][1]['err_code']}")

        # 根据当前模式获取数据
        key = self.data_keys[self.display_mode]
        data1 = self.result['outputs'][0][key][:]
        data2 = self.result['outputs'][1][key][:]

        # 更新数据显示
        self.left_data.config(text=f"J1-J7: [{format_vector(data1)}]")
        self.right_data.config(text=f"J1-J7: [{format_vector(data2)}]")

        # 根据状态值更新颜色
        pid_colors = ["gray", "green"]
        speed_colors = ["green", "gray"]
        pid_color_1 = pid_colors[self.result['outputs'][0]['tip_di'][0]]
        pid_color_2 = pid_colors[self.result['outputs'][0]['tip_di'][0]]

        speed_color_1 = speed_colors[self.result['outputs'][1]['low_speed_flag'][0]]
        speed_color_2 = speed_colors[self.result['outputs'][1]['low_speed_flag'][0]]

        self.left_state_1.config(bg=pid_color_1, fg="white")
        self.right_state_1.config(bg=pid_color_2, fg="white")

        self.left_state_2.config(bg=speed_color_1, fg="white")
        self.right_state_2.config(bg=speed_color_2, fg="white")

        # 数据背景色
        data_bg = "#f5f5f5" if self.connected else "#e0e0e0"
        self.left_data.config(bg=data_bg)
        self.right_data.config(bg=data_bg)


    def select_period_file(self, robot_id):
        file_path = filedialog.askopenfilename(
            defaultextension=".r50pth",
            filetypes=[("path files", "*.r50pth"), ("All files", "*.*")],
            title="选择1#周期运行文件"
        )
        if file_path:
            if robot_id == 'A':
                self.period_file_path_1.set(file_path)
                # messagebox.showinfo("成功", f"1#周期运行文件已选择: {os.path.basename(file_path)}")
            elif robot_id == 'B':
                self.period_file_path_2.set(file_path)
                messagebox.showinfo("成功", f"2#周期运行文件已选择: {os.path.basename(file_path)}")

    def run_period_file(self, robot_id):
        if self.connected:

            try:
                if robot_id == 'A':
                    with open(self.period_file_path_1.get(), 'r', encoding='utf-8') as file:
                        lines = file.readlines()

                    for i, line in enumerate(lines):
                        # 处理当前行
                        processed_line = self.process_line(i, line)
                        # print(f'processed_line:{processed_line}')
                        robot.clear_set()
                        robot.set_joint_cmd_pose(arm='A', joints=processed_line)
                        robot.send_cmd()
                        # 50Hz频率 = 每0.02秒一行
                        time.sleep(0.02)
                elif robot_id == 'B':
                    with open(self.period_file_path_2.get(), 'r', encoding='utf-8') as file:
                        lines = file.readlines()

                    for i, line in enumerate(lines):
                        # 处理当前行
                        processed_line = self.process_line(i, line)
                        # print(f'processed_line:{processed_line}')
                        robot.clear_set()
                        robot.set_joint_cmd_pose(arm='B', joints=processed_line)
                        robot.send_cmd()
                        # 50Hz频率 = 每0.02秒一行
                        time.sleep(0.02)
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("错误", f"读取文件时出错: {str(e)}"))
        else:
            messagebox.showerror('error', '请先连接机器人')

    def process_line(self, line_num, line):
        """处理单行数据，将其转换为浮点数列表"""
        try:
            # 去除行尾的换行符和多余空格
            cleaned_line = line.strip()
            # 分割字符串（假设数据由空格或制表符分隔）
            elements = cleaned_line.split()
            # 确保每行有7个元素
            if len(elements) != 7:
                return f"错误: 第 {line_num + 1} 行有 {len(elements)} 个元素，但需要7个"
            # 尝试将每个元素转换为浮点数
            float_list = [float(element) for element in elements]
            return float_list
        except ValueError as e:
            return f"错误: 第 {line_num + 1} 行包含非数值数据 - {str(e)}"
        except Exception as e:
            return f"错误: 处理第 {line_num + 1} 行时发生未知错误 - {str(e)}"

    def add_current_joints(self, robot_id):
        if self.connected:
            self.entry.delete(0, tk.END)
            if robot_id == 'A':
                self.entry.insert(0, str([0.0 if abs(round(x, 2)) < 1e-5 else round(x, 2) for x in
                                          self.result['outputs'][0]['fb_joint_pos']]))
            elif robot_id == 'B':
                self.entry.insert(0, str([0.0 if abs(round(x, 2)) < 1e-5 else round(x, 2) for x in
                                          self.result['outputs'][1]['fb_joint_pos']]))
        else:
            messagebox.showerror('error', '请先连接机器人')

    def validate_point(self, point_str):
        """验证输入是否为长度为7的列表"""
        try:
            point_list = ast.literal_eval(point_str)
            # 检查是否为列表且长度为7
            if not isinstance(point_list, list):
                return False, "输入必须是一个列表"
            if len(point_list) != 7:
                return False, "列表长度必须为7"
            # 检查所有元素是否为数字
            for item in point_list:
                if not isinstance(item, (int, float)):
                    return False, "列表中的所有元素必须是数字"
            return True, point_list
        except (ValueError, SyntaxError):
            return False, "输入格式不正确，必须是有效的列表格式，如 [0,0,0,0,0,0,0]"

    def is_duplicate(self, point_list, target_list):
        """检查点是否已经在列表中存在（去重功能）"""
        # 将点列表转换为元组以便比较（列表不可哈希）
        point_tuple = tuple(point_list)
        # 检查目标列表中是否已存在相同的点
        for existing_point_str in target_list:
            try:
                existing_point = ast.literal_eval(existing_point_str)
                if tuple(existing_point) == point_tuple:
                    return True
            except:
                continue
        return False



    def is_duplicate_command(self,point_list, target_list):
        """检查点是否已经在列表中存在（去重功能）"""
        for existing_point_str in target_list:
            if existing_point_str == point_list:
                return True
        return False

    def add_point1(self):
        """添加点到1#列表"""
        point_str = self.entry_var.get()
        is_valid, result = self.validate_point(point_str)
        if is_valid:
            # 检查是否已存在相同的点
            if self.is_duplicate(result, self.points1):
                messagebox.showwarning("重复点", "该点已存在于1#列表中")
                return
            # 将列表转换为字符串并存储
            point_repr = str(result)
            self.points1.insert(0, point_repr)
            self.update_comboboxes()
            # messagebox.showinfo("成功", "点已添加到1#列表")
        else:
            messagebox.showwarning("输入错误", result)

    def add_point2(self):
        """添加点到2#列表"""
        point_str = self.entry_var.get()
        is_valid, result = self.validate_point(point_str)
        if is_valid:
            # 检查是否已存在相同的点
            if self.is_duplicate(result, self.points2):
                messagebox.showwarning("重复点", "该点已存在于2#列表中")
                return
            # 将列表转换为字符串并存储
            point_repr = str(result)
            self.points2.insert(0, point_repr)
            self.update_comboboxes()
            # messagebox.showinfo("成功", "点已添加到2#列表")
        else:
            messagebox.showwarning("输入错误", result)

    def delete_point1(self):
        """从1#列表删除选中的点"""
        selected_index = self.combo1.current()
        if selected_index != -1 and selected_index < len(self.points1):
            self.points1.pop(selected_index)
            self.update_comboboxes()
            # messagebox.showinfo("成功", "点已从1#列表中删除")
        else:
            messagebox.showwarning("警告", "请选择要删除的点")

    def delete_point2(self):
        """从2#列表删除选中的点"""
        selected_index = self.combo2.current()
        if selected_index != -1 and selected_index < len(self.points2):
            self.points2.pop(selected_index)
            self.update_comboboxes()
            # messagebox.showinfo("成功", "点已从2#列表中删除")
        else:
            messagebox.showwarning("警告", "请选择要删除的点")

    def save_points1(self):
        """保存1#列表到TXT文件"""
        if not self.points1:
            messagebox.showwarning("警告", "1#列表为空，没有内容可保存")
            return
        file_path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            title="保存1#点列表"
        )
        if file_path:
            try:
                with open(file_path, 'w') as f:
                    for point in self.points1:
                        f.write(point + '\n')
                # messagebox.showinfo("成功", f"1#点列表已保存到: {os.path.basename(file_path)}")
            except Exception as e:
                messagebox.showerror("错误", f"保存文件时出错: {str(e)}")
    def save_points2(self):
        """保存2#列表到TXT文件"""
        if not self.points2:
            messagebox.showwarning("警告", "2#列表为空，没有内容可保存")
            return
        file_path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            title="保存2#点列表"
        )
        if file_path:
            try:
                with open(file_path, 'w') as f:
                    for point in self.points2:
                        f.write(point + '\n')
                # messagebox.showinfo("成功", f"2#点列表已保存到: {os.path.basename(file_path)}")
            except Exception as e:
                messagebox.showerror("错误", f"保存文件时出错: {str(e)}")

    def load_points1(self):
        """从TXT文件导入到1#列表"""
        file_path = filedialog.askopenfilename(
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            title="选择要导入到1#的文件"
        )
        if file_path:
            try:
                with open(file_path, 'r') as f:
                    lines = f.readlines()
                # 验证并导入点
                valid_points = []
                invalid_lines = []
                for i, line in enumerate(lines, 1):
                    line = line.strip()
                    if line:  # 跳过空行
                        is_valid, result = self.validate_point(line)
                        if is_valid:
                            # 检查是否重复
                            if not self.is_duplicate(result, self.points1 + valid_points):
                                valid_points.append(str(result))
                        else:
                            invalid_lines.append(f"第{i}行: {line}")
                # 添加有效点
                if valid_points:
                    # self.points1.extend(valid_points)
                    self.points1 = valid_points
                    self.update_comboboxes()
                    # messagebox.showinfo("成功", f"从文件导入了 {len(valid_points)} 个点到1#列表")
                # 显示无效行
                if invalid_lines:
                    messagebox.showwarning("警告",
                                           f"以下行格式无效，已跳过:\n" +
                                           "\n".join(invalid_lines[:10]) +
                                           ("\n..." if len(invalid_lines) > 10 else ""))
            except Exception as e:
                messagebox.showerror("错误", f"读取文件时出错: {str(e)}")

    def load_points2(self):
        """从TXT文件导入到2#列表"""
        file_path = filedialog.askopenfilename(
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            title="选择要导入到2#的文件"
        )
        if file_path:
            try:
                with open(file_path, 'r') as f:
                    lines = f.readlines()
                # 验证并导入点
                valid_points = []
                invalid_lines = []
                for i, line in enumerate(lines, 1):
                    line = line.strip()
                    if line:  # 跳过空行
                        is_valid, result = self.validate_point(line)
                        if is_valid:
                            # 检查是否重复
                            if not self.is_duplicate(result, self.points2 + valid_points):
                                valid_points.append(str(result))
                        else:
                            invalid_lines.append(f"第{i}行: {line}")
                # 添加有效点
                if valid_points:
                    self.points2 = valid_points
                    self.update_comboboxes()
                    # messagebox.showinfo("成功", f"从文件导入了 {len(valid_points)} 个点到2#列表")
                # 显示无效行
                if invalid_lines:
                    messagebox.showwarning("警告",
                                           f"以下行格式无效，已跳过:\n" +
                                           "\n".join(invalid_lines[:10]) +
                                           ("\n..." if len(invalid_lines) > 10 else ""))
            except Exception as e:
                messagebox.showerror("错误", f"读取文件时出错: {str(e)}")



    def add_eef_command(self,robot_id):
        """添加点到1#列表"""
        command_str = self.eef_entry.get()
        if robot_id=='A':
            # 检查是否已存在相同的点
            if self.is_duplicate_command(command_str, self.command1):
                messagebox.showwarning("重复指令", "该指令已存在于1#列表中")
                return
            else:
                self.command1.insert(0, command_str)
        elif robot_id=='B':
            # 检查是否已存在相同的点
            if self.is_duplicate_command(command_str, self.command2):
                messagebox.showwarning("重复指令", "该指令已存在于1#列表中")
                return
            else:
                self.command2.insert(0, command_str)
        self.update_combo_eef()

    def delete_eef_command(self,robot_id):
        """从2#列表删除选中的点"""
        if robot_id=='A':
            selected_index = self.eef_combo1.current()
            if selected_index != -1 and selected_index < len(self.command1):
                self.command1.pop(selected_index)
                self.update_combo_eef()
            else:
                messagebox.showwarning("警告", "请选择要删除的通讯指令")
        elif robot_id=='B':
            selected_index = self.eef_combo1.current()
            if selected_index != -1 and selected_index < len(self.command2):
                self.command2.pop(selected_index)
                self.update_combo_eef()
            else:
                messagebox.showwarning("警告", "请选择要删除的通讯指令")


    def update_comboboxes(self):
        """更新两个下拉框的内容"""
        self.combo1['values'] = self.points1
        self.combo2['values'] = self.points2

        # 如果有选项，选择第一个
        if self.points1:
            self.combo1.current(0)
        else:
            self.combo1.set('')
        if self.points2:
            self.combo2.current(0)
        else:
            self.combo2.set('')

    def update_combo_eef(self):
        # 更新eef commands列表
        self.eef_combo1['values'] = self.command1
        self.eef_combo2['values'] = self.command2
        # 如果有选项，选择第一个
        if self.command1:
            self.eef_combo1.current(0)
        else:
            self.eef_combo1.set('')
        if self.command2:
            self.eef_combo2.current(0)
        else:
            self.eef_combo2.set('')

    def run1(self):
        if self.connected:
            """1#运行按钮的功能"""
            selected = self.combo1.get()
            if selected:
                # 验证选中的点是否为有效的7元素列表
                is_valid, point_list = self.validate_point(selected)
                if is_valid:
                    # messagebox.showinfo("1#运行", f"运行选中的点: {point_list}")
                    robot.clear_set()
                    robot.set_joint_cmd_pose(arm='A', joints=point_list)
                    robot.send_cmd()
                else:
                    messagebox.showerror("错误", f"选中的点格式无效: {selected}")
            else:
                messagebox.showwarning("警告", "没有可运行的点")
        else:
            messagebox.showerror('error', '请先连接机器人')

    def run2(self):
        if self.connected:
            """2#运行按钮的功能"""
            selected = self.combo2.get()
            if selected:
                # 验证选中的点是否为有效的7元素列表
                is_valid, point_list = self.validate_point(selected)
                if is_valid:
                    # messagebox.showinfo("2#运行", f"运行选中的点: {point_list}")
                    robot.clear_set()
                    robot.set_joint_cmd_pose(arm='B', joints=point_list)
                    robot.send_cmd()
                else:
                    messagebox.showerror("错误", f"选中的点格式无效: {selected}")
            else:
                messagebox.showwarning("警告", "没有可运行的点")
        else:
            messagebox.showerror('error', '请先连接机器人')

    def save_param(self, robot_id):
        if robot_id == 'A':
            self.params.append(str(ast.literal_eval(self.tool_a_entry.get())))
            self.params.append(str(ast.literal_eval(self.tool_a1_entry.get())))
            self.params.append(str(ast.literal_eval(self.k_a_entry.get())))
            self.params.append(str(ast.literal_eval(self.d_a_entry.get())))
            self.params.append(str(ast.literal_eval(self.cart_k_a_entry.get())))
            self.params.append(str(ast.literal_eval(self.cart_d_a_entry.get())))
            self.params.append(str(ast.literal_eval(self.vel_a_entry.get())))
            self.params.append(str(ast.literal_eval(self.acc_a_entry.get())))

        elif robot_id == 'A':
            self.params.append(str(ast.literal_eval(self.tool_b_entry.get())))
            self.params.append(str(ast.literal_eval(self.tool_b1_entry.get())))
            self.params.append(str(ast.literal_eval(self.k_b_entry.get())))
            self.params.append(str(ast.literal_eval(self.d_b_entry.get())))
            self.params.append(str(ast.literal_eval(self.cart_k_b_entry.get())))
            self.params.append(str(ast.literal_eval(self.cart_d_b_entry.get())))
            self.params.append(str(ast.literal_eval(self.vel_b_entry.get())))
            self.params.append(str(ast.literal_eval(self.acc_b_entry.get())))
        print(f'save params:{self.params}')

        file_path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            title="保存设置的运动参数"
        )
        if file_path:
            try:
                with open(file_path, 'w') as f:
                    for point in self.params:
                        f.write(point + '\n')
                # messagebox.showinfo("成功", f"运动参数已保存到: {os.path.basename(file_path)}")
            except Exception as e:
                messagebox.showerror("错误", f"保存文件时出错: {str(e)}")

    def load_param(self, robot_id):
        if robot_id == 'A':
            file_path = filedialog.askopenfilename(
                filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
                title="选择要导入到1#的参数文件"
            )
            if file_path:
                try:
                    with open(file_path, 'r') as f:
                        lines = f.readlines()
                    valid_points = []
                    for i, line in enumerate(lines, 1):
                        line = line.strip()
                        if line:  # 跳过空行
                            valid_points.append(line)
                    print(f'valid_points:{valid_points}')

                    # 添加有效点
                    if valid_points:
                        self.tool_a_entry.delete(0, tk.END)
                        self.tool_a_entry.insert(0, valid_points[0])
                        self.tool_a1_entry.delete(0, tk.END)
                        self.tool_a1_entry.insert(0, valid_points[1])
                        self.k_a_entry.delete(0, tk.END)
                        self.k_a_entry.insert(0, valid_points[2])
                        self.d_a_entry.delete(0, tk.END)
                        self.d_a_entry.insert(0, valid_points[3])
                        self.cart_k_a_entry.delete(0, tk.END)
                        self.cart_k_a_entry.insert(0, valid_points[4])
                        self.cart_d_a_entry.delete(0, tk.END)
                        self.cart_d_a_entry.insert(0, valid_points[5])
                        self.vel_a_entry.delete(0, tk.END)
                        self.vel_a_entry.insert(0, valid_points[6])
                        self.acc_a_entry.delete(0, tk.END)
                        self.acc_a_entry.insert(0, valid_points[7])
                        # messagebox.showinfo("成功", f"从文件导入了 {len(valid_points)} 参数到#1")

                except Exception as e:
                    messagebox.showerror("错误", f"读取文件时出错: {str(e)}")

        elif robot_id == 'B':
            file_path = filedialog.askopenfilename(
                filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
                title="选择要导入到2#的参数文件"
            )

            if file_path:
                try:
                    with open(file_path, 'r') as f:
                        lines = f.readlines()
                    valid_points = []
                    for i, line in enumerate(lines, 1):
                        line = line.strip()
                        if line:  # 跳过空行
                            valid_points.append(line)
                    print(f'valid_points:{valid_points}')

                    # 添加有效点
                    if valid_points:
                        self.tool_b_entry.delete(0, tk.END)
                        self.tool_b_entry.insert(0, valid_points[0])
                        self.tool_b1_entry.delete(0, tk.END)
                        self.tool_b1_entry.insert(0, valid_points[1])
                        self.k_b_entry.delete(0, tk.END)
                        self.k_b_entry.insert(0, valid_points[2])
                        self.d_b_entry.delete(0, tk.END)
                        self.d_b_entry.insert(0, valid_points[3])
                        self.cart_k_b_entry.delete(0, tk.END)
                        self.cart_k_b_entry.insert(0, valid_points[4])
                        self.cart_d_b_entry.delete(0, tk.END)
                        self.cart_d_b_entry.insert(0, valid_points[5])
                        self.vel_b_entry.delete(0, tk.END)
                        self.vel_b_entry.insert(0, valid_points[6])
                        self.acc_b_entry.delete(0, tk.END)
                        self.acc_b_entry.insert(0, valid_points[7])
                        # messagebox.showinfo("成功", f"从文件导入了 {len(valid_points)} 参数到#2")

                except Exception as e:
                    messagebox.showerror("错误", f"读取文件时出错: {str(e)}")

    def stop_command(self):
        if self.connected:
            robot.soft_stop('AB')
        else:
            messagebox.showerror('error', '请先连接机器人')

    def reset_robot(self, robot_id):
        if self.connected:
            robot.clear_set()
            robot.set_state(arm=robot_id, state=0)  # state=0 下伺服
            robot.send_cmd()
        else:
            messagebox.showerror('error', '请先连接机器人')

    def pvt_mode(self, robot_id):
        if self.connected:
            robot.clear_set()
            robot.set_state(arm=robot_id, state=2)
            robot.send_cmd()
        else:
            messagebox.showerror('error', '请先连接机器人')

    def position_mode(self, robot_id):
        if self.connected:
            robot.clear_set()
            robot.set_state(arm=robot_id, state=1)
            robot.send_cmd()
        else:
            messagebox.showerror('error', '请先连接机器人')

    def cr_state(self, robot_id):
        if self.connected:
            robot.clear_set()
            robot.set_state(arm=robot_id, state=4)
            robot.send_cmd()
        else:
            messagebox.showerror('error', '请先连接机器人')

    def imded_j_mode(self, robot_id):
        if self.connected:
            robot.clear_set()
            robot.set_state(arm=robot_id, state=3)
            robot.set_impedance_type(arm=robot_id, type=1)
            robot.send_cmd()
        else:
            messagebox.showerror('error', '请先连接机器人')

    def imded_c_mode(self, robot_id):
        if self.connected:
            robot.clear_set()
            robot.set_state(arm=robot_id, state=3)
            robot.set_impedance_type(arm=robot_id, type=2)
            robot.send_cmd()
        else:
            messagebox.showerror('error', '请先连接机器人')

    def imded_f_mode(self, robot_id):
        if self.connected:
            directions = [0, 0, 0, 0, 0, 0]
            if robot_id == 'A':

                force_a = float(self.f_a_entry.get())
                adjustment_a = float(self.f_a_adj_entry.get())
                selected_axis_a = self.axis_combobox_a.get()
                if selected_axis_a == 'X':
                    directions[0] = 1
                elif selected_axis_a == 'Y':
                    directions[1] = 1
                elif selected_axis_a == 'Z':
                    directions[2] = 1

                robot.clear_set()
                robot.set_force_control_params(arm=robot_id, fcType=0, fxDirection=directions,
                                               fcCtrlpara=[0, 0, 0, 0, 0, 0, 0],
                                               fcAdjLmt=adjustment_a)
                robot.set_force_cmd(arm=robot_id, f=force_a)
                robot.set_state(arm=robot_id, state=3)
                robot.set_impedance_type(arm=robot_id, type=3)
                robot.send_cmd()

            elif robot_id == 'B':
                force_b = float(self.f_b_entry.get())
                adjustment_b = float(self.f_b_adj_entry.get())
                selected_axis_b = self.axis_combobox_b.get()
                if selected_axis_b == 'X':
                    directions[0] = 1
                elif selected_axis_b == 'Y':
                    directions[1] = 1
                elif selected_axis_b == 'Z':
                    directions[2] = 1

                robot.clear_set()
                robot.set_force_control_params(arm=robot_id, fcType=0, fxDirection=directions,
                                               fcCtrlpara=[0, 0, 0, 0, 0, 0, 0],
                                               fcAdjLmt=adjustment_b)
                robot.set_force_cmd(arm=robot_id, f=force_b)
                robot.set_state(arm=robot_id, state=3)
                robot.set_impedance_type(arm=robot_id, type=3)
                robot.send_cmd()
        else:
            messagebox.showerror('error', '请先连接机器人')

    def drag_j(self, robot_id):
        idx=0
        if robot_id=='A' :
            idx=0
        elif robot_id=='B' :
            idx=1
        if self.result["states"][idx]["cur_state"] == 3 and self.result["inputs"][idx]["imp_type"] == 1:
            robot.clear_set()
            robot.set_drag_space(arm=robot_id, dgType=1)
            robot.send_cmd()
        else:
            messagebox.showerror('error', '请设置为关节阻抗模式后再选择关节拖动')

    def drag_x(self, robot_id):
        idx = 0
        if robot_id == 'A':
            idx = 0
        elif robot_id == 'B':
            idx = 1
        if self.result["states"][idx]["cur_state"] == 3 and self.result["inputs"][idx]["imp_type"] == 2:
            robot.clear_set()
            robot.set_drag_space(arm=robot_id, dgType=2)
            robot.send_cmd()
        else:
            messagebox.showerror('error', '请设置为笛卡尔阻抗模式后再选择笛卡尔X拖动')

    def drag_y(self, robot_id):
        idx = 0
        if robot_id == 'A':
            idx = 0
        elif robot_id == 'B':
            idx = 1
        if self.result["states"][idx]["cur_state"] == 3 and self.result["inputs"][idx]["imp_type"] == 2:
            robot.clear_set()
            robot.set_drag_space(arm=robot_id, dgType=3)
            robot.send_cmd()
        else:
            messagebox.showerror('error', '请设置为笛卡尔阻抗模式后再选择笛卡尔Y拖动')

    def drag_z(self, robot_id):
        idx = 0
        if robot_id == 'A':
            idx = 0
        elif robot_id == 'B':
            idx = 1
        if self.result["states"][idx]["cur_state"] == 3 and self.result["inputs"][idx]["imp_type"] == 2:
            robot.clear_set()
            robot.set_drag_space(arm=robot_id, dgType=4)
            robot.send_cmd()
        else:
            messagebox.showerror('error', '请设置为笛卡尔阻抗模式后再选择笛卡尔Z拖动')

    def drag_r(self, robot_id):
        idx = 0
        if robot_id == 'A':
            idx = 0
        elif robot_id == 'B':
            idx = 1
        if self.result["states"][idx]["cur_state"] == 3 and self.result["inputs"][idx]["imp_type"] == 2:
            robot.clear_set()
            robot.set_drag_space(arm=robot_id, dgType=5)
            robot.send_cmd()
        else:
            messagebox.showerror('error', '请设置为笛卡尔阻抗模式后再选择笛卡尔R拖动')

    def drag_exit(self, robot_id):
        robot.clear_set()
        robot.set_drag_space(arm=robot_id, dgType=0)
        robot.send_cmd()

    def thread_drag_save(self, robot_id):
        """在新线程中执行drag_save"""
        thread = threading.Thread(target=self.drag_save, args=(robot_id))
        thread.daemon = True
        thread.start()

    def drag_save(self, robot_id):
        stage1 = 1
        stage2 = 0
        cols = 7
        rows = 1000000
        idd = 0
        idx = [0, 1, 2, 3, 4, 5, 6,
               0, 0, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0]

        if robot_id == 'A':
            idd = 0
            idx = [0, 1, 2, 3, 4, 5, 6,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0]
        elif robot_id == 'B':
            idd = 1
            idx = [100, 101, 102, 103, 104, 105, 106,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0]

        while stage1 == 1:
            # 检查是否需要停止（可选功能）
            if hasattr(self, '_stop_thread') and self._stop_thread:
                return

            if (self.result["states"][idd]["cur_state"] == 3 and
                    self.result["inputs"][idd]["imp_type"] in [1, 2] and
                    self.result["inputs"][idd]["drag_sp_type"] in [1, 2] and
                    self.result['outputs'][idd]['tip_di'][0] == 1):
                print(f"----{idd},dip:{self.result['outputs'][idd]['tip_di'][0]}")
                robot.clear_set()
                robot.collect_data(targetNum=cols, targetID=idx, recordNum=rows)
                robot.send_cmd()
                time.sleep(0.5)
                stage2 = 1
                stage1 = 0
                break

            time.sleep(0.01)

        while stage2 == 1:
            # 检查是否需要停止（可选功能）
            if hasattr(self, '_stop_thread') and self._stop_thread:
                robot.clear_set()
                robot.stop_collect_data()
                robot.send_cmd()
                return

            if self.result['outputs'][idd]['tip_di'][0] != 1:
                robot.clear_set()
                robot.stop_collect_data()
                robot.send_cmd()
                time.sleep(1)
                stage2 = 0
                break
            time.sleep(0.01)

        # 使用after方法在GUI线程中执行文件对话框和消息框
        self.root.after(0, self._save_data_dialog, robot_id)

    def _save_data_dialog(self, robot_id):
        """在GUI主线程中执行文件保存操作"""
        file_path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt")],
            title="保存拖动轨迹数据"
        )

        if file_path:
            try:
                robot.save_collected_data_to_path(file_path)
                time.sleep(2)
                messagebox.showinfo("成功", f"拖动轨迹数据已保存到: {os.path.basename(file_path)}，\n请退出拖动。")
            except Exception as e:
                messagebox.showerror("错误", f"保存文件时出错: {str(e)}")

    def error_get(self, robot_id):
        if self.connected:
            errors = robot.get_servo_error_code(robot_id)
            print(f'servo error:{errors}')
            if errors:
                messagebox.showinfo(f'{robot_id} arm error', errors)
        else:
            messagebox.showerror('error', '请先连接机器人')

    def error_clear(self, robot_id):
        if self.connected:
            robot.clear_error(robot_id)
        else:
            messagebox.showerror('error', '请先连接机器人')

    def brake(self, robot_id):
        if self.connected:
            if robot_id == 'A':
                robot.set_param('int', 'BRAK0', 1)
            elif robot_id == 'B':
                robot.set_param('int', 'BRAK1', 1)
        else:
            messagebox.showerror('error', '请先连接机器人')

    def release_brake(self, robot_id):
        if self.connected:
            if robot_id == 'A':
                robot.set_param('int', 'BRAK0', 2)
            elif robot_id == 'B':
                robot.set_param('int', 'BRAK1', 2)
        else:
            messagebox.showerror('error', '请先连接机器人')

    def send_pvt(self, robot_id):
        if self.connected:
            file_path = filedialog.askopenfilename(
                title="选择数据文件",
                filetypes=[("文本文件", "*.txt"), ("fmv文件", "*.fmv"), ("所有文件", "*.*")]
            )
            if file_path:
                print(f'pvt file_path:{file_path}')
                if robot_id == 'A':
                    print(f'pvt id:{int(self.pvt_a_entry.get())}')
                    robot.send_pvt_file(arm=robot_id, pvt_path=file_path, id=int(self.pvt_a_entry.get()))
                elif robot_id == 'B':
                    print(f'pvt id:{int(self.pvt_b_entry.get())}')
                    robot.send_pvt_file(arm=robot_id, pvt_path=file_path, id=int(self.pvt_b_entry.get()))
        else:
            messagebox.showerror('error', '请先连接机器人')

    def run_pvt(self, robot_id):
        if self.connected:
            if robot_id == 'A':
                robot.clear_set()
                robot.set_state(arm=robot_id, state=2)  # PVT
                robot.set_pvt_id(arm=robot_id, id=int(self.pvt_a_entry.get()))
                robot.send_cmd()
            elif robot_id == 'B':
                robot.clear_set()
                robot.set_state(arm=robot_id, state=2)  # PVT
                robot.set_pvt_id(arm=robot_id, id=int(self.pvt_b_entry.get()))
                robot.send_cmd()
        else:
            messagebox.showerror('error', '请先连接机器人')

    def tool_set(self, robot_id):
        if self.connected:
            kine_p = 0
            dyn_p = 0
            if robot_id == 'A':
                kine_p = ast.literal_eval(self.tool_a1_entry.get())
                dyn_p = ast.literal_eval(self.tool_a_entry.get())
                # print(f'a:  kine_p:{kine_p}, dyn_p:{dyn_p}')
            elif robot_id == 'B':
                kine_p = ast.literal_eval(self.tool_b1_entry.get())
                dyn_p = ast.literal_eval(self.tool_b_entry.get())
                # print(f'b:  kine_p:{kine_p}, dyn_p:{dyn_p}')
            if not kine_p:
                messagebox.showerror("错误", "工具运动学参数不能为空！")
            if len(kine_p) != 6:
                messagebox.showerror("错误", f"工具运动学必须为6个，当前有{len(kine_p)}个数据！")
            try:
                kine_p = [float(item) for item in kine_p]
            except ValueError:
                messagebox.showerror("错误", "工具运动学参数必须是有效的数值！")

            if not dyn_p:
                messagebox.showerror("错误", "工具动力学参数不能为空！")
            if len(dyn_p) != 10:
                messagebox.showerror("错误", f"工具动力学参数必须为10个，当前有{len(dyn_p)}个数据！")
            try:
                dyn_p = [float(item) for item in dyn_p]
            except ValueError:
                messagebox.showerror("错误", "工具动力学参数必须是有效的数值！")
            robot.clear_set()
            robot.set_tool(arm=robot_id, kineParams=kine_p, dynamicParams=dyn_p)
            robot.send_cmd()

            tool_mat = kk1.xyzabc_to_mat4x4(xyzabc=kine_p)
            if robot_id == "A":
                kk1.set_tool_kine(robot_serial=0, tool_mat=tool_mat)
            elif robot_id == "B":
                kk2.set_tool_kine(robot_serial=1, tool_mat=tool_mat)

            '''save in txt and send it to controller'''
            if not self.tool_result:
                lines = ['0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n',
                         '0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n']
                # 写回文件
                with open(self.tools_txt, 'w', encoding='utf-8') as file:
                    file.writelines(lines)
                file.close()
            else:
                from python.fx_robot import update_text_file_simple
                full_tool = dyn_p + kine_p
                update_text_file_simple(robot_id, full_tool, self.tools_txt)
                robot.send_file(self.tools_txt, os.path.join('/home/fusion/', self.tools_txt))
                time.sleep(1)
        else:
            messagebox.showerror('error', '请先连接机器人')

    def vel_acc_set(self, robot_id):
        if self.connected:
            vel = acc = 0
            if robot_id == 'A':
                vel = int(self.vel_a_entry.get())
                acc = int(self.acc_a_entry.get())
            elif robot_id == 'B':
                vel = int(self.vel_b_entry.get())
                acc = int(self.acc_b_entry.get())

            robot.clear_set()
            robot.set_vel_acc(arm=robot_id, velRatio=vel, AccRatio=acc)
            robot.send_cmd()
        else:
            messagebox.showerror('error', '请先连接机器人')

    def joint_kd_set(self, robot_id):
        if self.connected:
            k = 0
            d = 0
            if robot_id == 'A':
                k = ast.literal_eval(self.k_a_entry.get())
                d = ast.literal_eval(self.d_a_entry.get())
            elif robot_id == 'B':
                k = ast.literal_eval(self.k_b_entry.get())
                d = ast.literal_eval(self.d_b_entry.get())
            if not k:
                messagebox.showerror("错误", "关节K参数不能为空！")
            if len(k) != 7:
                messagebox.showerror("错误", f"关节K参数必须为7个，当前有{len(k)}个数据！")
            try:
                k = [float(item) for item in k]
            except ValueError:
                messagebox.showerror("错误", "关节K参数必须是有效的数值！")

            if not d:
                messagebox.showerror("错误", "关节D参数不能为空！")
            if len(d) != 7:
                messagebox.showerror("错误", f"关节D参数必须为7个，当前有{len(d)}个数据！")
            try:
                d = [float(item) for item in d]
            except ValueError:
                messagebox.showerror("错误", "关节D参数必须是有效的数值！")
            robot.clear_set()
            robot.set_joint_kd_params(arm=robot_id, K=k, D=d)
            robot.send_cmd()
        else:
            messagebox.showerror('error', '请先连接机器人')

    def cart_kd_set(self, robot_id):
        if self.connected:
            k = 0
            d = 0
            type = 0
            if robot_id == 'A':
                k = ast.literal_eval(self.cart_k_a_entry.get())
                d = ast.literal_eval(self.cart_d_a_entry.get())
                type = int(self.imped_a_entry.get())
            elif robot_id == 'B':
                k = ast.literal_eval(self.cart_k_b_entry.get())
                d = ast.literal_eval(self.cart_d_b_entry.get())
                type = int(self.imped_b_entry.get())

            if not k:
                messagebox.showerror("错误", "笛卡尔K参数不能为空！")
            if len(k) != 7:
                messagebox.showerror("错误", f"笛卡尔K参数必须为7个，当前有{len(k)}个数据！")
            try:
                k = [float(item) for item in k]
            except ValueError:
                messagebox.showerror("错误", "笛卡尔K参数必须是有效的数值！")

            if not d:
                messagebox.showerror("错误", "笛卡尔D参数不能为空！")
            if len(d) != 7:
                messagebox.showerror("错误", f"笛卡尔D参数必须为7个，当前有{len(d)}个数据！")
            try:
                d = [float(item) for item in d]
            except ValueError:
                messagebox.showerror("错误", "笛卡尔D参数必须是有效的数值！")
            robot.clear_set()
            robot.set_cart_kd_params(arm=robot_id, K=k, D=d, type=type)
            robot.send_cmd()
        else:
            messagebox.showerror('error', '请先连接机器人')

    def thread_collect_tool_data_no_load(self, robot_id):
        """在新线程中执行collect_tool_data_no_load"""
        thread = threading.Thread(target=self.collect_tool_data_no_load, args=(robot_id))
        thread.daemon = True
        thread.start()

    def collect_tool_data_no_load(self, robot_id):
        if self.connected:
            folder_path = filedialog.askdirectory(
                title="选择保存辨识数据的文件夹",
                mustexist=True
            )

            if folder_path:
                pvt_file = self.file_path_tool.get()
                robot.send_pvt_file(robot_id, pvt_file, 97)
                time.sleep(0.5)

                '''机器人运动前开始设置保存数据'''
                cols = 15
                if robot_id == 'A':
                    idx = [0, 1, 2, 3, 4, 5, 6,
                           50, 51, 52, 53, 54, 55, 56,
                           76, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0]
                elif robot_id == 'B':
                    idx = [100, 101, 102, 103, 104, 105, 106,
                           150, 151, 152, 153, 154, 155, 156,
                           176, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0]
                else:
                    raise ValueError('wrong robot_id')
                rows = 1000000
                robot.clear_set()
                robot.collect_data(targetNum=cols, targetID=idx, recordNum=rows)
                robot.send_cmd()
                time.sleep(0.5)

                '''设置运行的PVT 号'''
                robot.clear_set()
                robot.set_pvt_id(robot_id, 97)
                robot.send_cmd()

                time.sleep(60)  # 模拟跑轨迹时间

                '''停止采集'''
                robot.stop_collect_data()
                time.sleep(0.5)

                '''保存采集数据'''
                save_pvt_path = os.path.join(folder_path, 'pvt.txt')
                robot.save_collected_data_to_path(save_pvt_path)

                time.sleep(1)

                '''数据预处理'''
                processed_data = []
                with open(save_pvt_path, 'r') as file:
                    lines = file.readlines()
                    # 删除首行
                lines = lines[1:]
                for i, line in enumerate(lines):
                    # 移除行末的换行符并按'$'分割
                    parts = line.strip().split('$')
                    # 提取每个字段的数字部分（去掉非数字前缀）
                    numbers = []
                    for part in parts:
                        if part:  # 忽略空字符串
                            # 找到最后一个空格后的数字部分
                            num_str = part.split()[-1]
                            numbers.append(num_str)

                    # 删除前两列（索引0和1），保留剩余列
                    if len(numbers) >= 2:
                        numbers = numbers[2:]
                    processed_data.append(numbers)
                time.sleep(0.5)
                os.remove(save_pvt_path)
                time.sleep(0.5)
                save_csv_path = os.path.join(folder_path, 'NoLoadData.csv')
                with open(save_csv_path, 'w') as out_file:
                    for row in processed_data:
                        out_file.write(','.join(row) + '\n')
                out_file.close()
                messagebox.showinfo('success', f'成功保存{robot_id}臂空载辨识数据')
        else:
            messagebox.showerror('error', '请先连接机器人')

    def thread_collect_tool_data_with_load(self, robot_id):
        """在新线程中执行collect_tool_data_with_load"""
        thread = threading.Thread(target=self.collect_tool_data_with_load, args=(robot_id))
        thread.daemon = True
        thread.start()

    def collect_tool_data_with_load(self, robot_id):
        if self.connected:
            folder_path = filedialog.askdirectory(
                title="选择保存辨识数据的文件夹",
                mustexist=True
            )

            if folder_path:
                pvt_file = self.file_path_tool.get()
                robot.send_pvt_file(robot_id, pvt_file, 97)
                time.sleep(0.5)

                '''机器人运动前开始设置保存数据'''
                cols = 15
                if robot_id == 'A':
                    idx = [0, 1, 2, 3, 4, 5, 6,
                           50, 51, 52, 53, 54, 55, 56,
                           76, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0]
                elif robot_id == 'B':
                    idx = [100, 101, 102, 103, 104, 105, 106,
                           150, 151, 152, 153, 154, 155, 156,
                           176, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0]
                else:
                    raise ValueError('wrong robot_id')
                rows = 1000000
                robot.clear_set()
                robot.collect_data(targetNum=cols, targetID=idx, recordNum=rows)
                robot.send_cmd()
                time.sleep(0.5)

                '''设置运行的PVT 号'''
                robot.clear_set()
                robot.set_pvt_id(robot_id, 97)
                robot.send_cmd()

                time.sleep(60)  # 模拟跑轨迹时间

                '''停止采集'''
                robot.stop_collect_data()
                time.sleep(0.5)

                '''保存采集数据'''
                save_pvt_path = os.path.join(folder_path, 'pvt.txt')
                robot.save_collected_data_to_path(save_pvt_path)

                time.sleep(1)

                '''数据预处理'''
                processed_data = []
                with open(save_pvt_path, 'r') as file:
                    lines = file.readlines()
                    # 删除首行
                lines = lines[1:]
                for i, line in enumerate(lines):
                    # 移除行末的换行符并按'$'分割
                    parts = line.strip().split('$')
                    # 提取每个字段的数字部分（去掉非数字前缀）
                    numbers = []
                    for part in parts:
                        if part:  # 忽略空字符串
                            # 找到最后一个空格后的数字部分
                            num_str = part.split()[-1]
                            numbers.append(num_str)

                    # 删除前两列（索引0和1），保留剩余列
                    if len(numbers) >= 2:
                        numbers = numbers[2:]
                    processed_data.append(numbers)
                time.sleep(0.5)
                os.remove(save_pvt_path)
                time.sleep(0.5)
                save_csv_path = os.path.join(folder_path, 'LoadData.csv')
                with open(save_csv_path, 'w') as out_file:
                    for row in processed_data:
                        out_file.write(','.join(row) + '\n')
                out_file.close()
                messagebox.showinfo('success', f'成功保存{robot_id}臂带载辨识数据')
        else:
            messagebox.showerror('error', '请先连接机器人')

    def tool_dyn_identy(self):
        # 格式化数据
        def format_vector6(vector):
            return ", ".join([f"{v:.6f}" for v in vector])

        print(f"ccs srs:{self.type_select_combobox_1.get()}")
        print(f"tool data:{self.save_tool_data_path}")
        if self.type_select_combobox_1.get() == 'CCS':
            tool_identy_tag, identy_results = kk1.identify_tool_dyn(robot_type=1, ipath=self.save_tool_data_path)
            print(f'tool_identy_tag:{tool_identy_tag}, identy_results:{identy_results}')
            if tool_identy_tag == False:
                messagebox.showerror('wrong', f'工具动力学参数辨识错误提示:{identy_results}')
            if tool_identy_tag:
                self.entry_tool_dyn.set(format_vector6(identy_results))
                messagebox.showinfo('success', '工具动力学参数辨识完成')

        else:
            tool_identy_tag, identy_results = kk1.identify_tool_dyn(robot_type=2, ipath=self.save_tool_data_path)
            print(f'tool_identy_tag:{tool_identy_tag}, identy_results:{identy_results}')
            if tool_identy_tag == False:
                messagebox.showerror('wrong', f'工具动力学参数辨识错误提示:{identy_results}')
            else:
                self.entry_tool_dyn.set(format_vector6(identy_results))
                messagebox.showinfo('success', '工具动力学参数辨识完成')

    def data_clear_preprocess(self, input, output):
        save_list = []
        with open(input, 'r') as file:
            lines = file.readlines()
        # 删除首行
        lines = lines[1:]
        for i, line in enumerate(lines):
            # 移除行末的换行符并按'$'分割
            parts = line.strip().split('$')
            # 提取每个字段的数字部分（去掉非数字前缀）
            numbers = []
            for part in parts:
                if part:  # 忽略空字符串
                    # 找到最后一个空格后的数字部分
                    num_str = part.split()[-1]
                    numbers.append(num_str)

            # 删除前两列（索引0和1），保留剩余列
            if len(numbers) >= 2:
                numbers = numbers[2:]
            save_list.append(numbers)
        with open(output, 'w') as out_file:
            for row in self.save_list:
                out_file.write(' '.join(row) + '\n')

    def collect_data_both(self):
        if self.connected:
            robot.clear_set()
            cols = 14
            idx = [0, 1, 2, 3, 4, 5, 6,
                   100, 101, 102, 103, 104, 105, 106,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0]
            rows = 100000
            robot.collect_data(targetNum=cols, targetID=idx, recordNum=rows)
            robot.send_cmd()
        else:
            messagebox.showerror('error', '请先连接机器人')

    def stop_collect_data_both(self):
        if self.connected:
            robot.clear_set()
            robot.stop_collect_data()
            robot.send_cmd()
        else:
            messagebox.showerror('error', '请先连接机器人')

    def save_collect_data_both(self):
        if self.connected:
            file_path = filedialog.asksaveasfilename(
                defaultextension=".txt",
                filetypes=[("Text files", "*.txt")],
                title="保存双臂运动数据"
            )
            if file_path:
                try:
                    robot.save_collected_data_to_path(file_path)
                    # messagebox.showinfo("成功", f"双臂运动数据已保存到: {os.path.basename(file_path)}")
                except Exception as e:
                    messagebox.showerror("错误", f"保存文件时出错: {str(e)}")
        else:
            messagebox.showerror('error', '请先连接机器人')

    def collect_data(self, robot_id):
        if self.connected:
            cols = 0
            idx = 0
            rows = 0
            if robot_id == 'A':
                cols = int(self.features_entry_1.get())
                idx = ast.literal_eval(self.feature_idx_entry_1.get())
                rows = int(self.lines_entry_1.get())

            if robot_id == 'B':
                cols = int(self.features_entry_2.get())
                idx = ast.literal_eval(self.feature_idx_entry_2.get())
                rows = int(self.lines_entry_2.get())
            if cols > 35:
                messagebox.showerror("错误", f"采集特征参数不能超过35个！")
            if len(idx) != 35:
                messagebox.showerror("错误", f"采集特征参数必须为35个，当前有{idx}个！")
            if 1000000 < rows:
                rows = 1000000
                messagebox.showerror("错误", f"数据最多采集一百万行，已设置为1000000")
            if rows < 1000:
                rows = 1000
                messagebox.showerror("错误", f"数据至少采集一千行，已设置为1000")
            robot.clear_set()
            robot.collect_data(targetNum=cols, targetID=idx, recordNum=rows)
            robot.send_cmd()
        else:
            messagebox.showerror('error', '请先连接机器人')

    def tool_trajectory(self):
        file_path = filedialog.askopenfilename(
            defaultextension=".fmv",
            filetypes=[("fmv files", "*.fmv"), ("All files", "*.*")],
            title="选择工具辨识的激励轨迹文件"
        )
        if file_path:
            self.save_tool_data_path = file_path.split('IdenTraj')[0]
            self.file_path_tool.set(file_path)

    def select_50_file(self):
        file_path = filedialog.askopenfilename(
            defaultextension=".txt",
            filetypes=[("txt files", "*.txt"), ("All files", "*.*")],
            title="选择下采样数据文件"
        )
        if file_path:
            self.file_path_50.set(file_path)
            # messagebox.showinfo("成功", f"下采样数据文件已选择: {os.path.basename(file_path)}")

            with open(file_path, 'r') as file:
                lines = file.readlines()
            # 删除首行
            lines = lines[1:]
            for i, line in enumerate(lines):
                # 每隔20行采集一次数据 (1KHz -> 50Hz)
                if i % 20 != 0:
                    continue
                # 移除行末的换行符并按'$'分割
                parts = line.strip().split('$')
                # 提取每个字段的数字部分（去掉非数字前缀）
                numbers = []
                for part in parts:
                    if part:  # 忽略空字符串
                        # 找到最后一个空格后的数字部分
                        num_str = part.split()[-1]
                        numbers.append(num_str)

                # 删除前两列（索引0和1），保留剩余列
                if len(numbers) >= 2:
                    numbers = numbers[2:]
                self.processed_data.append(numbers)

    def generate_50_file(self):
        """保存2#列表到TXT文件"""
        if len(self.processed_data) == 0:
            messagebox.showerror("错误", "重采样数据为空，没有内容可保存")

        file_path = filedialog.asksaveasfilename(
            defaultextension=".r50pth",
            filetypes=[("50pth files", "*.r50pth"), ("All files", "*.*")],
            title="保存下采样数据"
        )

        if file_path:
            try:
                # 将处理后的数据写入新文件或进行其他操作
                with open(file_path, 'w') as out_file:
                    for row in self.processed_data:
                        out_file.write(' '.join(row) + '\n')
                # messagebox.showinfo("成功", f"下采样已保存到: {os.path.basename(file_path)}")
            except Exception as e:
                messagebox.showerror("错误", f"保存文件时出错: {str(e)}")

    def get_sensor_offset(self, robot_id):
        if self.connected:
            if robot_id == 'A':
                axis = int(self.axis_select_combobox_1.get())
                data = robot.subscribe(dcss)
                self.m_sq_offset_1 = data['outputs'][0]['fb_joint_sToq'][axis]
                print(f'**** self.m_sq_offset_1:{self.m_sq_offset_1}')
                self.get_offset_entry_1.delete(0, tk.END)
                self.get_offset_entry_1.insert(0, self.m_sq_offset_1)
                # name_f = f"R.A0.L{axis}.BASIC.SensorK"
                # name_i = f"R.A0.L{axis}.BASIC.SensorOffset"
                #
                # re_flag1,i_para = robot.get_param(type='int', paraName=name_i)
                # re_flag2,f_para = robot.get_param(type='float', paraName=name_f)
                # if re_flag1==0 and re_flag2==0:
                #     temp="".join(f"{i_para * f_para:.2f}")
                #     self.get_offset_entry_1.delete(0, tk.END)
                #     self.get_offset_entry_1.insert(0, temp)
                # else:
                #     messagebox.showerror("error","获取参数错误")

            if robot_id == 'B':
                axis = int(self.axis_select_combobox_2.get())
                data = robot.subscribe(dcss)
                self.m_sq_offset_2 = data['outputs'][1]['fb_joint_sToq'][axis]
                print(f'**** self.m_sq_offset_2:{self.m_sq_offset_2}')
                self.get_offset_entry_2.delete(0, tk.END)
                self.get_offset_entry_2.insert(0, self.m_sq_offset_2)
                # name_f = f"R.A1.L{axis}.BASIC.SensorK"
                # name_i = f"R.A1.L{axis}.BASIC.SensorOffset"
                # re_flag1,i_para = robot.get_param(type='int', paraName=name_i)
                # re_flag2,f_para = robot.get_param(type='float', paraName=name_f)
                # if re_flag1 == 0 and re_flag2 == 0:
                #     print(f' *** int:{i_para}, float:{f_para}')
                #     temp="".join(f"{i_para * f_para:.2f}")
                #     self.get_offset_entry_2.delete(0, tk.END)
                #     self.get_offset_entry_2.insert(0, temp)
                # else:
                #     messagebox.showerror("error","获取参数错误")
        else:
            messagebox.showerror('error', '请先连接机器人')

    def set_sensor_offset(self, robot_id):  # todo
        if self.connected:
            if robot_id == 'A':
                axis = int(self.axis_select_combobox_1.get())
                name_f = f"R.A0.L{axis}.BASIC.SensorK"
                re_flag,m_sk = robot.get_param(type='float', paraName=name_f)
                if re_flag==0:
                    print(f' *** get m_sk:{m_sk}')
                    m_soft=float(self.get_offset_entry_1.get())/m_sk
                    print(f'**** set senor value:{m_soft}')
                    name_i = f"R.A0.L{axis}.BASIC.SensorOffset"
                    robot.set_param(type='int', paraName=name_i, value=m_soft)
                    re_flag__=robot.save_para_file()
                    if re_flag__!=0:
                        messagebox.showerror("error","保存参数失败")

            elif robot_id == 'B':
                axis = int(self.axis_select_combobox_2.get())
                name_f = f"R.A1.L{axis}.BASIC.SensorK"
                re_flag,m_sk = robot.get_param(type='float', paraName=name_f)
                if re_flag==0:
                    print(f' *** get m_sk:{m_sk}')
                    m_soft = float(self.get_offset_entry_2.get()) / m_sk
                    print(f'**** set senor value:{m_soft}')
                    name_i = f"R.A1.L{axis}.BASIC.SensorOffset"
                    robot.set_param(type='int', paraName=name_i, value=m_soft)
                    re_flag__ = robot.save_para_file()
                    if re_flag__ != 0:
                        messagebox.showerror("error", "保存参数失败")

        else:
            messagebox.showerror('error', '请先连接机器人')

    def clear_motor_as_zero(self, robot_id):
        if self.connected:
            if robot_id == 'A':
                if self.result['states'][0]["cur_state"] != 0:
                    messagebox.showerror('error', '左臂必须在复位状态才可电机编码器清零')
                else:
                    axis = int(self.motor_axis_select_combobox_1.get())
                    robot.set_param(type='int', paraName="RESETMOTENC0", value=axis)
            elif robot_id == 'B':
                if self.result['states'][1]["cur_state"] != 0:
                    messagebox.showerror('error', '右臂必须在复位状态才可电机编码器清零')
                else:
                    axis = int(self.motor_axis_select_combobox_11.get())
                    robot.set_param(type='int', paraName="RESETMOTENC1", value=axis)
        else:
            messagebox.showerror('error', '请先连接机器人')

    def clear_motorE_as_zero(self, robot_id):
        if self.connected:
            if robot_id == 'A':
                if self.result['states'][0]["cur_state"] != 0:
                    messagebox.showerror('error', '左臂必须在复位状态才可电机外编码器清零')
                else:
                    axis = int(self.motor_axis_select_combobox_1.get())
                    robot.set_param(type='int', paraName="RESETEXTENC0", value=axis)
            elif robot_id == 'B':
                if self.result['states'][1]["cur_state"] != 0:
                    messagebox.showerror('error', '右臂必须在复位状态才可电机外编码器清零')
                else:
                    axis = int(self.motor_axis_select_combobox_11.get())
                    robot.set_param(type='int', paraName="RESETEXTENC1", value=axis)
        else:
            messagebox.showerror('error', '请先连接机器人')

    def clear_motor_error(self, robot_id):
        if self.connected:
            if robot_id == 'A':
                if self.result['states'][0]["cur_state"] != 0:
                    messagebox.showerror('error', '左臂必须在复位状态才可电机编码器清错')
                else:
                    axis = int(self.motor_axis_select_combobox_1.get())
                    robot.set_param(type='int', paraName="CLEARMOTENC0", value=axis)
            elif robot_id == 'B':
                if self.result['states'][1]["cur_state"] != 0:
                    messagebox.showerror('error', '右臂必须在复位状态才可电机编码器清错')
                else:
                    axis = int(self.motor_axis_select_combobox_11.get())
                    robot.set_param(type='int', paraName="CLEARMOTENC1", value=axis)
        else:
            messagebox.showerror('error', '请先连接机器人')

    def send_data_eef(self, robot_id):
        if self.connected:
            try:
                com = 0
                com_ = ''
                sample_data = None
                robot.clear_485_cache(robot_id)
                time.sleep(0.5)
                if robot_id == 'A':
                    sample_data = self.com_entry_1.get()
                    com_ = self.com_select_combobox_1.get()
                elif robot_id == 'B':
                    sample_data = self.com_entry_2.get()
                    com_ = self.com_select_combobox_2.get()

                # 1：‘C’端; 2：com1; 3:com2
                if com_ == 'CAN':
                    com = 1
                elif com_ == 'COM1':
                    com = 2
                elif com_ == 'COM2':
                    com = 3
                # print(f'com:{com}')
                success, sdk_return = robot.set_485_data(robot_id, sample_data, len(sample_data), com)
                received_count, received_data = get_received_data()
                if received_data>0:
                    print(f'received_count:{received_count},  eef received:{received_data[0]}')
                if robot_id == 'A':
                    self.recv_text1.delete('1.0', tk.END)
                    self.recv_text1.insert(tk.END, received_data[0])
                if robot_id == 'B':
                    self.recv_text2.delete('1.0', tk.END)
                    self.recv_text2.insert(tk.END, received_data[0])
                if not success:
                    messagebox.showerror('error', f'send data must be hex string of bytes string')
            except Exception as e:
                messagebox.showerror('error', e)
        else:
            messagebox.showerror('error', '请先连接机器人')

    def receive_data_eef(self, robot_id):
        if self.connected:
            try:
                robot.clear_485_cache(robot_id)
                com = 0
                com_ = ''
                if robot_id == 'A':
                    com_ = self.com_select_combobox_1.get()
                elif robot_id == 'B':
                    com_ = self.com_select_combobox_2.get()

                # 1：‘C’端; 2：com1; 3:com2
                if com_ == 'CAN':
                    com = 1
                elif com_ == 'COM1':
                    com = 2
                elif com_ == 'COM2':
                    com = 3
                self.eef_thread=threading.Thread(target=read_data, args=(robot_id, com), daemon=True)
                self.eef_thread.start()

                received_count, received_data = get_received_data()
                if received_data > 0:
                    print(f'received_count:{received_count},  eef received:{received_data[0]}')
                if robot_id == 'A':
                    self.recv_text1.delete('1.0', tk.END)
                    self.recv_text1.insert(tk.END, received_data[0])
                if robot_id == 'B':
                    self.recv_text2.delete('1.0', tk.END)
                    self.recv_text2.insert(tk.END, received_data[0])
            except Exception as e:
                messagebox.showerror('error', e)
        else:
            messagebox.showerror('error', '请先连接机器人')


# 启动应用
if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
