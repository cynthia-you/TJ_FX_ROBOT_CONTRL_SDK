import tkinter as tk
from tkinter import messagebox, ttk, scrolledtext, filedialog, simpledialog
from PIL import Image, ImageTk
import threading
import time
import random
import queue
import os
import glob
import math
import sys
from python.fx_robot import Marvin_Robot, DCSS, arm_err_code
from python.fx_kine import Marvin_Kine
import ast
from PIL import Image, ImageDraw, ImageTk
from pathlib import Path
import difflib
import re

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
            time.sleep(0.2)  # 每0.2秒更新一次

    def stop(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)

class EmergencyStopButton:
    """急停按钮类"""
    def __init__(self, parent, radius=40, command=None, reset_command=None):
        self.parent = parent
        self.radius = radius
        self.command = command
        self.is_stopped = False
        self.stop_command = command  # 急停回调（使用原有参数名command）
        # self.reset_command = reset_command  # 复位回调
        # 创建按钮画布
        self.canvas = tk.Canvas(
            parent,
            width=radius * 2 + 10,
            height=radius * 2 + 10,
            bg='white',
            highlightthickness=0
        )
        self.canvas.pack()

        # 创建默认状态的按钮图像
        self.create_button_image()
        # 绑定点击事件
        self.canvas.bind("<Button-1>", self.on_click)

    def create_button_image(self):
        """创建按钮图像"""
        # 创建PIL图像
        img_size = (self.radius * 2 + 10, self.radius * 2 + 10)
        image = Image.new('RGBA', img_size, (255, 255, 255, 0))
        draw = ImageDraw.Draw(image)

        # 按钮位置
        center_x = img_size[0] // 2
        center_y = img_size[1] // 2

        # 绘制红色圆形（急停按钮主体）
        if self.is_stopped:
            # 已停止状态 - 灰色
            draw.ellipse(
                [center_x - self.radius, center_y - self.radius,
                 center_x + self.radius, center_y + self.radius],
                fill='#808080', outline='#404040', width=3
            )
        else:
            # 正常状态 - 红色
            draw.ellipse(
                [center_x - self.radius, center_y - self.radius,
                 center_x + self.radius, center_y + self.radius],
                fill='#ff4444', outline='#cc0000', width=3
            )

        # 绘制白色边框和阴影效果
        draw.ellipse(
            [center_x - self.radius + 2, center_y - self.radius + 2,
             center_x + self.radius - 2, center_y + self.radius - 2],
            outline='white', width=2
        )

        # 绘制"STOP"文字
        # 这里需要先创建一个临时的draw来测量文字大小
        temp_draw = ImageDraw.Draw(image)
        # 绘制白色STOP文字
        font_size = self.radius // 2
        try:
            from PIL import ImageFont
            font = ImageFont.truetype("arial.ttf", font_size)
        except:
            font = ImageFont.load_default()

        text = "STOP" if not self.is_stopped else "RESET"
        bbox = temp_draw.textbbox((0, 0), text, font=font)
        text_width = bbox[2] - bbox[0]
        text_height = bbox[3] - bbox[1]

        # 绘制文字阴影（深色）
        shadow_offset = 2
        draw.text(
            (center_x - text_width // 2 + shadow_offset,
             center_y - text_height // 2 + shadow_offset),
            text, fill='#880000' if not self.is_stopped else '#404040', font=font
        )
        # 绘制主文字（白色）
        draw.text(
            (center_x - text_width // 2, center_y - text_height // 2),
            text, fill='white', font=font
        )
        # 转换为PhotoImage
        self.photo = ImageTk.PhotoImage(image)
        # 显示图像
        self.canvas.delete("all")
        self.canvas.create_image(
            center_x, center_y,
            image=self.photo,
            anchor=tk.CENTER
        )

    def on_click(self, event):
        """点击事件处理"""
        if not self.is_stopped:
            # 触发急停
            success = False

            if self.stop_command:
                try:
                    success = self.stop_command()  # 执行急停回调
                except Exception as e:
                    print(f"急停回调函数执行失败: {e}")
                    success = False
            else:
                success = True  # 没有回调函数时默认成功

            if success:
                self.is_stopped = True
                self.parent.bell()
                self.create_button_image()
                messagebox.showwarning(
                    "急停触发",
                    "机器人已紧急停止！\n请检查系统安全后再按复位按钮。"
                )
            else:
                self.parent.bell()
                print("急停操作失败，按钮状态未改变")

        else:
            # 复位按钮
            self.is_stopped = False
            self.create_button_image()
            # # 复位按钮
            # success = True  # 默认复位成功
            # if self.reset_command:
            #     try:
            #         success = self.reset_command()  # 执行复位回调
            #     except Exception as e:
            #         print(f"复位回调函数执行失败: {e}")
            #         success = False
            #
            # if success:
            #     self.is_stopped = False
            #     self.create_button_image()

    def reset(self):
        """复位急停按钮"""
        self.is_stopped = False
        self.create_button_image()

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("MarvinPlatform")
        self.root.geometry("1350x800")
        self.root.configure(bg="#f0f0f0")

        self.version = ''
        self.drag_mode = False
        self.tools_txt = 'tool_dyn_kine.txt'
        self.tool_result = None

        # 初始化两个点的列表
        self.points1 = []
        self.points2 = []
        self.command1=[]
        self.command2=[]

        # 初始化参数列表
        self.params = []
        self.processed_data = []
        self.period_file_path_1 = tk.StringVar()
        self.period_file_path_2 = tk.StringVar()
        self.file_path_tool = tk.StringVar()
        self.file_path_50 = tk.StringVar()

        # 文件路径
        self.source_file = "robot.ini"  # 初始文件
        self.target_file = None  # 选择的对比文件

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

        self.display_mode = 0
        self.mode_names = ["位置数据", "速度数据", "传感器数据", "电流数据", "温度数据", "外编位置数据", "指令位置数据",
                           "轴外力数据"]
        self.data_keys = [('fb_joint_pos'), ('fb_joint_vel'), ('fb_joint_sToq'), ('fb_joint_cToq'), ('fb_joint_them'),
                          ('fb_joint_posE'), ('fb_joint_cmd'), ('est_joint_force')]
        # 存储组件引用
        self.widgets = {}
        # 创建控制面板
        self.create_control_components()
        # 创建主内容区域 - 修改为左右布局
        self.create_main_content()
        # 创建左右臂组件（按新布局）
        self.create_left_arm_components()
        self.create_separator()
        self.create_right_arm_components()
        self.create_emergency_stop_components()
        # 创建底部状态栏
        self.create_status_bar()
        # 绑定窗口关闭事件
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        # 密码设置
        self.correct_password = "1"
        # 初始未连接
        self.connected = True
        self.data_subscriber = None
        # 初始化工具参数变量
        self.init_tool_variables()
        # 初始化阻抗参数
        self.init_kd_variables()

        # self.stop_flag = False
        # self.thread = None
        # self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None

    def load_file(self, filepath):
        """加载文件内容"""
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                return f.read()
        except FileNotFoundError:
            messagebox.showerror("错误", f"文件 {filepath} 不存在！")
            return ""
        except Exception as e:
            messagebox.showerror("错误", f"读取文件失败: {str(e)}")
            return ""

    def init_tool_variables(self):
        """初始化工具参数相关的StringVar变量"""
        # 工具动力学参数 (10个值)
        self.tool_a_entry = tk.StringVar(value="0,0,0,0,0,0,0,0,0,0")
        self.tool_b_entry = tk.StringVar(value="0,0,0,0,0,0,0,0,0,0")
        self.entry_tool_dyn = tk.StringVar(value="0,0,0,0,0,0,0,0,0,0")
        # 工具运动学参数 (6个值)
        self.tool_a1_entry = tk.StringVar(value="0,0,0,0,0,0")
        self.tool_b1_entry = tk.StringVar(value="0,0,0,0,0,0")
        # 保存有效值的变量
        self._last_valid_tool_a = "0,0,0,0,0,0,0,0,0,0"
        self._last_valid_tool_b = "0,0,0,0,0,0,0,0,0,0"

    def init_kd_variables(self):
        self.cart_k_b_entry = tk.StringVar(value="2000,2000,2000,60,60,60,20")
        self.cart_k_a_entry = tk.StringVar(value="2000,2000,2000,60,60,60,20")
        self.cart_d_a_entry = tk.StringVar(value="0.4,0.4,0.4,0.4,0.4,0.4,0.4")
        self.cart_d_b_entry = tk.StringVar(value="0.4,0.4,0.4,0.4,0.4,0.4,0.4")
        self.k_a_entry=tk.StringVar(value="2,2,2,1.6,1,1,1")
        self.k_b_entry=tk.StringVar(value="2,2,2,1.6,1,1,1")
        self.d_a_entry=tk.StringVar(value="0.4,0.4,0.4,0.4,0.4,0.4,0.4")
        self.d_b_entry=tk.StringVar(value="0.4,0.4,0.4,0.4,0.4,0.4,0.4")

    def create_main_content(self):
        """创建主内容区域 - 使用左右布局"""
        # 主容器 - 使用左右布局
        self.main_container = tk.Frame(self.root, bg="white", padx=5, pady=10)
        self.main_container.pack(fill="both", expand=True)
        # 在main_container中使用Canvas实现滚动
        self.main_canvas = tk.Canvas(self.main_container, bg="white", highlightthickness=0)
        self.main_scrollbar = ttk.Scrollbar(self.main_container, orient="vertical", command=self.main_canvas.yview)
        # 可滚动的框架
        self.scrollable_frame = tk.Frame(self.main_canvas, bg="white")
        #急停按钮框架
        self.stop_frame=tk.Frame(self.main_canvas,bg='white')
        # 绑定滚动区域
        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: self.main_canvas.configure(scrollregion=self.main_canvas.bbox("all"))
        )
        self.main_canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.main_canvas.configure(yscrollcommand=self.main_scrollbar.set)
        # 布局
        self.main_canvas.pack(side="left", fill="both", expand=True, padx=(0, 5))
        self.main_scrollbar.pack(side="right", fill="y")
        # 鼠标滚轮支持
        self.main_canvas.bind_all("<MouseWheel>", self.on_mousewheel)

    def update_vertical_scrollbar(self, *args):
        """更新垂直滚动条"""
        self.v_scrollbar.set(*args)
        self.main_canvas.yview(*args)

    def update_horizontal_scrollbar(self, *args):
        """更新水平滚动条"""
        self.h_scrollbar.set(*args)
        self.main_canvas.xview(*args)

    def scroll_horizontally(self, *args):
        """水平滚动命令"""
        self.main_canvas.xview(*args)

    def on_mousewheel(self, event):
        """垂直滚动 - 支持Windows/Mac"""
        self.main_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    def on_shift_mousewheel(self, event):
        """水平滚动 - Shift+滚轮"""
        self.main_canvas.xview_scroll(int(-1 * (event.delta / 120)), "units")

    def on_mousewheel(self, event):
        """垂直滚动"""
        if event.delta:
            self.main_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
        else:
            # Linux系统通常使用event.num
            if event.num == 4:
                self.main_canvas.yview_scroll(-1, "units")
            elif event.num == 5:
                self.main_canvas.yview_scroll(1, "units")

    def on_horizontal_mousewheel(self, event):
        """水平滚动"""
        if event.delta:
            self.main_canvas.xview_scroll(int(-1 * (event.delta / 120)), "units")
        else:
            # Linux系统
            if event.num == 4:
                self.main_canvas.xview_scroll(-1, "units")
            elif event.num == 5:
                self.main_canvas.xview_scroll(1, "units")

    def create_separator(self):
        separator = tk.Frame(self.scrollable_frame, height=2, bg="#7F888C")
        separator.pack(fill="x", pady=(5,10))

    def create_emergency_stop_components(self):
        stop_control_frame=tk.Frame(self.stop_frame,bg='white')
        stop_control_frame.pack(fill="x", pady=15)

        # 急停按钮
        self.emergency_btn = EmergencyStopButton(
            stop_control_frame,
            radius=35,
            command=self.emergency_stop_action
            # command=lambda: self.emergency_stop_action,
        )
        self.emergency_btn.canvas.pack(side="left", padx=5, pady=(10, 10))


    def create_left_arm_components(self):
        """创建左臂组件 - 新布局：左状态，中控制，右新增功能"""
        # 左臂主容器
        left_arm_container = tk.Frame(self.scrollable_frame, bg="white", pady=5)
        left_arm_container.pack(fill="x", pady=(0, 5))

        # 左臂内容容器 - 三列布局
        left_content = tk.Frame(left_arm_container, bg="white")
        left_content.pack(fill="x")

        '''第一列：左臂状态'''
        left_status_frame = tk.Frame(left_content, bg="white", width=arm_main_state_with)
        left_status_frame.pack(side="left", fill="y", padx=(0, 10))
        left_status_frame.pack_propagate(False)  # 保持固定宽度

        # 状态标题区域
        status_title_frame = tk.Frame(left_status_frame, bg="white")
        status_title_frame.pack(fill="x", pady=(0, 10))

        # 左臂图标
        try:
            img_left = Image.open('src/left.png')
            img_left = img_left.resize((45, 75), Image.Resampling.LANCZOS)
            arm_image_left = ImageTk.PhotoImage(img_left)
            self.left_arm_image = arm_image_left
            img_label_left = tk.Label(status_title_frame, image=arm_image_left, bg="white")
            img_label_left.pack(side="left", padx=(10, 10), pady=20)
        except:
            # 如果图片不存在，使用文字代替
            tk.Label(status_title_frame, text="左臂", font=('Arial', 12), bg="white").pack(side="left")

        # 状态标签
        tk.Label(status_title_frame, text="左臂", font=('Arial', 12, 'bold'), fg='#2c3e50', bg="white").pack(
            side="left", pady=20)

        # 状态信息区域
        status_info_frame = tk.Frame(left_status_frame, bg="white")
        status_info_frame.pack(fill="both", expand=True)

        # 控制状态
        control_row = tk.Frame(status_info_frame, bg="white")
        control_row.pack(fill="x", pady=(0, 5))
        tk.Label(control_row, text="控制状态:", font=('Arial', 9), fg='#2c3e50', width=10, anchor='e', bg="white").pack(
            side="left", padx=(0, 5))
        self.left_state_main = tk.Label(
            control_row,
            text='下使能',
            font=('Arial', 9),
            fg='#34495e',
            bg='white',
            pady=3,
            anchor='w',
            relief=tk.SUNKEN,
        )
        self.left_state_main.pack(side="left", fill="x", expand=True)

        # 拖动按钮
        drag_row = tk.Frame(status_info_frame, bg="white")
        drag_row.pack(fill="x", pady=(0, 5))
        tk.Label(drag_row, text="拖动按钮:", font=('Arial', 9), fg='#2c3e50', width=10, anchor='e', bg="white").pack(
            side="left", padx=(0, 5))
        self.left_state_1 = tk.Label(
            drag_row,
            text='0',
            font=('Arial', 9),
            fg='#34495e',
            bg='white',
            pady=3,
            anchor='w',
            relief=tk.SUNKEN,
            bd=1
        )
        self.left_state_1.pack(side="left", fill="x", expand=True)

        # 低速标志
        speed_row = tk.Frame(status_info_frame, bg="white")
        speed_row.pack(fill="x", pady=(0, 5))
        tk.Label(speed_row, text="低速标志:", font=('Arial', 9), fg='#2c3e50', width=10, anchor='e', bg="white").pack(
            side="left", padx=(0, 5))
        self.left_state_2 = tk.Label(
            speed_row,
            text='0',
            font=('Arial', 9),
            fg='#34495e',
            bg='white',
            pady=3,
            anchor='w',
            relief=tk.SUNKEN,
            bd=1
        )
        self.left_state_2.pack(side="left", fill="x", expand=True)

        # 错误码
        error_row = tk.Frame(status_info_frame, bg="white")
        error_row.pack(fill="x", pady=(0, 5))
        tk.Label(error_row, text="臂错误码:", font=('Arial', 9), fg='#2c3e50', width=10, anchor='e', bg="white").pack(
            side="left", padx=(0, 5))
        self.left_state_3 = tk.Label(
            error_row,
            text='0',
            font=('Arial', 9),
            fg='#34495e',
            bg='white',
            pady=3,
            anchor='w',
            relief=tk.SUNKEN,
            bd=1
        )
        self.left_state_3.pack(side="left", fill="x", expand=True)

        # 错误内容
        error_detail_row = tk.Frame(status_info_frame, bg="white")
        error_detail_row.pack(fill="x", pady=(0, 5))
        self.left_arm_error = tk.Label(
            error_detail_row,
            text="",
            font=('Arial', 9),
            fg='#2c3e50',
            bg='white',
            pady=5,
            anchor='w',
            wraplength=120,
            justify='left'
        )
        self.left_arm_error.pack(fill="x", padx=5)

        '''第二列：左臂控制功能（状态切换、异常处理、参数设置、机械臂数据）'''
        left_control_frame = tk.Frame(left_content, bg="white",width=300)#不受宽度控制
        left_control_frame.pack(side="left", fill="y", expand=True, padx=(0, 15))

        # 上方：状态切换和异常处理（水平排列）
        left_control_top = tk.Frame(left_control_frame, bg="white")
        left_control_top.pack(fill="x", pady=(0, 10))

        # 左侧：状态切换
        state_switch_frame = ttk.LabelFrame(
            left_control_top,
            text="状态切换",
            padding=10,
            relief=tk.GROOVE,
            borderwidth=2,
            style="MyCustom.TLabelframe"
        )
        state_switch_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))

        # 复位按钮
        self.reset_button = tk.Button(
            state_switch_frame,
            text="复位",
            width=12,
            command=lambda: self.reset_robot_state('A'),
            bg="#2196F3",
            fg="white",
            font=("Arial", 10, "bold")
        )
        self.reset_button.pack(pady=(0, 10))

        # 状态选择下拉框
        state_options = ["关节跟随", "关节阻抗", "笛卡尔阻抗", "PVT", "拖动"]
        self.state_var = tk.StringVar()
        self.state_var.set(state_options[0])
        self.state_combobox = ttk.Combobox(
            state_switch_frame,
            textvariable=self.state_var,
            values=state_options,
            state="readonly",
            width=18
        )
        self.state_combobox.pack()
        self.state_combobox.bind("<<ComboboxSelected>>", lambda e: self.on_state_selected('A'))

        # 右侧：异常处理
        error_handle_frame = ttk.LabelFrame(
            left_control_top,
            text="异常处理",
            padding=10,
            relief=tk.GROOVE,
            borderwidth=2,
            style="MyCustom.TLabelframe"
        )
        error_handle_frame.pack(side="left", fill="both", expand=True)

        # 伺服错误处理按钮
        servo_frame = tk.Frame(error_handle_frame, bg="white")
        servo_frame.pack(fill="x", pady=(0, 10))

        self.clear_servo_error_left_btn = tk.Button(
            servo_frame,
            text="伺服清错",
            width=10,
            command=lambda: self.error_clear('A'),
            bg="white",
            fg="red",
            font=("Arial", 10, "bold"),
            relief=tk.RAISED,
            bd=2
        )
        self.clear_servo_error_left_btn.pack(side="left", padx=(0, 5))

        self.get_servo_error_left_btn = tk.Button(
            servo_frame,
            text="获取错误",
            width=10,
            command=lambda: self.error_get('A'),
            font=("Arial", 10, "bold")
        )
        self.get_servo_error_left_btn.pack(side="left")

        # 协作控制和刹车按钮
        control_frame = tk.Frame(error_handle_frame,bg='white')
        control_frame.pack(fill="x")

        self.release_collab_left_btn = tk.Button(
            control_frame,
            text="协作释放",
            width=10,
            command=lambda: self.cr_state('A'),
            bg="#4CAF50",
            fg="white",
            font=("Arial", 10, "bold")
        )
        self.release_collab_left_btn.pack(side="left", padx=(0, 5))

        self.release_brake_left_btn = tk.Button(
            control_frame,
            text="强制松闸",
            width=10,
            command=lambda: self.release_brake('A'),
            font=("Arial", 10, "bold")
        )
        self.release_brake_left_btn.pack(side="left", padx=(0, 5))

        self.hold_brake_left_btn = tk.Button(
            control_frame,
            text="强制抱闸",
            width=10,
            command=lambda: self.brake('A'),
            font=("Arial", 10, "bold")
        )
        self.hold_brake_left_btn.pack(side="left")

        # 中间：参数设置和机械臂数据（竖直排列）
        left_control_middle = tk.Frame(left_control_frame, bg="white")
        left_control_middle.pack(fill="y", expand=True)

        # 参数设置
        param_frame = ttk.LabelFrame(
            left_control_middle,
            text="参数设置",
            padding=10,
            relief=tk.GROOVE,
            borderwidth=2,
            style="MyCustom.TLabelframe"
        )
        param_frame.pack(fill="x", pady=(0, 10))

        param_row = tk.Frame(param_frame, bg="white")
        param_row.pack(fill="x")

        # 速度设置
        tk.Label(param_row, text="速度:", font=('Arial', 9),bg='white' ).pack(side="left", padx=(0, 2))
        self.left_speed_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.left_speed_entry.pack(side="left")
        self.left_speed_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9),bg='white' ).pack(side="left", padx=(0, 5))

        # 加速度设置
        tk.Label(param_row, text="加速度:", font=('Arial', 9),bg='white'  ).pack(side="left", padx=(0, 2))
        self.left_accel_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.left_accel_entry.pack(side="left")
        self.left_accel_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9),bg='white' ).pack(side="left", padx=(0, 5))
        speed_btn1 = tk.Button(
            param_row,
            text="确认速度和加速度",
            width=15,
            command=lambda: self.vel_acc_set('A'),
            bg="#58C3EE",
            font=("Arial", 9, "bold")
        )
        speed_btn1.pack(side="left", padx=(0, 20))

        # 阻抗参数设置按钮
        self.left_impedance_btn = tk.Button(
            param_row,
            text="阻抗参数设置",
            width=15,
            command=lambda: self.show_impedance_dialog('A'),
            bg="#9C27B0",
            fg="white",
            font=("Arial", 9, "bold")
        )
        self.left_impedance_btn.pack(side="left")

        # 机械臂数据
        data_frame = ttk.LabelFrame(
            left_control_middle,
            text="机械臂实时数据",
            padding=10,
            relief=tk.GROOVE,
            borderwidth=2,
            style="MyCustom.TLabelframe"
        )
        data_frame.pack(fill="x",pady=(0, 10))

        # 关节数据
        tk.Label(data_frame, text="关节J1-J7:", font=('Arial', 10, 'bold'),bg='white').pack(anchor="w", pady=(0, 5))
        joint_frame = tk.Frame(data_frame, bg="white")
        joint_frame.pack(fill="x")

        self.left_joint_text = tk.Text(
            joint_frame,
            width=45,
            height=1,
            font=('Arial', 9),
            bg='white',
            relief=tk.SUNKEN,
            bd=1,
            wrap=tk.NONE
        )
        self.left_joint_text.tag_configure("center", justify='center')
        # self.left_joint_text.pack(fill="x", expand=True)
        self.left_joint_text.pack(fill="both", expand=True)
        self.left_joint_text.insert("1.0", "0.000,0.000,0.000,0.000,0.000,0.000,0.000")
        self.left_joint_text.tag_add("center", "1.0", "end")
        self.left_joint_text.config(state="disabled")

        # 笛卡尔数据
        tk.Label(data_frame, text="笛卡尔XYZABC（末端）:", font=('Arial', 10, 'bold'),bg='white').pack(
            anchor="w", pady=(10, 5))

        cartesian_frame = tk.Frame(data_frame, bg="white")
        cartesian_frame.pack(fill="x", pady=(0, 5))

        self.left_cartesian_text = tk.Text(
            cartesian_frame,
            width=45,
            height=1,
            font=('Arial', 9),
            relief=tk.SUNKEN,
            bd=1,
            wrap=tk.NONE
        )
        self.left_cartesian_text.tag_configure("center", justify='center')
        # self.left_cartesian_text.pack(fill="x", expand=True)
        self.left_cartesian_text.pack(fill="both", expand=True)
        self.left_cartesian_text.insert("1.0", "0.000,0.000,0.000,0.000,0.000,0.000")
        self.left_cartesian_text.tag_add("center", "1.0", "end")
        self.left_cartesian_text.config(state="disabled")


        '''第三列：力控指令和位置指令'''
        left_second_control_frame = tk.Frame(left_content, bg="white", width=520)
        left_second_control_frame.pack(side="left", fill="y")
        left_second_control_frame.pack_propagate(False)  # 保持固定宽度

        # 力控组件
        force_control_frame = ttk.LabelFrame(
            left_second_control_frame,
            text="力控指令",
            padding=10,
            relief=tk.GROOVE,
            borderwidth=2,
            style="MyCustom.TLabelframe"
        )
        force_control_frame.pack(fill="x", pady=(0, 10))

        # 力 输入框 N 调节量 输入框  毫米  方向三选一
        force_row = tk.Frame(force_control_frame,bg='white')
        force_row.pack(fill="x", pady=(0, 5))
        # 力
        tk.Label(force_row, text="力：", font=('Arial', 9),bg='white' ).pack(side="left", padx=(0, 0))
        self.left_force_entry = tk.Entry(force_row, width=5, font=('Arial', 9), justify='center')
        self.left_force_entry.pack(side="left")
        self.left_force_entry.insert(0, "10") #最大不超过60N
        tk.Label(force_row, text="1N-60N", font=('Arial', 9),bg='white').pack(side="left", padx=(0, 10))

        # 调节量
        tk.Label(force_row, text="调节量：", font=('Arial', 9),bg='white' ).pack(side="left", padx=(0, 0))
        self.left_force_adj_entry = tk.Entry(force_row, width=5, font=('Arial', 9), justify='center')
        self.left_force_adj_entry.pack(side="left")
        self.left_force_adj_entry.insert(0, "50") #最大不超过50mm
        tk.Label(force_row, text="1mm-50mm", font=('Arial', 9),bg='white').pack(side="left", padx=(0, 10))

        self.left_force_dir_btn1 = tk.Button(
            force_row,
            text="X方向",
            width=5,
            command=lambda: self.imped_f_mode(0,'A'),
            font=("Arial", 9, "bold"),
            bg='#E2F6FF'
        )
        self.left_force_dir_btn1.pack(side="left",padx=(0,5))

        self.left_force_dir_btn2 = tk.Button(
            force_row,
            text="y方向",
            width=5,
            command=lambda: self.imped_f_mode(1,'A'),
            font=("Arial", 9, "bold"),
            bg='#F6DFF6'
        )
        self.left_force_dir_btn2.pack(side="left",padx=(0,5))

        self.left_force_dir_btn3 = tk.Button(
            force_row,
            text="Z方向",
            width=5,
            command=lambda: self.imped_f_mode(2,'A'),
            font=("Arial", 9, "bold"),
            bg = '#F7F7CE'
        )
        self.left_force_dir_btn3.pack(side="left",padx=(0,5))

        # 关节指令
        joint_cmd_frame = ttk.LabelFrame(
            left_second_control_frame,
            text="位置指令",
            padding=10,
            relief=tk.GROOVE,
            borderwidth=2,
            style="MyCustom.TLabelframe"
        )
        joint_cmd_frame.pack(fill="x")

        # 第一行添加点
        joints_row = tk.Frame(joint_cmd_frame,bg='white')
        joints_row.pack(fill="x", pady=(0, 5))

        # 第四列：1#
        self.btn_add3 = tk.Button(joints_row, text="获取当前", width=8,command=lambda: self.add_current_joints('A'))
        self.btn_add3.pack(side="left", padx=(0, 5))

        # 第二列：输入文本框
        self.entry_var = tk.StringVar(value="0,0,0,0,0,0,0")
        self.entry = tk.Entry(joints_row, textvariable=self.entry_var, width=45)
        self.entry.pack(side="left", padx=(0, 5),expand=True)

        # 第一列：1#加点按钮
        self.btn_add1 = tk.Button(joints_row, text="加点",width=8, command=self.add_point1)
        self.btn_add1.pack(side="left", padx=(0, 5))

        # 第一行运行当前点位
        joints_row1 = tk.Frame(joint_cmd_frame,bg='white')
        joints_row1.pack(fill="x", pady=(0, 5))
        #删除
        self.btn_del1 = tk.Button(joints_row1, text="删除点",width=8, command=self.delete_point1)
        self.btn_del1.pack(side="left", padx=(0, 5))
        #
        self.combo1 = ttk.Combobox(joints_row1, state="readonly", width=45)
        self.combo1.pack(side="left", padx=(0, 5))

        # 第三列：1#运行按钮
        self.btn_run1 = tk.Button(joints_row1, text="运行", width=8,command=lambda :self.run_joints('A'),
                                  font=("Arial", 11, "bold"),fg='white', bg='#EC2A23',border=5)
        self.btn_run1.pack(side="left", padx=(0, 5))

        #第三行 保存导入
        joints_row2 = tk.Frame(joint_cmd_frame,bg='white')
        joints_row2.pack(fill="x", pady=(0, 5))
        # 第四列：1#保存按钮
        self.btn_save1 = tk.Button(joints_row2, text="保存", width=8,command=self.save_points1)
        self.btn_save1.pack(side="left", padx=(180, 5),pady=(0,10))

        # 第五列：1#导入按钮
        self.btn_load1 = tk.Button(joints_row2, text="导入", width=8,command=self.load_points1)
        self.btn_load1.pack(side="left", padx=(0, 5),pady=(0,10))
        # 第四行 轨迹复现
        joints_row3 = tk.Frame(joint_cmd_frame,bg='white')
        joints_row3.pack(fill="x", pady=(0, 5))

        self.run_period_1 = tk.Button(joints_row3, text="轨迹复现", width=8, command=lambda: self.thread_run_period('A'))
        self.run_period_1.pack(side="left", padx=(0, 5))

        self.period_path_entry_1 = tk.Entry(joints_row3, textvariable=self.period_file_path_1, width=45,
                                            font=("Arial", 9), bg='white')
        self.period_path_entry_1.pack(side="left", padx=(0, 5), expand=True)

        self.btn_load_file1 = tk.Button(joints_row3, text="选择文件", width=8,
                                        command=lambda: self.select_period_file('A'))
        self.btn_load_file1.pack(side="left", padx=(0, 5))


        '''第四列：急停'''
        stop_control_frame = tk.Frame(left_content, bg="white",width=800)
        stop_control_frame.pack(side="left", fill="y")
        stop_control_frame.pack_propagate(False)  # 保持固定宽度

        # 急停按钮
        self.left_emergency_btn = EmergencyStopButton(
            stop_control_frame,
            radius=35,
            command=self.emergency_stop_action,
        )
        self.left_emergency_btn.canvas.pack(side="left", padx=5, pady=(10, 10))

    def create_right_arm_components(self):
        """创建右臂组件 - 新布局：左状态，右控制"""
        # 右臂主容器
        right_arm_container = tk.Frame(self.scrollable_frame, bg="white", pady=10)
        right_arm_container.pack(fill="x", pady=(0, 15))

        # 右臂内容容器 - 左右布局
        right_content = tk.Frame(right_arm_container, bg="white")
        right_content.pack(fill="x")

        '''左侧：右臂状态'''
        right_status_frame = tk.Frame(right_content, bg="white", width=arm_main_state_with)
        right_status_frame.pack(side="left", fill="y", padx=(0, 10))
        right_status_frame.pack_propagate(False)  # 保持固定宽度

        # 状态标题区域
        status_title_frame = tk.Frame(right_status_frame, bg="white")
        status_title_frame.pack(fill="x", pady=(0, 10))

        # 右臂图标
        try:
            img_right = Image.open('src/right.png')
            img_right = img_right.resize((45, 75), Image.Resampling.LANCZOS)
            arm_image_right = ImageTk.PhotoImage(img_right)
            self.right_arm_image = arm_image_right
            img_label_right = tk.Label(status_title_frame, image=arm_image_right, bg="white")
            img_label_right.pack(side="left", padx=(10, 10), pady=20)
        except:
            # 如果图片不存在，使用文字代替
            tk.Label(status_title_frame, text="右臂", font=('Arial', 12), bg="white").pack(side="left")

        # 状态标签
        tk.Label(status_title_frame, text="右臂", font=('Arial', 12, 'bold'), fg='#2c3e50', bg="white").pack(
            side="left", pady=20)

        # 状态信息区域
        status_info_frame = tk.Frame(right_status_frame, bg="white")
        status_info_frame.pack(fill="both", expand=True)

        # 控制状态
        control_row = tk.Frame(status_info_frame, bg="white")
        control_row.pack(fill="x", pady=(0, 5))
        tk.Label(control_row, text="控制状态:", font=('Arial', 9), fg='#2c3e50', width=10, anchor='e', bg="white").pack(
            side="left", padx=(0, 5))
        self.right_state_main = tk.Label(
            control_row,
            text='下使能',
            font=('Arial', 9),
            fg='#34495e',
            bg='white',
            pady=3,
            anchor='w',
            relief=tk.SUNKEN,
            bd=1
        )
        self.right_state_main.pack(side="left", fill="x", expand=True)

        # 拖动按钮
        drag_row = tk.Frame(status_info_frame, bg="white")
        drag_row.pack(fill="x", pady=(0, 5))
        tk.Label(drag_row, text="拖动按钮:", font=('Arial', 9), fg='#2c3e50', width=10, anchor='e', bg="white").pack(
            side="left", padx=(0, 5))
        self.right_state_1 = tk.Label(
            drag_row,
            text='0',
            font=('Arial', 9),
            fg='#34495e',
            bg='white',
            pady=3,
            anchor='w',
            relief=tk.SUNKEN,
            bd=1
        )
        self.right_state_1.pack(side="left", fill="x", expand=True)

        # 低速标志
        speed_row = tk.Frame(status_info_frame, bg="white")
        speed_row.pack(fill="x", pady=(0, 5))
        tk.Label(speed_row, text="低速标志:", font=('Arial', 9), fg='#2c3e50', width=10, anchor='e', bg="white").pack(
            side="left", padx=(0, 5))
        self.right_state_2 = tk.Label(
            speed_row,
            text='0',
            font=('Arial', 9),
            fg='#34495e',
            bg='white',
            pady=3,
            anchor='w',
            relief=tk.SUNKEN,
            bd=1
        )
        self.right_state_2.pack(side="left", fill="x", expand=True)

        # 错误码
        error_row = tk.Frame(status_info_frame, bg="white")
        error_row.pack(fill="x", pady=(0, 5))
        tk.Label(error_row, text="臂错误码:", font=('Arial', 9), fg='#2c3e50', width=10, anchor='e', bg="white").pack(
            side="left", padx=(0, 5))
        self.right_state_3 = tk.Label(
            error_row,
            text='1',
            font=('Arial', 9),
            fg='#34495e',
            bg='white',
            pady=3,
            anchor='w',
            relief=tk.SUNKEN,
            bd=1
        )
        self.right_state_3.pack(side="left", fill="x", expand=True)

        # 错误内容
        error_detail_row = tk.Frame(status_info_frame, bg="white")
        error_detail_row.pack(fill="x", pady=(0, 5))
        self.right_arm_error = tk.Label(
            error_detail_row,
            text="",
            font=('Arial', 9),
            fg='#2c3e50',
            bg='white',
            pady=3,
            anchor='w',  # 改成左对齐，换行时会更自然
            wraplength=120,  # 设置自动换行的宽度（像素）
            justify='left'  # 多行文本左对齐
        )
        self.right_arm_error.pack(fill="x", padx=5)

        '''第二列'''
        '''右侧：右臂控制功能'''
        right_control_frame = tk.Frame(right_content, bg="white")
        right_control_frame.pack(side="left", fill="y", expand=True,padx=(0, 15))

        # 右上方：状态切换和异常处理（水平排列）
        right_control_top = tk.Frame(right_control_frame, bg="white")
        right_control_top.pack(fill="x", pady=(0, 10))

        # 左侧：状态切换
        state_switch_frame = ttk.LabelFrame(
            right_control_top,
            text="状态切换",
            padding=10,
            relief=tk.GROOVE,
            borderwidth=2,
            style="MyCustom.TLabelframe"
        )
        state_switch_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))

        # 复位按钮
        self.reset_button_r = tk.Button(
            state_switch_frame,
            text="复位",
            width=12,
            command=lambda: self.reset_robot_state('B'),
            bg="#2196F3",
            fg="white",
            font=("Arial", 10, "bold")
        )
        self.reset_button_r.pack(pady=(0, 10))

        # 状态选择下拉框
        state_options = ["关节跟随", "关节阻抗", "笛卡尔阻抗", "PVT", "拖动"]
        self.state_var_r = tk.StringVar()
        self.state_var_r.set(state_options[0])
        self.state_combobox_r = ttk.Combobox(
            state_switch_frame,
            textvariable=self.state_var_r,
            values=state_options,
            state="readonly",
            width=18
        )
        self.state_combobox_r.pack()
        self.state_combobox_r.bind("<<ComboboxSelected>>", lambda e: self.on_state_selected('B'))

        # 右侧：异常处理
        error_handle_frame = ttk.LabelFrame(
            right_control_top,
            text="异常处理",
            padding=10,
            relief=tk.GROOVE,
            borderwidth=2,
            style="MyCustom.TLabelframe"
        )
        error_handle_frame.pack(side="left", fill="both", expand=True)

        # 伺服错误处理按钮
        servo_frame = tk.Frame(error_handle_frame, bg="white")
        servo_frame.pack(fill="x", pady=(0, 10))

        self.clear_servo_error_right_btn = tk.Button(
            servo_frame,
            text="伺服清错",
            width=10,
            command=lambda: self.error_clear('B'),
            bg="white",
            fg="red",
            font=("Arial", 10, "bold"),
            relief=tk.RAISED,
            bd=2
        )
        self.clear_servo_error_right_btn.pack(side="left", padx=(0, 5))

        self.get_servo_error_right_btn = tk.Button(
            servo_frame,
            text="获取错误",
            width=10,
            command=lambda: self.error_get('B'),
            font=("Arial", 10, "bold")
        )
        self.get_servo_error_right_btn.pack(side="left")

        # 协作控制和刹车按钮
        control_frame = tk.Frame(error_handle_frame,bg='white')
        control_frame.pack(fill="x")

        self.release_collab_right_btn = tk.Button(
            control_frame,
            text="协作释放",
            width=10,
            command=lambda: self.cr_state('B'),
            bg="#4CAF50",
            fg="white",
            font=("Arial", 10, "bold")
        )
        self.release_collab_right_btn.pack(side="left", padx=(0, 5))

        self.release_brake_right_btn = tk.Button(
            control_frame,
            text="强制松闸",
            width=10,
            command=lambda: self.release_brake('B'),
            font=("Arial", 10, "bold")
        )
        self.release_brake_right_btn.pack(side="left", padx=(0, 5))

        self.hold_brake_right_btn = tk.Button(
            control_frame,
            text="强制抱闸",
            width=10,
            command=lambda: self.brake('B'),
            font=("Arial", 10, "bold")
        )
        self.hold_brake_right_btn.pack(side="left")

        # 右下方：参数设置、机械臂数据和新增组件（竖直排列）
        right_control_bottom = tk.Frame(right_control_frame, bg="white")
        right_control_bottom.pack(fill="y", expand=True)

        # 参数设置
        param_frame = ttk.LabelFrame(
            right_control_bottom,
            text="参数设置",
            padding=10,
            relief=tk.GROOVE,
            borderwidth=2,
            style="MyCustom.TLabelframe"
        )
        param_frame.pack(fill="x", pady=(0, 10))

        param_row = tk.Frame(param_frame, bg="white")
        param_row.pack(fill="x")

        # 速度设置
        tk.Label(param_row, text="速度:", font=('Arial', 9),bg='white' ).pack(side="left", padx=(0, 2))
        self.right_speed_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.right_speed_entry.pack(side="left")
        self.right_speed_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9),bg='white' ).pack(side="left", padx=(0, 5))

        # 加速度设置
        tk.Label(param_row, text="加速度:", font=('Arial', 9),bg='white'  ).pack(side="left", padx=(0, 2))
        self.right_accel_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.right_accel_entry.pack(side="left")
        self.right_accel_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9),bg='white' ).pack(side="left", padx=(0, 5))
        speed_btn=tk.Button(
            param_row,
            text="确认速度和加速度",
            width=15,
            command=lambda: self.vel_acc_set('B'),
            bg="#58C3EE",
            font=("Arial", 9, "bold")
        )
        speed_btn.pack(side="left",padx=(0,20))

        # 阻抗参数设置按钮
        self.right_impedance_btn = tk.Button(
            param_row,
            text="阻抗参数设置",
            width=15,
            command=lambda: self.show_impedance_dialog('B'),
            bg="#9C27B0",
            fg="white",
            font=("Arial", 9, "bold")
        )
        self.right_impedance_btn.pack(side="left")

        # 机械臂数据
        data_frame = ttk.LabelFrame(
            right_control_bottom,
            text="机械臂实时数据",
            padding=10,
            relief=tk.GROOVE,
            borderwidth=2,
            style="MyCustom.TLabelframe"
        )
        data_frame.pack(fill="x",pady=(0, 10))

        # 关节数据
        tk.Label(data_frame, text="关节J1-J7:", font=('Arial', 10, 'bold'), bg="white").pack(anchor="w", pady=(0, 5))
        joint_frame = tk.Frame(data_frame, bg="white")
        joint_frame.pack(fill="x")
        self.right_joint_text = tk.Text(
            joint_frame,
            width=45,
            height=1,
            font=('Arial', 9),
            bg='white',
            relief=tk.SUNKEN,
            bd=1,
            wrap=tk.NONE
        )
        self.right_joint_text.tag_configure("center", justify='center')
        # self.right_joint_text.pack(fill="x", expand=True)
        self.right_joint_text.pack(fill="both", expand=True)
        self.right_joint_text.insert("1.0", "0.000,0.000,0.000,0.000,0.000,0.000,0.000")
        self.right_joint_text.tag_add("center", "1.0", "end")
        self.right_joint_text.config(state="disabled")

        # 笛卡尔数据
        tk.Label(data_frame, text="笛卡尔XYZABC（末端）:", font=('Arial', 10, 'bold'), bg="white").pack(
            anchor="w", pady=(10, 5))

        cartesian_frame = tk.Frame(data_frame, bg="white")
        cartesian_frame.pack(fill="x", pady=(0, 5))
        self.right_cartesian_text = tk.Text(
            cartesian_frame,
            width=45,
            height=1,
            font=('Arial', 9),
            bg='white',
            relief=tk.SUNKEN,
            bd=1,
            wrap=tk.NONE
        )
        self.right_cartesian_text.tag_configure("center", justify='center')
        # self.right_cartesian_text.pack(fill="x", expand=True)
        self.right_cartesian_text.pack(fill="both", expand=True)
        self.right_cartesian_text.insert("1.0", "0.000,0.000,0.000,0.000,0.000,0.000")
        self.right_cartesian_text.tag_add("center", "1.0", "end")
        self.right_cartesian_text.config(state="disabled")


        '''第三列：力控指令和位置指令'''
        right_second_control_frame = tk.Frame(right_content, bg="white", width=520)
        right_second_control_frame.pack(side="left", fill="y")
        right_second_control_frame.pack_propagate(False)  # 保持固定宽度

        # 力控组件
        force_control_frame = ttk.LabelFrame(
            right_second_control_frame,
            text="力控指令",
            padding=10,
            relief=tk.GROOVE,
            borderwidth=2,
            style="MyCustom.TLabelframe"
        )
        force_control_frame.pack(fill="x", pady=(0, 10))

        # 力 输入框 N 调节量 输入框  毫米  方向三选一
        force_row = tk.Frame(force_control_frame, bg='white')
        force_row.pack(fill="x", pady=(0, 5))
        # 力
        tk.Label(force_row, text="力：", font=('Arial', 9),bg='white').pack(side="left", padx=(0, 0))
        self.right_force_entry = tk.Entry(force_row, width=5, font=('Arial', 9), justify='center')
        self.right_force_entry.pack(side="left")
        self.right_force_entry.insert(0, "10")  # 最大不超过60N
        tk.Label(force_row, text="1N-60N", font=('Arial', 9),bg='white').pack(side="left", padx=(0, 10))

        # 调节量
        tk.Label(force_row, text="调节量：", font=('Arial', 9),bg='white').pack(side="left", padx=(0, 0))
        self.right_force_adj_entry = tk.Entry(force_row, width=5, font=('Arial', 9), justify='center')
        self.right_force_adj_entry.pack(side="left")
        self.right_force_adj_entry.insert(0, "50")  # 最大不超过50mm
        tk.Label(force_row, text="1mm-50mm", font=('Arial', 9),bg='white').pack(side="left", padx=(0, 10))

        self.right_force_dir_btn1 = tk.Button(
            force_row,
            text="X方向",
            width=5,
            command=lambda: self.imped_f_mode(0, 'B'),
            font=("Arial", 9, "bold"),
            bg='#E2F6FF'
        )
        self.right_force_dir_btn1.pack(side="left", padx=(0, 5))

        self.right_force_dir_btn2 = tk.Button(
            force_row,
            text="y方向",
            width=5,
            command=lambda: self.imped_f_mode(1, 'B'),
            font=("Arial", 9, "bold"),
            bg='#F6DFF6'
        )
        self.right_force_dir_btn2.pack(side="left", padx=(0, 5))

        self.right_force_dir_btn3 = tk.Button(
            force_row,
            text="Z方向",
            width=5,
            command=lambda: self.imped_f_mode(2, 'B'),
            font=("Arial", 9, "bold"),
            bg='#F7F7CE'
        )
        self.right_force_dir_btn3.pack(side="left", padx=(0, 5))

        # 关节指令
        joint_cmd_frame = ttk.LabelFrame(
            right_second_control_frame,
            text="位置指令",
            padding=10,
            relief=tk.GROOVE,
            borderwidth=2,
            style="MyCustom.TLabelframe"
        )
        joint_cmd_frame.pack(fill="x")

        # 第一行添加点
        joints_row = tk.Frame(joint_cmd_frame,bg='white')
        joints_row.pack(fill="x", pady=(0, 5))

        # 第四列：1#
        self.btn_add3 = tk.Button(joints_row, text="获取当前", width=8, command=lambda: self.add_current_joints('B'))
        self.btn_add3.pack(side="left", padx=(0, 5))

        # 第二列：输入文本框
        self.entry_var1 = tk.StringVar(value="0,0,0,0,0,0,0")
        self.entry1 = tk.Entry(joints_row, textvariable=self.entry_var1, width=45)
        self.entry1.pack(side="left", padx=(0, 5), expand=True)

        # 第一列：1#加点按钮
        self.btn_add1 = tk.Button(joints_row, text="加点", width=8, command=self.add_point2)
        self.btn_add1.pack(side="left", padx=(0, 5))

        # 第一行运行当前点位
        joints_row1 = tk.Frame(joint_cmd_frame,bg='white')
        joints_row1.pack(fill="x", pady=(0, 5))

        # 删除
        self.btn_del1 = tk.Button(joints_row1, text="删除点", width=8, command=self.delete_point2)
        self.btn_del1.pack(side="left", padx=(0, 5))
        #
        self.combo2 = ttk.Combobox(joints_row1, state="readonly", width=45)
        self.combo2.pack(side="left", padx=(0, 5))

        # 第三列：1#运行按钮
        self.btn_run2 = tk.Button(joints_row1, text="运行", width=8, command=lambda: self.run_joints('B'),
                                  font=("Arial", 11, "bold"),fg='white', bg='#EC2A23',border=5)
        self.btn_run2.pack(side="left", padx=(0, 5))

        # 第三行 保存导入
        joints_row2 = tk.Frame(joint_cmd_frame,bg='white')
        joints_row2.pack(fill="x", pady=(0, 5))

        # 第四列：1#保存按钮
        self.btn_save2 = tk.Button(joints_row2, text="保存", width=8, command=self.save_points2)
        self.btn_save2.pack(side="left", padx=(180, 5),pady=(0,10))

        # 第五列：1#导入按钮
        self.btn_load2 = tk.Button(joints_row2, text="导入", width=8, command=self.load_points2)
        self.btn_load2.pack(side="left", padx=(0, 5),pady=(0,10))

        # 第四行 轨迹复现
        joints_row3 = tk.Frame(joint_cmd_frame,bg='white')
        joints_row3.pack(fill="x", pady=(0, 5))

        self.run_period_2 = tk.Button(joints_row3, text="轨迹复现",width=8,  command=lambda: self.thread_run_period('B'))
        self.run_period_2.pack(side="left", padx=(0, 5))

        self.period_path_entry_2 = tk.Entry(joints_row3, textvariable=self.period_file_path_2, width=45,
                                            font=("Arial", 9), bg='white')
        self.period_path_entry_2.pack(side="left", padx=(0, 5),expand=True)

        self.btn_load_file2 = tk.Button(joints_row3, text="选择文件",width=8,  command=lambda: self.select_period_file('B'))
        self.btn_load_file2.pack(side="left", padx=(0, 5))

        '''第四列：急停'''
        stop_control_frame = tk.Frame(right_content, bg="white",width=800)
        stop_control_frame.pack(side="left", fill="y")
        stop_control_frame.pack_propagate(False)  # 保持固定宽度

        # 急停按钮
        self.right_emergency_btn = EmergencyStopButton(
            stop_control_frame,
            radius=35,
            command=self.emergency_stop_action,
        )
        self.right_emergency_btn.canvas.pack(side="left", padx=5, pady=(10, 10))

    def show_more_features(self):
        """显示更多功能菜单"""
        # 创建菜单
        menu = tk.Menu(self.root, tearoff=0)
        # 添加菜单项
        menu.add_command(label="系统升级", command=self.system_update_dialog)
        menu.add_separator()
        menu.add_command(label="传感器与编码器", command=self.sensor_decoder_dialog)
        menu.add_separator()
        menu.add_command(label="IMU计算", command=self.additional_settings)
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

    def sensor_decoder_dialog(self):
        """显示隐藏功能选择窗口"""
        button_w = 10
        sensor_encoder_window = tk.Toplevel(self.root)
        sensor_encoder_window.title("传感器与编码器功能")
        sensor_encoder_window.geometry("600x400")
        sensor_encoder_window.configure(bg="white")
        sensor_encoder_window.transient(self.root)  # 设置为主窗口的子窗口
        sensor_encoder_window.grab_set()  # 模态窗口
        # 功能按钮框架

        self.sensor_frame_2 = tk.Frame(sensor_encoder_window, bg="white")
        self.sensor_frame_2.pack(fill="x",padx=5,pady=(15,10))
        # 第1 :text
        self.sensor_main_tex = tk.Label(self.sensor_frame_2, text="传感器偏置获取与设置", bg="#2196F3",
                                      fg="white", font=("Arial", 10, "bold"))
        self.sensor_main_tex.pack(fill='x')


        self.sensor_frame_1 = tk.Frame(sensor_encoder_window, bg="white")
        self.sensor_frame_1.pack(fill="x")

        self.axis_text_ = tk.Label(self.sensor_frame_1, text="左臂", bg="#D8F4F3")
        self.axis_text_.grid(row=0, column=0, padx=(5,5))
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

        '''right arm'''
        self.sensor_frame_11 = tk.Frame(sensor_encoder_window, bg="white")
        self.sensor_frame_11.pack(fill="x")
        self.axis_text__ = tk.Label(self.sensor_frame_11, text="右臂", bg="#F4E4D8")
        self.axis_text__.grid(row=0, column=0, padx=(5,5))
        # 第2列：sensor select
        self.axis_text_2 = tk.Label(self.sensor_frame_11, text="轴", bg="white")
        self.axis_text_2.grid(row=0, column=1, padx=5)
        # 第3列：axis select
        self.axis_select_combobox_2 = ttk.Combobox(
            self.sensor_frame_11,
            values=["0", "1", "2", "3", "4", "5", "6"],
            width=3,
            state="readonly"  # 禁止直接输入
        )
        self.axis_select_combobox_2.current(0)  # 默认选中第一个选项
        self.axis_select_combobox_2.grid(row=0, column=2, padx=5)
        # 第4列：get offset
        self.get_offset_btn_2 = tk.Button(self.sensor_frame_11, text="获取偏置",
                                          command=lambda: self.get_sensor_offset('B'))
        self.get_offset_btn_2.grid(row=0, column=3, padx=5)
        # 第5列：get offset value
        self.get_offset_entry_2 = tk.Entry(self.sensor_frame_11, width=5)
        self.get_offset_entry_2.insert(0, '0.0')
        self.get_offset_entry_2.grid(row=0, column=4, padx=5)
        # 第6列：set offset
        self.set_offset_btn_2 = tk.Button(self.sensor_frame_11, text="设置偏置",
                                          command=lambda: self.set_sensor_offset('B'))
        self.set_offset_btn_2.grid(row=0, column=5, padx=5, pady=5)

        '''encoder'''
        self.encoder_frame_1 = tk.Frame(sensor_encoder_window, bg="white")
        self.encoder_frame_1.pack(fill="x",padx=5,pady=(25,10))
        # 第1 :text
        self.encoder_frame_1 = tk.Label(self.encoder_frame_1, text="电机编码器清零与清错", bg="#2196F3",
                                      fg="white", font=("Arial", 10, "bold"))
        self.encoder_frame_1.pack(fill='x')
        '''left arm'''
        self.motor_frame_1 = tk.Frame(sensor_encoder_window, bg="white")
        self.motor_frame_1.pack(fill="x")
        # 第1 :text
        self.motor_text_1 = tk.Label(self.motor_frame_1, text="左臂", bg="#D8F4F3")
        self.motor_text_1.grid(row=0, column=0, padx=(5,5))
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
        self.motor_btn_1 = tk.Button(self.motor_frame_1, text="电机内编清零",
                                     command=lambda: self.clear_motor_as_zero('A'))
        self.motor_btn_1.grid(row=0, column=3, padx=5, pady=5)
        # 第5列：电机外编
        self.motor_btn_2 = tk.Button(self.motor_frame_1, text="电机外编清零",
                                     command=lambda: self.clear_motorE_as_zero('A'))
        self.motor_btn_2.grid(row=0, column=4, padx=5)
        # 第7列：编码器清错
        self.motor_btn_3 = tk.Button(self.motor_frame_1, text="编码器清错", bg="#7ED2B4",
                                     command=lambda: self.clear_motor_error('A'))
        self.motor_btn_3.grid(row=0, column=5, padx=5)

        '''right arm'''
        self.motor_frame_2 = tk.Frame(sensor_encoder_window, bg="white")
        self.motor_frame_2.pack(fill="x")
        # 第1 :text
        self.motor_text_1 = tk.Label(self.motor_frame_2, text="右臂", bg="#F4E4D8")
        self.motor_text_1.grid(row=0, column=0, padx=(5, 5))

        # 第2列：axis select
        self.motor_axis_text_11 = tk.Label(self.motor_frame_2, text="轴", bg="white")
        self.motor_axis_text_11.grid(row=0, column=1, padx=5)

        # 第3列：axis select
        self.motor_axis_select_combobox_11 = ttk.Combobox(
            self.motor_frame_2,
            values=["0", "1", "2", "3", "4", "5", "6"],
            width=3,
            state="readonly"  # 禁止直接输入
        )
        self.motor_axis_select_combobox_11.current(0)  # 默认选中第一个选项
        self.motor_axis_select_combobox_11.grid(row=0, column=2, padx=5)

        # 第4列：电机内编
        self.motor_btn_11 = tk.Button(self.motor_frame_2, text="电机内编清零",
                                      command=lambda: self.clear_motor_as_zero('B'))
        self.motor_btn_11.grid(row=0, column=3, padx=5)

        # 第5列：电机外编
        self.motor_btn_21 = tk.Button(self.motor_frame_2, text="电机外编清零",
                                      command=lambda: self.clear_motorE_as_zero('B'))
        self.motor_btn_21.grid(row=0, column=4, padx=5)

        # 第7列：编码器清错
        self.motor_btn_31 = tk.Button(self.motor_frame_2, text="编码器清错", bg="#7ED2B4",
                                      command=lambda: self.clear_motor_error('B'))
        self.motor_btn_31.grid(row=0, column=5, padx=5)

    def system_update_dialog(self):
        """显示隐藏功能选择窗口"""
        button_w = 10
        hidden_window = tk.Toplevel(self.root)
        hidden_window.title("系统升级")
        hidden_window.geometry("600x400")
        hidden_window.configure(bg="white")
        hidden_window.transient(self.root)  # 设置为主窗口的子窗口
        hidden_window.grab_set()  # 模态窗口

        # 标题
        title_frame = tk.Frame(hidden_window, bg="white")
        title_frame.pack(fill="x",padx=5,pady=(15,10))
        # 第1 :text
        title_label = tk.Label(title_frame, text="参数更新", bg="#2196F3",
                                      fg="white", font=("Arial", 10, "bold"))
        title_label.pack(fill='x')


        '''第二行'''
        state_a_frame = tk.Frame(hidden_window, bg="white")
        state_a_frame.pack(fill="x", pady=5)

        hand_text_frame=tk.Label(state_a_frame,text="手动更新配置文件",bg='#CCCCFF')
        hand_text_frame.pack(side='left',expand=True)

        reset_a_button = tk.Button(state_a_frame, text="获取机器人参数文件", width=button_w + 10,
                                   command=self.get_ini)
        reset_a_button.pack(side='left',expand=True)
        pvt_a_button = tk.Button(state_a_frame, text="更新机器人参数文件", width=button_w + 10,
                                 command=self.update_ini)
        pvt_a_button.pack(side='left',expand=True)

        state_a_frame_ = tk.Frame(hidden_window, bg="white")
        state_a_frame_.pack(fill="x", pady=5)
        hand_text_frame_ = tk.Label(state_a_frame_, text="自动对比更新配置文件",bg='#CCCCFF')
        hand_text_frame_.pack(side='left', expand=True)
        param_c_btn=tk.Button(state_a_frame_,text='参数对比更新',width=button_w+10,command=self.compare_parameters_dialog, fg='#033341',bg='#DFC88C')
        param_c_btn.pack(side='left', expand=True)


        title_frame_ = tk.Frame(hidden_window, bg="white")
        title_frame_.pack(fill="x",padx=5,pady=(25,10))
        # 第1 :text
        title_label_ = tk.Label(title_frame_, text="系统升级", bg="#2196F3",
                                      fg="white", font=("Arial", 10, "bold"))
        title_label_.pack(fill='x',expand=True)

        state_a_frame1 = tk.Frame(hidden_window, bg="white")
        state_a_frame1.pack(fill="x", pady=5)
        reset_a_button = tk.Button(state_a_frame1, text="更新系统", width=10,
                                   command=self.update_sys, bg="#F6FC39",
                                   fg="#151513",
                                   font=("Arial", 10, "bold")).pack(side='left', expand=True)
        # reset_a_button.pack(side="left")

        state_a_frame2 = tk.Frame(hidden_window, bg="white")
        state_a_frame2.pack(fill="x", pady=5)
        label = tk.Label(state_a_frame2,
                         text='首次用软件更新系统后，方可在软件主页面底部查看到小版本，\n\n '
                              '参数配置文件robot.ini如果有更新，先获取参数文件，\n在存到本地的文件上面对比修改，再更新机器人参数文件.\n\n '
                              '更新系统选择更新包*.MV_SYS_UPDATE', bg='white')
        label.pack(padx=5, pady=10)

    def compare_parameters_dialog(self):
        """参数对比更新对话框"""
        robot.receive_file('robot.ini', "/home/FUSION/Config/cfg/robot.ini")
        time.sleep(1)
        # 加载源文件内容
        self.source_content = self.load_file(self.source_file)
        compare_window = tk.Toplevel(self.root)
        compare_window.title("参数对比更新")
        compare_window.geometry("1000x900")
        compare_window.configure(bg="#f0f0f0")
        compare_window.transient(self.root)
        compare_window.grab_set()

        # 设置对话框居中显示
        main_frame = ttk.Frame(compare_window, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(2, weight=1)

        # 文件选择区域
        file_frame = ttk.LabelFrame(main_frame, text="文件选择", padding="10")
        file_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))

        # 源文件显示
        ttk.Label(file_frame, text="源文件:").grid(row=0, column=0, sticky=tk.W)
        self.source_label = ttk.Label(file_frame, text=self.source_file, foreground="blue")
        self.source_label.grid(row=0, column=1, sticky=tk.W, padx=(5, 20))

        # 目标文件选择和显示
        ttk.Label(file_frame, text="对比文件:").grid(row=1, column=0, sticky=tk.W, pady=(10, 0))
        self.target_label = ttk.Label(file_frame, text="未选择", foreground="red")
        self.target_label.grid(row=1, column=1, sticky=tk.W, padx=(5, 10), pady=(10, 0))

        ttk.Button(file_frame, text="选择对比文件",
                   command=self.select_target_file).grid(row=1, column=2, padx=(10, 0), pady=(10, 0))

        # 状态标签
        self.status_label = ttk.Label(main_frame, text="", foreground="blue")
        self.status_label.grid(row=1, column=0, columnspan=2, sticky=tk.W, pady=(0, 10))

        # 对比显示区域
        compare_frame = ttk.LabelFrame(main_frame, text="文件对比", padding="10")
        compare_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 10))
        compare_frame.columnconfigure(0, weight=1)
        compare_frame.columnconfigure(1, weight=1)
        compare_frame.rowconfigure(0, weight=1)

        # 源文件文本框
        ttk.Label(compare_frame, text="源文件内容").grid(row=0, column=0, sticky=tk.W)
        self.source_text = tk.Text(compare_frame, wrap=tk.WORD, width=50, height=30)
        self.source_text.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 5))

        # 对比文件文本框
        ttk.Label(compare_frame, text="对比文件内容").grid(row=0, column=1, sticky=tk.W)
        self.target_text = tk.Text(compare_frame, wrap=tk.WORD, width=50, height=30)
        self.target_text.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(5, 0))

        # 创建一个共同的滚动条
        v_scrollbar = ttk.Scrollbar(compare_frame, orient=tk.VERTICAL)
        v_scrollbar.grid(row=1, column=2, sticky=(tk.N, tk.S))

        # 配置滚动条和文本框
        self.source_text.config(yscrollcommand=v_scrollbar.set)
        self.target_text.config(yscrollcommand=v_scrollbar.set)
        v_scrollbar.config(command=self.sync_scroll)

        # 操作按钮区域
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=3, column=0, columnspan=2, sticky=tk.E, pady=(0, 10))

        self.compare_button = ttk.Button(button_frame, text="对比文件",
                                         command=self.compare_files, state="disabled")
        self.compare_button.pack(side=tk.LEFT, padx=(0, 10))

        self.apply_button = ttk.Button(button_frame, text="确认修改",
                                       command=self.apply_changes, state="disabled")
        self.apply_button.pack(side=tk.LEFT)

        self.update_ini_button = ttk.Button(button_frame, text="上传文件",
                                       command=self.update_ini2)
        self.update_ini_button.pack(side=tk.LEFT)

    def sync_scroll(self, *args):
        """同步滚动两个文本框"""
        self.source_text.yview(*args)
        self.target_text.yview(*args)

    def select_target_file(self):
        """选择对比文件"""
        file_path = filedialog.askopenfilename(
            title="选择对比文件",
            filetypes=[("INI files", "*.ini"), ("All files", "*.*")]
        )

        if file_path:
            self.target_file = file_path
            self.target_label.config(text=file_path, foreground="green")
            self.compare_button.config(state="normal")
            self.update_status("已选择对比文件，点击'对比文件'开始对比")

    def update_status(self, message=""):
        """更新状态标签"""
        if not message:
            if self.target_file:
                message = f"已选择对比文件: {Path(self.target_file).name}"
            else:
                message = "请选择对比文件"
        self.status_label.config(text=message)

    def compare_files(self):
        """对比两个文件的内容"""
        if not self.target_file:
            messagebox.showwarning("警告", "请先选择对比文件")
            return

        # 加载目标文件内容
        target_content = self.load_file(self.target_file)
        if not target_content:
            return

        # 清空文本框
        self.source_text.delete(1.0, tk.END)
        self.target_text.delete(1.0, tk.END)

        # 显示文件内容并高亮差异
        self.highlight_differences(self.source_content, target_content)

        # 启用应用按钮
        self.apply_button.config(state="normal")
        self.update_status("文件对比完成，可以查看差异并应用修改")

    def highlight_differences(self, source_content, target_content):
        """高亮显示文件差异"""
        # 将内容分割成行
        source_lines = source_content.splitlines()
        target_lines = target_content.splitlines()

        # 使用difflib找出差异
        diff = difflib.SequenceMatcher(None, source_lines, target_lines)

        # 显示源文件内容
        for i, line in enumerate(source_lines):
            self.source_text.insert(tk.END, line + '\n')

            # 检查是否有差异
            tag_added = False
            for tag in diff.get_opcodes():
                if tag[0] != 'equal':
                    if tag[1] <= i < tag[2]:
                        self.source_text.tag_add('diff', f"{i + 1}.0", f"{i + 1}.end")
                        tag_added = True
                        break

            # 如果这一行在源文件中有但在目标文件中没有，用红色背景
            if not tag_added:
                # 检查是否在目标文件中存在
                in_target = any(line == target_line for target_line in target_lines)
                if not in_target and line.strip():  # 忽略空行
                    self.source_text.tag_add('removed', f"{i + 1}.0", f"{i + 1}.end")

        # 显示目标文件内容
        for i, line in enumerate(target_lines):
            self.target_text.insert(tk.END, line + '\n')

            # 检查是否有差异
            for tag in diff.get_opcodes():
                if tag[0] != 'equal':
                    if tag[3] <= i < tag[4]:
                        self.target_text.tag_add('diff', f"{i + 1}.0", f"{i + 1}.end")
                        break

            # 如果这一行在目标文件中有但在源文件中没有，用绿色背景
            in_source = any(line == source_line for source_line in source_lines)
            if not in_source and line.strip():  # 忽略空行
                self.target_text.tag_add('added', f"{i + 1}.0", f"{i + 1}.end")

        # 配置标签样式
        self.source_text.tag_config('diff', background='#FFD700')  # 金色表示修改
        self.source_text.tag_config('removed', background='#FFCCCC')  # 浅红色表示删除

        self.target_text.tag_config('diff', background='#FFD700')  # 金色表示修改
        self.target_text.tag_config('added', background='#CCFFCC')  # 浅绿色表示新增

    def parse_ini_by_sections(self, content):
        """按组别解析INI文件，返回结构化的数据"""
        lines = content.splitlines()
        sections = {}
        current_section = None
        current_section_lines = []
        section_start_line = 0
        protected_keys = set()  # 存储受保护的键

        # 记录节在文件中的顺序
        section_order = []

        for i, line in enumerate(lines):
            stripped = line.strip()

            # 检查是否为节头
            if stripped.startswith('[') and stripped.endswith(']'):
                # 保存前一个节
                if current_section is not None:
                    sections[current_section] = {
                        'start_line': section_start_line,
                        'lines': current_section_lines,
                        'params': self.extract_params(current_section_lines, protected_keys)
                    }
                    section_order.append(current_section)

                # 开始新节
                current_section = stripped[1:-1]
                current_section_lines = [line]
                section_start_line = i
            else:
                # 检查是否包含受保护的键（带*号）
                if '=' in stripped and '*' in stripped:
                    key_part = stripped.split('=', 1)[0].strip()
                    if '*' in key_part:
                        # 提取干净的键名
                        clean_key = key_part.rstrip('*').strip()
                        protected_keys.add(clean_key)

                if current_section is not None:
                    current_section_lines.append(line)
                else:
                    # 文件开头的全局内容
                    if 'global' not in sections:
                        sections['global'] = {
                            'start_line': 0,
                            'lines': [],
                            'params': {}
                        }
                        section_order.append('global')
                    sections['global']['lines'].append(line)

        # 保存最后一个节
        if current_section is not None:
            sections[current_section] = {
                'start_line': section_start_line,
                'lines': current_section_lines,
                'params': self.extract_params(current_section_lines, protected_keys)
            }
            section_order.append(current_section)

        return sections, protected_keys, section_order

    def extract_params(self, section_lines, protected_keys):
        """从节内容中提取参数"""
        params = {}

        for line in section_lines:
            stripped = line.strip()
            if stripped and '=' in stripped and not stripped.startswith(';'):
                # 处理键值对
                key_part, value_part = stripped.split('=', 1)
                key = key_part.strip()

                # 检查是否有*号
                has_star = False
                if key.endswith('*'):
                    has_star = True
                    key = key.rstrip('*').strip()

                # 存储值，标记是否有*号
                params[key] = {
                    'value': value_part.strip(),
                    'has_star': has_star,
                    'original_line': line
                }

        return params

    def apply_changes(self):
        """应用修改到源文件"""
        if not self.target_file:
            return

        # 确认操作
        if not messagebox.askyesno("确认", "确定要应用修改到源文件吗？"):
            return

        try:
            # 加载目标文件内容
            target_content = self.load_file(self.target_file)
            if not target_content:
                return

            # 解析两个文件
            source_sections, source_protected, source_order = self.parse_ini_by_sections(self.source_content)
            target_sections, target_protected, target_order = self.parse_ini_by_sections(target_content)

            # 应用修改规则
            new_content = self.apply_modification_rules_by_sections(
                source_sections, target_sections, source_protected, target_protected,
                source_order, target_order, self.source_content
            )

            if new_content != self.source_content:
                # 保存修改后的源文件
                with open(self.source_file, 'w', encoding='utf-8') as f:
                    f.write(new_content)

                # 重新加载源文件内容
                self.source_content = new_content

                # 重新对比显示
                self.compare_files()

                messagebox.showinfo("成功", "修改已成功应用到源文件")
                self.update_status("修改已应用，已重新加载源文件")
            else:
                messagebox.showinfo("信息", "没有需要修改的内容")

        except Exception as e:
            messagebox.showerror("错误", f"应用修改失败: {str(e)}")

    def apply_modification_rules_by_sections(self, source_sections, target_sections,
                                             source_protected, target_protected,
                                             source_order, target_order, original_source_content):
        """按组别应用修改规则"""
        new_lines = []
        processed_sections = set()

        # 处理文件开头的非节内容（注释等）
        original_lines = original_source_content.splitlines()
        for line in original_lines:
            stripped = line.strip()
            if not stripped or stripped.startswith(';'):
                new_lines.append(line)
            elif not stripped.startswith('['):
                new_lines.append(line)
            else:
                break

        # 处理源文件中存在的节
        for section_name in source_order:
            if section_name == 'global' or section_name not in source_sections:
                continue

            processed_sections.add(section_name)

            if section_name in target_sections:
                # 两个文件都有这个节
                new_section_lines = self.merge_section(
                    source_sections[section_name],
                    target_sections[section_name],
                    source_protected
                )
                # 添加节之前检查是否需要空行
                if new_lines and not self.should_add_empty_line_before(new_lines[-1], new_section_lines[0]):
                    # 检查原始文件中这个节前是否有空行
                    section_start_line = source_sections[section_name]['start_line']
                    if section_start_line > 0 and original_lines[section_start_line - 1].strip() == "":
                        new_lines.append("")
                new_lines.extend(new_section_lines)
            else:
                # 源文件有，目标文件没有这个节
                # 检查是否有受保护的键，如果没有则删除整个节
                has_protected_keys = False
                for key in source_sections[section_name]['params']:
                    if key in source_protected:
                        has_protected_keys = True
                        break

                if has_protected_keys:
                    # 有受保护键，保留整个节
                    # 添加节之前检查是否需要空行
                    if new_lines and not self.should_add_empty_line_before(new_lines[-1],
                                                                           source_sections[section_name]['lines'][0]):
                        # 检查原始文件中这个节前是否有空行
                        section_start_line = source_sections[section_name]['start_line']
                        if section_start_line > 0 and original_lines[section_start_line - 1].strip() == "":
                            new_lines.append("")
                    new_lines.extend(source_sections[section_name]['lines'])

        # 处理目标文件中独有的新节
        new_sections_to_add = []
        for section_name in target_order:
            if section_name == 'global' or section_name in processed_sections:
                continue
            new_sections_to_add.append((section_name, target_order.index(section_name)))

        # 按目标文件中的顺序排序
        new_sections_to_add.sort(key=lambda x: x[1])

        # 智能插入新节：保持目标文件中的相对顺序
        for section_name, _ in new_sections_to_add:
            # 在目标文件中找到这个新节的前一个节
            insert_after = None
            target_index = target_order.index(section_name)

            # 向前查找在源文件中也存在的节
            for j in range(target_index - 1, -1, -1):
                prev_section = target_order[j]
                if prev_section in processed_sections:
                    insert_after = prev_section
                    break

            if insert_after:
                # 在new_lines中找到这个节的位置
                insert_index = -1
                for idx in range(len(new_lines)):
                    if new_lines[idx].strip() == f"[{insert_after}]":
                        # 找到这个节的结束位置
                        for k in range(idx, len(new_lines)):
                            if k + 1 < len(new_lines) and new_lines[k + 1].strip().startswith('['):
                                insert_index = k + 1
                                break
                        if insert_index == -1:
                            insert_index = len(new_lines)
                        break

                if insert_index >= 0:
                    # 插入新节
                    new_section_lines = [f"[{section_name}]"]
                    for key, param_info in target_sections[section_name]['params'].items():
                        star_marker = '*' if key in target_protected else ''
                        new_section_lines.append(f"{key}{star_marker}={param_info['value']}")

                    # 在正确位置插入
                    new_lines[insert_index:insert_index] = new_section_lines
            else:
                # 没有找到合适的位置，添加到末尾
                new_lines.append(f"[{section_name}]")
                for key, param_info in target_sections[section_name]['params'].items():
                    star_marker = '*' if key in target_protected else ''
                    new_lines.append(f"{key}{star_marker}={param_info['value']}")

        # 清理多余的空行（最多保留一个空行）
        cleaned_lines = []
        last_was_empty = False
        for line in new_lines:
            if line.strip() == "":
                if not last_was_empty:
                    cleaned_lines.append(line)
                    last_was_empty = True
            else:
                cleaned_lines.append(line)
                last_was_empty = False

        # 确保文件末尾没有多余空行
        while cleaned_lines and cleaned_lines[-1].strip() == "":
            cleaned_lines.pop()

        return '\n'.join(cleaned_lines)

    def should_add_empty_line_before(self, last_line, current_section_header):
        """判断是否需要在节前添加空行"""
        if not last_line.strip():
            return True  # 最后一行已经是空行
        if last_line.strip().startswith(';'):
            return True  # 最后一行是注释，需要空行
        if last_line.strip().startswith('['):
            return False  # 最后一行是节头，不需要空行
        return True  # 其他情况需要空行

    def merge_section(self, source_section, target_section, source_protected):
        """合并两个相同节的内容"""
        new_section_lines = []

        # 添加节头
        new_section_lines.append(source_section['lines'][0])

        # 处理源文件中的参数
        for key, source_param in source_section['params'].items():
            if key in source_protected:
                # 受保护的键，保持原样
                new_section_lines.append(source_param['original_line'])
            elif key in target_section['params']:
                # 两个文件都有的键，使用目标文件的值
                target_param = target_section['params'][key]
                new_section_lines.append(f"{key}={target_param['value']}")
            else:
                # 源文件有但目标文件没有的键，删除
                pass

        # 添加目标文件中有但源文件中没有的新键
        for key, target_param in target_section['params'].items():
            if key not in source_section['params'] and key not in source_protected:
                # 新键，添加到节末尾
                new_section_lines.append(f"{key}={target_param['value']}")

        # 保留节中的注释和空行，保持原始顺序
        # 首先收集所有的注释和空行
        comments_and_blanks = []
        for line in source_section['lines'][1:]:  # 跳过节头
            stripped = line.strip()
            if not stripped or stripped.startswith(';'):
                comments_and_blanks.append(line)

        # 然后将注释和空行插入到合适的位置
        if comments_and_blanks:
            # 找到第一个参数的位置
            first_param_index = -1
            for i, line in enumerate(new_section_lines[1:], 1):
                if '=' in line:
                    first_param_index = i
                    break

            if first_param_index > 0:
                # 在第一个参数前插入注释
                new_section_lines[first_param_index:first_param_index] = comments_and_blanks

        return new_section_lines

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

    def update_ini2(self):
        if self.connected:
            tag=robot.send_file('robot.ini', "/home/FUSION/Config/cfg/robot.ini")
            time.sleep(1)
            if tag:
                messagebox.showinfo('success', '参数已上传到控制器')
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
                time.sleep(1)
                if tag:
                    messagebox.showinfo('success', '参数已保存')
        else:
            messagebox.showerror('error', '请先连接机器人')

    def update_sys(self):
        if self.connected:
            result = messagebox.askokcancel("确认操作", "文件上传后重启将自动更新系统版本，确认上传吗吗？")
            if result:
                file_path = filedialog.askopenfilename(
                    filetypes=[("All files", "*.*")],
                    title="选择系统更新文件"
                )
                if file_path:
                    tag1 = robot.send_file(file_path, "/home/FUSION/Tmp/ctrl_package.tar")
                    if tag1:
                        messagebox.showinfo('success', '系统文件已上传，请重启控制器自动更新。')
                    else:
                        messagebox.showinfo('error', '系统文件上传失败,请重新上传。')
        else:
            messagebox.showerror('error', '请先连接机器人')

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
        # # 网络设置选项卡
        # network_frame = ttk.Frame(notebook, padding="10")
        # notebook.add(network_frame, text="网络设置")

        # # 界面设置选项卡
        # interface_frame = ttk.Frame(notebook, padding="10")
        # notebook.add(interface_frame, text="界面设置")
        # 填充浮动基座设置选项卡
        self.create_floating_base_tab(floating_base_frame)
        # 填充网络设置选项卡
        # self.create_network_settings_tab(network_frame)
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
        if row2_val == "x":
            self.row2_selection[0] = 1
        if row2_val == "-x":
            self.row2_selection[0] = -1
        if row2_val == "y":
            self.row2_selection[1] = 1
        if row2_val == "-y":
            self.row2_selection[1] = -1
        if row2_val == "z":
            self.row2_selection[2] = 1
        if row2_val == "-z":
            self.row2_selection[2] = -1

        # 更新第三行选择
        row3_val = self.row3_var.get()
        if row3_val == "x":
            self.row3_selection[0] = 1
        if row3_val == "-x":
            self.row3_selection[0] = -1
        if row3_val == "y":
            self.row3_selection[1] = 1
        if row3_val == "-y":
            self.row3_selection[1] = -1
        if row3_val == "z":
            self.row3_selection[2] = 1
        if row3_val == "-z":
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

    def imped_f_mode(self,dir, robot_id):
        if self.connected:
            result = messagebox.askokcancel("确认操作", "机器人将切换为阻抗力控状态，确认切换吗？")
            if result:
                directions = [0, 0, 0, 0, 0, 0]
                directions[dir]=1
                robot.clear_set()
                robot.set_state(arm=robot_id, state=3)
                robot.set_impedance_type(arm=robot_id, type=3)
                robot.send_cmd()
                time.sleep(0.001)

                force=0
                adj=0
                if robot_id == 'A':
                    force = float(self.left_force_entry.get())
                    adj= float(self.left_force_adj_entry.get())
                elif robot_id == 'B':
                    force= float(self.right_force_entry.get())
                    adj= float(self.right_force_adj_entry.get())

                robot.clear_set()
                robot.set_force_control_params(arm=robot_id, fcType=0, fxDirection=directions,
                                               fcCtrlpara=[0, 0, 0, 0, 0, 0, 0],
                                               fcAdjLmt=adj)
                robot.send_cmd()
                time.sleep(0.02)
                robot.clear_set()
                robot.set_force_cmd(arm=robot_id, f=force)
                robot.send_cmd()
            else:
                print("切换力控操作取消")
        else:
            messagebox.showerror('error', '请先连接机器人')

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

    def validate_point(self,point_str, nums):
        try:
            point_str = point_str.strip()
            if not point_str:
                return False, "输入不能为空"
            values = point_str.split(',')
            if len(values) != nums:
                return False, f"请输入{nums}个用逗号隔开的数字"
            # 检查每个值是否为数字字符
            validated_values = []
            for value in values:
                value = value.strip()
                if not value:  # 检查是否为空字符串
                    return False, "所有位置都必须有数字，不能为空"
                if not value.isdigit():
                    try:
                        float(value)
                    except ValueError:
                        return False, f"'{value}' 不是有效的数字"
                validated_values.append(value)
            if len(validated_values) != nums:
                return False, f"列表长度必须为{nums}"
            normalized_str = ','.join(validated_values)
            return True, normalized_str
        except Exception as e:
            return False, f"输入格式不正确: {str(e)}"

    def run_joints(self,robot_id):
        if self.connected:
            selected=None
            if robot_id=='A':
                selected = self.combo1.get()
            else:
                selected=self.combo2.get()
            if selected:
                # 验证选中的点是否为有效的7元素列表
                is_valid, point_list = self.validate_point(selected,7)
                if is_valid:
                    # messagebox.showinfo("1#运行", f"运行选中的点: {point_list}")
                    values = point_list.split(',')
                    point_list = [float(value.strip()) for value in values]
                    robot.clear_set()
                    robot.set_joint_cmd_pose(arm=robot_id, joints=point_list)
                    robot.send_cmd()
                else:
                    messagebox.showerror("错误", f"选中的点格式无效: {selected}")
            else:
                messagebox.showwarning("警告", "没有可运行的点")
        else:
            messagebox.showerror('error', '请先连接机器人')

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

    def add_current_joints(self, robot_id):
        if self.connected:
            idx=0
            if robot_id=='A':
                idx=0
            elif robot_id=='B':
                idx=1
            cartesian_text_r = (f"{self.result['outputs'][idx]['fb_joint_pos'][0]:.2f},"
                                f"{self.result['outputs'][idx]['fb_joint_pos'][1]:.2f},"
                                f"{self.result['outputs'][idx]['fb_joint_pos'][2]:.2f},"
                                f"{self.result['outputs'][idx]['fb_joint_pos'][3]:.2f}, "
                                f"{self.result['outputs'][idx]['fb_joint_pos'][4]:.2f}, "
                                f"{self.result['outputs'][idx]['fb_joint_pos'][5]:.2f}, "
                                f"{self.result['outputs'][idx]['fb_joint_pos'][6]:.2f}")

            if robot_id=='A':
                self.entry.delete(0, tk.END)
                self.entry.insert(0, cartesian_text_r)
            elif robot_id=='B':
                self.entry1.delete(0, tk.END)
                self.entry1.insert(0, cartesian_text_r)
        else:
            messagebox.showerror('error', '请先连接机器人')

    def add_point1(self):
        """添加点到1#列表"""
        point_str = self.entry_var.get()
        is_valid, result = self.validate_point(point_str,7)
        print(f'#1 add point:{result}')
        if is_valid:
            # 检查是否已存在相同的点
            if self.is_duplicate(result, self.points1):
                messagebox.showwarning("重复点", "该点已存在于1#列表中")
                return
            # 将列表转换为字符串并存储
            # point_repr = str(result)
            self.points1.insert(0, result)
            self.update_comboboxes()
            # messagebox.showinfo("成功", "点已添加到1#列表")
        else:
            messagebox.showwarning("输入错误", result)

    def add_point2(self):
        """添加点到2#列表"""
        point_str = self.entry_var1.get()
        is_valid, result = self.validate_point(point_str,7)
        print(f'#2 add point:{result}')
        if is_valid:
            # 检查是否已存在相同的点
            if self.is_duplicate(result, self.points2):
                messagebox.showwarning("重复点", "该点已存在于2#列表中")
                return
            # 将列表转换为字符串并存储
            # point_repr = str(result)
            self.points2.insert(0, result)
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
                        is_valid, result = self.validate_point(line,7)
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
                        is_valid, result = self.validate_point(line,7)
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

    def select_period_file(self, robot_id):
        result = messagebox.askokcancel("提示", "轨迹复现须在关节跟随或阻抗模式下进行？")
        if result:
            file_path = filedialog.askopenfilename(
                defaultextension=".r50pth",
                filetypes=[("path files", "*.r50pth"), ("All files", "*.*")],
                title="选择周期运行文件"
            )
            if file_path:
                if robot_id == 'A':
                    self.period_file_path_1.set(file_path)
                    # messagebox.showinfo("成功", f"1#周期运行文件已选择: {os.path.basename(file_path)}")
                elif robot_id == 'B':
                    self.period_file_path_2.set(file_path)
                    # messagebox.showinfo("成功", f"2#周期运行文件已选择: {os.path.basename(file_path)}")
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

    def thread_run_period(self, robot_id):
        """在新线程中执行周期运行"""
        # # 重置停止标志
        # with self.lock:
        #     self.stop_flag = False
        self.stop_event.clear()
        self.thread = threading.Thread(
            target=self.run_period_file,
            args=(robot_id)
        )
        self.thread.daemon = True
        self.thread.start()


    def run_period_file(self, robot_id):
        if self.connected:
            try:
                # while True:
                #     # 检查停止标志
                #     with self.lock:
                #         if self.stop_flag:
                #             break

                if robot_id == 'A':
                    with open(self.period_file_path_1.get(), 'r', encoding='utf-8') as file:
                        lines = file.readlines()
                    for i, line in enumerate(lines):
                        if self.stop_event.is_set():
                            print("Thread interrupted by external signal")
                            return
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
                        if self.stop_event.is_set():
                            print("Thread interrupted by external signal")
                            return
                        # 处理当前行
                        processed_line = self.process_line(i, line)
                        # print(f'processed_line:{processed_line}')
                        robot.clear_set()
                        robot.set_joint_cmd_pose(arm='B', joints=processed_line)
                        robot.send_cmd()
                        # 50Hz频率 = 每0.02秒一行
                        time.sleep(0.02)
            except Exception as e:
                messagebox.showerror("错误", f"读取文件时出错: {e}")
                # self.root.after(0, lambda: messagebox.showerror("错误", f"读取文件时出错: {e}"))
        else:
            messagebox.showerror('error', '请先连接机器人')

    def vel_acc_set(self, robot_id):
        if self.connected:
            vel = acc = 0
            if robot_id == 'A':
                vel = int(self.left_speed_entry.get())
                acc = int(self.left_accel_entry.get())
            elif robot_id == 'B':
                vel = int(self.right_speed_entry.get())
                acc = int(self.right_accel_entry.get())

            robot.clear_set()
            robot.set_vel_acc(arm=robot_id, velRatio=vel, AccRatio=acc)
            robot.send_cmd()
        else:
            messagebox.showerror('error', '请先连接机器人')

    def show_impedance_dialog(self, robot_id):
        """显示阻抗参数设置对话框"""
        if not self.connected:
            messagebox.showwarning("未连接", "请先连接机器人")
            return

        # 创建阻抗参数设置对话框
        impedance_dialog = tk.Toplevel(self.root)
        impedance_dialog.title(f"阻抗参数设置")
        impedance_dialog.geometry("1200x500")
        impedance_dialog.configure(bg="white")
        impedance_dialog.transient(self.root)
        impedance_dialog.grab_set()

        arm_side = ''
        if robot_id=='A':
            arm_side = '左臂'
            # 创建主框架
            main_frame = tk.Frame(impedance_dialog, padx=20, pady=20, bg='white')
            main_frame.pack(fill="both", expand=True)
            # 标题
            title_label = tk.Label(
                main_frame,
                text=f"{arm_side}阻抗参数设置",
                font=('Arial', 10, 'bold'),
                fg='#2c3e50'
            )
            title_label.pack(pady=(0, 10))

            # 创建参数输入区域
            params_frame = tk.Frame(main_frame, bg='white')
            params_frame.pack(fill="x", pady=(5, 10))
            # row 1 0保存参数   1设置关节阻抗参数   2 K  3 K entry  4 D  5 D entry
            # set joint kd
            joint_kd_a_button = tk.Button(params_frame, text="设置关节阻抗参数", width=20,
                                          command=lambda: self.joint_kd_set('A'))
            joint_kd_a_button.grid(row=0, column=0, padx=5,pady=10)
            # k laebl
            k_a_label = tk.Label(params_frame, text='K:', width=5, bg="white")
            k_a_label.grid(row=0, column=1)

            # k entry
            k_a_entry = tk.Entry(params_frame, textvariable=self.k_a_entry,width=50)
            k_a_entry.grid(row=0, column=2, sticky="ew")

            # d laebl
            d_a_label = tk.Label(params_frame, text='D:', width=5, bg="white")
            d_a_label.grid(row=0, column=3)

            # d entry
            d_a_entry = tk.Entry(params_frame,textvariable=self.d_a_entry, width=30)
            d_a_entry.grid(row=0, column=4)

            # 创建参数输入区域
            params_save_frame = tk.Frame(main_frame,bg='white')
            params_save_frame.pack(fill="x", pady=(0, 10))
            # SAVE PARA
            save_param_a_button = tk.Button(params_save_frame, text="保存参数", command=lambda: self.save_param('A'))
            save_param_a_button.pack(side='left',expand=True)
            # SAVE PARA
            load_param_a_button = tk.Button(params_save_frame, text="导入参数", command=lambda: self.load_param('A'))
            load_param_a_button.pack(side='left',expand=True)

            # set joint kd
            cart_kd_a_button = tk.Button(params_frame, text="设置笛卡尔阻抗参数", width=20,
                                         command=lambda: self.cart_kd_set('A'))
            cart_kd_a_button.grid(row=1, column=0, padx=5,pady=(20,10))

            # k laebl
            k_a_label_ = tk.Label(params_frame, text='K:', width=5, bg="white")
            k_a_label_.grid(row=1, column=1)

            # k entry
            cart_k_a_entry = tk.Entry(params_frame, textvariable=self.cart_k_a_entry,width=50)
            cart_k_a_entry.grid(row=1, column=2, sticky="ew")

            # d laebl
            d_a_label_ = tk.Label(params_frame, text='D:', width=5, bg="white")
            d_a_label_.grid(row=1, column=3)

            # d entry
            cart_d_a_entry = tk.Entry(params_frame,textvariable=self.cart_d_a_entry,width=30)
            cart_d_a_entry.grid(row=1, column=4)



            '''right_arm'''

        elif robot_id=='B':
            arm_side = '右臂'
            # 创建主框架
            main_frame1 = tk.Frame(impedance_dialog, padx=20, pady=20,bg='white')
            main_frame1.pack(fill="both", expand=True)
            # 标题
            title_label1 = tk.Label(
                main_frame1,
                text=f"{arm_side}阻抗参数设置",
                font=('Arial', 10, 'bold'),
                fg='#2c3e50'
            )
            title_label1.pack(pady=(0, 10))

            # 创建参数输入区域
            params_frame1 = tk.Frame(main_frame1, bg='white')
            params_frame1.pack(fill="x", pady=(5, 10))

            # row 1 0保存参数   1设置关节阻抗参数   2 K  3 K entry  4 D  5 D entry
            # set joint kd
            joint_kd_a_button1 = tk.Button(params_frame1, text="设置关节阻抗参数", width=20,
                                          command=lambda: self.joint_kd_set('B'))
            joint_kd_a_button1.grid(row=0, column=0, padx=5, pady=10)
            # k laebl
            k_a_label1 = tk.Label(params_frame1, text='K:', width=5, bg="white")
            k_a_label1.grid(row=0, column=1)

            # k entry
            k_b_entry = tk.Entry(params_frame1,textvariable=self.k_b_entry, width=50)
            k_b_entry.grid(row=0, column=2, sticky="ew")

            # d laebl
            d_b_label = tk.Label(params_frame1, text='D:', width=5, bg="white")
            d_b_label.grid(row=0, column=3)

            # d entry
            d_b_entry = tk.Entry(params_frame1,textvariable=self.d_b_entry, width=30)
            d_b_entry.grid(row=0, column=4)

            # 创建参数输入区域
            params_save_frame1 = tk.Frame(main_frame1, bg='white')
            params_save_frame1.pack(fill="x", pady=(0, 10))
            # SAVE PARA
            save_param_b_button = tk.Button(params_save_frame1, text="保存参数", command=lambda: self.save_param('B'))
            save_param_b_button.pack(side='left', expand=True)
            # SAVE PARA
            load_param_b_button = tk.Button(params_save_frame1, text="导入参数", command=lambda: self.load_param('B'))
            load_param_b_button.pack(side='left', expand=True)

            # set joint kd
            cart_kd_b_button = tk.Button(params_frame1, text="设置笛卡尔阻抗参数", width=20,
                                         command=lambda: self.cart_kd_set('B'))
            cart_kd_b_button.grid(row=1, column=0, padx=5,pady=(20,10))

            # k laebl
            k_b_label_ = tk.Label(params_frame1, text='K:', width=5, bg="white")
            k_b_label_.grid(row=1, column=1)

            # k entry
            cart_k_b_entry = tk.Entry(params_frame1,textvariable=self.cart_k_b_entry, width=50)
            cart_k_b_entry.grid(row=1, column=2, sticky="ew")

            # d laebl
            d_b_label_ = tk.Label(params_frame1, text='D:', width=5, bg="white")
            d_b_label_.grid(row=1, column=3)

            # d entry
            cart_d_b_entry = tk.Entry(params_frame1,textvariable=self.cart_d_b_entry, width=30)
            cart_d_b_entry.grid(row=1, column=4)

    def joint_kd_set(self, robot_id):
        if self.connected:
            k = 0
            d = 0
            if robot_id == 'A':
                k_ = self.k_a_entry.get()
                d_ = self.d_a_entry.get()
                is_valid, point_list = self.validate_point(k_, 7)
                if is_valid:
                    values = point_list.split(',')
                    k = [float(value.strip()) for value in values]
                is_valid, point_list = self.validate_point(d_, 7)
                if is_valid:
                    values = point_list.split(',')
                    d = [float(value.strip()) for value in values]
                self.k_a_entry.set(k_)
                self.d_a_entry.set(d_)
            elif robot_id == 'B':
                k_=self.k_b_entry.get()
                d_=self.d_b_entry.get()
                is_valid, point_list = self.validate_point(k_, 7)
                if is_valid:
                    values = point_list.split(',')
                    k = [float(value.strip()) for value in values]
                is_valid, point_list = self.validate_point(d_, 7)
                if is_valid:
                    values = point_list.split(',')
                    d = [float(value.strip()) for value in values]
                self.k_b_entry.set(k_)
                self.d_b_entry.set(d_)
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
            if robot_id == 'A':
                k_ = self.cart_k_a_entry.get()
                d_ = self.cart_d_a_entry.get()
                is_valid, point_list = self.validate_point(k_, 7)
                if is_valid:
                    values = point_list.split(',')
                    k = [float(value.strip()) for value in values]
                is_valid, point_list = self.validate_point(d_, 7)
                if is_valid:
                    values = point_list.split(',')
                    d = [float(value.strip()) for value in values]
                self.cart_k_a_entry.set(k_)
                self.cart_d_a_entry.set(d_)
            elif robot_id == 'B':
                k_ = self.cart_k_b_entry.get()
                d_ = self.cart_d_b_entry.get()
                is_valid, point_list = self.validate_point(k_, 7)
                if is_valid:
                    values = point_list.split(',')
                    k = [float(value.strip()) for value in values]
                is_valid, point_list = self.validate_point(d_, 7)
                if is_valid:
                    values = point_list.split(',')
                    d = [float(value.strip()) for value in values]
                self.cart_k_b_entry.set(k_)
                self.cart_d_b_entry.set(d_)

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
            robot.set_cart_kd_params(arm=robot_id, K=k, D=d, type=2)
            robot.send_cmd()


        else:
            messagebox.showerror('error', '请先连接机器人')

    def save_param(self, robot_id):
        if robot_id == 'A':
            print(f'self.k_a_entry.get():{self.k_a_entry.get()}')
            print(f'self.d_a_entry.get():{self.d_a_entry.get()}')
            self.params.append(str(ast.literal_eval(self.k_a_entry.get())))
            self.params.append(str(ast.literal_eval(self.d_a_entry.get())))
            self.params.append(str(ast.literal_eval(self.cart_k_a_entry.get())))
            self.params.append(str(ast.literal_eval(self.cart_d_a_entry.get())))

        elif robot_id == 'B':
            print(f'self.k_b_entry.get():{self.k_b_entry.get()}')
            print(f'self.d_b_entry.get():{self.d_b_entry.get()}')
            self.params.append(str(ast.literal_eval(self.k_b_entry.get())))
            self.params.append(str(ast.literal_eval(self.d_b_entry.get())))
            self.params.append(str(ast.literal_eval(self.cart_k_b_entry.get())))
            self.params.append(str(ast.literal_eval(self.cart_d_b_entry.get())))
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
                title="选择要导入到左臂的参数文件"
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
                        # self.k_a_entry.delete(0, tk.END)
                        self.k_a_entry.set(valid_points[0])
                        # self.d_a_entry.delete(0, tk.END)
                        self.d_a_entry.set(valid_points[1])
                        # self.cart_k_a_entry.delete(0, tk.END)
                        self.cart_k_a_entry.set(valid_points[2])
                        # self.cart_d_a_entry.delete(0, tk.END)
                        self.cart_d_a_entry.set(valid_points[3])

                except Exception as e:
                    messagebox.showerror("错误", f"读取文件时出错: {str(e)}")

        elif robot_id == 'B':
            file_path = filedialog.askopenfilename(
                filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
                title="选择要导入到右臂的参数文件"
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
                        self.k_b_entry.delete(0, tk.END)
                        self.k_b_entry.insert(0, valid_points[0])
                        self.d_b_entry.delete(0, tk.END)
                        self.d_b_entry.insert(0, valid_points[1])
                        self.cart_k_b_entry.delete(0, tk.END)
                        self.cart_k_b_entry.insert(0, valid_points[2])
                        self.cart_d_b_entry.delete(0, tk.END)
                        self.cart_d_b_entry.insert(0, valid_points[3])
                except Exception as e:
                    messagebox.showerror("错误", f"读取文件时出错: {str(e)}")

    def reset_robot_state(self, robot_id):
        """复位按钮点击事件"""
        if not self.connected:
            messagebox.showwarning("未连接", "请先连接机器人")
            return

        else:
            try:
                robot.clear_set()
                robot.set_state(arm=robot_id, state=0)
                robot.send_cmd()

                # 添加急停按钮的复位
                if robot_id == 'A' and hasattr(self, 'left_emergency_btn'):
                    self.left_emergency_btn.reset()
                elif robot_id == 'B' and hasattr(self, 'right_emergency_btn'):
                    self.right_emergency_btn.reset()
                self.status_label.config(text="已连接", foreground='green')
            except Exception as e:
                messagebox.showerror("错误", f"复位过程中发生错误: {str(e)}")

    def on_state_selected(self, robot_id):
        """状态选择事件"""
        if not self.connected:
            messagebox.showwarning("未连接", "请先连接机器人")
            return

        selected_state=None
        if robot_id == 'A':
            selected_state = self.state_var.get()
        elif robot_id == 'B':
            selected_state = self.state_var_r.get()

        # 处理阻抗模式
        imp_type=None
        if selected_state == '关节阻抗' or selected_state == "笛卡尔阻抗":
            if selected_state == '关节阻抗':
                imp_type=1
            elif selected_state == "笛卡尔阻抗":
                imp_type= 2
            if messagebox.askyesno("确认状态切换", f"确定要切换到 {selected_state} 模式吗？"):
                try:
                    robot.clear_set()
                    robot.set_state(arm=robot_id, state=3)
                    robot.set_impedance_type(arm=robot_id, type=imp_type)
                    robot.send_cmd()
                except Exception as e:
                    messagebox.showerror("错误", f"状态切换过程中发生错误: {str(e)}")
                    self.state_var.set("关节跟随")

        # 处理PVT模式
        elif selected_state == 'PVT':
            if messagebox.askyesno("确认状态切换", f"确定要切换到 {selected_state} 模式吗？"):
                try:
                    robot.clear_set()
                    robot.set_state(arm=robot_id, state=2)
                    robot.send_cmd()
                except Exception as e:
                    messagebox.showerror("错误", f"状态切换过程中发生错误: {str(e)}")
                    self.state_var.set("关节跟随")
                # 创建拖动选择弹窗
                drag_dialog = tk.Toplevel(self.root)
                drag_dialog.title("拖动选项")
                drag_dialog.geometry("500x350")  # 增加高度以容纳新选项
                drag_dialog.transient(self.root)
                drag_dialog.grab_set()

                # 设置对话框居中显示
                drag_dialog.update_idletasks()
                x = (drag_dialog.winfo_screenwidth() - drag_dialog.winfo_width()) // 2
                y = (drag_dialog.winfo_screenheight() - drag_dialog.winfo_height()) // 2
                drag_dialog.geometry(f"+{x}+{y}")

                # 第一行：是否保存拖动数据复选框
                pvt_frame = tk.Frame(drag_dialog)
                pvt_frame.pack(fill="x", pady=(15, 10), padx=20)

                # 2选择PVT号
                pvt_b_text_label = tk.Label(pvt_frame, text="选择PVT号1~99", bg='white')
                pvt_b_text_label.grid(row=0, column=0, padx=5, sticky="ew")
                # 3PVT id
                self.pvt_b_entry = tk.Entry(pvt_frame, )
                self.pvt_b_entry.insert(0, "1")
                self.pvt_b_entry.grid(row=0, column=1, padx=5)
                # 4上传PVT
                send_pvt_b_button = tk.Button(pvt_frame, text="上传PVT",
                                              command=lambda: self.send_pvt(robot_id))
                send_pvt_b_button.grid(row=0, column=2, padx=5)
                # 5运行PVT
                run_pvt_b_button = tk.Button(pvt_frame, text="运行PVT",
                                             command=lambda: self.run_pvt(robot_id))
                run_pvt_b_button.grid(row=0, column=3, padx=5)

        # 处理关节跟随模式
        elif selected_state == '关节跟随':
            if messagebox.askyesno("确认状态切换", f"确定要切换到 {selected_state} 模式吗？"):
                try:
                    robot.clear_set()
                    robot.set_state(arm=robot_id, state=1)
                    robot.send_cmd()
                except Exception as e:
                    messagebox.showerror("错误", f"状态切换过程中发生错误: {str(e)}")
                    self.state_var.set("关节跟随")

        # 处理拖动模式 - 新增部分
        elif selected_state == '拖动':
            # 创建拖动选择弹窗
            drag_dialog = tk.Toplevel(self.root)
            drag_dialog.title("拖动选项")
            drag_dialog.geometry("300x500")
            drag_dialog.transient(self.root)
            drag_dialog.grab_set()

            # 设置对话框居中显示
            drag_dialog.update_idletasks()
            x = (drag_dialog.winfo_screenwidth() - drag_dialog.winfo_width()) // 2
            y = (drag_dialog.winfo_screenheight() - drag_dialog.winfo_height()) // 2
            drag_dialog.geometry(f"+{x}+{y}")

            # 创建变量存储选择
            drag_var = tk.StringVar()
            drag_type_var = tk.StringVar()  # 用于存储关节/笛卡尔拖动类型
            axis_var = tk.StringVar()  # 用于存储轴选择
            save_drag_var = tk.BooleanVar(value=False)  # 新增：是否保存拖动数据

            # 第一行：是否保存拖动数据复选框
            save_frame = tk.Frame(drag_dialog)
            save_frame.pack(fill="x", pady=(15, 10), padx=20)

            save_checkbox = tk.Checkbutton(
                save_frame,
                text="保存拖动数据",
                variable=save_drag_var,
                font=('Arial', 10),
                # bg='white'
            )
            save_checkbox.pack(anchor='w')

            # 添加分隔线
            separator1 = ttk.Separator(drag_dialog, orient='horizontal')
            separator1.pack(fill="x", padx=20, pady=(0, 10))

            # 第二行：拖动类型标题
            tk.Label(drag_dialog, text="选择拖动类型:", font=('Arial', 10)).pack(pady=(0, 5), anchor='w', padx=20)

            # 关节拖动选项
            joint_frame = tk.Frame(drag_dialog)
            joint_frame.pack(anchor='w', padx=40, pady=5)

            tk.Radiobutton(
                joint_frame,
                text="关节拖动",
                variable=drag_type_var,
                value="joint_drag",
                font=('Arial', 9),
                command=lambda: axis_var.set("")  # 清空轴选择
            ).pack(anchor='w')

            # 第三行：笛卡尔拖动单选按钮
            cartesian_frame = tk.Frame(drag_dialog)
            cartesian_frame.pack(anchor='w', padx=40, pady=5)

            tk.Radiobutton(
                cartesian_frame,
                text="笛卡尔拖动",
                variable=drag_type_var,
                value="cartesian_drag",
                font=('Arial', 9)
            ).pack(anchor='w')

            # 笛卡尔拖动的轴选择
            axis_frame = tk.Frame(drag_dialog)
            axis_frame.pack(anchor='w', padx=60, pady=5)

            tk.Label(axis_frame, text="选择拖动轴:", font=('Arial', 9)).pack(anchor='w', pady=(5, 0))

            # 创建轴选择单选按钮组
            axis_options = ["X拖动", "Y拖动", "Z拖动", "R拖动"]

            for i, axis in enumerate(axis_options):
                tk.Radiobutton(
                    axis_frame,
                    text=axis,
                    variable=axis_var,
                    value=axis,
                    font=('Arial', 9),
                    state=tk.DISABLED  # 初始禁用，等待选择笛卡尔拖动
                ).pack(anchor='w')

            # 更新笛卡尔拖动选择时启用轴选择
            def enable_axis_selection():
                if drag_type_var.get() == "cartesian_drag":
                    for widget in axis_frame.winfo_children():
                        if isinstance(widget, tk.Radiobutton):
                            widget.config(state=tk.NORMAL)
                else:
                    for widget in axis_frame.winfo_children():
                        if isinstance(widget, tk.Radiobutton):
                            widget.config(state=tk.DISABLED)
                    axis_var.set("")  # 清空轴选择

            # 绑定拖动类型改变事件
            drag_type_var.trace('w', lambda *args: enable_axis_selection())

            # 确认按钮
            def confirm_drag_selection():
                self.drag_type = drag_type_var.get()
                self.selected_axis = axis_var.get()
                save_drag_data = save_drag_var.get()  # 获取是否保存拖动数据

                if not self.drag_type:
                    messagebox.showwarning("选择错误", "请选择拖动类型")
                    return

                if self.drag_type == "cartesian_drag" and not self.selected_axis:
                    messagebox.showwarning("选择错误", "请选择笛卡尔拖动的轴")
                    return

                # 映射拖动类型到机器人状态
                drag_mapping = {
                    "joint_drag": ("joint_drag", "关节拖动"),
                    "cartesian_drag_X": ("cartesian_drag_X", "笛卡尔X拖动"),
                    "cartesian_drag_Y": ("cartesian_drag_Y", "笛卡尔Y拖动"),
                    "cartesian_drag_Z": ("cartesian_drag_Z", "笛卡尔Z拖动"),
                    "cartesian_drag_R": ("cartesian_drag_R", "笛卡尔R拖动")
                }

                if self.drag_type == "joint_drag":
                    state_key, state_name = drag_mapping["joint_drag"]
                else:
                    axis_map = {"X拖动": "X", "Y拖动": "Y", "Z拖动": "Z", "R拖动": "R"}
                    axis_key = axis_map[self.selected_axis]
                    state_key, state_name = drag_mapping[f"cartesian_drag_{axis_key}"]

                if messagebox.askyesno("确认拖动设置", f"确定要设置为 {state_name} 模式吗？"):
                    try:
                        # 如果勾选了保存拖动数据，启动保存线程
                        if save_drag_data:
                            self.thread_drag_save(robot_id)

                        if self.drag_mode == True:
                            robot.clear_set()
                            robot.set_drag_space(arm=robot_id, dgType=0)
                            robot.send_cmd()
                            time.sleep(0.02)
                            self.drag_mode = False

                        if self.drag_type == 'joint_drag':
                            robot.clear_set()
                            robot.set_state(arm=robot_id, state=3)
                            robot.set_impedance_type(arm=robot_id, type=1)
                            robot.send_cmd()
                            time.sleep(0.02)
                            robot.clear_set()
                            robot.set_drag_space(arm=robot_id, dgType=1)
                            robot.send_cmd()
                            time.sleep(0.02)
                            self.drag_mode = True
                        elif self.drag_type == 'cartesian_drag':
                            if self.selected_axis == "X拖动":
                                robot.clear_set()
                                robot.set_state(arm=robot_id, state=3)
                                robot.set_impedance_type(arm=robot_id, type=2)
                                robot.send_cmd()
                                time.sleep(0.02)
                                robot.clear_set()
                                robot.set_drag_space(arm=robot_id, dgType=2)
                                robot.send_cmd()
                                time.sleep(0.02)
                                self.drag_mode = True
                            elif self.selected_axis == "Y拖动":
                                robot.clear_set()
                                robot.set_state(arm=robot_id, state=3)
                                robot.set_impedance_type(arm=robot_id, type=2)
                                robot.send_cmd()
                                time.sleep(0.02)
                                robot.clear_set()
                                robot.set_drag_space(arm=robot_id, dgType=3)
                                robot.send_cmd()
                                time.sleep(0.02)
                                self.drag_mode = True
                            elif self.selected_axis == "Z拖动":
                                robot.clear_set()
                                robot.set_state(arm=robot_id, state=3)
                                robot.set_impedance_type(arm=robot_id, type=2)
                                robot.send_cmd()
                                time.sleep(0.02)
                                robot.clear_set()
                                robot.set_drag_space(arm=robot_id, dgType=4)
                                robot.send_cmd()
                                time.sleep(0.02)
                                self.drag_mode = True
                            elif self.selected_axis == "R拖动":
                                robot.clear_set()
                                robot.set_state(arm=robot_id, state=3)
                                robot.set_impedance_type(arm=robot_id, type=2)
                                robot.send_cmd()
                                time.sleep(0.02)
                                robot.clear_set()
                                robot.set_drag_space(arm=robot_id, dgType=5)
                                robot.send_cmd()
                                time.sleep(0.02)
                                self.drag_mode = True
                        self.state_var.set(f"{state_name}")
                        drag_dialog.destroy()
                    except Exception as e:
                        messagebox.showerror("错误", f"设置拖动模式时发生错误: {str(e)}")
                else:
                    self.state_var.set("关节跟随")
                    drag_dialog.destroy()

            # 取消按钮
            def cancel_drag_selection():
                self.state_var.set("关节跟随")
                drag_dialog.destroy()

            # 添加分隔线
            separator2 = ttk.Separator(drag_dialog, orient='horizontal')
            separator2.pack(fill="x", padx=20, pady=(10, 5))

            # 按钮框架
            button_frame = tk.Frame(drag_dialog)
            button_frame.pack(pady=10)

            tk.Button(
                button_frame,
                text="确定",
                width=10,
                command=confirm_drag_selection,
                bg="#4CAF50",
                fg="white"
            ).pack(side="left", padx=5, pady=5)

            tk.Button(
                button_frame,
                text="取消",
                width=10,
                command=cancel_drag_selection,
                bg="#F44336",
                fg="white"
            ).pack(side="left", padx=5, pady=5)

            # 等待对话框关闭
            self.root.wait_window(drag_dialog)
        else:
            self.state_var.set("关节跟随")

    def send_pvt(self, robot_id):
        if self.connected:
            file_path = filedialog.askopenfilename(
                title="选择数据文件",
                filetypes=[("文本文件", "*.txt"), ("fmv文件", "*.fmv"), ("所有文件", "*.*")]
            )
            if file_path:
                print(f'pvt file_path:{file_path}')
                if robot_id == 'A':
                    print(f'pvt id:{int(self.pvt_b_entry.get())}')
                    robot.send_pvt_file(arm=robot_id, pvt_path=file_path, id=int(self.pvt_b_entry.get()))
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
                robot.set_pvt_id(arm=robot_id, id=int(self.pvt_b_entry.get()))
                robot.send_cmd()
            elif robot_id == 'B':
                robot.clear_set()
                robot.set_state(arm=robot_id, state=2)  # PVT
                robot.set_pvt_id(arm=robot_id, id=int(self.pvt_b_entry.get()))
                robot.send_cmd()
        else:
            messagebox.showerror('error', '请先连接机器人')

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
                time.sleep(0.01)
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
                time.sleep(1)
                messagebox.showinfo("成功", f"拖动轨迹数据已保存到: {os.path.basename(file_path)}，\n请退出拖动。")
            except Exception as e:
                messagebox.showerror("错误", f"保存文件时出错: {str(e)}")

    def create_control_components(self):
        """创建顶部控制面板"""
        self.control_frame = tk.Frame(self.root, bg="#e0e0e0", pady=5)
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
        self.connect_btn.pack(side="left", padx=5)

        self.arm_ip_entry = tk.Entry(self.control_frame)
        self.arm_ip_entry.insert(0, "192.168.1.190")
        self.arm_ip_entry.pack(side="left", padx=5)

        # 更多功能菜单按钮
        self.more_features_btn = tk.Button(
            self.control_frame,
            text="更多功能",
            width=15,
            command=self.show_more_features,
            bg="#3BA4FD",
            fg="white",
            font=("Arial", 10, "bold")
        )
        self.more_features_btn.pack(side="right", padx=5)

        '''###########################mor more more############################'''

        # 数采及处理
        self.mode_btn = tk.Button(
            self.control_frame,
            text="数采及处理",
            width=15,
            command=self.data_collect_and_process_dialog,
            bg="#3BA4FD",
            fg="white",
            font=("Arial", 10, "bold"))
        self.mode_btn.pack(side="right", padx=5)

        # 末端透传
        self.mode_btn = tk.Button(
            self.control_frame,
            text="末端透传",
            width=15,
            command=self.eef_dialog,
            bg="#3BA4FD",
            fg="white",
            font=("Arial", 10, "bold"))
        self.mode_btn.pack(side="right", padx=5)

        # 工具参数设置
        self.mode_btn = tk.Button(
            self.control_frame,
            text="工具参数设置",
            width=15,
            command=self.set_tool_dialog,
            bg="#3BA4FD",
            fg="white",
            font=("Arial", 10, "bold"))
        self.mode_btn.pack(side="right", padx=5)

        # 工具动力学辨识
        self.mode_btn = tk.Button(
            self.control_frame,
            text="工具动力学辨识",
            width=15,
            command=self.tool_identy_dialog,
            bg="#3BA4FD",
            fg="white",
            font=("Arial", 10, "bold"))
        self.mode_btn.pack(side="right", padx=5)

        # 模式切换按钮
        self.mode_btn = tk.Button(
            self.control_frame,
            text="位置数据",
            width=15,
            command=self.toggle_display_mode,
            bg="#3BA4FD",
            fg="white",
            font=("Arial", 10, "bold"))
        self.mode_btn.pack(side="right", padx=5)

        # 状态指示灯
        status_frame = tk.Frame(self.control_frame, bg="#e0e0e0")
        status_frame.pack(side="right", padx=5)
        self.status_light = tk.Label(status_frame, text="●", font=("Arial", 16), fg="red")
        self.status_light.pack(side="left", padx=5)
        self.status_label = tk.Label(status_frame, text="未连接", bg="#e0e0e0", font=("Arial", 9))
        self.status_label.pack(side="left")

    def create_status_bar(self):
        """创建底部状态栏"""
        self.status_bar = tk.Frame(self.root, height=20)
        self.status_bar.pack(side="bottom", fill="x")
        self.version_label = tk.Label(
            self.status_bar, text=f"", fg="black", font=("Arial", 9))
        self.version_label.pack(side="left", padx=15)
        self.time_label = tk.Label(
            self.status_bar, text="", fg="black", font=("Arial", 9))
        self.time_label.pack(side="right", padx=15)
        self.update_time()

    def update_time(self):
        """更新时间显示"""
        current_time = time.strftime("%Y-%m-%d %H:%M:%S")
        self.time_label.config(text=current_time)
        self.version_label.config(text=f"controller version:{self.version}")
        self.root.after(1000, self.update_time)

    def on_mousewheel(self, event):
        """鼠标滚轮事件"""
        self.main_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    def on_menu_click(self, item):
        """菜单点击事件"""
        print(f"菜单点击: {item}")

    def on_close(self):
        """窗口关闭事件"""
        if messagebox.askokcancel("退出", "确定要退出应用程序吗?"):
            '''save tools txt'''
            robot.send_file(self.tools_txt, os.path.join('/home/fusion/', self.tools_txt))
            time.sleep(0.2)
            if os.path.exists(self.tools_txt):
                os.remove(self.tools_txt)
            if self.data_subscriber:
                self.data_subscriber.stop()
            self.root.destroy()
            robot.release_robot()

    def error_get(self, robot_id):
        if self.connected:
            errors = robot.get_servo_error_code(robot_id)
            print(f'servo error:{errors}')
            if errors:
                messagebox.showinfo(f'{robot_id} arm error:\n', errors)
        else:
            messagebox.showerror('error', '请先连接机器人')

    def cr_state(self, robot_id):
        if self.connected:
            robot.clear_set()
            robot.set_state(arm=robot_id, state=4)
            robot.send_cmd()
        else:
            messagebox.showerror('error', '请先连接机器人')

    def error_clear(self, robot_id):
        if self.connected:
            robot.clear_error(robot_id)
        else:
            messagebox.showerror('error', '请先连接机器人')

    def brake(self, robot_id):
        if self.connected:
            messagebox.showinfo('提示', '请确实伺服参数是否为166混合控制模式')
            if robot_id == 'A':
                robot.set_param('int', 'BRAK0', 1)
            elif robot_id == 'B':
                robot.set_param('int', 'BRAK1', 1)
        else:
            messagebox.showerror('error', '请先连接机器人')

    def release_brake(self, robot_id):
        if self.connected:
            messagebox.showinfo('提示', '请确实伺服参数是否为166混合控制模式')
            if robot_id == 'A':
                robot.set_param('int', 'BRAK0', 2)
            elif robot_id == 'B':
                robot.set_param('int', 'BRAK1', 2)
        else:
            messagebox.showerror('error', '请先连接机器人')

    def data_collect_and_process_dialog(self):
        drag_dialog = tk.Toplevel(self.root)
        drag_dialog.title("数据采集及处理")
        drag_dialog.geometry("1200x600")  # 调整尺寸以适应内容
        drag_dialog.resizable(True, True)  # 允许调整大小以便查看完整内容
        drag_dialog.transient(self.root)
        drag_dialog.grab_set()

        # 设置对话框居中显示
        drag_dialog.update_idletasks()
        x = (drag_dialog.winfo_screenwidth() - drag_dialog.winfo_width()) // 2
        y = (drag_dialog.winfo_screenheight() - drag_dialog.winfo_height()) // 2
        drag_dialog.geometry(f"+{x}+{y}")

        # 确保属性已初始化
        if not hasattr(self, 'file_path_50'):
            self.file_path_50 = tk.StringVar()
        if not hasattr(self, 'file_path_collect'):
            self.file_path_collect=tk.StringVar()

        # 创建主框架，使用pack布局
        parent = tk.Frame(drag_dialog, bg="white", padx=10, pady=10)
        parent.pack(fill="both", expand=True)

        self.frame_data_11 = tk.Frame(parent, bg="white")
        self.frame_data_11.pack(fill="x", pady=15)
        # 查看文档
        self.read_file_button = tk.Button(self.frame_data_11, text="采集ID说明", width=15, command=preview_text_file_1,
                                          font=("Arial", 10, "bold"))
        self.read_file_button.grid(row=0, column=0, padx=5)

        self.frame_data_2 = tk.Frame(parent, bg="white")
        self.frame_data_2.pack(fill="x",pady=15)
        # 第一列：collect 1 arm' data
        self.collect_btn_1 = tk.Button(self.frame_data_2, text="左臂开始采集", command=lambda: self.collect_data('A'))
        self.collect_btn_1.grid(row=0, column=0, padx=5)

        # 第8列：stop collect
        self.stop_collect_btn_1 = tk.Button(self.frame_data_2, text="停止", command=self.stop_collect_data_both)
        self.stop_collect_btn_1.grid(row=0, column=1, padx=5)

        # 第3列：save collect
        self.save_collect_btn_1 = tk.Button(self.frame_data_2, text="保存", command=self.save_collect_data_both)
        self.save_collect_btn_1.grid(row=0, column=2, padx=5)

        # 第2列：特征个数
        self.feature_1 = tk.Label(self.frame_data_2, text="特征个数", bg='white')
        self.feature_1.grid(row=0, column=3, padx=5)

        # 第3列：特征个数
        self.features_entry_1 = tk.Entry(self.frame_data_2, width=3)
        self.features_entry_1.insert(0, '7')
        self.features_entry_1.grid(row=0, column=4, padx=5)

        # 第4列：特征
        self.feature_idx_1 = tk.Label(self.frame_data_2, text="特征IDX", bg='white')
        self.feature_idx_1.grid(row=0, column=5, padx=5)

        # 第5列：特征
        self.entry_var_raw_1 = tk.StringVar(
            value="[0,1,2,3,4,5,6,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]")
        self.feature_idx_entry_1 = tk.Entry(self.frame_data_2, textvariable=self.entry_var_raw_1, width=100)
        self.feature_idx_entry_1.grid(row=0, column=6, padx=5, sticky="ew")

        # 第6列：行数文本
        self.lines_1 = tk.Label(self.frame_data_2, text="行数", bg='white')
        self.lines_1.grid(row=0, column=7, padx=5)

        # 第7列：行数
        self.lines_entry_1 = tk.Entry(self.frame_data_2, width=5)
        self.lines_entry_1.insert(0, '1000')
        self.lines_entry_1.grid(row=0, column=8, padx=5)



        self.frame_data_3 = tk.Frame(parent, bg="white")
        self.frame_data_3.pack(fill="x",pady=15)
        # 第一列：collect 1 arm' data
        self.collect_btn_2 = tk.Button(self.frame_data_3, text="右臂开始采集", command=lambda: self.collect_data('B'))
        self.collect_btn_2.grid(row=0, column=0, padx=5)

        # 第8列：stop collect
        self.stop_collect_btn_2 = tk.Button(self.frame_data_3, text="停止", command=self.stop_collect_data_both)
        self.stop_collect_btn_2.grid(row=0, column=1, padx=5)

        # 第3列：save collect
        self.save_collect_btn_2 = tk.Button(self.frame_data_3, text="保存", command=self.save_collect_data_both)
        self.save_collect_btn_2.grid(row=0, column=2, padx=5, pady=5)

        # 第2列：特征个数
        self.feature_2 = tk.Label(self.frame_data_3, text="特征个数", bg='white')
        self.feature_2.grid(row=0, column=3, padx=5)

        # 第3列：特征个数
        self.features_entry_2 = tk.Entry(self.frame_data_3, width=3)
        self.features_entry_2.insert(0, '7')
        self.features_entry_2.grid(row=0, column=4, padx=5)

        # 第4列：特征
        self.feature_idx_2 = tk.Label(self.frame_data_3, text="特征IDX", bg='white')
        self.feature_idx_2.grid(row=0, column=5, padx=5)

        # 第5列：特征
        self.entry_var_raw_2 = tk.StringVar(
            value="[100,101,102,103,104,105,106,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]")
        self.feature_idx_entry_2 = tk.Entry(self.frame_data_3, textvariable=self.entry_var_raw_2, width=100)
        self.feature_idx_entry_2.grid(row=0, column=6, padx=5, sticky="ew")

        # 第6列：行数文本
        self.lines_2 = tk.Label(self.frame_data_3, text="行数", bg='white')
        self.lines_2.grid(row=0, column=7, padx=5)

        # 第7列：行数
        self.lines_entry_2 = tk.Entry(self.frame_data_3, width=5)
        self.lines_entry_2.insert(0, '1000')
        self.lines_entry_2.grid(row=0, column=8, padx=5)

        self.frame_data_1 = tk.Frame(parent, bg="white")
        self.frame_data_1.pack(fill="x",pady=15)

        # 第一列：collect 2 arms' data
        self.collect_both_btn = tk.Button(self.frame_data_1, text="同时采集两个手臂的关节位置数据", command=self.collect_data_both)
        self.collect_both_btn.grid(row=0, column=0, padx=5)

        # 第2列：stop collect
        self.stop_collect_both_btn = tk.Button(self.frame_data_1, text="停止", command=self.stop_collect_data_both)
        self.stop_collect_both_btn.grid(row=0, column=1, padx=5)

        # 第3列：save collect
        self.save_collect_both_btn = tk.Button(self.frame_data_1, text="保存", command=self.save_collect_data_both)
        self.save_collect_both_btn.grid(row=0, column=2, padx=5)

        self.frame_data_4 = tk.Frame(parent, bg="white")
        self.frame_data_4.pack(fill="x",pady=15)

        self.text_50_load_file = tk.Label(self.frame_data_4, text='数据下采样50HZ', bg='#cde6c7')
        self.text_50_load_file.grid(row=0, column=0, padx=3)

        self.btn_load_file_50 = tk.Button(self.frame_data_4, text="选择文件", command=self.select_50_file)
        self.btn_load_file_50.grid(row=0, column=1, padx=5)

        self.path_50 = tk.Entry(self.frame_data_4, textvariable=self.file_path_50, width=75,
                                font=("Arial", 7), state="readonly")
        self.path_50.grid(row=0, column=2, padx=5, sticky="ew")

        self.run_generate_50 = tk.Button(self.frame_data_4, text="生成50点位", command=self.generate_50_file)
        self.run_generate_50.grid(row=0, column=3, padx=5)


        self.frame_data_6 = tk.Frame(parent, bg="white")
        self.frame_data_6.pack(fill="x", pady=15)

        self.text_collect_to_pvt = tk.Label(self.frame_data_6, text='采集数据处理为PVT格式', bg='#6FF2E0')
        self.text_collect_to_pvt.grid(row=0, column=0, padx=3)

        self.btn_load_collect = tk.Button(self.frame_data_6, text="选择文件", command=self.select_collect_file)
        self.btn_load_collect.grid(row=0, column=1, padx=5)

        self.path_collect = tk.Entry(self.frame_data_6, textvariable=self.file_path_collect, width=75,
                                font=("Arial", 7), state="readonly")
        self.path_collect.grid(row=0, column=2, padx=5, sticky="ew")

        self.run_generate_pvt = tk.Button(self.frame_data_6, text="处理并保存", command=self.generate_pvt_file)
        self.run_generate_pvt.grid(row=0, column=3, padx=5)


        self.frame_data_5 = tk.Frame(parent, bg="white")
        self.frame_data_5.pack(fill="x",pady=15)

        self.text_50_load_file = tk.Label(self.frame_data_5, text='提示：改变特征个数，需要退出机器人连接后，重连再采集数据。（如果先启动位置同步采集，需要重连后再采单臂；先采单臂，要采双臂数据，也需要重连）', bg='#FBFAD4')
        self.text_50_load_file.grid(row=0, column=0, padx=3)

    def select_collect_file(self):
        file_path = filedialog.askopenfilename(
            defaultextension=".txt",
            filetypes=[("txt files", "*.txt"), ("All files", "*.*")],
            title="选择数据文件"
        )
        if file_path:
            self.file_path_collect.set(file_path)

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
    def generate_pvt_file(self):
        result = messagebox.askokcancel("确认操作", "数据将保存为PVT格式，保存路径与源文件一致，以‘proceesed_'开头，确认处理并保存吗？")
        if result:
            process_and_downsample(file_path= self.file_path_collect.get(),format_unify=True)

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
                print(f'idx:{idx}')
                rows = int(self.lines_entry_1.get())

            if robot_id == 'B':
                cols = int(self.features_entry_2.get())
                idx = ast.literal_eval(self.feature_idx_entry_2.get())
                print(f'idx:{idx}')
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

    def get_sensor_offset(self, robot_id):
        if self.connected:
            if robot_id == 'A':
                axis = int(self.axis_select_combobox_1.get())
                self.m_sq_offset_1 = self.result['outputs'][0]['fb_joint_sToq'][axis]
                print(f'**** self.m_sq_offset_1:{self.m_sq_offset_1}')
                self.get_offset_entry_1.delete(0, tk.END)
                self.get_offset_entry_1.insert(0, self.m_sq_offset_1)
            if robot_id == 'B':
                axis = int(self.axis_select_combobox_2.get())
                self.m_sq_offset_2 = self.result['outputs'][1]['fb_joint_sToq'][axis]
                print(f'**** self.m_sq_offset_2:{self.m_sq_offset_2}')
                self.get_offset_entry_2.delete(0, tk.END)
                self.get_offset_entry_2.insert(0, self.m_sq_offset_2)

        else:
            messagebox.showerror('error', '请先连接机器人')

    def set_sensor_offset(self, robot_id):  # todo
        if self.connected:
            if robot_id == 'A':
                if self.result['states'][0]["cur_state"] != 0:
                    messagebox.showerror('error', '左臂必须在复位状态才可设置传感器偏移')
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
                if self.result['states'][1]["cur_state"] != 0:
                    messagebox.showerror('error', '右臂必须在复位状态才可设置传感器偏移')
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
            result = messagebox.askokcancel("确认操作", "编码器清零后，机器人将丢失原点，确认清零吗？")
            if result:
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
                print('取消编码器清零')
        else:
            messagebox.showerror('error', '请先连接机器人')

    def clear_motorE_as_zero(self, robot_id):
        if self.connected:
            result = messagebox.askokcancel("确认操作", "编码器清零后，机器人将丢失原点，确认清零吗？")
            if result:
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
                print('取消外编清零')
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

    def eef_dialog(self):
        drag_dialog = tk.Toplevel(self.root)
        drag_dialog.title("末端工具通讯")
        drag_dialog.geometry("1300x400")  # 调整尺寸以适应内容
        drag_dialog.resizable(True, True)  # 允许调整大小以便查看完整内容
        drag_dialog.transient(self.root)
        drag_dialog.grab_set()

        # 设置对话框居中显示
        drag_dialog.update_idletasks()
        x = (drag_dialog.winfo_screenwidth() - drag_dialog.winfo_width()) // 2
        y = (drag_dialog.winfo_screenheight() - drag_dialog.winfo_height()) // 2
        drag_dialog.geometry(f"+{x}+{y}")

        # 创建主框架，使用pack布局
        parent = tk.Frame(drag_dialog, bg="white", padx=10, pady=10)
        parent.pack(fill="both", expand=True)

        self.eef_frame_1 = tk.Frame(parent, bg="white")
        self.eef_frame_1.pack(fill="x")
        # 第1 :text
        self.eef_text_1 = tk.Button(self.eef_frame_1, text="左臂发送", command=lambda: self.send_data_eef('A'))
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

        self.eef_delet_1 = tk.Button(self.eef_frame_1, text="删除选中", command=lambda: self.delete_eef_command('A'))
        self.eef_delet_1.grid(row=0, column=3, padx=5, pady=5)

        self.eef_combo1 = ttk.Combobox(self.eef_frame_1, state="readonly", width=120)
        self.eef_combo1.grid(row=0, column=4, padx=5)

        self.eef_bt_1 = tk.Button(self.eef_frame_1, text="左臂接收", command=lambda: self.receive_data_eef('A'))
        self.eef_bt_1.grid(row=0, column=5, padx=5)

        self.eef_frame_1_2 = tk.Frame(parent, bg="white")
        self.eef_frame_1_2.pack(fill="x")

        self.eef1_2_b1 = tk.Label(self.eef_frame_1_2, text="", bg="white", width=7)
        self.eef1_2_b1.grid(row=0, column=0, padx=5)

        self.eef1_2_b2 = tk.Label(self.eef_frame_1_2, text="", bg="white", width=7)
        self.eef1_2_b2.grid(row=0, column=1, padx=5)

        self.eef1_2_b3 = tk.Label(self.eef_frame_1_2, text="", bg="white", width=7)
        self.eef1_2_b3.grid(row=0, column=2, padx=5)

        self.eef_add_1 = tk.Button(self.eef_frame_1_2, text='左臂加指令', command=lambda: self.add_eef_command('A'))
        self.eef_add_1.grid(row=0, column=3, padx=5)

        self.eef_entry = tk.Entry(self.eef_frame_1_2, width=120)
        self.eef_entry.insert(0, "01 06 00 00 00 01 48 0A")
        self.eef_entry.grid(row=0, column=4, padx=5, sticky="ew")

        self.eef_add_2 = tk.Button(self.eef_frame_1_2, text='右臂加指令', command=lambda: self.add_eef_command('B'))
        self.eef_add_2.grid(row=0, column=5, padx=5)

        self.eef_frame_2 = tk.Frame(parent, bg="white")
        self.eef_frame_2.pack(fill="x")
        # 第1 :text
        self.eef_bt_2 = tk.Button(self.eef_frame_2, text="右臂发送", command=lambda: self.send_data_eef('B'))
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

        self.eef_delet_2 = tk.Button(self.eef_frame_2, text="删除选中", command=lambda: self.delete_eef_command('B'))
        self.eef_delet_2.grid(row=0, column=3, padx=5, pady=5)

        self.eef_combo2 = ttk.Combobox(self.eef_frame_2, state="readonly", width=120)
        self.eef_combo2.grid(row=0, column=4, padx=5)

        self.eef_bt_4 = tk.Button(self.eef_frame_2, text="右臂接收", command=lambda: self.receive_data_eef('B'))
        self.eef_bt_4.grid(row=0, column=5, padx=5, pady=5)

        self.eef_frame_3 = tk.Frame(parent, bg="white")
        self.eef_frame_3.pack(fill="x")

        # 接收内容文本框
        recv_label1 = tk.Label(self.eef_frame_3, text="左臂接收内容:")
        recv_label1.grid(row=0, column=0, padx=5)

        # 间隔
        spacer = tk.Label(self.eef_frame_3, text="   ", bg='white')
        spacer.grid(row=0, column=1, padx=5)

        self.recv_text1 = scrolledtext.ScrolledText(self.eef_frame_3, width=70, height=8, wrap=tk.WORD)
        self.recv_text1.grid(row=1, column=0, padx=5)
        self.recv_text1.insert(tk.END,
                               '使用提示：\n请先选择端口：CAN/COM1/COM2, \n点击 1#末端接收按钮， \n输入发送数据，点击 1#末端发送按钮, \n接收到的末端信息以1khz频率刷新显示')

        # 间隔
        spacer1 = tk.Label(self.eef_frame_3, text="   ", bg='white')
        spacer1.grid(row=1, column=1, padx=5)

        # 接收内容文本框
        recv_label2 = tk.Label(self.eef_frame_3, text="右臂接收内容:")
        recv_label2.grid(row=0, column=2, padx=5)

        self.recv_text2 = scrolledtext.ScrolledText(self.eef_frame_3, width=70, height=8, wrap=tk.WORD)
        self.recv_text2.grid(row=1, column=2, padx=5)
        self.recv_text2.insert(tk.END,
                               '使用提示：\n请先选择端口：CAN/COM1/COM2, \n点击 2#末端接收按钮， \n输入发送数据，点击 2#末端发送按钮, \n接收到的末端信息以1khz频率刷新显示')

        # 添加状态显示区域
        status_display_frame_7 = tk.Frame(parent, bg="white", padx=10, pady=5)
        status_display_frame_7.pack(fill="x", pady=5)

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


    def send_data_eef(self, robot_id):
        if self.connected:
            try:
                com = 0
                com_ = ''
                sample_data = None
                robot.clear_485_cache(robot_id)
                time.sleep(0.5)
                if robot_id == 'A':
                    sample_data = self.eef_combo1.get()
                    print(f'sample_data:{sample_data}')
                    com_ = self.com_select_combobox_1.get()
                elif robot_id == 'B':
                    sample_data = self.eef_combo2.get()
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
                if received_count > 0:
                    if len(received_data) > 0:
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

    def delete_eef_command(self, robot_id):
        """从2#列表删除选中的点"""
        if robot_id == 'A':
            selected_index = self.eef_combo1.current()
            if selected_index != -1 and selected_index < len(self.command1):
                self.command1.pop(selected_index)
                self.update_combo_eef()
            else:
                messagebox.showwarning("警告", "请选择要删除的通讯指令")
        elif robot_id == 'B':
            selected_index = self.eef_combo1.current()
            if selected_index != -1 and selected_index < len(self.command2):
                self.command2.pop(selected_index)
                self.update_combo_eef()
            else:
                messagebox.showwarning("警告", "请选择要删除的通讯指令")

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
                self.eef_thread = threading.Thread(target=read_data, args=(robot_id, com), daemon=True)
                self.eef_thread.start()

                received_count, received_data = get_received_data()
                if received_count > 0:
                    if len(received_data) > 0:
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

    def set_tool_dialog(self):
        drag_dialog = tk.Toplevel(self.root)
        drag_dialog.title("工具参数设置")
        drag_dialog.geometry("800x300")  # 调整尺寸以适应内容
        drag_dialog.resizable(True, True)  # 允许调整大小以便查看完整内容
        drag_dialog.transient(self.root)
        drag_dialog.grab_set()

        # 设置对话框居中显示
        drag_dialog.update_idletasks()
        x = (drag_dialog.winfo_screenwidth() - drag_dialog.winfo_width()) // 2
        y = (drag_dialog.winfo_screenheight() - drag_dialog.winfo_height()) // 2
        drag_dialog.geometry(f"+{x}+{y}")

        # 创建主框架，使用pack布局
        main_frame = tk.Frame(drag_dialog, bg="white", padx=10, pady=10)
        main_frame.pack(fill="both", expand=True)

        # 确保变量已初始化
        if not hasattr(self, 'tool_a_entry'):
            self.init_tool_variables()

        # 1设置工具参数
        tool_a_button = tk.Button(main_frame, text="设置左臂工具参数", width=20, bg='#7CCEF0',
                                  command=lambda: self.tool_set('A'))
        tool_a_button.grid(row=2, column=0, padx=5, pady=(5, 50))

        tool_a_label_1 = tk.Label(main_frame, text="工具动力学参数(M~I_zz)", width=20, bg='white')
        tool_a_label_1.grid(row=0, column=0, padx=5, pady=5)

        # 2tool entry - 使用已有的StringVar
        tool_a_entry_widget = tk.Entry(main_frame, textvariable=self.tool_a_entry, width=70)
        tool_a_entry_widget.grid(row=0, column=1, padx=5, sticky="ew")

        # 1设置工具运动学参数
        tool_a_label_2 = tk.Label(main_frame, text="工具运动学参数", width=20, bg='white')
        tool_a_label_2.grid(row=1, column=0, pady=5)

        # 2tool entry - 使用已有的StringVar
        tool_a1_entry_widget = tk.Entry(main_frame, textvariable=self.tool_a1_entry, width=70)
        tool_a1_entry_widget.grid(row=1, column=1, padx=5)

        # 1设置工具参数
        tool_b_button = tk.Button(main_frame, text="设置右臂工具参数", width=20, bg='#7CCEF0', command=lambda: self.tool_set('B'))
        tool_b_button.grid(row=5, column=0, padx=5,pady=5)

        tool_b_label_1 = tk.Label(main_frame, text="工具动力学参数(M~I_zz)", width=25, bg='white')
        tool_b_label_1.grid(row=3, column=0, padx=5)

        # 2tool entry
        tool_b_entry = tk.Entry(main_frame, textvariable=self.tool_b_entry,width=70)
        tool_b_entry.grid(row=3, column=1, padx=5, sticky="ew")

        # 1设置工具运动学参数
        tool_b_label_2 = tk.Label(main_frame, text="工具运动学参数", width=25, bg='white')
        tool_b_label_2.grid(row=4, column=0)

        tool_b1_entry = tk.Entry(main_frame, textvariable=self.tool_b1_entry,width=70)
        tool_b1_entry.grid(row=4, column=1, padx=5)

    def tool_identy_dialog(self):
        drag_dialog = tk.Toplevel(self.root)
        drag_dialog.title("工具动力学辨识")
        drag_dialog.geometry("1000x400")  # 调整尺寸以适应内容
        drag_dialog.resizable(True, True)  # 允许调整大小以便查看完整内容
        drag_dialog.transient(self.root)
        drag_dialog.grab_set()

        # 设置对话框居中显示
        drag_dialog.update_idletasks()
        x = (drag_dialog.winfo_screenwidth() - drag_dialog.winfo_width()) // 2
        y = (drag_dialog.winfo_screenheight() - drag_dialog.winfo_height()) // 2
        drag_dialog.geometry(f"+{x}+{y}")

        # 创建主框架，使用pack布局
        main_frame = tk.Frame(drag_dialog, bg="white", padx=10, pady=10)
        main_frame.pack(fill="both", expand=True)

        # ============ 第一行：标题和文件选择 ============
        identy_tool_frame = tk.Frame(main_frame, bg="white")
        identy_tool_frame.pack(fill="x", pady=(0, 10))

        # 机型选择标签
        robot_type_label = tk.Label(identy_tool_frame, text="选择机型", bg='white', width=8)
        robot_type_label.grid(row=0, column=1, padx=5)

        # 机型选择下拉框
        self.type_select_combobox_1 = ttk.Combobox(
            identy_tool_frame,
            values=["CCS", "SRS"],
            width=5,
            state="readonly"
        )
        self.type_select_combobox_1.current(0)
        self.type_select_combobox_1.grid(row=0, column=2, padx=5)

        # 选择轨迹文件按钮
        self.tool_trajectory_file = tk.Button(identy_tool_frame, text="选择轨迹文件",
                                         command=self.tool_trajectory)
        self.tool_trajectory_file.grid(row=0, column=3, padx=5)

        # 文件路径显示
        self.file_path_tool = tk.StringVar()  # 如果需要在类中访问，可以定义为实例变量
        self.path_tool = tk.Entry(identy_tool_frame, textvariable=self.file_path_tool, width=60,
                             font=("Arial", 8), state="readonly")
        self.path_tool.grid(row=0, column=4, padx=5, sticky="ew")

        # 配置列权重，让路径输入框扩展
        identy_tool_frame.grid_columnconfigure(4, weight=1)

        # ============ 第二行：数据采集按钮 ============
        identy_tool_frame2 = tk.Frame(main_frame, bg="white")
        identy_tool_frame2.pack(fill="x", pady=(0, 10))

        # 左侧空白标签
        tool_blank = tk.Label(identy_tool_frame2, text=" ", width=15, bg="white")
        tool_blank.grid(row=0, column=0, padx=5)

        # 左臂按钮
        collect_tool_btn = tk.Button(identy_tool_frame2, text="左臂空载数据采集",
                                     command=lambda: self.thread_collect_tool_data_no_load('A'))
        collect_tool_btn.grid(row=0, column=1, padx=5)

        collect_tool_btn2 = tk.Button(identy_tool_frame2, text="左臂带载数据采集",
                                      command=lambda: self.thread_collect_tool_data_with_load('A'))
        collect_tool_btn2.grid(row=0, column=2, padx=5)

        # 中间空白
        tool_blank1 = tk.Label(identy_tool_frame2, text=" ", width=5, bg="white")
        tool_blank1.grid(row=0, column=3, padx=5)

        # 工具辨识按钮
        tool_dyn_identy_btn = tk.Button(identy_tool_frame2, text="工具动力学辨识", bg='#afb4db',
                                        command=self.tool_dyn_identy)
        tool_dyn_identy_btn.grid(row=0, column=4, padx=5)

        # 右侧空白
        tool_blank3 = tk.Label(identy_tool_frame2, text=" ", width=5, bg="white")
        tool_blank3.grid(row=0, column=5, padx=5)

        # 右臂按钮
        collect_tool_btn1 = tk.Button(identy_tool_frame2, text="右臂空载数据采集",
                                      command=lambda: self.thread_collect_tool_data_no_load('B'))
        collect_tool_btn1.grid(row=0, column=6, padx=5)

        collect_tool_btn22 = tk.Button(identy_tool_frame2, text="右臂带载数据采集",
                                       command=lambda: self.thread_collect_tool_data_with_load('B'))
        collect_tool_btn22.grid(row=0, column=7, padx=5)

        # ============ 第三行：参数显示 ============
        identy_tool_frame1 = tk.Frame(main_frame, bg="white")
        identy_tool_frame1.pack(fill="x", pady=(0, 10))

        # 左侧空白
        tool_blank1_left = tk.Label(identy_tool_frame1, text=" ", width=5, bg="white")
        tool_blank1_left.grid(row=0, column=0, padx=5)

        # 参数标签
        robot_type_choose1 = tk.Label(identy_tool_frame1,
                                      text="工具动力学参数[m,mx,my,mz,ixx,ixy,ixz,iyy,iyz,izz]",
                                      bg='white', width=40)
        robot_type_choose1.grid(row=0, column=1, padx=5, pady=5)

        # 参数输入框
        self.entry_tool_dyn  = tk.StringVar(value="0,0,0,0,0,0,0,0,0,0")
        tool_dyn_entry = tk.Entry(identy_tool_frame1, textvariable=self.entry_tool_dyn , width=60)
        tool_dyn_entry.grid(row=0, column=2, padx=5, sticky="ew")


        close_button = tk.Button(identy_tool_frame1, text="一键导入到左臂工具动力学参数", width=20,command=lambda :self.load_tool_dyn_to_tool_api('A'))
        close_button.grid(row=1, column=1, padx=(10,5), sticky="ew")

        save_button = tk.Button(identy_tool_frame1, text="一键导入到右臂工具动力学参数", width=20,command=lambda :self.load_tool_dyn_to_tool_api('B'))
        save_button.grid(row=1, column=2, padx=(5,10), sticky="ew")

    def tool_set(self, robot_id):
        if self.connected:
            kine_p = 0
            dyn_p = 0
            if robot_id == 'A':
                kine_p = self.tool_a1_entry.get()
                dyn_p = self.tool_a_entry.get()
                # print(f'kine_p:{kine_p}')
                is_valid,kine_p=self.validate_point(kine_p,6)
                if is_valid:
                    values = kine_p.split(',')
                    kine_p = [float(value.strip()) for value in values]
                else:
                    messagebox.showerror("错误", f"{kine_p}")

                is_valid,dyn_p=self.validate_point(dyn_p,10)
                if is_valid:
                    values = dyn_p.split(',')
                    dyn_p = [float(value.strip()) for value in values]
                else:
                    messagebox.showerror("错误", f"{dyn_p}")

            elif robot_id == 'B':
                kine_p = self.tool_b1_entry.get()
                dyn_p = self.tool_b_entry.get()
                # print(f'kine_p:{kine_p}')
                is_valid, kine_p = self.validate_point(kine_p, 6)
                if is_valid:
                    values = kine_p.split(',')
                    kine_p = [float(value.strip()) for value in values]
                else:
                    messagebox.showerror("错误", f"{kine_p}")
                is_valid, dyn_p = self.validate_point(dyn_p, 10)
                if is_valid:
                    values = dyn_p.split(',')
                    dyn_p = [float(value.strip()) for value in values]
                else:
                    messagebox.showerror("错误", f"{dyn_p}")

            robot.clear_set()
            robot.set_tool(arm=robot_id, kineParams=kine_p, dynamicParams=dyn_p)
            robot.send_cmd()

            tool_mat = kk1.xyzabc_to_mat4x4(xyzabc=kine_p)
            if robot_id == "A":
                kk1.set_tool_kine(tool_mat=tool_mat)
            elif robot_id == "B":
                kk2.set_tool_kine(tool_mat=tool_mat)

            '''save in txt and send it to controller'''
            if not self.tool_result:
                lines = ['0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n',
                         '0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n']
                # 写回文件
                with open(self.tools_txt, 'w', encoding='utf-8') as file:
                    file.writelines(lines)
                file.close()
                time.sleep(0.5)
            from python.fx_robot import update_text_file_simple
            full_tool = dyn_p + kine_p
            # print(f'full tool:{full_tool}')
            update_text_file_simple(robot_id, full_tool, self.tools_txt)
            robot.send_file(self.tools_txt, os.path.join('/home/fusion/', self.tools_txt))
            time.sleep(1)
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

    def load_tool_dyn_to_tool_api(self,robot_id):
        try:
            # 获取工具动力学参数值
            if hasattr(self, 'entry_tool_dyn'):
                tool_dyn_value = self.entry_tool_dyn.get()
                # 验证格式
                success, result = self.validate_point(tool_dyn_value, 10)
                if success:
                    if robot_id=='A':
                        # 如果tool_a_entry不存在，创建它
                        if not hasattr(self, 'tool_a_entry'):
                            self.tool_a_entry = tk.StringVar()
                            self._last_valid_tool_a = result
                        self.tool_a_entry.set(result)

                    elif robot_id=='B':
                        # 如果tool_a_entry不存在，创建它
                        if not hasattr(self, 'tool_b_entry'):
                            self.tool_b_entry = tk.StringVar()
                            self._last_valid_tool_a = result
                        self.tool_b_entry.set(result)
                    print(f"成功导入工具{robot_id}参数: {result}")
                else:
                    print(f"参数格式错误: {result}")
            else:
                print("找不到工具动力学参数输入框")
        except Exception as e:
            print(f"导入失败: {e}")

    def tool_dyn_identy(self):
        print(f"ccs srs:{self.type_select_combobox_1.get()}")
        print(f"tool data:{self.save_tool_data_path}")
        if self.type_select_combobox_1.get() == 'CCS':
            identy_results = kk1.identify_tool_dyn(robot_type=1, ipath=self.save_tool_data_path)
            if type(identy_results) == str:
                print('error:', identy_results)
                messagebox.showerror('wrong', f'工具动力学参数辨识错误提示:{identy_results}')
            else:
                print(f' identy_results:{identy_results}')
                tool_dyn_text = ""
                for i in range(10):
                    tool_dyn_text += f"{identy_results[i]:.3f}, "
                tool_dyn_text = tool_dyn_text.rstrip(", ")  # 移除最后一个逗号和空格
                self.entry_tool_dyn.set(tool_dyn_text)
                messagebox.showinfo('success', '工具动力学参数辨识完成')

        else:
            identy_results = kk1.identify_tool_dyn(robot_type=2, ipath=self.save_tool_data_path)
            if type(identy_results) == str:
                print('error:', identy_results)
                messagebox.showerror('wrong', f'工具动力学参数辨识错误提示:{identy_results}')
            else:
                print(f' identy_results:{identy_results}')
                tool_dyn_text = ""
                for i in range(10):
                    tool_dyn_text += f"{identy_results[i]:.3f}, "
                tool_dyn_text = tool_dyn_text.rstrip(", ")  # 移除最后一个逗号和空格
                self.entry_tool_dyn.set(tool_dyn_text)
                messagebox.showinfo('success', '工具动力学参数辨识完成')

    def clear_status_text(self):
        """清空状态文本"""
        if hasattr(self, 'status_text'):
            self.status_text.configure(state="normal")
            self.status_text.delete(1.0, tk.END)
            self.status_text.configure(state="disabled")

    def emergency_stop_action(self):
        """急停按钮回调函数"""
        if self.connected:
            try:
                robot.soft_stop('AB')
                """外部调用来停止线程"""
                self.stop_event.set()
                self.status_label.config(text="⚠️ 紧急停止已触发！", foreground='red')
                return True  # 返回成功
            except Exception as e:
                print(f"急停操作失败: {e}")
                messagebox.showerror('错误', f'急停操作失败: {e}')
                return False  # 返回失败
        else:
            messagebox.showerror('错误', '请先连接机器人')
            return False  # 返回失败

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
            '''judge '''
            time.sleep(0.01)
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
                # 更新连接设备
                self.connect_btn.config(text="断开连接", bg="#F44336")
                self.status_label.config(text="已连接")
                self.status_light.config(fg="green")
                self.mode_btn.config(state="normal")
                # get controller version
                ret, version = robot.get_param('int', 'VERSION')
                self.version = version

                '''启动读485数据'''
                self.data_subscriber = DataSubscriber(self.update_data)

                '''tool '''
                robot.receive_file(self.tools_txt, '/home/fusion/tool_dyn_kine.txt')
                time.sleep(1)
                self.tool_result = read_csv_file_to_float_strict(self.tools_txt, expected_columns=16)
                print(f'self.tool_result:{self.tool_result}')
                if self.tool_result == 0:
                    messagebox.showinfo('success', '机器人连接成功. 机器人未设置工具信息，如果带工具，请设置工具信息')
                elif self.tool_result == -1:
                    messagebox.showerror('failed', '机器人连接成功. 工具信息文件tool_dyn_kine.txt有误。')
                elif type(self.tool_result) == tuple:
                    arm_side = self.tool_result[0]
                    if arm_side == 'line1':
                        messagebox.showinfo('success', '机器人连接成功. 机器人已设置左臂工具信息，右臂未设置.')
                        tool_dyn_l = ""
                        tool_kine_l = ""
                        for i in range(10):
                            tool_dyn_l += f"{self.tool_result[1][i]:.3f},"
                            if i < 6:
                                tool_kine_l += f"{self.tool_result[1][10 + i]:.3f},"
                        tool_dyn_l = tool_dyn_l.rstrip(", ")
                        tool_kine_l = tool_kine_l.rstrip(", ")
                        self.tool_a_entry.set(tool_dyn_l)
                        self.tool_a1_entry.set(tool_kine_l)
                        tool_mat = kk1.xyzabc_to_mat4x4(self.tool_result[1][10:])
                        kk1.set_tool_kine(tool_mat=tool_mat)

                    elif arm_side == 'line2':
                        messagebox.showinfo('success', '机器人连接成功. 机器人已设置右臂工具信息，左臂未设置.')
                        tool_dyn_r = ""
                        tool_kine_r = ""
                        for i in range(10):
                            tool_dyn_r += f"{self.tool_result[1][i]:.3f},"
                            if i < 6:
                                tool_kine_r += f"{self.tool_result[1][10 + i]:.3f},"
                        tool_dyn_r = tool_dyn_r.rstrip(", ")
                        tool_kine_r = tool_kine_r.rstrip(", ")
                        self.tool_b_entry.set(tool_dyn_r)
                        self.tool_b1_entry.set(tool_kine_r)
                        tool_mat1 = kk2.xyzabc_to_mat4x4(self.tool_result[1][10:])
                        kk2.set_tool_kine(tool_mat=tool_mat1)
                else:
                    messagebox.showinfo('success', '机器人连接成功.  机器人已设置工具信息.')
                    if isinstance(self.tool_result[0], list):
                        # print(f"第一行: {self.tool_result[0]}")
                        # print(f"第二行: {self.tool_result[1]}")
                        tool_dyn_l=""
                        tool_dyn_r = ""
                        tool_kine_l=""
                        tool_kine_r=""
                        for i in range(10):
                            tool_dyn_l += f"{self.tool_result[0][i]:.3f},"
                            tool_dyn_r += f"{self.tool_result[1][i]:.3f},"
                            if i < 6:
                                tool_kine_l += f"{self.tool_result[0][10+i]:.3f},"
                                tool_kine_r += f"{self.tool_result[1][10+i]:.3f},"
                        tool_dyn_l = tool_dyn_l.rstrip(", ")
                        tool_dyn_r = tool_dyn_r.rstrip(", ")
                        tool_kine_l = tool_kine_l.rstrip(", ")
                        tool_kine_r = tool_kine_r.rstrip(", ")

                        self.tool_a_entry.set(tool_dyn_l)
                        self.tool_a1_entry.set(tool_kine_l)
                        self.tool_b_entry.set(tool_dyn_r)
                        self.tool_b1_entry.set(tool_kine_r)

                        # 从控制器加载的工具信息
                        robot.set_tool(arm='A', dynamicParams=self.tool_result[0][:10], kineParams=self.tool_result[0][10:])
                        robot.set_tool(arm='B', dynamicParams=self.tool_result[1][:10], kineParams=self.tool_result[1][10:])

                        tool_mat = kk1.xyzabc_to_mat4x4(self.tool_result[0][10:])
                        tool_mat1 = kk2.xyzabc_to_mat4x4(self.tool_result[1][10:])
                        kk1.set_tool_kine(tool_mat=tool_mat)
                        kk2.set_tool_kine(tool_mat=tool_mat1)

            if motion_tag == 0:
                messagebox.showerror('failed!', "机器人连接不成功，请重连")
                self.status_label.config(text="未连接")
                self.status_light.config(fg="red")
                self.mode_btn.config(state="disabled")

        else:
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
                    {'cur_state': 0, 'cmd_state': 0, 'err_code': 0},
                    {'cur_state': 0, 'cmd_state': 0, 'err_code': 0},
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

    def update_data(self, result):
        """更新订阅的数据"""
        self.result = result
        self.root.after(0, self.update_ui)
        self.root.after(0, self.update_6d)

    def toggle_display_mode(self):
        """切换数据显示模式"""
        self.display_mode = (self.display_mode + 1) % 8
        self.mode_btn.config(text=self.mode_names[self.display_mode])
        self.update_ui()

    def update_ui(self):
        type = ''
        self.left_state_main.config(text=f"{self.result['states'][0]['cur_state']}")
        self.left_state_1.config(text=f"{self.result['outputs'][0]['tip_di'][0]}")
        self.left_state_2.config(text=f"{self.result['outputs'][0]['low_speed_flag'][0]}")
        self.left_state_3.config(text=f"{self.result['states'][0]['err_code']}")
        arm_error=self.result['states'][0]['err_code']
        if str(arm_error) in arm_err_code:
            type = arm_err_code[str(arm_error)]
            # print(f'left arm error:type:{type}')
            self.left_arm_error.config(text=f"手臂错误码{arm_error}为:"+f"{type}")
        else:
            self.left_arm_error.config(text="")
        # self.left_arm_error.config(text=f"配置文件选择了浮动基座选项，但是UMI设置在配置文件未开")
        '''### sperate ###'''
        type1 = ''
        self.right_state_main.config(text=f"{self.result['states'][1]['cur_state']}")
        self.right_state_1.config(text=f"{self.result['outputs'][1]['tip_di'][0]}")
        self.right_state_2.config(text=f"{self.result['outputs'][1]['low_speed_flag'][0]}")
        self.right_state_3.config(text=f"{self.result['states'][1]['err_code']}")
        arm_error1 = self.result['states'][1]['err_code']
        if str(arm_error1) in arm_err_code:
            type1 = arm_err_code[str(arm_error1)]
            # print(f'right arm error:type:{type1}')
            self.right_arm_error.config(text=f"手臂错误码{arm_error}为:"+f"{type1}")
        else:
            self.right_arm_error.config(text="")
        # self.right_arm_error.config(text=f"配置文件选择了浮动基座选项，但是UMI设置在配置文件未开")

    def update_6d(self):
        """更新机械臂数据"""
        key = self.data_keys[self.display_mode]

        '''left'''
        joint_pos_l = self.result['outputs'][0][key]
        '''xyzabc'''
        fk_mat_l = kk1.fk(joints=joint_pos_l)
        pose_6d_l = kk1.mat4x4_to_xyzabc(pose_mat=fk_mat_l)  # 用关节正解的姿态转XYZABC
        # print(f'pose_6d_1:{pose_6d_1}')
        # 格式化笛卡尔数据为单行
        cartesian_text_l = f"{pose_6d_l[0]:.3f},{pose_6d_l[1]:.3f},{pose_6d_l[2]:.3f},{pose_6d_l[3]:.3f}, {pose_6d_l[4]:.3f}, {pose_6d_l[5]:.3f}"
        # 格式化关节数据为单行：
        joint_text_l = ""
        for i in range(7):
            joint_text_l += f"{joint_pos_l[i]:.3f}, "
        joint_text_l = joint_text_l.rstrip(", ")  # 移除最后一个逗号和空格
        # 更新对应臂的文本框
        self.left_cartesian_text.config(state="normal")
        self.left_cartesian_text.delete("1.0", tk.END)
        self.left_cartesian_text.insert("1.0", cartesian_text_l)
        self.left_cartesian_text.tag_add("center", "1.0", "end")
        self.left_cartesian_text.config(state="disabled")

        self.left_joint_text.config(state="normal")
        self.left_joint_text.delete("1.0", tk.END)
        self.left_joint_text.insert("1.0", joint_text_l)
        self.left_joint_text.tag_add("center", "1.0", "end")
        self.left_joint_text.config(state="disabled")

        '''right'''
        joint_pos_r = self.result['outputs'][1][key]
        '''xyzabc'''
        fk_mat_r = kk2.fk(joints=joint_pos_r)
        pose_6d_r = kk1.mat4x4_to_xyzabc(pose_mat=fk_mat_r)  # 用关节正解的姿态转XYZABC
        # print(f'pose_6d_2:{pose_6d_2}')
        # 格式化笛卡尔数据为单行
        cartesian_text_r = f"{pose_6d_r[0]:.3f},{pose_6d_r[1]:.3f},{pose_6d_r[2]:.3f},{pose_6d_r[3]:.3f}, {pose_6d_r[4]:.3f}, {pose_6d_r[5]:.3f}"
        # 格式化关节数据为单行：
        joint_text_r = ""
        for i in range(7):
            joint_text_r += f"{joint_pos_r[i]:.3f}, "
        joint_text_r = joint_text_r.rstrip(", ")  # 移除最后一个逗号和空格
        self.right_cartesian_text.config(state="normal")
        self.right_cartesian_text.delete("1.0", tk.END)
        self.right_cartesian_text.insert("1.0", cartesian_text_r)
        self.right_cartesian_text.tag_add("center", "1.0", "end")
        self.right_cartesian_text.config(state="disabled")

        self.right_joint_text.config(state="normal")
        self.right_joint_text.delete("1.0", tk.END)
        self.right_joint_text.insert("1.0", joint_text_r)
        self.right_joint_text.tag_add("center", "1.0", "end")
        self.right_joint_text.config(state="disabled")

def read_csv_file_to_float_strict(filename, expected_columns=16):
    """
    读取CSV格式的文件内容并转换为float，严格验证每列数量
    参数:
        filename: 文件名
        expected_columns: 期望的列数（默认16）

    返回:
        如果文件为空: 返回0
        如果文件有一行: 返回0
        如果文件有两行且其中一行全为0:
            - 返回 ('line1', [第一行数据])  # 如果第二行全为0
            - 返回 ('line2', [第二行数据])  # 如果第一行全为0
        如果文件有两行且都不为0: 返回 [[第一行数据], [第二行数据]]
        如果文件有两行且都全为0: 返回0
        如果文件不存在或转换失败: 返回-1
    """
    if not os.path.exists(filename):
        print(f"文件不存在: {filename}")
        return -1

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

        # 根据行数处理
        if len(all_float_data) == 1:
            # 文件只有一行，返回0
            return 0

        elif len(all_float_data) == 2:
            # 检查两行是否全为0
            line1_all_zero = all(x == 0.0 for x in all_float_data[0])
            line2_all_zero = all(x == 0.0 for x in all_float_data[1])

            if line1_all_zero and line2_all_zero:
                # 两行都全为0
                return 0
            elif line1_all_zero and not line2_all_zero:
                # 第一行全为0，第二行不为0
                return ('line2', all_float_data[1])
            elif not line1_all_zero and line2_all_zero:
                # 第一行不为0，第二行全为0
                return ('line1', all_float_data[0])
            else:
                # 两行都不为0
                return all_float_data
        else:
            print(f"文件包含{len(all_float_data)}行，只支持1-2行")
            return -1

    except Exception as e:
        print(f"读取文件时出错: {e}")
        return -1

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
                        )

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
    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            content = file.read()
            text_area.insert(tk.INSERT, content)
            text_area.config(state=tk.DISABLED)  # 设置为只读
    except Exception as e:
        messagebox.showerror("错误", f"无法读取文件:\n{str(e)}")

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

def process_and_downsample(file_path, format_unify=True):
    """
    完整处理：下采样 + 特征重映射 + 格式统一
    1. 第一行：将'='和'@'之间的数字改为7，@后的行数减半
    2. 下采样：每隔一行取一行（1000Hz->500Hz）
    3. 特征重映射：删除前两列，后面7列数据往前移动，重新分配字母标识
    4. 格式统一：统一使用\n换行符，数值格式化为6位小数

    参数：
    file_path: 要处理的文件路径
    format_unify: 是否统一数值格式为6位小数（默认True）
    """
    # 读取文件所有行
    with open(file_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    if not lines:
        print("文件为空")
        return

    processed_lines = []
    # 处理第一行
    first_line = lines[0].strip('\r\n')
    if '=' in first_line and '@' in first_line:
        # 分离@前后的部分
        parts = first_line.split('@')
        if len(parts) == 2:
            # 处理=和@之间的数字
            first_part = re.sub(r'(=)\d+(@?)', r'\g<1>7\2', parts[0])
            # 获取原始行数并减半（1000Hz->500Hz）
            try:
                original_rows = int(parts[1])
                new_rows = original_rows // 2  # 下采样行数减半
                if original_rows % 2 != 0:
                    print(f"注意：原始行数{original_rows}不是偶数，下采样后行数为{new_rows}")
            except ValueError:
                print(f"警告：无法解析行数 '{parts[1]}'，保持原样")
                new_rows = parts[1]
            processed_first_line = f"{first_part}@{new_rows}"
            processed_lines.append(processed_first_line + '\n')
            print(f"第一行已修改: {first_line} -> {processed_first_line}")
        else:
            processed_lines.append(first_line + '\n')
    else:
        processed_lines.append(first_line + '\n')

    # 新的特征字母顺序
    new_letters = ['X', 'Y', 'Z', 'A', 'B', 'C', 'U']
    # 下采样处理：从第1行开始，每隔一行取一行并处理
    original_data_lines = 0
    processed_data_lines = 0
    for i in range(1, len(lines)):
        line = lines[i]
        # 去除换行符
        line = line.rstrip('\r\n')
        if not line:
            continue
        original_data_lines += 1
        # 下采样：每隔一行取一行（保留奇数行）
        if (i - 1) % 2 == 0:
            # 格式统一：分离字母和数值
            if format_unify:
                # 按$分割各个特征
                features = [f.strip() for f in line.split('$') if f.strip()]
                if len(features) >= 9:
                    # 统一数值格式为6位小数
                    unified_features = []
                    for feature in features:
                        # 分离字母和数值
                        match = re.match(r'([A-Z])\s+(-?\d+\.?\d*)', feature)
                        if match:
                            letter = match.group(1)
                            value_str = match.group(2)
                            # 格式化为6位小数
                            try:
                                value = float(value_str)
                                formatted_value = f"{value:.6f}"
                                unified_features.append(f"{letter} {formatted_value}")
                            except ValueError:
                                unified_features.append(feature)
                        else:
                            unified_features.append(feature)

                    # 使用统一后的特征进行处理
                    if len(unified_features) >= 9:
                        features = unified_features
                    # 特征重映射处理
                    values = []
                    for feature in features:
                        # 分离字母和数值
                        parts = feature.split(' ', 1)
                        if len(parts) == 2:
                            values.append(parts[1].strip())
                        else:
                            # 查找第一个数字或负号或小数点的位置
                            match = re.search(r'[-]?\d+\.?\d*', feature)
                            if match:
                                value = match.group()
                                values.append(value)
                            else:
                                values.append('')

                    # 删除前两个值（X和Y），取后面7个值
                    if len(values) >= 9:
                        selected_values = values[2:9]  # 取索引2到8的7个值

                        # 创建新的特征行
                        new_features = []
                        for j in range(7):
                            new_features.append(f"{new_letters[j]} {selected_values[j]}")

                        processed_line = '$'.join(new_features) + '$'
                        processed_lines.append(processed_line + '\n')
                        processed_data_lines += 1
                    else:
                        processed_lines.append(line + '$\n')
                        processed_data_lines += 1
                        print(f"警告: 第{i + 1}行数值不足，保持原样")
                else:
                    processed_lines.append(line + '$\n')
                    processed_data_lines += 1
                    print(f"警告: 第{i + 1}行特征数不足9个，保持原样")
            else:
                # 不统一格式，只进行下采样和特征重映射
                # 按$分割各个特征，提取数值部分
                features = [f.strip() for f in line.split('$') if f.strip()]

                if len(features) >= 9:
                    # 提取各特征的数值部分
                    values = []
                    for feature in features:
                        # 分离字母和数值
                        parts = feature.split(' ', 1)
                        if len(parts) == 2:
                            values.append(parts[1].strip())
                        else:
                            # 查找第一个数字或负号或小数点的位置
                            match = re.search(r'[-]?\d+\.?\d*', feature)
                            if match:
                                value = match.group()
                                values.append(value)
                            else:
                                values.append('')

                    # 删除前两个值（X和Y），取后面7个值
                    if len(values) >= 9:
                        selected_values = values[2:9]  # 取索引2到8的7个值
                        # 创建新的特征行
                        new_features = []
                        for j in range(7):
                            new_features.append(f"{new_letters[j]} {selected_values[j]}")
                        processed_line = '$'.join(new_features) + '$'
                        processed_lines.append(processed_line + '\n')
                        processed_data_lines += 1
                    else:
                        processed_lines.append(line + '$\n')
                        processed_data_lines += 1
                        print(f"警告: 第{i + 1}行数值不足，保持原样")
                else:
                    processed_lines.append(line + '$\n')
                    processed_data_lines += 1
                    print(f"警告: 第{i + 1}行特征数不足9个，保持原样")

    print(f"下采样：从{original_data_lines}行减少到{processed_data_lines}行")
    # 另存为，不覆盖
    if '/'in file_path:
        file_path=file_path.split('/')[-1]
    file_path_save='processed_'+file_path
    with open(file_path_save, 'w', encoding='utf-8', newline='\n') as f:
        f.writelines(processed_lines)

    print(f"\n文件 '{file_path}' 处理完成并已保存为 {file_path_save}")
    print(f"频率：1000Hz -> 500Hz")
    print(f"行数：{original_data_lines} -> {processed_data_lines}")
    print(f"格式统一：{'是' if format_unify else '否'}")
    print("新的特征对应关系:")
    print("原始: X Y Z A B C U V W")
    print("新的: X Y Z A B C U")
    print("对应: - - Z→X A→Y B→Z C→A U→B V→C W→U")

if __name__ == "__main__":
    # 定义常量
    DBL_EPSILON = sys.float_info.epsilon
    arm_main_state_with = 120
    # 创建队列
    data_queue = queue.Queue()
    '''
    ini sdk
    '''
    crr_pth = os.getcwd()
    dcss = DCSS()
    robot = Marvin_Robot()

    kk1 = Marvin_Kine()
    kk2 = Marvin_Kine()
    ini_result1 = kk1.load_config(arm_type=0, config_path=glob.glob('config/*.MvKDCfg')[0])
    initial_kine_tag1 = kk1.initial_kine(robot_type=ini_result1['TYPE'][0],
                                         dh=ini_result1['DH'][0],
                                         pnva=ini_result1['PNVA'][0],
                                         j67=ini_result1['BD'][0])

    ini_result2 = kk2.load_config(arm_type=1, config_path=glob.glob('config/*.MvKDCfg')[0])
    initial_kine_tag2 = kk2.initial_kine(robot_type=ini_result2['TYPE'][0],
                                         dh=ini_result2['DH'][0],
                                         pnva=ini_result2['PNVA'][0],
                                         j67=ini_result2['BD'][0])

    if not ini_result1 or not ini_result2:
        messagebox.showerror('error', 'config/*.MvKDCfg 出错，请检查文件内容或路径')
    root = tk.Tk()

    style = ttk.Style()

    style.configure(
        "MyCustom.TLabelframe",
        font=("Arial", 12, "italic"),  # 字体
        foreground="darkblue",  # 文字颜色
        background="white"  # 标签背景色
    )
    app = App(root)

    root.mainloop()
