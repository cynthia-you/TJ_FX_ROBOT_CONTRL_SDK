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
from python.fx_robot import Marvin_Robot, DCSS, arm_err_code,arm_err_code_EN,tools_cfg
from python.fx_kine import Marvin_Kine
import ast
from PIL import Image, ImageDraw, ImageTk
from pathlib import Path
import difflib
import re
import json
from typing import Optional

class DataSubscriber:
    def __init__(self, callback):
        self.callback = callback
        self.running = True
        self.thread = threading.Thread(target=self.generate_data, daemon=True)
        self.thread.start()

    def generate_data(self):
        while self.running:
            result = robot.subscribe(dcss)
            self.callback(result)
            time.sleep(0.2)

    def stop(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)

class EmergencyStopButton:
    def __init__(self, parent, radius=40, command=None, reset_command=None):
        self.parent = parent
        self.radius = radius
        self.command = command
        self.is_stopped = False
        self.stop_command = command
        self.canvas = tk.Canvas(
            parent,
            width=radius * 2 + 10,
            height=radius * 2 + 10,
            bg='white',
            highlightthickness=0
        )
        self.canvas.pack()
        self.create_button_image()
        self.canvas.bind("<Button-1>", self.on_click)

    def create_button_image(self):
        img_size = (self.radius * 2 + 10, self.radius * 2 + 10)
        image = Image.new('RGBA', img_size, (255, 255, 255, 0))
        draw = ImageDraw.Draw(image)

        center_x = img_size[0] // 2
        center_y = img_size[1] // 2
        if self.is_stopped:
            draw.ellipse(
                [center_x - self.radius, center_y - self.radius,
                 center_x + self.radius, center_y + self.radius],
                fill='#808080', outline='#404040', width=3
            )
        else:
            draw.ellipse(
                [center_x - self.radius, center_y - self.radius,
                 center_x + self.radius, center_y + self.radius],
                fill='#ff4444', outline='#cc0000', width=3
            )
        draw.ellipse(
            [center_x - self.radius + 2, center_y - self.radius + 2,
             center_x + self.radius - 2, center_y + self.radius - 2],
            outline='white', width=2
        )
        temp_draw = ImageDraw.Draw(image)
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

        shadow_offset = 2
        draw.text(
            (center_x - text_width // 2 + shadow_offset,
             center_y - text_height // 2 + shadow_offset),
            text, fill='#880000' if not self.is_stopped else '#404040', font=font
        )
        draw.text(
            (center_x - text_width // 2, center_y - text_height // 2),
            text, fill='white', font=font
        )
        self.photo = ImageTk.PhotoImage(image)
        self.canvas.delete("all")
        self.canvas.create_image(
            center_x, center_y,
            image=self.photo,
            anchor=tk.CENTER
        )

    def on_click(self, event):
        if not self.is_stopped:
            if self.stop_command:
                try:
                    success = self.stop_command()
                except Exception as e:
                    print(f"Emergency stop callback function failed to execute: {e}")
                    success = False
            else:
                success = True

            if success:
                self.is_stopped = True
                self.parent.bell()
                self.create_button_image()
                messagebox.showwarning(
                    "Emergency stop trigger",
                    "The robot has been brought to an emergency stop! \nPlease check the system for safety before pressing the reset button."
                )
            else:
                self.parent.bell()
                print("Emergency stop operation failed; button status remained unchanged.")
        else:
            self.is_stopped = False
            self.create_button_image()

    def reset(self):
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
        if not hasattr(self, 'force_dir_a_entry'):
            self.force_dir_a_entry = tk.StringVar()
        if not hasattr(self, 'force_dir_b_entry'):
            self.force_dir_b_entry = tk.StringVar()
        self.force_dir_a_entry = tk.StringVar(value='1,1,1,1,1,1')
        self.force_dir_b_entry = tk.StringVar(value='1,1,1,1,1,1')

        self.source_file = "robot.ini"
        self.target_file = None

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
        self.mode_names = ["Position", "Velocity", "Torque", "Current", "Temperature", "PositionEx", "CmdPosition",
                           "ForceEST"]
        self.data_keys = [('fb_joint_pos'), ('fb_joint_vel'), ('fb_joint_sToq'), ('fb_joint_cToq'), ('fb_joint_them'),
                          ('fb_joint_posE'), ('fb_joint_cmd'), ('est_joint_force')]
        self.widgets = {}
        self.create_control_components()
        self.create_main_content()
        self.create_left_arm_components()
        self.create_separator()
        self.create_right_arm_components()
        self.create_emergency_stop_components()
        self.create_status_bar()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.correct_password = "1"
        self.connected = True
        self.data_subscriber = None
        self.tools_cfg_path = 'tools_cfg.json'
        self.tools_cfg = None
        self.init_tool_variables()
        self.init_kd_variables()
        self.stop_event = threading.Event()
        self.thread = None

    def load_file(self, filepath):
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                return f.read()
        except FileNotFoundError:
            messagebox.showerror("Error", f"file '{filepath}' doesn't exist！")
            return ""
        except Exception as e:
            messagebox.showerror("Error", f"File reading failed: {str(e)}")
            return ""

    def init_tool_variables(self):
        result = load_or_create_tools_config('tools_cfg.json')
        if result is False:
            try:
                with open('tools_cfg.json', 'r', encoding='utf-8') as f:
                    self.tools_cfg = json.load(f)
            except Exception as e:
                print(f"Reloading configuration file failed: {e}")
                return
        else:
            self.tools_cfg = result

        print(f'self.tools_cfg:{self.tools_cfg}')

        if "current_tool" not in self.tools_cfg:
            self.tools_cfg["current_tool"] = {"arm0": None, "arm1": None}

        self.tool_names = ["tool-1", "tool-2", "tool-3", "tool-4", "tool-5"]

        self.tool_a_entry = tk.StringVar()
        self.tool_b_entry = tk.StringVar()
        self.entry_tool_dyn = tk.StringVar(value="0,0,0,0,0,0,0,0,0,0")
        self.tool_a1_entry = tk.StringVar()
        self.tool_b1_entry = tk.StringVar()

        self._last_valid_tool_a = "0,0,0,0,0,0,0,0,0,0"
        self._last_valid_tool_a1 = "0,0,0,0,0,0"
        self._last_valid_tool_b = "0,0,0,0,0,0,0,0,0,0"
        self._last_valid_tool_b1 = "0,0,0,0,0,0"

        self.arm0_dyn_presets = []
        self.arm0_kine_presets = []
        self.arm1_dyn_presets = []
        self.arm1_kine_presets = []

        for tool_name in self.tool_names:
            if "arm0" in self.tools_cfg and tool_name in self.tools_cfg["arm0"]:
                dyn_str = ",".join(str(x) for x in self.tools_cfg["arm0"][tool_name]["dyn"])
                kine_str = ",".join(str(x) for x in self.tools_cfg["arm0"][tool_name]["kine"])
                self.arm0_dyn_presets.append(dyn_str)
                self.arm0_kine_presets.append(kine_str)
            if "arm1" in self.tools_cfg and tool_name in self.tools_cfg["arm1"]:
                dyn_str = ",".join(str(x) for x in self.tools_cfg["arm1"][tool_name]["dyn"])
                kine_str = ",".join(str(x) for x in self.tools_cfg["arm1"][tool_name]["kine"])
                self.arm1_dyn_presets.append(dyn_str)
                self.arm1_kine_presets.append(kine_str)

        if hasattr(self, 'tool_select_combobox_2'):
            current_arm0 = self.tools_cfg["current_tool"]["arm0"]
            if current_arm0 in self.tool_names:
                self.tool_select_combobox_2.set(current_arm0)
            else:
                self.tool_select_combobox_2.current(0)

        if hasattr(self, 'tool_select_combobox_3'):
            current_arm1 = self.tools_cfg["current_tool"]["arm1"]
            if current_arm1 in self.tool_names:
                self.tool_select_combobox_3.set(current_arm1)
            else:
                self.tool_select_combobox_3.current(0)

        if hasattr(self, 'tool_select_combobox_1'):
            current_arm0 = self.tools_cfg["current_tool"]["arm0"]
            if current_arm0 in self.tool_names:
                self.tool_select_combobox_1.set(current_arm0)
            else:
                self.tool_select_combobox_1.current(0)

        if hasattr(self, 'tool_select_combobox_11'):
            current_arm1 = self.tools_cfg["current_tool"]["arm1"]
            if current_arm1 in self.tool_names:
                self.tool_select_combobox_11.set(current_arm1)
            else:
                self.tool_select_combobox_11.current(0)

        self.update_tool_display('A','down_arm0')
        self.update_tool_display('B','down_rm1')

    def update_tool_display(self, arm_type,which_combobox:Optional[str]):
        if not hasattr(self, 'tools_cfg'):
            return
        selected_tool = None
        if arm_type == 'A':
            if which_combobox == 'up_arm0' and hasattr(self, 'tool_select_combobox_1'):
                selected_tool = self.tool_select_combobox_1.get()

            if which_combobox == 'down_arm0' and hasattr(self, 'tool_select_combobox_2'):
                selected_tool = self.tool_select_combobox_2.get()
            if selected_tool is None:
                selected_tool = "tool-1"

            if selected_tool and "arm0" in self.tools_cfg and selected_tool in self.tools_cfg["arm0"]:
                dyn_data = self.tools_cfg["arm0"][selected_tool]["dyn"]
                kine_data = self.tools_cfg["arm0"][selected_tool]["kine"]

                dyn_str = ",".join(str(x) for x in dyn_data)
                kine_str = ",".join(str(x) for x in kine_data)

                if hasattr(self, 'tool_a_combobox'):
                    self.tool_a_combobox.set(dyn_str)
                    self.tool_a_entry.set(dyn_str)

                if hasattr(self, 'tool_a1_combobox'):
                    self.tool_a1_combobox.set(kine_str)
                    self.tool_a1_entry.set(kine_str)

        elif arm_type == 'B':
            if which_combobox == 'up_arm1' and hasattr(self, 'tool_select_combobox_11'):
                selected_tool = self.tool_select_combobox_11.get()
            if which_combobox == 'down_arm1' and hasattr(self, 'tool_select_combobox_3'):
                selected_tool = self.tool_select_combobox_3.get()
            if selected_tool is None:
                selected_tool = "tool-1"

            if selected_tool and "arm1" in self.tools_cfg and selected_tool in self.tools_cfg["arm1"]:
                dyn_data = self.tools_cfg["arm1"][selected_tool]["dyn"]
                kine_data = self.tools_cfg["arm1"][selected_tool]["kine"]

                dyn_str = ",".join(str(x) for x in dyn_data)
                kine_str = ",".join(str(x) for x in kine_data)

                if hasattr(self, 'tool_b_combobox'):
                    self.tool_b_combobox.set(dyn_str)
                    self.tool_b_entry.set(dyn_str)

                if hasattr(self, 'tool_b1_combobox'):
                    self.tool_b1_combobox.set(kine_str)
                    self.tool_b1_entry.set(kine_str)

    def init_kd_variables(self):
        self.cart_k_b_entry = tk.StringVar(value="10000, 10000, 10000, 600, 600, 600, 20")
        self.cart_k_a_entry = tk.StringVar(value="10000, 10000, 10000, 600, 600, 600, 20")
        self.cart_d_a_entry = tk.StringVar(value="0.8, 0.8, 0.8, 0.2, 0.2, 0.2, 1")
        self.cart_d_b_entry = tk.StringVar(value="0.8, 0.8, 0.8, 0.2, 0.2, 0.2, 1")
        self.k_a_entry=tk.StringVar(value="12, 12, 12, 10, 9, 9, 7")
        self.k_b_entry=tk.StringVar(value="12, 12, 12, 10, 9, 9, 7")
        self.d_a_entry=tk.StringVar(value="0,0,0,0,0,0,0")
        self.d_b_entry=tk.StringVar(value="0,0,0,0,0,0,0")

    def create_main_content(self):
        self.main_container = tk.Frame(self.root, bg="white", padx=5, pady=10)
        self.main_container.pack(fill="both", expand=True)
        self.main_canvas = tk.Canvas(self.main_container, bg="white", highlightthickness=0)
        self.main_scrollbar = ttk.Scrollbar(self.main_container, orient="vertical", command=self.main_canvas.yview)
        self.scrollable_frame = tk.Frame(self.main_canvas, bg="white")
        self.stop_frame=tk.Frame(self.main_canvas,bg='white')
        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: self.main_canvas.configure(scrollregion=self.main_canvas.bbox("all"))
        )
        self.main_canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.main_canvas.configure(yscrollcommand=self.main_scrollbar.set)
        self.main_canvas.pack(side="left", fill="both", expand=True, padx=(0, 5))
        self.main_scrollbar.pack(side="right", fill="y")
        self.main_canvas.bind_all("<MouseWheel>", self.on_mousewheel)

    def update_vertical_scrollbar(self, *args):
        self.v_scrollbar.set(*args)
        self.main_canvas.yview(*args)

    def update_horizontal_scrollbar(self, *args):
        self.h_scrollbar.set(*args)
        self.main_canvas.xview(*args)

    def scroll_horizontally(self, *args):
        self.main_canvas.xview(*args)

    def on_mousewheel(self, event):
        self.main_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    def on_shift_mousewheel(self, event):
        self.main_canvas.xview_scroll(int(-1 * (event.delta / 120)), "units")

    def on_mousewheel(self, event):
        if event.delta:
            self.main_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
        else:
            if event.num == 4:
                self.main_canvas.yview_scroll(-1, "units")
            elif event.num == 5:
                self.main_canvas.yview_scroll(1, "units")

    def on_horizontal_mousewheel(self, event):
        if event.delta:
            self.main_canvas.xview_scroll(int(-1 * (event.delta / 120)), "units")
        else:
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

        self.emergency_btn = EmergencyStopButton(
            stop_control_frame,
            radius=35,
            command=self.emergency_stop_action
        )
        self.emergency_btn.canvas.pack(side="left", padx=5, pady=(10, 10))

    def create_left_arm_components(self):
        left_arm_container = tk.Frame(self.scrollable_frame, bg="white", pady=5)
        left_arm_container.pack(fill="x", pady=(0, 5))

        left_content = tk.Frame(left_arm_container, bg="white")
        left_content.pack(fill="x")

        '''第一列：左臂状态'''
        left_status_frame = tk.Frame(left_content, bg="white", width=arm_main_state_with)
        left_status_frame.pack(side="left", fill="y", padx=(0, 10))
        left_status_frame.pack_propagate(False)

        status_title_frame = tk.Frame(left_status_frame, bg="white")
        status_title_frame.pack(fill="x", pady=(0, 10))

        try:
            img_left = Image.open('src/left.png')
            img_left = img_left.resize((45, 75), Image.Resampling.LANCZOS)
            arm_image_left = ImageTk.PhotoImage(img_left)
            self.left_arm_image = arm_image_left
            img_label_left = tk.Label(status_title_frame, image=arm_image_left, bg="white")
            img_label_left.pack(side="left", padx=(10, 10), pady=20)
            tk.Label(status_title_frame, text="LEFT ARM", font=('Arial', 9, 'bold'), fg='#2c3e50', bg="white").pack(
            side="left", pady=20)
        except:
            tk.Label(status_title_frame, text="LEFT ARM", font=('Arial', 9, 'bold'), fg='#2c3e50', bg="white").pack(
            side="left", pady=20)

        status_info_frame = tk.Frame(left_status_frame, bg="white")
        status_info_frame.pack(fill="both", expand=True)

        control_row = tk.Frame(status_info_frame, bg="white")
        control_row.pack(fill="x", pady=(0, 5))
        tk.Label(control_row, text="Status:", font=('Arial', 9), fg='#2c3e50', width=10, anchor='e', bg="white").pack(
            side="left", padx=(0, 5))
        self.left_state_main = tk.Label(
            control_row,
            text='disable',
            font=('Arial', 9),
            fg='#34495e',
            bg='white',
            pady=3,
            anchor='w',
            relief=tk.SUNKEN,
        )
        self.left_state_main.pack(side="left", fill="x", expand=True)

        drag_row = tk.Frame(status_info_frame, bg="white")
        drag_row.pack(fill="x", pady=(0, 5))
        tk.Label(drag_row, text="Drag:", font=('Arial', 9), fg='#2c3e50', width=10, anchor='e', bg="white").pack(
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

        speed_row = tk.Frame(status_info_frame, bg="white")
        speed_row.pack(fill="x", pady=(0, 5))
        tk.Label(speed_row, text="LowSpeed:", font=('Arial', 9), fg='#2c3e50', width=10, anchor='e', bg="white").pack(
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

        error_row = tk.Frame(status_info_frame, bg="white")
        error_row.pack(fill="x", pady=(0, 5))
        tk.Label(error_row, text="ArmError:", font=('Arial', 9), fg='#2c3e50', width=10, anchor='e', bg="white").pack(
            side="left", padx=(0, 5))
        self.left_state_3 = tk.Label(
            error_row,
            text='None',
            font=('Arial', 9),
            fg='#34495e',
            bg='white',
            pady=3,
            anchor='w',
            relief=tk.SUNKEN,
            bd=1
        )
        self.left_state_3.pack(side="left", fill="x", expand=True)

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

        left_control_top = tk.Frame(left_control_frame, bg="white")
        left_control_top.pack(fill="x", pady=(0, 10))
        state_switch_frame = ttk.LabelFrame(
            left_control_top,
            text="Status switching",
            padding=10,
            relief=tk.GROOVE,
            borderwidth=2,
            style="MyCustom.TLabelframe"
        )
        state_switch_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))

        # 复位按钮
        self.reset_button = tk.Button(
            state_switch_frame,
            text="Disable",
            width=10,
            command=lambda: self.reset_robot_state('A'),
            bg="#2196F3",
            fg="white",
            font=("Arial", 10, "bold")
        )
        self.reset_button.pack(pady=(0, 10))

        state_options = ["Position", "Joint Impedance", "Cartesian Impedance", "PVT", "Drag","Flange Cart.Imp."]
        self.state_var = tk.StringVar()
        self.state_var.set(state_options[0])
        self.state_combobox = ttk.Combobox(
            state_switch_frame,
            textvariable=self.state_var,
            values=state_options,
            state="readonly",
            width=15
        )
        self.state_combobox.pack()
        self.state_combobox.bind("<<ComboboxSelected>>", lambda e: self.on_state_selected('A'))

        error_handle_frame = ttk.LabelFrame(
            left_control_top,
            text="Error Handling",
            padding=10,
            relief=tk.GROOVE,
            borderwidth=2,
            style="MyCustom.TLabelframe"
        )
        error_handle_frame.pack(side="left", fill="both", expand=True)

        servo_frame = tk.Frame(error_handle_frame, bg="white")
        servo_frame.pack(fill="x", pady=(0, 10))

        self.clear_servo_error_left_btn = tk.Button(
            servo_frame,
            text="ClearServoErr",
            width=12,
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
            text="GetError",
            width=12,
            command=lambda: self.error_get('A'),
            font=("Arial", 10, "bold")
        )
        self.get_servo_error_left_btn.pack(side="left",padx=(0, 20))

        self.servo_reset_left_label= tk.Label(servo_frame, text="J:",font=("Arial", 9, "bold"), bg='white' ).pack(side="left", padx=(0, 0))
        self.servo_axis_select_combobox_left = ttk.Combobox(
            servo_frame,
            values=["0", "1", "2", "3", "4", "5", "6"],
            width=2,
            state="readonly"
        )
        self.servo_axis_select_combobox_left.current(0)
        self.servo_axis_select_combobox_left.pack(side="left",padx=(0, 5))

        self.servo_reset_btn_left= tk.Button(
            servo_frame,
            text="SoftDisable",
            width=10,
            command=lambda: self.servo_set('A'),
            font=("Arial", 10, "bold"),
            bg='#F5F5DC'
        )
        self.servo_reset_btn_left.pack(side="left",)

        # 协作控制和刹车按钮
        control_frame = tk.Frame(error_handle_frame,bg='white')
        control_frame.pack(fill="x")

        self.release_collab_left_btn = tk.Button(
            control_frame,
            text="Collaboration",
            width=12,
            command=lambda: self.cr_state('A'),
            bg="#4CAF50",
            fg="white",
            font=("Arial", 10, "bold")
        )
        self.release_collab_left_btn.pack(side="left", padx=(0, 5))

        self.release_brake_left_btn = tk.Button(
            control_frame,
            text="BrakeRelease",
            width=12,
            command=lambda: self.release_brake('A'),
            font=("Arial", 10, "bold")
        )
        self.release_brake_left_btn.pack(side="left", padx=(0, 5))

        self.hold_brake_left_btn = tk.Button(
            control_frame,
            text="BrakeHold",
            width=12,
            command=lambda: self.brake('A'),
            font=("Arial", 10, "bold")
        )
        self.hold_brake_left_btn.pack(side="left")

        left_control_middle = tk.Frame(left_control_frame, bg="white")
        left_control_middle.pack(fill="y", expand=True)

        param_frame = ttk.LabelFrame(
            left_control_middle,
            text="Parameters",
            padding=10,
            relief=tk.GROOVE,
            borderwidth=2,
            style="MyCustom.TLabelframe"
        )
        param_frame.pack(fill="x", pady=(0, 10))

        param_row = tk.Frame(param_frame, bg="white")
        param_row.pack(fill="x")

        # 速度设置
        tk.Label(param_row, text="Speed:", font=('Arial', 9),bg='white' ).pack(side="left", padx=(0, 2))
        self.left_speed_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.left_speed_entry.pack(side="left")
        self.left_speed_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9),bg='white' ).pack(side="left", padx=(0, 5))

        # 加速度设置
        tk.Label(param_row, text="Accel:", font=('Arial', 9),bg='white'  ).pack(side="left", padx=(0, 2))
        self.left_accel_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.left_accel_entry.pack(side="left")
        self.left_accel_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9),bg='white' ).pack(side="left", padx=(0, 5))
        speed_btn1 = tk.Button(
            param_row,
            text="Confirm Speed",
            width=15,
            command=lambda: self.vel_acc_set('A'),
            bg="#58C3EE",
            font=("Arial", 9, "bold")
        )
        speed_btn1.pack(side="left", padx=(0, 20))

        # 阻抗参数设置按钮
        self.left_impedance_btn = tk.Button(
            param_row,
            text="Impedance Params",
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
            text="Realtime Data",
            padding=10,
            relief=tk.GROOVE,
            borderwidth=2,
            style="MyCustom.TLabelframe"
        )
        data_frame.pack(fill="x",pady=(0, 10))

        # 关节数据
        tk.Label(data_frame, text="J1-J7:", font=('Arial', 10, 'bold'),bg='white').pack(anchor="w", pady=(0, 5))
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
        self.left_joint_text.pack(fill="both", expand=True)
        self.left_joint_text.insert("1.0", "0.000,0.000,0.000,0.000,0.000,0.000,0.000")
        self.left_joint_text.tag_add("center", "1.0", "end")
        self.left_joint_text.config(state="disabled")

        # 笛卡尔数据
        tk.Label(data_frame, text="XYZABC(flange):", font=('Arial', 10, 'bold'),bg='white').pack(
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
        self.left_cartesian_text.pack(fill="both", expand=True)
        self.left_cartesian_text.insert("1.0", "0.000,0.000,0.000,0.000,0.000,0.000")
        self.left_cartesian_text.tag_add("center", "1.0", "end")
        self.left_cartesian_text.config(state="disabled")


        '''第三列：力控指令和位置指令'''
        left_second_control_frame = tk.Frame(left_content, bg="white", width=650)
        left_second_control_frame.pack(side="left", fill="y")
        left_second_control_frame.pack_propagate(False)  # 保持固定宽度

        # 力控组件
        force_control_frame = ttk.LabelFrame(
            left_second_control_frame,
            text="Force Control",
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
        tk.Label(force_row, text="Force:", font=('Arial', 9),bg='white' ).pack(side="left", padx=(0, 0))
        self.left_force_entry = tk.Entry(force_row, width=5, font=('Arial', 9), justify='center')
        self.left_force_entry.pack(side="left")
        self.left_force_entry.insert(0, "10") #最大不超过60N
        tk.Label(force_row, text="1N-60N", font=('Arial', 9),bg='white').pack(side="left", padx=(0, 10))

        # 调节量
        tk.Label(force_row, text="Adjust:", font=('Arial', 9),bg='white' ).pack(side="left", padx=(0, 0))
        self.left_force_adj_entry = tk.Entry(force_row, width=5, font=('Arial', 9), justify='center')
        self.left_force_adj_entry.pack(side="left")
        self.left_force_adj_entry.insert(0, "50") #最大不超过50mm
        tk.Label(force_row, text="1mm-50mm", font=('Arial', 9),bg='white').pack(side="left", padx=(0, 10))

        self.left_force_dir_btn1 = tk.Button(
            force_row,
            text="X Dir.",
            width=5,
            command=lambda: self.imped_f_mode(0,'A'),
            font=("Arial", 9, "bold"),
            bg='#E2F6FF'
        )
        self.left_force_dir_btn1.pack(side="left",padx=(0,5))

        self.left_force_dir_btn2 = tk.Button(
            force_row,
            text="Y Dir.",
            width=5,
            command=lambda: self.imped_f_mode(1,'A'),
            font=("Arial", 9, "bold"),
            bg='#F6DFF6'
        )
        self.left_force_dir_btn2.pack(side="left",padx=(0,5))

        self.left_force_dir_btn3 = tk.Button(
            force_row,
            text="Z Dir.",
            width=5,
            command=lambda: self.imped_f_mode(2,'A'),
            font=("Arial", 9, "bold"),
            bg = '#F7F7CE'
        )
        self.left_force_dir_btn3.pack(side="left",padx=(0,5))

        # 力方向 用户定义
        force_row1 = tk.Frame(force_control_frame,bg='white')
        force_row1.pack(fill="x", pady=(5, 5))
        # 力
        tk.Label(force_row1, text="Any Dir.:", font=('Arial', 9),bg='white' ).pack(side="left", padx=(0, 5))

        left_force_entry1 = tk.Entry(force_row1, textvariable=self.force_dir_a_entry,font=('Arial', 9),width=30)
        left_force_entry1.pack(side="left",padx=(0,5))

        self.left_force_dir_btn3 = tk.Button(
            force_row1,
            text="User Defined",
            width=10,
            command=lambda: self.imped_f_mode(3,'A'),
            font=("Arial", 9, "bold"),
            bg='#CCCCFF'
        )
        self.left_force_dir_btn3.pack(side="left",padx=(5,5))

        # 关节指令
        joint_cmd_frame = ttk.LabelFrame(
            left_second_control_frame,
            text="Position Cmd",
            padding=10,
            relief=tk.GROOVE,
            borderwidth=2,
            style="MyCustom.TLabelframe"
        )
        joint_cmd_frame.pack(fill="x")

        joints_row = tk.Frame(joint_cmd_frame,bg='white')
        joints_row.pack(fill="x", pady=(0, 5))

        # 第四列：1#
        self.btn_add3 = tk.Button(joints_row, text="GetCurrentPos", width=15,command=lambda: self.add_current_joints('A'))
        self.btn_add3.pack(side="left", padx=(0, 5))

        self.entry_var = tk.StringVar(value="0,0,0,0,0,0,0")
        self.entry = tk.Entry(joints_row, textvariable=self.entry_var, width=45)
        self.entry.pack(side="left", padx=(0, 5),expand=True)

        self.btn_add1 = tk.Button(joints_row, text="Add",width=8, command=self.add_point1)
        self.btn_add1.pack(side="left", padx=(0, 5))

        joints_row1 = tk.Frame(joint_cmd_frame,bg='white')
        joints_row1.pack(fill="x", pady=(0, 5))

        self.btn_del1 = tk.Button(joints_row1, text="Delete",width=8, command=self.delete_point1)
        self.btn_del1.pack(side="left", padx=(0, 5))
        #
        self.combo1 = ttk.Combobox(joints_row1, state="readonly", width=45)
        self.combo1.pack(side="left", padx=(0, 5))

        self.btn_run1 = tk.Button(joints_row1, text="Run", width=8,command=lambda :self.run_joints('A'),
                                  font=("Arial", 11, "bold"),fg='white', bg='#EC2A23',border=5)
        self.btn_run1.pack(side="left", padx=(0, 5))

        joints_row2 = tk.Frame(joint_cmd_frame,bg='white')
        joints_row2.pack(fill="x", pady=(0, 5))

        self.btn_save1 = tk.Button(joints_row2, text="Save", width=8,command=self.save_points1)
        self.btn_save1.pack(side="left", padx=(180, 5),pady=(0,10))

        self.btn_load1 = tk.Button(joints_row2, text="Import", width=8,command=self.load_points1)
        self.btn_load1.pack(side="left", padx=(0, 5),pady=(0,10))

        joints_row3 = tk.Frame(joint_cmd_frame,bg='white')
        joints_row3.pack(fill="x", pady=(0, 5))

        self.run_period_1 = tk.Button(joints_row3, text="Trajectory", width=8, command=lambda: self.thread_run_period('A'))
        self.run_period_1.pack(side="left", padx=(0, 5))

        self.period_path_entry_1 = tk.Entry(joints_row3, textvariable=self.period_file_path_1, width=45,
                                            font=("Arial", 9), bg='white')
        self.period_path_entry_1.pack(side="left", padx=(0, 5), expand=True)

        self.btn_load_file1 = tk.Button(joints_row3, text="Select File", width=8,
                                        command=lambda: self.select_period_file('A'))
        self.btn_load_file1.pack(side="left", padx=(0, 5))

        '''第四列：急停'''
        stop_control_frame = tk.Frame(left_content, bg="white",width=800)
        stop_control_frame.pack(side="left", fill="y")
        stop_control_frame.pack_propagate(False)

        self.left_emergency_btn = EmergencyStopButton(
            stop_control_frame,
            radius=35,
            command=self.emergency_stop_action,
        )
        self.left_emergency_btn.canvas.pack(side="left", padx=5, pady=(10, 10))

    def create_right_arm_components(self):
        right_arm_container = tk.Frame(self.scrollable_frame, bg="white", pady=10)
        right_arm_container.pack(fill="x", pady=(0, 15))

        right_content = tk.Frame(right_arm_container, bg="white")
        right_content.pack(fill="x")

        '''左侧：右臂状态'''
        right_status_frame = tk.Frame(right_content, bg="white", width=arm_main_state_with)
        right_status_frame.pack(side="left", fill="y", padx=(0, 10))
        right_status_frame.pack_propagate(False)

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
            tk.Label(status_title_frame, text="RIGHT ARM", font=('Arial', 9, 'bold'), fg='#2c3e50', bg="white").pack(
            side="left", pady=20)
        except:
            tk.Label(status_title_frame, text="RIGHT ARM", font=('Arial', 9, 'bold'), fg='#2c3e50', bg="white").pack(
            side="left", pady=20)

        # # 状态标签
        # tk.Label(status_title_frame, text="RIGHT ARM", font=('Arial', 9, 'bold'), fg='#2c3e50', bg="white").pack(
        #     side="left", pady=20)

        # 状态信息区域
        status_info_frame = tk.Frame(right_status_frame, bg="white")
        status_info_frame.pack(fill="both", expand=True)

        # 控制状态
        control_row = tk.Frame(status_info_frame, bg="white")
        control_row.pack(fill="x", pady=(0, 5))
        tk.Label(control_row, text="Status:", font=('Arial', 9), fg='#2c3e50', width=10, anchor='e', bg="white").pack(
            side="left", padx=(0, 5))
        self.right_state_main = tk.Label(
            control_row,
            text='Disable',
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
        tk.Label(drag_row, text="Drag:", font=('Arial', 9), fg='#2c3e50', width=10, anchor='e', bg="white").pack(
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
        tk.Label(speed_row, text="LowSpeed:", font=('Arial', 9), fg='#2c3e50', width=10, anchor='e', bg="white").pack(
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
        tk.Label(error_row, text="ArmError:", font=('Arial', 9), fg='#2c3e50', width=10, anchor='e', bg="white").pack(
            side="left", padx=(0, 5))
        self.right_state_3 = tk.Label(
            error_row,
            text='None',
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
            anchor='w',
            wraplength=120,
            justify='left'
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
            text="switching",
            padding=10,
            relief=tk.GROOVE,
            borderwidth=2,
            style="MyCustom.TLabelframe"
        )
        state_switch_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))

        # 复位按钮
        self.reset_button_r = tk.Button(
            state_switch_frame,
            text="Disable",
            width=10,
            command=lambda: self.reset_robot_state('B'),
            bg="#2196F3",
            fg="white",
            font=("Arial", 10, "bold")
        )
        self.reset_button_r.pack(pady=(0, 10))

        # 状态选择下拉框
        state_options = ["Position", "Joint Impedance", "Cartesian Impedance", "PVT", "Drag","Flange Cart.Imp."]
        self.state_var_r = tk.StringVar()
        self.state_var_r.set(state_options[0])
        self.state_combobox_r = ttk.Combobox(
            state_switch_frame,
            textvariable=self.state_var_r,
            values=state_options,
            state="readonly",
            width=15
        )
        self.state_combobox_r.pack()
        self.state_combobox_r.bind("<<ComboboxSelected>>", lambda e: self.on_state_selected('B'))

        # 右侧：异常处理
        error_handle_frame = ttk.LabelFrame(
            right_control_top,
            text="Error Handling",
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
            text="ClearServoErr",
            width=12,
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
            text="GetError",
            width=12,
            command=lambda: self.error_get('B'),
            font=("Arial", 10, "bold")
        )
        self.get_servo_error_right_btn.pack(side="left",padx=(0, 20))

        self.servo_reset_right_label = tk.Label(servo_frame, text="J:", font=("Arial", 9, "bold"), bg='white').pack(
            side="left", padx=(0, 0))
        self.servo_axis_select_combobox_right = ttk.Combobox(
            servo_frame,
            values=["0", "1", "2", "3", "4", "5", "6"],
            width=2,
            state="readonly"  # 禁止直接输入
        )
        self.servo_axis_select_combobox_right.current(0)
        self.servo_axis_select_combobox_right.pack(side="left", padx=(0, 5))

        self.servo_reset_btn_right = tk.Button(
            servo_frame,
            text="SoftDisable",
            width=10,
            command=lambda: self.servo_set('B'),
            font=("Arial", 10, "bold"),
            bg='#F5F5DC'
        )
        self.servo_reset_btn_right.pack(side="left", )

        # 协作控制和刹车按钮
        control_frame = tk.Frame(error_handle_frame,bg='white')
        control_frame.pack(fill="x")

        self.release_collab_right_btn = tk.Button(
            control_frame,
            text="Collaboration",
            width=12,
            command=lambda: self.cr_state('B'),
            bg="#4CAF50",
            fg="white",
            font=("Arial", 10, "bold")
        )
        self.release_collab_right_btn.pack(side="left", padx=(0, 5))

        self.release_brake_right_btn = tk.Button(
            control_frame,
            text="BrakeRelease",
            width=12,
            command=lambda: self.release_brake('B'),
            font=("Arial", 10, "bold")
        )
        self.release_brake_right_btn.pack(side="left", padx=(0, 5))

        self.hold_brake_right_btn = tk.Button(
            control_frame,
            text="BrakeHold",
            width=12,
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
            text="Parameters",
            padding=10,
            relief=tk.GROOVE,
            borderwidth=2,
            style="MyCustom.TLabelframe"
        )
        param_frame.pack(fill="x", pady=(0, 10))

        param_row = tk.Frame(param_frame, bg="white")
        param_row.pack(fill="x")

        # 速度设置
        tk.Label(param_row, text="Speed:", font=('Arial', 9),bg='white' ).pack(side="left", padx=(0, 2))
        self.right_speed_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.right_speed_entry.pack(side="left")
        self.right_speed_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9),bg='white' ).pack(side="left", padx=(0, 5))

        # 加速度设置
        tk.Label(param_row, text="Accel:", font=('Arial', 9),bg='white'  ).pack(side="left", padx=(0, 2))
        self.right_accel_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.right_accel_entry.pack(side="left")
        self.right_accel_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9),bg='white' ).pack(side="left", padx=(0, 5))
        speed_btn=tk.Button(
            param_row,
            text="Confirm Speed",
            width=15,
            command=lambda: self.vel_acc_set('B'),
            bg="#58C3EE",
            font=("Arial", 9, "bold")
        )
        speed_btn.pack(side="left",padx=(0,20))

        # 阻抗参数设置按钮
        self.right_impedance_btn = tk.Button(
            param_row,
            text="Impedance Params",
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
            text="Realtime Data",
            padding=10,
            relief=tk.GROOVE,
            borderwidth=2,
            style="MyCustom.TLabelframe"
        )
        data_frame.pack(fill="x",pady=(0, 10))

        # 关节数据
        tk.Label(data_frame, text="J1-J7:", font=('Arial', 10, 'bold'), bg="white").pack(anchor="w", pady=(0, 5))
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
        self.right_joint_text.pack(fill="both", expand=True)
        self.right_joint_text.insert("1.0", "0.000,0.000,0.000,0.000,0.000,0.000,0.000")
        self.right_joint_text.tag_add("center", "1.0", "end")
        self.right_joint_text.config(state="disabled")

        # 笛卡尔数据
        tk.Label(data_frame, text="XYZABC(flange):", font=('Arial', 10, 'bold'), bg="white").pack(
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
        self.right_cartesian_text.pack(fill="both", expand=True)
        self.right_cartesian_text.insert("1.0", "0.000,0.000,0.000,0.000,0.000,0.000")
        self.right_cartesian_text.tag_add("center", "1.0", "end")
        self.right_cartesian_text.config(state="disabled")


        '''第三列：力控指令和位置指令'''
        right_second_control_frame = tk.Frame(right_content, bg="white", width=650)
        right_second_control_frame.pack(side="left", fill="y")
        right_second_control_frame.pack_propagate(False)  # 保持固定宽度

        # 力控组件
        force_control_frame = ttk.LabelFrame(
            right_second_control_frame,
            text="Force Control",
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
        tk.Label(force_row, text="Force:", font=('Arial', 9),bg='white').pack(side="left", padx=(0, 0))
        self.right_force_entry = tk.Entry(force_row, width=5, font=('Arial', 9), justify='center')
        self.right_force_entry.pack(side="left")
        self.right_force_entry.insert(0, "10")  # 最大不超过60N
        tk.Label(force_row, text="1N-60N", font=('Arial', 9),bg='white').pack(side="left", padx=(0, 10))

        # 调节量
        tk.Label(force_row, text="Adjust:", font=('Arial', 9),bg='white').pack(side="left", padx=(0, 0))
        self.right_force_adj_entry = tk.Entry(force_row, width=5, font=('Arial', 9), justify='center')
        self.right_force_adj_entry.pack(side="left")
        self.right_force_adj_entry.insert(0, "50")  # 最大不超过50mm
        tk.Label(force_row, text="1mm-50mm", font=('Arial', 9),bg='white').pack(side="left", padx=(0, 10))

        self.right_force_dir_btn1 = tk.Button(
            force_row,
            text="X Dir.",
            width=5,
            command=lambda: self.imped_f_mode(0, 'B'),
            font=("Arial", 9, "bold"),
            bg='#E2F6FF'
        )
        self.right_force_dir_btn1.pack(side="left", padx=(0, 5))

        self.right_force_dir_btn2 = tk.Button(
            force_row,
            text="Y Dir.",
            width=5,
            command=lambda: self.imped_f_mode(1, 'B'),
            font=("Arial", 9, "bold"),
            bg='#F6DFF6'
        )
        self.right_force_dir_btn2.pack(side="left", padx=(0, 5))

        self.right_force_dir_btn3 = tk.Button(
            force_row,
            text="Z Dir.",
            width=5,
            command=lambda: self.imped_f_mode(2, 'B'),
            font=("Arial", 9, "bold"),
            bg='#F7F7CE'
        )
        self.right_force_dir_btn3.pack(side="left", padx=(0, 5))

        force_row1 = tk.Frame(force_control_frame, bg='white')
        force_row1.pack(fill="x", pady=(5, 5))
        tk.Label(force_row1, text="Any Dir.:", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 5))

        left_force_entry1 = tk.Entry(force_row1, textvariable=self.force_dir_b_entry, font=('Arial', 9), width=30)
        left_force_entry1.pack(side="left", padx=(0, 5))

        self.right_force_dir_btn3 = tk.Button(
            force_row1,
            text="User Defined",
            width=10,
            command=lambda: self.imped_f_mode(3, 'B'),
            font=("Arial", 9, "bold"),
            bg='#CCCCFF'
        )
        self.right_force_dir_btn3.pack(side="left", padx=(5, 5))

        # 关节指令
        joint_cmd_frame = ttk.LabelFrame(
            right_second_control_frame,
            text="Position Cmd",
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
        self.btn_add3 = tk.Button(joints_row,  text="GetCurrentPos", width=15, command=lambda: self.add_current_joints('B'))
        self.btn_add3.pack(side="left", padx=(0, 5))

        # 第二列：输入文本框
        self.entry_var1 = tk.StringVar(value="0,0,0,0,0,0,0")
        self.entry1 = tk.Entry(joints_row, textvariable=self.entry_var1, width=45)
        self.entry1.pack(side="left", padx=(0, 5), expand=True)

        # 第一列：1#加点按钮
        self.btn_add1 = tk.Button(joints_row, text="Add", width=8, command=self.add_point2)
        self.btn_add1.pack(side="left", padx=(0, 5))

        # 第一行运行当前点位
        joints_row1 = tk.Frame(joint_cmd_frame,bg='white')
        joints_row1.pack(fill="x", pady=(0, 5))

        # 删除
        self.btn_del1 = tk.Button(joints_row1, text="Delete", width=8, command=self.delete_point2)
        self.btn_del1.pack(side="left", padx=(0, 5))
        #
        self.combo2 = ttk.Combobox(joints_row1, state="readonly", width=45)
        self.combo2.pack(side="left", padx=(0, 5))

        # 第三列：1#运行按钮
        self.btn_run2 = tk.Button(joints_row1, text="Run", width=8, command=lambda: self.run_joints('B'),
                                  font=("Arial", 11, "bold"),fg='white', bg='#EC2A23',border=5)
        self.btn_run2.pack(side="left", padx=(0, 5))

        # 第三行 保存导入
        joints_row2 = tk.Frame(joint_cmd_frame,bg='white')
        joints_row2.pack(fill="x", pady=(0, 5))

        # 第四列：1#保存按钮
        self.btn_save2 = tk.Button(joints_row2, text="Save", width=8, command=self.save_points2)
        self.btn_save2.pack(side="left", padx=(180, 5),pady=(0,10))

        # 第五列：1#导入按钮
        self.btn_load2 = tk.Button(joints_row2, text="Import", width=8, command=self.load_points2)
        self.btn_load2.pack(side="left", padx=(0, 5),pady=(0,10))

        # 第四行 轨迹复现
        joints_row3 = tk.Frame(joint_cmd_frame,bg='white')
        joints_row3.pack(fill="x", pady=(0, 5))

        self.run_period_2 = tk.Button(joints_row3, text="Trajectory",width=8,  command=lambda: self.thread_run_period('B'))
        self.run_period_2.pack(side="left", padx=(0, 5))

        self.period_path_entry_2 = tk.Entry(joints_row3, textvariable=self.period_file_path_2, width=45,
                                            font=("Arial", 9), bg='white')
        self.period_path_entry_2.pack(side="left", padx=(0, 5),expand=True)

        self.btn_load_file2 = tk.Button(joints_row3, text="Select File",width=8,  command=lambda: self.select_period_file('B'))
        self.btn_load_file2.pack(side="left", padx=(0, 5))

        '''第四列：急停'''
        stop_control_frame = tk.Frame(right_content, bg="white",width=800)
        stop_control_frame.pack(side="left", fill="y")
        stop_control_frame.pack_propagate(False)

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
        menu.add_command(label="System Upgrade", command=self.system_update_dialog)
        menu.add_separator()
        # menu.add_command(label="Planning", command=self.movl_dialog)
        # menu.add_separator()
        menu.add_command(label="Sensors & Encoders", command=self.sensor_decoder_dialog)
        menu.add_separator()
        menu.add_command(label="IMU Calculation", command=self.additional_settings)
        # menu.add_command(label="数据管理", command=self.open_data_management)
        # menu.add_command(label="日志查看", command=self.open_log_viewer)
        # menu.add_separator()
        # menu.add_command(label="校准工具", command=self.open_calibration_tool)
        # menu.add_command(label="诊断工具", command=self.open_diagnostic_tool)
        menu.add_separator()
        menu.add_command(label="Docs", command=self.open_doc)
        # menu.add_command(label="关于软件", command=self.open_about)

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
        button_w = 10
        sensor_encoder_window = tk.Toplevel(self.root)
        sensor_encoder_window.title("Sensor and encoder functions")
        sensor_encoder_window.geometry("800x400")
        sensor_encoder_window.configure(bg="white")
        sensor_encoder_window.transient(self.root)
        sensor_encoder_window.resizable(True, True)
        sensor_encoder_window.grab_set()
        # 功能按钮框架

        self.sensor_frame_2 = tk.Frame(sensor_encoder_window, bg="white")
        self.sensor_frame_2.pack(fill="x",padx=5,pady=(15,10))
        # 第1 :text
        self.sensor_main_tex = tk.Label(self.sensor_frame_2, text="Sensor bias acquisition and setting", bg="#2196F3",
                                      fg="white", font=("Arial", 10, "bold"))
        self.sensor_main_tex.pack(fill='x')


        self.sensor_frame_1 = tk.Frame(sensor_encoder_window, bg="white")
        self.sensor_frame_1.pack(fill="x")

        self.axis_text_ = tk.Label(self.sensor_frame_1, text="Left   arm", bg="#D8F4F3")
        self.axis_text_.grid(row=0, column=0, padx=(5,5))
        # 第2列：sensor select
        self.axis_text_1 = tk.Label(self.sensor_frame_1, text="J:", bg="white")
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
        self.get_offset_btn_1 = tk.Button(self.sensor_frame_1, text="Get offset",
                                          command=lambda: self.get_sensor_offset('A'))
        self.get_offset_btn_1.grid(row=0, column=3, padx=5)

        # 第5列：get offset value

        self.get_offset_entry_1 = tk.Entry(self.sensor_frame_1, width=5)
        self.get_offset_entry_1.insert(0, '0.0')
        self.get_offset_entry_1.grid(row=0, column=4, padx=5)

        # 第6列：set offset
        self.set_offset_btn_1 = tk.Button(self.sensor_frame_1, text="Set offset",
                                          command=lambda: self.set_sensor_offset('A'))
        self.set_offset_btn_1.grid(row=0, column=5, padx=5)

        '''right arm'''
        self.sensor_frame_11 = tk.Frame(sensor_encoder_window, bg="white")
        self.sensor_frame_11.pack(fill="x")
        self.axis_text__ = tk.Label(self.sensor_frame_11, text="Right arm", bg="#F4E4D8")
        self.axis_text__.grid(row=0, column=0, padx=(5,5))
        # 第2列：sensor select
        self.axis_text_2 = tk.Label(self.sensor_frame_11, text="J:", bg="white")
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
        self.get_offset_btn_2 = tk.Button(self.sensor_frame_11, text="Get offset",
                                          command=lambda: self.get_sensor_offset('B'))
        self.get_offset_btn_2.grid(row=0, column=3, padx=5)
        # 第5列：get offset value
        self.get_offset_entry_2 = tk.Entry(self.sensor_frame_11, width=5)
        self.get_offset_entry_2.insert(0, '0.0')
        self.get_offset_entry_2.grid(row=0, column=4, padx=5)
        # 第6列：set offset
        self.set_offset_btn_2 = tk.Button(self.sensor_frame_11, text="Set offset",
                                          command=lambda: self.set_sensor_offset('B'))
        self.set_offset_btn_2.grid(row=0, column=5, padx=5, pady=5)

        '''encoder'''
        self.encoder_frame_1 = tk.Frame(sensor_encoder_window, bg="white")
        self.encoder_frame_1.pack(fill="x",padx=5,pady=(25,10))
        self.encoder_frame_1 = tk.Label(self.encoder_frame_1, text="Motor encoder zeroing and error clearing", bg="#2196F3",
                                      fg="white", font=("Arial", 10, "bold"))
        self.encoder_frame_1.pack(fill='x')
        '''left arm'''
        self.motor_frame_1 = tk.Frame(sensor_encoder_window, bg="white")
        self.motor_frame_1.pack(fill="x")
        self.motor_text_1 = tk.Label(self.motor_frame_1, text="Left   arm", bg="#D8F4F3")
        self.motor_text_1.grid(row=0, column=0, padx=(5,5))
        self.motor_axis_text_1 = tk.Label(self.motor_frame_1, text="J:", bg="white")
        self.motor_axis_text_1.grid(row=0, column=1, padx=5)
        self.motor_axis_select_combobox_1 = ttk.Combobox(
            self.motor_frame_1,
            values=["0", "1", "2", "3", "4", "5", "6"],
            width=3,
            state="readonly"
        )
        self.motor_axis_select_combobox_1.current(0)
        self.motor_axis_select_combobox_1.grid(row=0, column=2, padx=5)

        self.motor_btn_1 = tk.Button(self.motor_frame_1, text="Motor internal encoder zeroing",
                                     command=lambda: self.clear_motor_as_zero('A'))
        self.motor_btn_1.grid(row=0, column=3, padx=5, pady=5)

        self.motor_btn_2 = tk.Button(self.motor_frame_1, text="Motor external encoder zeroing",
                                     command=lambda: self.clear_motorE_as_zero('A'))
        self.motor_btn_2.grid(row=0, column=4, padx=5)

        self.motor_btn_3 = tk.Button(self.motor_frame_1, text="Encoder clearing error", bg="#7ED2B4",
                                     command=lambda: self.clear_motor_error('A'))
        self.motor_btn_3.grid(row=0, column=5, padx=5)

        '''right arm'''
        self.motor_frame_2 = tk.Frame(sensor_encoder_window, bg="white")
        self.motor_frame_2.pack(fill="x")

        self.motor_text_1 = tk.Label(self.motor_frame_2, text="Right arm", bg="#F4E4D8")
        self.motor_text_1.grid(row=0, column=0, padx=(5, 5))

        self.motor_axis_text_11 = tk.Label(self.motor_frame_2, text="J:", bg="white")
        self.motor_axis_text_11.grid(row=0, column=1, padx=5)

        self.motor_axis_select_combobox_11 = ttk.Combobox(
            self.motor_frame_2,
            values=["0", "1", "2", "3", "4", "5", "6"],
            width=3,
            state="readonly"
        )
        self.motor_axis_select_combobox_11.current(0)
        self.motor_axis_select_combobox_11.grid(row=0, column=2, padx=5)

        self.motor_btn_11 = tk.Button(self.motor_frame_2, text="Motor internal encoder zeroing",
                                      command=lambda: self.clear_motor_as_zero('B'))
        self.motor_btn_11.grid(row=0, column=3, padx=5)

        self.motor_btn_21 = tk.Button(self.motor_frame_2, text="Motor external encoder zeroing",
                                      command=lambda: self.clear_motorE_as_zero('B'))
        self.motor_btn_21.grid(row=0, column=4, padx=5)

        self.motor_btn_31 = tk.Button(self.motor_frame_2, text="Encoder clearing error", bg="#7ED2B4",
                                      command=lambda: self.clear_motor_error('B'))
        self.motor_btn_31.grid(row=0, column=5, padx=5)

    def system_update_dialog(self):
        button_w = 10
        hidden_window = tk.Toplevel(self.root)
        hidden_window.title("System Upgrade")
        hidden_window.geometry("600x400")
        hidden_window.configure(bg="white")
        hidden_window.transient(self.root)
        hidden_window.resizable(True, True)
        hidden_window.grab_set()

        title_frame = tk.Frame(hidden_window, bg="white")
        title_frame.pack(fill="x",padx=5,pady=(15,10))
        title_label = tk.Label(title_frame, text="Robot configuration file update", bg="#2196F3",
                                      fg="white", font=("Arial", 10, "bold"))
        title_label.pack(fill='x')


        '''第二行'''
        state_a_frame = tk.Frame(hidden_window, bg="white")
        state_a_frame.pack(fill="x", pady=5)

        hand_text_frame=tk.Label(state_a_frame,text="Manually update configuration files",bg='#CCCCFF')
        hand_text_frame.pack(side='left',expand=True)

        reset_a_button = tk.Button(state_a_frame, text="Download robot configuration file", width=button_w + 10,
                                   command=self.get_ini)
        reset_a_button.pack(side='left',expand=True)
        pvt_a_button = tk.Button(state_a_frame, text="Update robot configuration file", width=button_w + 10,
                                 command=self.update_ini)
        pvt_a_button.pack(side='left',expand=True)

        state_a_frame_ = tk.Frame(hidden_window, bg="white")
        state_a_frame_.pack(fill="x", pady=5)
        hand_text_frame_ = tk.Label(state_a_frame_, text="Automatically compare and update configuration files",bg='#CCCCFF')
        hand_text_frame_.pack(side='left', expand=True)
        param_c_btn=tk.Button(state_a_frame_,text='Configuration comparison update',width=button_w+10,command=self.compare_parameters_dialog, fg='#033341',bg='#DFC88C')
        param_c_btn.pack(side='left', expand=True)

        title_frame_ = tk.Frame(hidden_window, bg="white")
        title_frame_.pack(fill="x",padx=5,pady=(25,10))

        title_label_ = tk.Label(title_frame_, text="System Upgrade", bg="#2196F3",
                                      fg="white", font=("Arial", 10, "bold"))
        title_label_.pack(fill='x',expand=True)

        state_a_frame1 = tk.Frame(hidden_window, bg="white")
        state_a_frame1.pack(fill="x", pady=5)
        reset_a_button = tk.Button(state_a_frame1, text="Update system", width=10,
                                   command=self.update_sys, bg="#F6FC39",
                                   fg="#151513",
                                   font=("Arial", 10, "bold")).pack(side='left', expand=True)
        # reset_a_button.pack(side="left")

        state_a_frame2 = tk.Frame(hidden_window, bg="white")
        state_a_frame2.pack(fill="x", pady=5)
        label = tk.Label(state_a_frame2,
                         text='The system update package file selected is: *.MV_SYS_UPDATE', bg='white')
        label.pack(padx=5, pady=10)

    def compare_parameters_dialog(self):
        robot.receive_file('robot.ini', "/home/FUSION/Config/cfg/robot.ini")
        time.sleep(1)
        self.source_content = self.load_file(self.source_file)
        compare_window = tk.Toplevel(self.root)
        compare_window.title("Configuration comparison update")
        compare_window.geometry("1000x900")
        compare_window.configure(bg="#f0f0f0")
        compare_window.transient(self.root)
        compare_window.resizable(True, True)
        compare_window.grab_set()

        main_frame = ttk.Frame(compare_window, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(2, weight=1)

        file_frame = ttk.LabelFrame(main_frame, text="File selection", padding="10")
        file_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))

        ttk.Label(file_frame, text="Src file:").grid(row=0, column=0, sticky=tk.W)
        self.source_label = ttk.Label(file_frame, text=self.source_file, foreground="blue")
        self.source_label.grid(row=0, column=1, sticky=tk.W, padx=(5, 20))

        ttk.Label(file_frame, text="Compare file:").grid(row=1, column=0, sticky=tk.W, pady=(10, 0))
        self.target_label = ttk.Label(file_frame, text="未选择", foreground="red")
        self.target_label.grid(row=1, column=1, sticky=tk.W, padx=(5, 10), pady=(10, 0))

        ttk.Button(file_frame, text="Select comparison file",
                   command=self.select_target_file).grid(row=1, column=2, padx=(10, 0), pady=(10, 0))

        self.status_label = ttk.Label(main_frame, text="", foreground="blue")
        self.status_label.grid(row=1, column=0, columnspan=2, sticky=tk.W, pady=(0, 10))

        # 对比显示区域
        compare_frame = ttk.LabelFrame(main_frame, text="File compare", padding="10")
        compare_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 10))
        compare_frame.columnconfigure(0, weight=1)
        compare_frame.columnconfigure(1, weight=1)
        compare_frame.rowconfigure(0, weight=1)

        ttk.Label(compare_frame, text="Source file contents").grid(row=0, column=0, sticky=tk.W)
        self.source_text = tk.Text(compare_frame, wrap=tk.WORD, width=50, height=30)
        self.source_text.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 5))

        ttk.Label(compare_frame, text="Compare file contents").grid(row=0, column=1, sticky=tk.W)
        self.target_text = tk.Text(compare_frame, wrap=tk.WORD, width=50, height=30)
        self.target_text.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(5, 0))

        v_scrollbar = ttk.Scrollbar(compare_frame, orient=tk.VERTICAL)
        v_scrollbar.grid(row=1, column=2, sticky=(tk.N, tk.S))

        self.source_text.config(yscrollcommand=v_scrollbar.set)
        self.target_text.config(yscrollcommand=v_scrollbar.set)
        v_scrollbar.config(command=self.sync_scroll)

        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=3, column=0, columnspan=2, sticky=tk.E, pady=(0, 10))

        self.compare_button = ttk.Button(button_frame, text="Compare Files",
                                         command=self.compare_files, state="disabled")
        self.compare_button.pack(side=tk.LEFT, padx=(0, 10))

        self.apply_button = ttk.Button(button_frame, text="Confirm changes",
                                       command=self.apply_changes, state="disabled")
        self.apply_button.pack(side=tk.LEFT)

        self.update_ini_button = ttk.Button(button_frame, text="Upload file",
                                       command=self.update_ini2)
        self.update_ini_button.pack(side=tk.LEFT)

    def sync_scroll(self, *args):
        self.source_text.yview(*args)
        self.target_text.yview(*args)

    def select_target_file(self):
        file_path = filedialog.askopenfilename(
            title="Select comparison file",
            filetypes=[("INI files", "*.ini"), ("All files", "*.*")]
        )

        if file_path:
            self.target_file = file_path
            self.target_label.config(text=file_path, foreground="green")
            self.compare_button.config(state="normal")
            self.update_status("You have selected the files to compare. Click 'Compare Files' to begin the comparison.")

    def update_status(self, message=""):
        if not message:
            if self.target_file:
                message = f"Comparison files selected: {Path(self.target_file).name}"
            else:
                message = "Please select comparison file"
        self.status_label.config(text=message)

    def compare_files(self):
        if not self.target_file:
            messagebox.showwarning("Warning", "Please select the comparison file first.")
            return

        target_content = self.load_file(self.target_file)
        if not target_content:
            return

        self.source_text.delete(1.0, tk.END)
        self.target_text.delete(1.0, tk.END)

        self.highlight_differences(self.source_content, target_content)

        self.apply_button.config(state="normal")
        self.update_status("The file comparison is complete; you can now view the differences and apply the changes.")

    def highlight_differences(self, source_content, target_content):
        source_lines = source_content.splitlines()
        target_lines = target_content.splitlines()
        diff = difflib.SequenceMatcher(None, source_lines, target_lines)
        for i, line in enumerate(source_lines):
            self.source_text.insert(tk.END, line + '\n')
            tag_added = False
            for tag in diff.get_opcodes():
                if tag[0] != 'equal':
                    if tag[1] <= i < tag[2]:
                        self.source_text.tag_add('diff', f"{i + 1}.0", f"{i + 1}.end")
                        tag_added = True
                        break
            if not tag_added:
                in_target = any(line == target_line for target_line in target_lines)
                if not in_target and line.strip():
                    self.source_text.tag_add('removed', f"{i + 1}.0", f"{i + 1}.end")

        for i, line in enumerate(target_lines):
            self.target_text.insert(tk.END, line + '\n')

            for tag in diff.get_opcodes():
                if tag[0] != 'equal':
                    if tag[3] <= i < tag[4]:
                        self.target_text.tag_add('diff', f"{i + 1}.0", f"{i + 1}.end")
                        break

            in_source = any(line == source_line for source_line in source_lines)
            if not in_source and line.strip():
                self.target_text.tag_add('added', f"{i + 1}.0", f"{i + 1}.end")

        self.source_text.tag_config('diff', background='#FFD700')
        self.source_text.tag_config('removed', background='#FFCCCC')

        self.target_text.tag_config('diff', background='#FFD700')
        self.target_text.tag_config('added', background='#CCFFCC')

    def parse_ini_by_sections(self, content):
        lines = content.splitlines()
        sections = {}
        current_section = None
        current_section_lines = []
        section_start_line = 0
        protected_keys = set()
        section_order = []
        for i, line in enumerate(lines):
            stripped = line.strip()
            if stripped.startswith('[') and stripped.endswith(']'):
                if current_section is not None:
                    sections[current_section] = {
                        'start_line': section_start_line,
                        'lines': current_section_lines,
                        'params': self.extract_params(current_section_lines, protected_keys)
                    }
                    section_order.append(current_section)

                current_section = stripped[1:-1]
                current_section_lines = [line]
                section_start_line = i
            else:
                if '=' in stripped and '*' in stripped:
                    key_part = stripped.split('=', 1)[0].strip()
                    if '*' in key_part:
                        clean_key = key_part.rstrip('*').strip()
                        protected_keys.add(clean_key)

                if current_section is not None:
                    current_section_lines.append(line)
                else:
                    if 'global' not in sections:
                        sections['global'] = {
                            'start_line': 0,
                            'lines': [],
                            'params': {}
                        }
                        section_order.append('global')
                    sections['global']['lines'].append(line)

        if current_section is not None:
            sections[current_section] = {
                'start_line': section_start_line,
                'lines': current_section_lines,
                'params': self.extract_params(current_section_lines, protected_keys)
            }
            section_order.append(current_section)

        return sections, protected_keys, section_order

    def extract_params(self, section_lines, protected_keys):
        params = {}
        for line in section_lines:
            stripped = line.strip()
            if stripped and '=' in stripped and not stripped.startswith(';'):
                key_part, value_part = stripped.split('=', 1)
                key = key_part.strip()
                has_star = False
                if key.endswith('*'):
                    has_star = True
                    key = key.rstrip('*').strip()

                params[key] = {
                    'value': value_part.strip(),
                    'has_star': has_star,
                    'original_line': line
                }

        return params

    def apply_changes(self):
        if not self.target_file:
            return
        if not messagebox.askyesno("Confirm", "Are you sure you want to apply the changes to the source file?"):
            return
        try:
            target_content = self.load_file(self.target_file)
            if not target_content:
                return

            source_sections, source_protected, source_order = self.parse_ini_by_sections(self.source_content)
            target_sections, target_protected, target_order = self.parse_ini_by_sections(target_content)

            new_content = self.apply_modification_rules_by_sections(
                source_sections, target_sections, source_protected, target_protected,
                source_order, target_order, self.source_content
            )
            if new_content != self.source_content:
                with open(self.source_file, 'w', encoding='utf-8') as f:
                    f.write(new_content)
                self.source_content = new_content
                self.compare_files()

                messagebox.showinfo("success", "The modifications have been successfully applied to the source file.")
                self.update_status("The changes have been applied, and the source file has been reloaded.")
            else:
                messagebox.showinfo("Information", "No modifications are needed.")

        except Exception as e:
            messagebox.showerror("Error", f"Modification failed: {str(e)}")

    def apply_modification_rules_by_sections(self, source_sections, target_sections,
                                             source_protected, target_protected,
                                             source_order, target_order, original_source_content):
        new_lines = []
        processed_sections = set()
        original_lines = original_source_content.splitlines()
        for line in original_lines:
            stripped = line.strip()
            if not stripped or stripped.startswith(';'):
                new_lines.append(line)
            elif not stripped.startswith('['):
                new_lines.append(line)
            else:
                break
        for section_name in source_order:
            if section_name == 'global' or section_name not in source_sections:
                continue
            processed_sections.add(section_name)
            if section_name in target_sections:
                new_section_lines = self.merge_section(
                    source_sections[section_name],
                    target_sections[section_name],
                    source_protected
                )
                if new_lines and not self.should_add_empty_line_before(new_lines[-1], new_section_lines[0]):
                    section_start_line = source_sections[section_name]['start_line']
                    if section_start_line > 0 and original_lines[section_start_line - 1].strip() == "":
                        new_lines.append("")
                new_lines.extend(new_section_lines)
            else:
                has_protected_keys = False
                for key in source_sections[section_name]['params']:
                    if key in source_protected:
                        has_protected_keys = True
                        break

                if has_protected_keys:
                    if new_lines and not self.should_add_empty_line_before(new_lines[-1],
                                                                           source_sections[section_name]['lines'][0]):
                        section_start_line = source_sections[section_name]['start_line']
                        if section_start_line > 0 and original_lines[section_start_line - 1].strip() == "":
                            new_lines.append("")
                    new_lines.extend(source_sections[section_name]['lines'])

        new_sections_to_add = []
        for section_name in target_order:
            if section_name == 'global' or section_name in processed_sections:
                continue
            new_sections_to_add.append((section_name, target_order.index(section_name)))
        new_sections_to_add.sort(key=lambda x: x[1])
        for section_name, _ in new_sections_to_add:
            insert_after = None
            target_index = target_order.index(section_name)
            for j in range(target_index - 1, -1, -1):
                prev_section = target_order[j]
                if prev_section in processed_sections:
                    insert_after = prev_section
                    break

            if insert_after:
                insert_index = -1
                for idx in range(len(new_lines)):
                    if new_lines[idx].strip() == f"[{insert_after}]":
                        for k in range(idx, len(new_lines)):
                            if k + 1 < len(new_lines) and new_lines[k + 1].strip().startswith('['):
                                insert_index = k + 1
                                break
                        if insert_index == -1:
                            insert_index = len(new_lines)
                        break

                if insert_index >= 0:
                    new_section_lines = [f"[{section_name}]"]
                    for key, param_info in target_sections[section_name]['params'].items():
                        star_marker = '*' if key in target_protected else ''
                        new_section_lines.append(f"{key}{star_marker}={param_info['value']}")

                    new_lines[insert_index:insert_index] = new_section_lines
            else:
                new_lines.append(f"[{section_name}]")
                for key, param_info in target_sections[section_name]['params'].items():
                    star_marker = '*' if key in target_protected else ''
                    new_lines.append(f"{key}{star_marker}={param_info['value']}")

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

        while cleaned_lines and cleaned_lines[-1].strip() == "":
            cleaned_lines.pop()
        return '\n'.join(cleaned_lines)

    def should_add_empty_line_before(self, last_line, current_section_header):
        if not last_line.strip():
            return True
        if last_line.strip().startswith(';'):
            return True
        if last_line.strip().startswith('['):
            return False
        return True

    def merge_section(self, source_section, target_section, source_protected):
        new_section_lines = []
        new_section_lines.append(source_section['lines'][0])
        for key, source_param in source_section['params'].items():
            if key in source_protected:
                new_section_lines.append(source_param['original_line'])
            elif key in target_section['params']:
                target_param = target_section['params'][key]
                new_section_lines.append(f"{key}={target_param['value']}")
            else:
                pass
        for key, target_param in target_section['params'].items():
            if key not in source_section['params'] and key not in source_protected:
                new_section_lines.append(f"{key}={target_param['value']}")
        comments_and_blanks = []
        for line in source_section['lines'][1:]:
            stripped = line.strip()
            if not stripped or stripped.startswith(';'):
                comments_and_blanks.append(line)
        if comments_and_blanks:
            first_param_index = -1
            for i, line in enumerate(new_section_lines[1:], 1):
                if '=' in line:
                    first_param_index = i
                    break
            if first_param_index > 0:
                new_section_lines[first_param_index:first_param_index] = comments_and_blanks
        return new_section_lines

    def get_ini(self):
        if self.connected:
            file_path = filedialog.asksaveasfilename(
                defaultextension=".ini",
                filetypes=[("ini files", "*.ini"), ("All files", "*.*")],
                title="Save robot configuration parameter file"
            )
            if file_path:
                tag = robot.receive_file(file_path, "/home/FUSION/Config/cfg/robot.ini")
                time.sleep(1)
                if tag:
                    messagebox.showinfo('Success', 'Configuration file has been saved')
        else:
            messagebox.showerror('Error', 'Please connect robot')

    def update_ini2(self):
        if self.connected:
            tag=robot.send_file('robot.ini', "/home/FUSION/Config/cfg/robot.ini")
            time.sleep(1)
            if tag:
                messagebox.showinfo('Success', 'The configuration file have been uploaded to the controller.')
        else:
            messagebox.showerror('Error', 'Please connect robot')


    def update_ini(self):
        if self.connected:
            file_path = filedialog.askopenfilename(
                defaultextension=".ini",
                filetypes=[("ini files", "*.ini"), ("All files", "*.*")],
                title="Select robot configuration parameter file"
            )
            if file_path:
                tag = robot.send_file(file_path, "/home/FUSION/Config/cfg/robot.ini")
                time.sleep(1)
                if tag:
                    messagebox.showinfo('Success', 'Configuration file has been saved')
        else:
            messagebox.showerror('Error', 'Please connect robot')

    def update_sys(self):
        if self.connected:
            result = messagebox.askokcancel("Confirm", "After uploading the file, restarting will automatically update the system version. \nConfirm the upload?")
            if result:
                file_path = filedialog.askopenfilename(
                    filetypes=[("All files", "*.*")],
                    title="Select system update file"
                )
                if file_path:
                    tag1 = robot.update_SDK(file_path)
                    if tag1:
                        messagebox.showinfo('Success', 'The system files have been uploaded. Please restart the controller to update automatically.')
                    else:
                        messagebox.showinfo('Error', 'System file upload failed, please upload again.')
        else:
            messagebox.showerror('Error', 'Please connect robot')

    def additional_settings(self):
        """打开系统设置窗口"""
        settings_window = tk.Toplevel(self.root)
        settings_window.title("Additional features")
        settings_window.geometry("800x600")
        settings_window.configure(bg="#f0f0f0")
        settings_window.transient(self.root)
        settings_window.resizable(True,True)
        settings_window.grab_set()

        notebook = ttk.Notebook(settings_window)
        notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        floating_base_frame = ttk.Frame(notebook, padding="10")
        notebook.add(floating_base_frame, text="Floating base parameter calculation")
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

        ttk.Button(button_frame, text="Save settings",
                   command=lambda: self.save_all_settings(notebook)).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Close",
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
        self.row2_selection = [0, 0, 0]  # 对应X,Y,Z
        self.row3_selection = [0, 0, 0]  # 对应X,Y,Z

        self.row2_var = tk.StringVar()
        self.row3_var = tk.StringVar()

        self.row2_var.trace('w', lambda *args: self.on_selection_change(2))
        self.row3_var.trace('w', lambda *args: self.on_selection_change(3))

        ttk.Label(parent, text="Floating base parameter calculation", font=("Arial", 14, "bold")).pack(pady=10)

        # 第一行
        row1_frame = ttk.Frame(parent)
        row1_frame.pack(fill="x", pady=5)

        ttk.Label(row1_frame, text="The coordinate directions of the base (x-axis and y-axis)").pack(side="left", padx=5)
        ttk.Label(row1_frame, text="UMI coordinate orientation (option to align base with UMI coordinate orientation)").pack(side="right", padx=5)

        axis_frame = ttk.Frame(parent)
        axis_frame.pack(fill="x", pady=10)

        ttk.Label(axis_frame, text="X-axis").grid(row=0, column=0, padx=10, pady=10, sticky="w")
        ttk.Label(axis_frame, text="Y-axis").grid(row=1, column=0, padx=10, pady=10, sticky="w")

        options = ["x", "-x", "y", "-y", "z", "-z"]
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

        self.result_frame = ttk.LabelFrame(parent, text="Calculation results")
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
            self.result_text.insert(tk.END, "Please complete the selections in both rows to view the calculation results.")

    def update_selection_lists(self):
        """根据当前选择更新选择列表"""
        self.row2_selection = [0, 0, 0]
        self.row3_selection = [0, 0, 0]

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
        result = f"The base coordinate direction is as follows during the rotation of the gyroscope IMU:\n"
        result += "=" * 20 + "\n"

        try:
            abc = main_function(self.row2_selection, self.row3_selection)
            result += abc
            result += "\n"
        except Exception as e:
            result += f"Calculation error: {str(e)}\n"

        result += "=" * 20 + "\n\n"
        result += ("Please update the three angles A, B, and C to the [R.A0.BASIC] group in robot.ini respectively:\n"
                    "GYROSETA, GYROSETB, GYROSETC\n"
                    "Please note that the left and right arms should be calculated sequentially, with [R.A0.BASIC] representing the left arm and [R.A1.BASIC] representing the right arm.")
        return result

    def display_result(self, result):
        self.result_text.delete(1.0, tk.END)
        self.result_text.insert(tk.END, result)

    def save_all_settings(self, notebook):
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
            result = messagebox.askokcancel("Confirm", "The robot will switch to impedance force control mode. Confirm the switch?")
            if result:
                robot.clear_set()
                robot.set_state(arm=robot_id, state=3)
                robot.set_impedance_type(arm=robot_id, type=3)
                robot.send_cmd()
                time.sleep(0.001)
                directions = [0, 0, 0, 0, 0, 0]
                if dir==3:
                    flag=None
                    dir=None
                    if robot_id=='A':
                        flag,dir=self.validate_point(self.force_dir_a_entry.get(),6)
                    elif robot_id=='B':
                        flag, dir = self.validate_point(self.force_dir_b_entry.get(), 6)
                    if flag:
                        values = dir.split(',')
                        directions = [float(value.strip()) for value in values]
                else:
                    directions[dir]=1
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
                print("Cancel switching force control operation")
        else:
            messagebox.showerror('Error', 'Please connect robot')

    def is_duplicate(self, point_list, target_list):
        """检查点是否已经在列表中存在（去重功能）"""
        point_tuple = tuple(point_list)
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
                return False, "The input cannot be empty."
            values = point_str.split(',')
            if len(values) != nums:
                return False, f"Please enter {nums} comma-separated numbers"
            # 检查每个值是否为数字字符
            validated_values = []
            for value in values:
                value = value.strip()
                if not value:
                    return False, "All positions must contain numbers and cannot be empty."
                if not value.isdigit():
                    try:
                        float(value)
                    except ValueError:
                        return False, f"'{value}' is not a valid number"
                validated_values.append(value)
            if len(validated_values) != nums:
                return False, f"The list length must be {nums}"
            normalized_str = ','.join(validated_values)
            return True, normalized_str
        except Exception as e:
            return False, f"Incorrect input format: {str(e)}"

    def run_joints(self,robot_id):
        if self.connected:
            selected=None
            if robot_id=='A':
                selected = self.combo1.get()
            else:
                selected=self.combo2.get()
            if selected:
                is_valid, point_list = self.validate_point(selected,7)
                if is_valid:
                    values = point_list.split(',')
                    point_list = [float(value.strip()) for value in values]
                    robot.clear_set()
                    robot.set_joint_cmd_pose(arm=robot_id, joints=point_list)
                    robot.send_cmd()
                else:
                    messagebox.showerror("Error", f"Invalid format of selected points: {selected}")
            else:
                messagebox.showwarning("Warning", "No workable points.")
        else:
            messagebox.showerror('Error', 'Please connect robot')

    def update_comboboxes(self):
        """更新两个下拉框的内容"""
        self.combo1['values'] = self.points1
        self.combo2['values'] = self.points2

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
            messagebox.showerror('Error', 'Please connect robot')

    def add_point1(self):
        """添加点到1#列表"""
        point_str = self.entry_var.get()
        is_valid, result = self.validate_point(point_str,7)
        print(f'#1 add point:{result}')
        if is_valid:
            if self.is_duplicate(result, self.points1):
                messagebox.showwarning("Duplicate point", "This point already exists in list left arm")
                return
            self.points1.insert(0, result)
            self.update_comboboxes()
        else:
            messagebox.showwarning("Wrong inputs", result)

    def add_point2(self):
        """添加点到2#列表"""
        point_str = self.entry_var1.get()
        is_valid, result = self.validate_point(point_str,7)
        print(f'#2 add point:{result}')
        if is_valid:
            if self.is_duplicate(result, self.points2):
                messagebox.showwarning("Duplicate point", "This point already exists in list right arm")
                return
            self.points2.insert(0, result)
            self.update_comboboxes()
        else:
            messagebox.showwarning("Wrong inputs", result)

    def delete_point1(self):
        """从1#列表删除选中的点"""
        selected_index = self.combo1.current()
        if selected_index != -1 and selected_index < len(self.points1):
            self.points1.pop(selected_index)
            self.update_comboboxes()
        else:
            messagebox.showwarning("Warning", "Please select the point to delete")

    def delete_point2(self):
        """从2#列表删除选中的点"""
        selected_index = self.combo2.current()
        if selected_index != -1 and selected_index < len(self.points2):
            self.points2.pop(selected_index)
            self.update_comboboxes()
        else:
            messagebox.showwarning("Warning", "Please select the point to delete")

    def save_points1(self):
        """保存1#列表到TXT文件"""
        if not self.points1:
            messagebox.showwarning("Warning", "The left arm list is empty; there is no content to save.")
            return
        file_path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            title="Save points of left arm"
        )
        if file_path:
            try:
                with open(file_path, 'w') as f:
                    for point in self.points1:
                        f.write(point + '\n')
            except Exception as e:
                messagebox.showerror("Error", f"Error saving file: {str(e)}")

    def save_points2(self):
        """保存2#列表到TXT文件"""
        if not self.points2:
            messagebox.showwarning("Warning", "The right arm list is empty; there is no content to save.")
            return
        file_path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            title="Save points of right arm"
        )
        if file_path:
            try:
                with open(file_path, 'w') as f:
                    for point in self.points2:
                        f.write(point + '\n')
            except Exception as e:
                messagebox.showerror("Error", f"Error saving file: {str(e)}")

    def load_points1(self):
        """从TXT文件导入到1#列表"""
        file_path = filedialog.askopenfilename(
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            title="Select the file you want to import into left arm."
        )
        if file_path:
            try:
                with open(file_path, 'r') as f:
                    lines = f.readlines()
                valid_points = []
                invalid_lines = []
                for i, line in enumerate(lines, 1):
                    line = line.strip()
                    if line:
                        is_valid, result = self.validate_point(line,7)
                        if is_valid:
                            if not self.is_duplicate(result, self.points1 + valid_points):
                                valid_points.append(str(result))
                        else:
                            invalid_lines.append(f"Line {i}: {line}")
                if valid_points:
                    self.points1 = valid_points
                    self.update_comboboxes()
                if invalid_lines:
                    messagebox.showwarning("Warining",
                                           f"The following line has been skipped as it has invalid formatting:\n" +
                                           "\n".join(invalid_lines[:10]) +
                                           ("\n..." if len(invalid_lines) > 10 else ""))
            except Exception as e:
                messagebox.showerror("Error", f"Error reading file: {str(e)}")

    def load_points2(self):
        """从TXT文件导入到2#列表"""
        file_path = filedialog.askopenfilename(
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            title="Select the file you want to import into right arm."
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
                    if line:
                        is_valid, result = self.validate_point(line,7)
                        if is_valid:
                            if not self.is_duplicate(result, self.points2 + valid_points):
                                valid_points.append(str(result))
                        else:
                            invalid_lines.append(f"Line {i}: {line}")
                if valid_points:
                    self.points2 = valid_points
                    self.update_comboboxes()
                if invalid_lines:
                    messagebox.showwarning("Warining",
                                           f"The following line has been skipped as it has invalid formatting:\n" +
                                           "\n".join(invalid_lines[:10]) +
                                           ("\n..." if len(invalid_lines) > 10 else ""))
            except Exception as e:
                messagebox.showerror("Error", f"Error reading file: {str(e)}")

    def select_period_file(self, robot_id):
        result = messagebox.askokcancel("Confirm", "Trajectory reproduction must be performed in joint following or impedance mode？")
        if result:
            file_path = filedialog.askopenfilename(
                defaultextension=".r50pth",
                filetypes=[("path files", "*.r50pth"), ("All files", "*.*")],
                title="Select the file to run periodically"
            )
            if file_path:
                if robot_id == 'A':
                    self.period_file_path_1.set(file_path)
                elif robot_id == 'B':
                    self.period_file_path_2.set(file_path)
    def process_line(self, line_num, line):
        """处理单行数据，将其转换为浮点数列表"""
        try:
            cleaned_line = line.strip()
            elements = cleaned_line.split()
            if len(elements) != 7:
                return f"Error: Line {line_num + 1} has {len(elements)} elements, but requires 7."
            float_list = [float(element) for element in elements]
            return float_list
        except ValueError as e:
            return f"Error: Line {line_num + 1} contains non-numeric data - {str(e)}"
        except Exception as e:
            return f"Error: An unknown error occurred while processing line {line_num + 1} - {str(e)}"

    def thread_run_period(self, robot_id):
        """在新线程中执行周期运行"""
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
                if robot_id == 'A':
                    with open(self.period_file_path_1.get(), 'r', encoding='utf-8') as file:
                        lines = file.readlines()
                    for i, line in enumerate(lines):
                        if self.stop_event.is_set():
                            print("Thread interrupted by external signal")
                            return
                        processed_line = self.process_line(i, line)
                        robot.clear_set()
                        robot.set_joint_cmd_pose(arm='A', joints=processed_line)
                        robot.send_cmd()
                        time.sleep(0.02)
                elif robot_id == 'B':
                    with open(self.period_file_path_2.get(), 'r', encoding='utf-8') as file:
                        lines = file.readlines()
                    for i, line in enumerate(lines):
                        if self.stop_event.is_set():
                            print("Thread interrupted by external signal")
                            return
                        processed_line = self.process_line(i, line)
                        robot.clear_set()
                        robot.set_joint_cmd_pose(arm='B', joints=processed_line)
                        robot.send_cmd()
                        time.sleep(0.02)
            except Exception as e:
                messagebox.showerror("Error", f"Error reading file: {e}")
                # self.root.after(0, lambda: messagebox.showerror("错误", f"读取文件时出错: {e}"))
        else:
            messagebox.showerror('Error', 'Please connect robot')

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
            messagebox.showerror('Error', 'Please connect robot')

    def show_impedance_dialog(self, robot_id):
        """显示阻抗参数设置对话框"""
        if not self.connected:
            messagebox.showwarning('Error', 'Please connect robot')
            return

        # 创建阻抗参数设置对话框
        impedance_dialog = tk.Toplevel(self.root)
        impedance_dialog.title(f"Impedance parameter settings")
        impedance_dialog.geometry("1200x500")
        impedance_dialog.configure(bg="white")
        impedance_dialog.transient(self.root)
        impedance_dialog.resizable(True,True)
        impedance_dialog.grab_set()

        arm_side = ''
        if robot_id=='A':
            arm_side = 'Left arm'
            main_frame = tk.Frame(impedance_dialog, padx=20, pady=20, bg='white')
            main_frame.pack(fill="both", expand=True)
            title_label = tk.Label(
                main_frame,
                text=f"Set the impedance parameters of the {arm_side}",
                font=('Arial', 10, 'bold'),
                fg='#2c3e50'
            )
            title_label.pack(pady=(0, 10))

            params_frame = tk.Frame(main_frame, bg='white')
            params_frame.pack(fill="x", pady=(5, 10))
            joint_kd_a_button = tk.Button(params_frame, text="Set joint impedance parameters", width=20,
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
            save_param_a_button = tk.Button(params_save_frame, text="Save parameters", command=lambda: self.save_param('A'))
            save_param_a_button.pack(side='left',expand=True)
            # SAVE PARA
            load_param_a_button = tk.Button(params_save_frame, text="Import parameters", command=lambda: self.load_param('A'))
            load_param_a_button.pack(side='left',expand=True)

            # set joint kd
            cart_kd_a_button = tk.Button(params_frame, text="Set cartesian impedance parameters", width=20,
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
            arm_side = 'right arm'
            # 创建主框架
            main_frame1 = tk.Frame(impedance_dialog, padx=20, pady=20,bg='white')
            main_frame1.pack(fill="both", expand=True)
            # 标题
            title_label1 = tk.Label(
                main_frame1,
                text=f"Set the impedance parameters of the {arm_side}",
                font=('Arial', 10, 'bold'),
                fg='#2c3e50'
            )
            title_label1.pack(pady=(0, 10))

            # 创建参数输入区域
            params_frame1 = tk.Frame(main_frame1, bg='white')
            params_frame1.pack(fill="x", pady=(5, 10))

            # row 1 0保存参数   1设置关节阻抗参数   2 K  3 K entry  4 D  5 D entry
            # set joint kd
            joint_kd_a_button1 = tk.Button(params_frame1, text="Set joint impedance parameters", width=20,
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
            save_param_b_button = tk.Button(params_save_frame1, text="Save parameters", command=lambda: self.save_param('B'))
            save_param_b_button.pack(side='left', expand=True)
            # SAVE PARA
            load_param_b_button = tk.Button(params_save_frame1, text="Import parameters", command=lambda: self.load_param('B'))
            load_param_b_button.pack(side='left', expand=True)

            # set joint kd
            cart_kd_b_button = tk.Button(params_frame1, text="Set cartesian impedance parameters", width=20,
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
                messagebox.showerror("Error",  "Joint K parameter cannot be empty!")
            if len(k) != 7:
                messagebox.showerror("Error", f"The joint K parameter must have 7 values; currently there are {len(k)} data points!")
            try:
                k = [float(item) for item in k]
            except ValueError:
                messagebox.showerror("Error", "The joint K parameter must be a valid value！")

            if not d:
                messagebox.showerror("Error", "The joint D parameter cannot be empty！")
            if len(d) != 7:
                messagebox.showerror("Error", f"The joint D parameter must have 7 values; currently there are {len(d)} data points!")
            try:
                d = [float(item) for item in d]
            except ValueError:
                messagebox.showerror("Error", "The joint D parameter must be a valid value！")
            robot.clear_set()
            robot.set_joint_kd_params(arm=robot_id, K=k, D=d)
            robot.send_cmd()
        else:
            messagebox.showerror('Error', 'Please connect robot')

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
                messagebox.showerror("Error", "The Cartesian parameter k cannot be empty！")
            if len(k) != 7:
                messagebox.showerror("Error", f"The Cartesian parameter K must have 7 parameters, and there are currently {len(k)} data points!")
            try:
                k = [float(item) for item in k]
            except ValueError:
                messagebox.showerror("Error", "The Cartesian parameter K must be a valid value！")

            if not d:
                messagebox.showerror("Error", "The Cartesian parameter D cannot be empty.！")
            if len(d) != 7:
                messagebox.showerror("Error", f"The Cartesian parameter D must have 7 parameters, and there are currently {len(d)} data points!")
            try:
                d = [float(item) for item in d]
            except ValueError:
                messagebox.showerror("Error", "The Cartesian parameter D must be a valid value！")
            robot.clear_set()
            robot.set_cart_kd_params(arm=robot_id, K=k, D=d, type=2)
            robot.send_cmd()
        else:
            messagebox.showerror('Error', 'Please connect robot')

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
            title="Save impedance parameters"
        )
        if file_path:
            try:
                with open(file_path, 'w') as f:
                    for point in self.params:
                        f.write(point + '\n')
            except Exception as e:
                messagebox.showerror("Error", f"Error saving file: {str(e)}")

    def load_param(self, robot_id):
        if robot_id == 'A':
            file_path = filedialog.askopenfilename(
                filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
                title="Select the parameter file to import into the left arm."
            )
            if file_path:
                try:
                    with open(file_path, 'r') as f:
                        lines = f.readlines()
                    valid_points = []
                    for i, line in enumerate(lines, 1):
                        line = line.strip()
                        if line:
                            valid_points.append(line)
                    print(f'valid_points:{valid_points}')

                    if valid_points:
                        self.k_a_entry.set(valid_points[0])
                        self.d_a_entry.set(valid_points[1])
                        self.cart_k_a_entry.set(valid_points[2])
                        self.cart_d_a_entry.set(valid_points[3])

                except Exception as e:
                    messagebox.showerror("Error", f"Error reading file: {str(e)}")

        elif robot_id == 'B':
            file_path = filedialog.askopenfilename(
                filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
                title="Select the parameter file to import into the Right arm."
            )
            if file_path:
                try:
                    with open(file_path, 'r') as f:
                        lines = f.readlines()
                    valid_points = []
                    for i, line in enumerate(lines, 1):
                        line = line.strip()
                        if line:
                            valid_points.append(line)
                    print(f'valid_points:{valid_points}')

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
                    messagebox.showerror("Error", f"Error reading file: {str(e)}")

    def reset_robot_state(self, robot_id):
        """复位按钮点击事件"""
        if not self.connected:
            messagebox.showerror('Error', 'Please connect robot')
            return

        else:
            try:
                robot.clear_set()
                robot.set_state(arm=robot_id, state=0)
                robot.send_cmd()

                if robot_id == 'A' and hasattr(self, 'left_emergency_btn'):
                    self.left_emergency_btn.reset()
                elif robot_id == 'B' and hasattr(self, 'right_emergency_btn'):
                    self.right_emergency_btn.reset()
                self.status_label.config(text="Connected", foreground='green')
            except Exception as e:
                messagebox.showerror("Error", f"Error occurred during the reset process.: {str(e)}")

    def on_state_selected(self, robot_id):
        """状态选择事件"""
        if not self.connected:
            messagebox.showerror('Error', 'Please connect robot')
            return

        selected_state=None
        if robot_id == 'A':
            selected_state = self.state_var.get()
        elif robot_id == 'B':
            selected_state = self.state_var_r.get()

        imp_type=None

        if selected_state == 'Joint Impedance' or selected_state == "Cartesian Impedance" :
            if selected_state == 'Joint Impedance':
                imp_type=1
            elif selected_state == "Cartesian Impedance" :
                imp_type= 2
            if messagebox.askyesno("confirm status switch", f"Are you sure you want to switch to {selected_state} mode?"):
                try:
                    robot.clear_set()
                    robot.set_state(arm=robot_id, state=3)
                    robot.set_impedance_type(arm=robot_id, type=imp_type)
                    robot.send_cmd()
                except Exception as e:
                    messagebox.showerror("Error", f"An error occurred during the state transition.: {str(e)}")
                    self.state_var.set("Position")

        #handle eef cart impedance
        elif selected_state=="Flange Cart.Imp.":
            if messagebox.askyesno("confirm status switch",
                                   f"Are you sure you want to switch to {selected_state} mode?"):

                try:
                    robot.clear_set()
                    robot.set_state(arm=robot_id, state=3)
                    robot.set_impedance_type(arm=robot_id, type=2)
                    robot.send_cmd()
                except Exception as e:
                    messagebox.showerror("Error", f"An error occurred during the state transition.: {str(e)}")
                    self.state_var.set("Position")

                eefcart_dialog = tk.Toplevel(self.root)
                eefcart_dialog.title("Flange Cartesian Impedance")
                eefcart_dialog.geometry("700x350")
                eefcart_dialog.configure(bg="white")
                eefcart_dialog.resizable(True, True)
                eefcart_dialog.transient(self.root)
                eefcart_dialog.grab_set()


                eefcart_dialog.update_idletasks()
                x = (eefcart_dialog.winfo_screenwidth() - eefcart_dialog.winfo_width()) // 2
                y = (eefcart_dialog.winfo_screenheight() - eefcart_dialog.winfo_height()) // 2
                eefcart_dialog.geometry(f"+{x}+{y}")

                adaptive_frame = tk.Frame(eefcart_dialog,bg='white')
                adaptive_frame.pack(fill="x", padx=(5,5),pady=(10,30))

                adaptive_btn = tk.Button(adaptive_frame, text="Realtime Cartesian Impedance",width=30,bg='#F8F8FF',
                                              command=lambda: self.usr_set_eefcart(2,robot_id)
                                         )
                adaptive_btn.pack(pady=(10,30))

                horizontal_line1 = tk.Frame(adaptive_frame, height=2, bg="#%02x%02x%02x" % (50, 150, 200))
                horizontal_line1.pack(fill="x", pady=(10, 10))

                #usrdfine
                usr_define_frame = tk.Frame(eefcart_dialog,bg='white')
                usr_define_frame.pack(fill="x", padx=(5,5),pady=(10,30))

                usr_text_label = tk.Label(usr_define_frame, text="Custom rotation direction")
                usr_text_label.grid(row=0, column=0, padx=(5,0))

                self.eefcart_rot_entry = tk.Entry(usr_define_frame, )
                self.eefcart_rot_entry.insert(0, "0,0,0")
                self.eefcart_rot_entry.grid(row=0, column=1, padx=(3,3), sticky="ew")
                get_crt_rot_button = tk.Button(usr_define_frame, text="Current eef rotation information",command=lambda: self.get_crt_rot(robot_id))
                get_crt_rot_button.grid(row=0, column=2, padx=(0,5))
                usr_set_button = tk.Button(usr_define_frame, text="Settings take effect",
                                             command=lambda: self.usr_set_eefcart(1,robot_id))
                usr_set_button.grid(row=0, column=3, padx=5)



        # 处理PVT模式
        elif selected_state == 'PVT':
            if messagebox.askyesno("confirm status switch",
                                   f"Are you sure you want to switch to {selected_state} mode?"):
                try:
                    robot.clear_set()
                    robot.set_state(arm=robot_id, state=2)
                    robot.send_cmd()
                except Exception as e:
                    messagebox.showerror("Error", f"An error occurred during the state transition.: {str(e)}")
                    self.state_var.set("Position")
                # 创建拖动选择弹窗
                drag_dialog = tk.Toplevel(self.root)
                drag_dialog.title("PVT")
                drag_dialog.geometry("500x200")
                drag_dialog.configure(bg="white")
                drag_dialog.transient(self.root)
                drag_dialog.resizable(True,True)
                drag_dialog.grab_set()

                drag_dialog.update_idletasks()
                x = (drag_dialog.winfo_screenwidth() - drag_dialog.winfo_width()) // 2
                y = (drag_dialog.winfo_screenheight() - drag_dialog.winfo_height()) // 2
                drag_dialog.geometry(f"+{x}+{y}")

                # 第一行：是否保存拖动数据复选框
                pvt_frame = tk.Frame(drag_dialog,bg='white')
                pvt_frame.pack(fill="x", pady=(15, 10), padx=20)

                # 2选择PVT号
                pvt_b_text_label = tk.Label(pvt_frame, text="Select PVT number:1~99", bg='white')
                pvt_b_text_label.grid(row=0, column=0, padx=5, sticky="ew")
                # 3PVT id
                self.pvt_b_entry = tk.Entry(pvt_frame, )
                self.pvt_b_entry.insert(0, "1")
                self.pvt_b_entry.grid(row=0, column=1, padx=5)
                # 4上传PVT
                send_pvt_b_button = tk.Button(pvt_frame, text="Upload PVT",
                                              command=lambda: self.send_pvt(robot_id))
                send_pvt_b_button.grid(row=0, column=2, padx=5)
                # 5运行PVT
                run_pvt_b_button = tk.Button(pvt_frame, text="Run PVT",
                                             command=lambda: self.run_pvt(robot_id))
                run_pvt_b_button.grid(row=0, column=3, padx=5)

        elif selected_state == 'Position':
            if messagebox.askyesno("confirm status switch", f"Are you sure you want to switch to {selected_state} mode?"):
                try:
                    robot.clear_set()
                    robot.set_state(arm=robot_id, state=1)
                    robot.send_cmd()
                except Exception as e:
                    messagebox.showerror("Error", f"An error occurred during the state transition.: {str(e)}")
                    self.state_var.set("Position")

        # 处理拖动模式 - 新增部分
        elif selected_state == 'Drag':
            # 创建拖动选择弹窗
            drag_dialog = tk.Toplevel(self.root)
            drag_dialog.title("Drag")
            drag_dialog.geometry("300x500")
            drag_dialog.configure(bg="white")
            drag_dialog.transient(self.root)
            drag_dialog.grab_set()

            drag_dialog.update_idletasks()
            x = (drag_dialog.winfo_screenwidth() - drag_dialog.winfo_width()) // 2
            y = (drag_dialog.winfo_screenheight() - drag_dialog.winfo_height()) // 2
            drag_dialog.geometry(f"+{x}+{y}")

            drag_var = tk.StringVar()
            drag_type_var = tk.StringVar()
            axis_var = tk.StringVar()
            save_drag_var = tk.BooleanVar(value=False)

            save_frame = tk.Frame(drag_dialog,bg='white')
            save_frame.pack(fill="x", pady=(15, 10), padx=20)

            save_checkbox = tk.Checkbutton(
                save_frame,
                    text="Save drag data",
                variable=save_drag_var,
                font=('Arial', 10),
                bg='white'
            )
            save_checkbox.pack(anchor='w')

            # 添加分隔线
            separator1 = ttk.Separator(drag_dialog, orient='horizontal')
            separator1.pack(fill="x", padx=20, pady=(0, 10))

            # 第二行：拖动类型标题
            tk.Label(drag_dialog, text="Select drag type:", bg='white',font=('Arial', 10)).pack(pady=(0, 5), anchor='w', padx=20)

            # 关节拖动选项
            joint_frame = tk.Frame(drag_dialog,bg='white')
            joint_frame.pack(anchor='w', padx=40, pady=5)

            tk.Radiobutton(
                joint_frame,
                text="Joint drag",
                variable=drag_type_var,
                value="joint_drag",
                bg='white',
                font=('Arial', 9),
                command=lambda: axis_var.set("")  # 清空轴选择
            ).pack(anchor='w')

            # 第三行：笛卡尔拖动单选按钮
            cartesian_frame = tk.Frame(drag_dialog,bg='white')
            cartesian_frame.pack(anchor='w', padx=40, pady=5)

            tk.Radiobutton(
                cartesian_frame,
                text="Cart drag",
                variable=drag_type_var,
                value="cartesian_drag",
                bg='white',
                font=('Arial', 9)
            ).pack(anchor='w')

            # 笛卡尔拖动的轴选择
            axis_frame = tk.Frame(drag_dialog,bg='white')
            axis_frame.pack(anchor='w', padx=60, pady=5)

            tk.Label(axis_frame, text="Select drag axis:",bg='white', font=('Arial', 9)).pack(anchor='w', pady=(5, 0))

            # 创建轴选择单选按钮组
            axis_options = ["X-drag", "Y-drag", "Z-drag", "R-drag"]

            for i, axis in enumerate(axis_options):
                tk.Radiobutton(
                    axis_frame,
                    text=axis,
                    variable=axis_var,
                    value=axis,
                    font=('Arial', 9),
                    bg='white',
                    state=tk.DISABLED
                ).pack(anchor='w')

            def enable_axis_selection():
                if drag_type_var.get() == "cartesian_drag":
                    for widget in axis_frame.winfo_children():
                        if isinstance(widget, tk.Radiobutton):
                            widget.config(state=tk.NORMAL)
                else:
                    for widget in axis_frame.winfo_children():
                        if isinstance(widget, tk.Radiobutton):
                            widget.config(state=tk.DISABLED)
                    axis_var.set("")

            drag_type_var.trace('w', lambda *args: enable_axis_selection())

            # 确认按钮
            def confirm_drag_selection():
                self.drag_type = drag_type_var.get()
                self.selected_axis = axis_var.get()
                save_drag_data = save_drag_var.get()  # 获取是否保存拖动数据

                if not self.drag_type:
                    messagebox.showwarning("Error", "Please select drag type")
                    return

                if self.drag_type == "cartesian_drag" and not self.selected_axis:
                    messagebox.showwarning("Error", "Please select the axis to drag using cartesian.")
                    return

                drag_mapping = {
                    "joint_drag": ("joint_drag", "joint drag"),
                    "cartesian_drag_X": ("cartesian_drag_X", "cart X drag"),
                    "cartesian_drag_Y": ("cartesian_drag_Y", "cart Y drag"),
                    "cartesian_drag_Z": ("cartesian_drag_Z", "cart Z drag"),
                    "cartesian_drag_R": ("cartesian_drag_R", "cart R drag")
                }

                if self.drag_type == "joint_drag":
                    state_key, state_name = drag_mapping["joint_drag"]
                else:
                    axis_map = {"X-drag": "X", "Y-drag": "Y", "Z-drag": "Z", "R-drag": "R"}
                    axis_key = axis_map[self.selected_axis]
                    state_key, state_name = drag_mapping[f"cartesian_drag_{axis_key}"]

                if messagebox.askyesno("confirm drag settings", f"Are you sure you want to set it to the {state_name} pattern?"):
                    try:
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
                            if self.selected_axis == "X-drag":
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
                            elif self.selected_axis == "Y-drag":
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
                            elif self.selected_axis == "Z-drag":
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
                            elif self.selected_axis == "R-drag":
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
                        messagebox.showerror("Error", f"An error occurred while setting drag mode: {str(e)}")
                else:
                    self.state_var.set("Position")
                    drag_dialog.destroy()

            # 取消按钮
            def cancel_drag_selection():
                self.state_var.set("Position")
                drag_dialog.destroy()

            # 添加分隔线
            separator2 = ttk.Separator(drag_dialog, orient='horizontal')
            separator2.pack(fill="x", padx=20, pady=(10, 5))

            # 按钮框架
            button_frame = tk.Frame(drag_dialog,bg='white')
            button_frame.pack(pady=10)

            tk.Button(
                button_frame,
                text="Confirm",
                width=10,
                command=confirm_drag_selection,
                bg="#4CAF50",
                fg="white"
            ).pack(side="left", padx=5, pady=5)

            tk.Button(
                button_frame,
                text="Cancel",
                width=10,
                command=cancel_drag_selection,
                bg="#F44336",
                fg="white"
            ).pack(side="left", padx=5, pady=5)

            self.root.wait_window(drag_dialog)
        else:
            self.state_var.set("Position")

    def get_crt_rot(self,robot_id):
        rot_text=''
        if robot_id=='A':
            rot_text=f"{self.pose_6d_l[3]:.3f}, {self.pose_6d_l[4]:.3f}, {self.pose_6d_l[5]:.3f}"
        elif robot_id=='B':
            rot_text=f"{self.pose_6d_r[3]: .3f}, {self.pose_6d_r[4]: .3f}, {self.pose_6d_r[5]: .3f}"
        self.eefcart_rot_entry.delete(0,tk.END)
        self.eefcart_rot_entry.insert(0,rot_text)

    def usr_set_eefcart(self,type,robot_id):
        if type==2:
            robot.set_EefCart_control_params(arm=robot_id,fcType=2,CartCtrlPara=[0]*7)
        if type==1:
            rot=self.eefcart_rot_entry.get()
            ret,rot_value=self.validate_point(rot,3)
            if ret:
                values = rot_value.split(',')
                rot = [float(value.strip()) for value in values]
                robot.set_EefCart_control_params(arm=robot_id, fcType=1, CartCtrlPara=[rot[0],rot[1],rot[2],0,0,0,0])
            else:
                messagebox.showerror("Error", f"The selected point format is invalid: {rot_value}")

    def send_pvt(self, robot_id):
        if self.connected:
            file_path = filedialog.askopenfilename(
                title="Select data file",
                filetypes=[("text file", "*.txt"), ("fmv file", "*.fmv"), ("all files", "*.*")]
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
            messagebox.showerror('Error', 'Please connect robot')

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
            messagebox.showerror('Error', 'Please connect robot')

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
        self.root.after(0, self._save_data_dialog, robot_id)

    def _save_data_dialog(self, robot_id):
        """在GUI主线程中执行文件保存操作"""
        file_path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt")],
            title="Save drag data"
        )

        if file_path:
            try:
                robot.save_collected_data_to_path(file_path)
                time.sleep(1)
                messagebox.showinfo("Success", f"The drag trajectory data has been saved to: {os.path.basename(file_path)}，\nPlease exit drag mode.")
            except Exception as e:
                messagebox.showerror("Error", f"Error saving file: {str(e)}")

    def create_control_components(self):
        """创建顶部控制面板"""
        self.control_frame = tk.Frame(self.root, bg="#e0e0e0", pady=5)
        self.control_frame.pack(fill="x")

        # 连接按钮
        self.connect_btn = tk.Button(
            self.control_frame,
            text="Connect Robot",
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
            text="More Features",
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
            text="Data Acquisition",
            width=15,
            command=self.data_collect_and_process_dialog,
            bg="#3BA4FD",
            fg="white",
            font=("Arial", 10, "bold"))
        self.mode_btn.pack(side="right", padx=5)

        # 末端透传
        self.mode_btn = tk.Button(
            self.control_frame,
            text="CAN/485",
            width=15,
            command=self.eef_dialog,
            bg="#3BA4FD",
            fg="white",
            font=("Arial", 10, "bold"))
        self.mode_btn.pack(side="right", padx=5)

        # 工具动力学辨识
        self.mode_btn = tk.Button(
            self.control_frame,
            text="Tool Settings",
            width=15,
            command=self.tool_identy_dialog,
            bg="#3BA4FD",
            fg="white",
            font=("Arial", 10, "bold"))
        self.mode_btn.pack(side="right", padx=5)

        # 模式切换按钮
        self.mode_btn = tk.Button(
            self.control_frame,
            text="Position",
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
        self.status_label = tk.Label(status_frame, text="disconnected", bg="#e0e0e0", font=("Arial", 9))
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

    def save_tools_config(self):
        """保存工具配置到文件"""
        try:
            if hasattr(self, 'tools_cfg'):
                with open('tools_cfg.json', 'w', encoding='utf-8') as f:
                    json.dump(self.tools_cfg, f, indent=4, ensure_ascii=False)
                print("tools cfg saved")
            else:
                print("warning:no tools cfg")
        except Exception as e:
            print(f"Failed to save tools cfg: {e}")

    def on_close(self):
        """窗口关闭事件"""
        if messagebox.askokcancel("Exit", "Are you sure you want to exit the application??"):
            '''save tools cfg'''
            self.save_tools_config()
            print(f'last save tools:{self.tools_cfg}')
            time.sleep(1)
            robot.send_file(self.tools_cfg_path, '/home/fusion/tools_cfg.json')
            time.sleep(1)
            if os.path.exists(self.tools_txt):
                os.remove(self.tools_txt)
            if self.data_subscriber:
                self.data_subscriber.stop()
            self.root.destroy()
            robot.release_robot()

    def error_get(self, robot_id):
        if self.connected:
            errors = robot.get_servo_error_code(robot_id,lang='EN')
            print(f'servo error:{errors}')
            if errors:
                messagebox.showinfo(f'{robot_id} arm error:\n', errors)
        else:
            messagebox.showerror('Error', 'Please connect robot')

    def servo_set(self,robot_id):
        if self.connected:
            try:
                axis=0
                if robot_id=='A':
                    axis=self.servo_axis_select_combobox_left.get()
                if robot_id=='B':
                    axis=self.servo_axis_select_combobox_right.get()
                robot.servo_reset(arm=robot_id,axis=axis)
            except Exception as e:
                messagebox.showerror("Error", f"Joint soft reset error: {str(e)}")
        else:
            messagebox.showerror('Error', 'Please connect robot')

    def cr_state(self, robot_id):
        if self.connected:
            robot.clear_set()
            robot.set_state(arm=robot_id, state=4)
            robot.send_cmd()
        else:
            messagebox.showerror('Error', 'Please connect robot')

    def error_clear(self, robot_id):
        if self.connected:
            robot.clear_error(robot_id)
        else:
            messagebox.showerror('Error', 'Please connect robot')

    def brake(self, robot_id):
        if self.connected:
            messagebox.showinfo('Attention', 'Please confirm that the servo parameters are set to 166 hybrid control mode.')
            if robot_id == 'A':
                robot.set_param('int', 'BRAK0', 1)
            elif robot_id == 'B':
                robot.set_param('int', 'BRAK1', 1)
        else:
            messagebox.showerror('Error', 'Please connect robot')

    def release_brake(self, robot_id):
        if self.connected:
            messagebox.showinfo('Attention', 'Please confirm that the servo parameters are set to 166 hybrid control mode.')
            if robot_id == 'A':
                robot.set_param('int', 'BRAK0', 2)
            elif robot_id == 'B':
                robot.set_param('int', 'BRAK1', 2)
        else:
            messagebox.showerror('Error', 'Please connect robot')

    def data_collect_and_process_dialog(self):
        drag_dialog = tk.Toplevel(self.root)
        drag_dialog.title("Data acquisition and processing")
        drag_dialog.geometry("1200x600")
        drag_dialog.resizable(True, True)
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
        self.read_file_button = tk.Button(self.frame_data_11, text="ID Instructions", width=15, command=preview_text_file_1,
                                          font=("Arial", 10, "bold"))
        self.read_file_button.grid(row=0, column=0, padx=5)

        self.frame_data_2 = tk.Frame(parent, bg="white")
        self.frame_data_2.pack(fill="x",pady=15)

        self.collect_btn_1 = tk.Button(self.frame_data_2, text="L Arm start collect", command=lambda: self.collect_data('A'))
        self.collect_btn_1.grid(row=0, column=0, padx=5)

        self.stop_collect_btn_1 = tk.Button(self.frame_data_2, text="Stop", command=self.stop_collect_data_both)
        self.stop_collect_btn_1.grid(row=0, column=1, padx=5)

        self.save_collect_btn_1 = tk.Button(self.frame_data_2, text="Save", command=self.save_collect_data_both)
        self.save_collect_btn_1.grid(row=0, column=2, padx=5)

        self.feature_1 = tk.Label(self.frame_data_2, text="Feat.Count.", bg='white')
        self.feature_1.grid(row=0, column=3, padx=5)

        self.features_entry_1 = tk.Entry(self.frame_data_2, width=3)
        self.features_entry_1.insert(0, '7')
        self.features_entry_1.grid(row=0, column=4, padx=5)

        self.feature_idx_1 = tk.Label(self.frame_data_2, text="Feat.IDX", bg='white')
        self.feature_idx_1.grid(row=0, column=5, padx=5)

        self.entry_var_raw_1 = tk.StringVar(
            value="[0,1,2,3,4,5,6,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]")
        self.feature_idx_entry_1 = tk.Entry(self.frame_data_2, textvariable=self.entry_var_raw_1, width=100)
        self.feature_idx_entry_1.grid(row=0, column=6, padx=5, sticky="ew")

        self.lines_1 = tk.Label(self.frame_data_2, text="Lines", bg='white')
        self.lines_1.grid(row=0, column=7, padx=5)

        self.lines_entry_1 = tk.Entry(self.frame_data_2, width=5)
        self.lines_entry_1.insert(0, '1000')
        self.lines_entry_1.grid(row=0, column=8, padx=5)


        self.frame_data_3 = tk.Frame(parent, bg="white")
        self.frame_data_3.pack(fill="x",pady=15)

        self.collect_btn_2 = tk.Button(self.frame_data_3, text="R Arm start collect", command=lambda: self.collect_data('B'))
        self.collect_btn_2.grid(row=0, column=0, padx=5)

        # 第8列：stop collect
        self.stop_collect_btn_2 = tk.Button(self.frame_data_3, text="Stop", command=self.stop_collect_data_both)
        self.stop_collect_btn_2.grid(row=0, column=1, padx=5)

        # 第3列：save collect
        self.save_collect_btn_2 = tk.Button(self.frame_data_3, text="Save", command=self.save_collect_data_both)
        self.save_collect_btn_2.grid(row=0, column=2, padx=5, pady=5)

        # 第2列：特征个数
        self.feature_2 = tk.Label(self.frame_data_3, text="Feat.Count.", bg='white')
        self.feature_2.grid(row=0, column=3, padx=5)

        # 第3列：特征个数
        self.features_entry_2 = tk.Entry(self.frame_data_3, width=3)
        self.features_entry_2.insert(0, '7')
        self.features_entry_2.grid(row=0, column=4, padx=5)

        # 第4列：特征
        self.feature_idx_2 = tk.Label(self.frame_data_3, text="Feat.IDX", bg='white')
        self.feature_idx_2.grid(row=0, column=5, padx=5)

        # 第5列：特征
        self.entry_var_raw_2 = tk.StringVar(
            value="[100,101,102,103,104,105,106,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]")
        self.feature_idx_entry_2 = tk.Entry(self.frame_data_3, textvariable=self.entry_var_raw_2, width=100)
        self.feature_idx_entry_2.grid(row=0, column=6, padx=5, sticky="ew")

        self.lines_2 = tk.Label(self.frame_data_3, text="lines", bg='white')
        self.lines_2.grid(row=0, column=7, padx=5)

        self.lines_entry_2 = tk.Entry(self.frame_data_3, width=5)
        self.lines_entry_2.insert(0, '1000')
        self.lines_entry_2.grid(row=0, column=8, padx=5)

        self.frame_data_1 = tk.Frame(parent, bg="white")
        self.frame_data_1.pack(fill="x",pady=15)

        self.collect_both_btn = tk.Button(self.frame_data_1, text="Collect Both Arms", command=self.collect_data_both)
        self.collect_both_btn.grid(row=0, column=0, padx=5)

        self.stop_collect_both_btn = tk.Button(self.frame_data_1, text="Stop", command=self.stop_collect_data_both)
        self.stop_collect_both_btn.grid(row=0, column=1, padx=5)

        self.save_collect_both_btn = tk.Button(self.frame_data_1, text="Save", command=self.save_collect_data_both)
        self.save_collect_both_btn.grid(row=0, column=2, padx=5)

        self.frame_data_4 = tk.Frame(parent, bg="white")
        self.frame_data_4.pack(fill="x",pady=15)

        self.text_50_load_file = tk.Label(self.frame_data_4, text='Downsample 50Hz', bg='#cde6c7')
        self.text_50_load_file.grid(row=0, column=0, padx=3)

        self.btn_load_file_50 = tk.Button(self.frame_data_4, text="Select file", command=self.select_50_file)
        self.btn_load_file_50.grid(row=0, column=1, padx=5)

        self.path_50 = tk.Entry(self.frame_data_4, textvariable=self.file_path_50, width=75,
                                font=("Arial", 7), state="readonly")
        self.path_50.grid(row=0, column=2, padx=5, sticky="ew")

        self.run_generate_50 = tk.Button(self.frame_data_4, text="Gen. 50 Points", command=self.generate_50_file)
        self.run_generate_50.grid(row=0, column=3, padx=5)


        self.frame_data_6 = tk.Frame(parent, bg="white")
        self.frame_data_6.pack(fill="x", pady=15)

        self.text_collect_to_pvt = tk.Label(self.frame_data_6, text='To PVT Format', bg='#6FF2E0')
        self.text_collect_to_pvt.grid(row=0, column=0, padx=3)

        self.btn_load_collect = tk.Button(self.frame_data_6, text="Select file", command=self.select_collect_file)
        self.btn_load_collect.grid(row=0, column=1, padx=5)

        self.path_collect = tk.Entry(self.frame_data_6, textvariable=self.file_path_collect, width=75,
                                font=("Arial", 7), state="readonly")
        self.path_collect.grid(row=0, column=2, padx=5, sticky="ew")

        self.run_generate_pvt = tk.Button(self.frame_data_6, text="Process & Save", command=self.generate_pvt_file)
        self.run_generate_pvt.grid(row=0, column=3, padx=5)


        self.frame_data_5 = tk.Frame(parent, bg="white")
        self.frame_data_5.pack(fill="x",pady=15)

        self.text_50_load_file = tk.Label(self.frame_data_5, text='Note: To change the number of features, you need to disconnect from the robot and reconnect before collecting data. \n(If you start position synchronization data acquisition first, you need to reconnect before collecting data from a single arm; \nif you collect data from a single arm first and then want to collect data from both arms, you also need to reconnect.)', bg='#FBFAD4')
        self.text_50_load_file.grid(row=0, column=0, padx=3)

    def select_collect_file(self):
        file_path = filedialog.askopenfilename(
            defaultextension=".txt",
            filetypes=[("txt files", "*.txt"), ("All files", "*.*")],
            title="Select data file"
        )
        if file_path:
            self.file_path_collect.set(file_path)

    def select_50_file(self):
        file_path = filedialog.askopenfilename(
            defaultextension=".txt",
            filetypes=[("txt files", "*.txt"), ("All files", "*.*")],
            title="Select the downsampled data file"
        )
        if file_path:
            self.file_path_50.set(file_path)

            if len(self.processed_data) != 0:
                self.processed_data = []

            with open(file_path, 'r') as file:
                lines = file.readlines()
            lines = lines[1:]
            for i, line in enumerate(lines):
                if i % 20 != 0:
                    continue
                parts = line.strip().split('$')
                numbers = []
                for part in parts:
                    if part:
                        num_str = part.split()[-1]
                        numbers.append(num_str)

                if len(numbers) >= 2:
                    numbers = numbers[2:]
                self.processed_data.append(numbers)
    def generate_pvt_file(self):
        result = messagebox.askokcancel("Confirm", "The data will be saved in PVT format, at the same path as the source file, \nstarting with 'proceesed_'. Confirm processing and saving?")
        if result:
            process_and_downsample(file_path= self.file_path_collect.get(),format_unify=True)

    def generate_50_file(self):
        """保存2#列表到TXT文件"""
        if len(self.processed_data) == 0:
            messagebox.showerror("Error", "The resampled data is empty; there is no content to save.")

        file_path = filedialog.asksaveasfilename(
            defaultextension=".r50pth",
            filetypes=[("50pth files", "*.r50pth"), ("All files", "*.*")],
            title="Save downsampling data"
        )

        if file_path:
            try:
                with open(file_path, 'w') as out_file:
                    for row in self.processed_data:
                        out_file.write(' '.join(row) + '\n')
            except Exception as e:
                messagebox.showerror("Error", f"Error saving file: {str(e)}")

    def data_clear_preprocess(self, input, output):
        save_list = []
        with open(input, 'r') as file:
            lines = file.readlines()
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
            messagebox.showerror('Error', 'Please connect robot')

    def stop_collect_data_both(self):
        if self.connected:
            robot.clear_set()
            robot.stop_collect_data()
            robot.send_cmd()
        else:
            messagebox.showerror('Error', 'Please connect robot')

    def save_collect_data_both(self):
        if self.connected:
            file_path = filedialog.asksaveasfilename(
                defaultextension=".txt",
                filetypes=[("Text files", "*.txt")],
                title="Save toe arms data"
            )
            if file_path:
                try:
                    robot.save_collected_data_to_path(file_path)
                except Exception as e:
                    messagebox.showerror("Error", f"Error saving file: {str(e)}")
        else:
            messagebox.showerror('Error', 'Please connect robot')

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
                messagebox.showerror("Error", f"The number of feature parameters collected cannot exceed 35！")
            if len(idx) != 35:
                messagebox.showerror("Error", f"The number of feature parameters collected must be 35, and there are currently {idx} of them!")
            if 1000000 < rows:
                rows = 1000000
                messagebox.showerror("Error", f"The maximum data collection limit is one million rows, which has been set to 1,000,000.")
            if rows < 1000:
                rows = 1000
                messagebox.showerror("Error", f"At least 1,000 rows of data must be collected; this has been set to 1,000.")
            robot.clear_set()
            robot.collect_data(targetNum=cols, targetID=idx, recordNum=rows)
            robot.send_cmd()
        else:
            messagebox.showerror('Error', 'Please connect robot')

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
            messagebox.showerror('error', 'Please connect robot')

    def set_sensor_offset(self, robot_id):  # todo
        if self.connected:
            if robot_id == 'A':
                if self.result['states'][0]["cur_state"] != 0:
                    messagebox.showerror('error', "The sensor offset can only be set in the left arm's disable state.")
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
                        messagebox.showerror("error","Save parameters fail")

            elif robot_id == 'B':
                if self.result['states'][1]["cur_state"] != 0:
                    messagebox.showerror('Error', "The sensor offset can only be set in the right arm's disable state.")
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
                        messagebox.showerror("Error", "Save parameters fail")

        else:
            messagebox.showerror('Error', 'Please connect robot')

    def clear_motor_as_zero(self, robot_id):
        if self.connected:
            result = messagebox.askokcancel("Confirm", "After the encoder is zeroed, the robot will lose its origin. Do you want to confirm the zeroing?")
            if result:
                if robot_id == 'A':
                    if self.result['states'][0]["cur_state"] != 0:
                        messagebox.showerror('Error', 'The left arm must in disable state before the motor encoder can be zeroed.')
                    else:
                        axis = int(self.motor_axis_select_combobox_1.get())
                        robot.set_param(type='int', paraName="RESETMOTENC0", value=axis)
                elif robot_id == 'B':
                    if self.result['states'][1]["cur_state"] != 0:
                        messagebox.showerror('Error', 'The right arm must in disable state before the motor encoder can be zeroed.')
                    else:
                        axis = int(self.motor_axis_select_combobox_11.get())
                        robot.set_param(type='int', paraName="RESETMOTENC1", value=axis)
            else:
                print('cancel')
        else:
            messagebox.showerror('Error', 'Please connect robot')

    def clear_motorE_as_zero(self, robot_id):
        if self.connected:
            result = messagebox.askokcancel("Confirm", "After the encoder is zeroed, the robot will lose its origin. Do you want to confirm the zeroing?")
            if result:
                if robot_id == 'A':
                    if self.result['states'][0]["cur_state"] != 0:
                        messagebox.showerror('Error', 'The left arm must be in disable state before the external encoder of the motor can be zeroed.')
                    else:
                        axis = int(self.motor_axis_select_combobox_1.get())
                        robot.set_param(type='int', paraName="RESETEXTENC0", value=axis)
                elif robot_id == 'B':
                    if self.result['states'][1]["cur_state"] != 0:
                        messagebox.showerror('Error', 'The right arm must be in disable state before the external encoder of the motor can be zeroed.')
                    else:
                        axis = int(self.motor_axis_select_combobox_11.get())
                        robot.set_param(type='int', paraName="RESETEXTENC1", value=axis)
            else:
                print('取消外编清零')
        else:
            messagebox.showerror('error', 'Please connect robot')

    def clear_motor_error(self, robot_id):
        if self.connected:
            if robot_id == 'A':
                if self.result['states'][0]["cur_state"] != 0:
                    messagebox.showerror('Error', 'The left arm must be in the disable state before the motor encoder can be cleared.')
                else:
                    axis = int(self.motor_axis_select_combobox_1.get())
                    robot.set_param(type='int', paraName="CLEARMOTENC0", value=axis)
            elif robot_id == 'B':
                if self.result['states'][1]["cur_state"] != 0:
                    messagebox.showerror('Error', 'The right arm must be in the disable state before the motor encoder can be cleared.')
                else:
                    axis = int(self.motor_axis_select_combobox_11.get())
                    robot.set_param(type='int', paraName="CLEARMOTENC1", value=axis)
        else:
            messagebox.showerror('Error', 'Please connect robot')

    def eef_dialog(self):
        drag_dialog = tk.Toplevel(self.root)
        drag_dialog.title("Eef Tool Communication")
        drag_dialog.geometry("1300x400")  
        drag_dialog.resizable(True, True)  
        drag_dialog.transient(self.root)
        drag_dialog.grab_set()

        drag_dialog.update_idletasks()
        x = (drag_dialog.winfo_screenwidth() - drag_dialog.winfo_width()) // 2
        y = (drag_dialog.winfo_screenheight() - drag_dialog.winfo_height()) // 2
        drag_dialog.geometry(f"+{x}+{y}")

        parent = tk.Frame(drag_dialog, bg="white", padx=10, pady=10)
        parent.pack(fill="both", expand=True)

        self.eef_frame_1 = tk.Frame(parent, bg="white")
        self.eef_frame_1.pack(fill="x")
        # 第1 :text
        self.eef_text_1 = tk.Button(self.eef_frame_1, text="L ARM Send", command=lambda: self.send_data_eef('A'))
        self.eef_text_1.grid(row=0, column=0, padx=5, pady=5)

        # 第2列：sensor select
        self.com_text_1 = tk.Label(self.eef_frame_1, text="Port", bg="white", width=5)
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

        self.eef_delet_1 = tk.Button(self.eef_frame_1, text="Delete Selected", command=lambda: self.delete_eef_command('A'))
        self.eef_delet_1.grid(row=0, column=3, padx=5, pady=5)

        self.eef_combo1 = ttk.Combobox(self.eef_frame_1, state="readonly", width=120)
        self.eef_combo1.grid(row=0, column=4, padx=5)

        self.eef_bt_1 = tk.Button(self.eef_frame_1, text="L ARM Receive", command=lambda: self.receive_data_eef('A'))
        self.eef_bt_1.grid(row=0, column=5, padx=5)

        self.eef_frame_1_2 = tk.Frame(parent, bg="white")
        self.eef_frame_1_2.pack(fill="x")

        self.eef1_2_b1 = tk.Label(self.eef_frame_1_2, text="", bg="white", width=7)
        self.eef1_2_b1.grid(row=0, column=0, padx=5)

        self.eef1_2_b2 = tk.Label(self.eef_frame_1_2, text="", bg="white", width=7)
        self.eef1_2_b2.grid(row=0, column=1, padx=5)

        self.eef1_2_b3 = tk.Label(self.eef_frame_1_2, text="", bg="white", width=7)
        self.eef1_2_b3.grid(row=0, column=2, padx=5)

        self.eef_add_1 = tk.Button(self.eef_frame_1_2, text='L ARM Add Cmd', command=lambda: self.add_eef_command('A'))
        self.eef_add_1.grid(row=0, column=3, padx=5)

        self.eef_entry = tk.Entry(self.eef_frame_1_2, width=120)
        self.eef_entry.insert(0, "01 06 00 00 00 01 48 0A")
        self.eef_entry.grid(row=0, column=4, padx=5, sticky="ew")

        self.eef_add_2 = tk.Button(self.eef_frame_1_2, text='R ARM Add Cmd', command=lambda: self.add_eef_command('B'))
        self.eef_add_2.grid(row=0, column=5, padx=5)

        self.eef_frame_2 = tk.Frame(parent, bg="white")
        self.eef_frame_2.pack(fill="x")
        # 第1 :text
        self.eef_bt_2 = tk.Button(self.eef_frame_2, text="R ARM Send", command=lambda: self.send_data_eef('B'))
        self.eef_bt_2.grid(row=0, column=0, padx=5)

        # 第2列：sensor select
        self.com_text_2 = tk.Label(self.eef_frame_2, text="Port", bg="white", width=5)
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

        self.eef_delet_2 = tk.Button(self.eef_frame_2, text="Delete Selected", command=lambda: self.delete_eef_command('B'))
        self.eef_delet_2.grid(row=0, column=3, padx=5, pady=5)

        self.eef_combo2 = ttk.Combobox(self.eef_frame_2, state="readonly", width=120)
        self.eef_combo2.grid(row=0, column=4, padx=5)

        self.eef_bt_4 = tk.Button(self.eef_frame_2, text="R ARM Receive", command=lambda: self.receive_data_eef('B'))
        self.eef_bt_4.grid(row=0, column=5, padx=5, pady=5)

        self.eef_frame_3 = tk.Frame(parent, bg="white")
        self.eef_frame_3.pack(fill="x")

        # 接收内容文本框
        recv_label1 = tk.Label(self.eef_frame_3, text="Left Receive:")
        recv_label1.grid(row=0, column=0, padx=5)

        # 间隔
        spacer = tk.Label(self.eef_frame_3, text="   ", bg='white')
        spacer.grid(row=0, column=1, padx=5)

        self.recv_text1 = scrolledtext.ScrolledText(self.eef_frame_3, width=70, height=8, wrap=tk.WORD)
        self.recv_text1.grid(row=1, column=0, padx=5)
        self.recv_text1.insert(tk.END,
                               'Usage Tips: First, select the port: CAN/COM1/COM2, \nthen click the "L ARM Receive" button, \nenter the data to send, \n and click the "L ARM Send" button. \nThe received end information will be refreshed and displayed at a 1kHz frequency.')

        # 间隔
        spacer1 = tk.Label(self.eef_frame_3, text="   ", bg='white')
        spacer1.grid(row=1, column=1, padx=5)

        # 接收内容文本框
        recv_label2 = tk.Label(self.eef_frame_3, text="Right Receive:")
        recv_label2.grid(row=0, column=2, padx=5)

        self.recv_text2 = scrolledtext.ScrolledText(self.eef_frame_3, width=70, height=8, wrap=tk.WORD)
        self.recv_text2.grid(row=1, column=2, padx=5)
        self.recv_text2.insert(tk.END,
                                                              'Usage Tips: First, select the port: CAN/COM1/COM2, \nthen click the "R ARM Receive" button, \nenter the data to send, \n and click the "R ARM Send" button. \nThe received end information will be refreshed and displayed at a 1kHz frequency.')
        # 添加状态显示区域
        status_display_frame_7 = tk.Frame(parent, bg="white", padx=10, pady=5)
        status_display_frame_7.pack(fill="x", pady=5)

    def add_eef_command(self,robot_id):
        """添加点到1#列表"""
        command_str = self.eef_entry.get()
        if robot_id=='A':
            # 检查是否已存在相同的点
            if self.is_duplicate_command(command_str, self.command1):
                messagebox.showwarning("Repeat instructions", "This instruction already exists in list left arm.")
                return
            else:
                self.command1.insert(0, command_str)
        elif robot_id=='B':
            # 检查是否已存在相同的点
            if self.is_duplicate_command(command_str, self.command2):
                messagebox.showwarning("Repeat instructions", "This instruction already exists in list right arm.")
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
            messagebox.showerror('error', 'Please connect robot')

    def delete_eef_command(self, robot_id):
        """从2#列表删除选中的点"""
        if robot_id == 'A':
            selected_index = self.eef_combo1.current()
            if selected_index != -1 and selected_index < len(self.command1):
                self.command1.pop(selected_index)
                self.update_combo_eef()
            else:
                messagebox.showwarning("Warning", "Please select the communication command to delete.")
        elif robot_id == 'B':
            selected_index = self.eef_combo1.current()
            if selected_index != -1 and selected_index < len(self.command2):
                self.command2.pop(selected_index)
                self.update_combo_eef()
            else:
                messagebox.showwarning("Warning", "Please select the communication command to delete.")

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
                # self.eef_thread = threading.Thread(target=read_data, args=(robot_id, com), daemon=True)
                # self.eef_thread.start()
                # received_count, received_data = get_received_data()
                # if received_count > 0:
                #     if len(received_data) > 0:
                #         print(f'received_count:{received_count},  eef received:{received_data[0]}')
                #         if robot_id == 'A':
                #             self.recv_text1.delete('1.0', tk.END)
                #             self.recv_text1.insert(tk.END, received_data[0])
                #         if robot_id == 'B':
                #             self.recv_text2.delete('1.0', tk.END)
                #             self.recv_text2.insert(tk.END, received_data[0])
                tag, receive_hex_data = robot.get_485_data(robot_id, com)
                if tag >= 1:
                    if robot_id == 'A':
                        self.recv_text1.delete('1.0', tk.END)
                        self.recv_text1.insert(tk.END, receive_hex_data)
                    if robot_id == 'B':
                        self.recv_text2.delete('1.0', tk.END)
                        self.recv_text2.insert(tk.END, receive_hex_data)
            except Exception as e:
                messagebox.showerror('error', e)
        else:
            messagebox.showerror('error', 'Please connect robot')

    def movl_dialog(self):
        pln_window = tk.Toplevel(self.root)
        pln_window.title("直线规划功能")
        pln_window.geometry("1000x400")
        pln_window.configure(bg="white")
        pln_window.transient(self.root)  # 设置为主窗口的子窗口
        pln_window.resizable(True, True)
        pln_window.grab_set()  # 模态窗口
        # 功能按钮框架

        self.pln_frame_1 = tk.Frame(pln_window, bg="white")
        self.pln_frame_1.pack(fill="x", padx=5, pady=(15, 10))
        #显示当前关节和当前xyzabc
        self.arm_text = tk.Label(self.pln_frame_1, text="关节实时数据", bg="#2196F3",
                                        fg="white", font=("Arial", 10, "bold"))
        self.arm_text.pack(fill='x')

        self.pln_frame_2 = tk.Frame(pln_window, bg="white")
        self.pln_frame_2.pack(fill="x", padx=5, pady=(10, 10))

        self.choose_arm = tk.Label(self.pln_frame_2, text="选择手臂" ,bg='white')
        self.choose_arm.grid(row=0, column=0, padx=(5, 5))

        self.arm_select_combobox= ttk.Combobox(
            self.pln_frame_2,
            values=["0", "1"],
            width=3,
            state="readonly"  # 禁止直接输入
        )
        self.arm_select_combobox.current(0)  # 默认选中第一个选项
        self.arm_select_combobox.grid(row=0, column=1, padx=(5, 5))
        # 绑定选择事件
        self.arm_select_combobox.bind("<<ComboboxSelected>>", self.update_pln_rt_data)

        self.jv_text = tk.Label(self.pln_frame_2, text="当前关节",bg='white')
        self.jv_text.grid(row=0, column=2, padx=(5, 5))

        self.crt_jv_entry = tk.Entry(self.pln_frame_2, width=50)
        self.crt_jv_entry.insert(0, '0,0,0,0,0,0,0')
        self.crt_jv_entry.grid(row=0, column=3, padx=(5, 5),sticky='ew')

        self.cart_text = tk.Label(self.pln_frame_2, text="当前末端姿态",bg='white')
        self.cart_text.grid(row=0, column=4, padx=(5, 5))

        self.crt_cart_entry = tk.Entry(self.pln_frame_2, width=50)
        self.crt_cart_entry.insert(0, '0,0,0,0,0,0')
        self.crt_cart_entry.grid(row=0, column=5, padx=(5, 5))

        #
        self.pln_frame_3 = tk.Frame(pln_window, bg="white")
        self.pln_frame_3.pack(fill="x", padx=5, pady=(15, 10))
        #显示当前关节和当前xyzabc
        self.arm_text_1 = tk.Label(self.pln_frame_3, text="末端直线", bg="#2196F3",
                                        fg="white", font=("Arial", 10, "bold"))
        self.arm_text_1.pack(fill='x')

        self.pln_frame_31 = tk.Frame(pln_window, bg="white")
        self.pln_frame_31.pack(fill="x", padx=5, pady=(10, 5))
        #点动距离，速度
        tk.Label(self.pln_frame_31 , text="规划速度：", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 0))
        self.pln_speed_entry_1 = tk.Entry(self.pln_frame_31 , width=5, font=('Arial', 9), justify='center')
        self.pln_speed_entry_1.pack(side="left")
        self.pln_speed_entry_1.insert(0, "100")
        tk.Label(self.pln_frame_31 , text="1mm/s-500mm/s", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 40))

        tk.Label(self.pln_frame_31 , text="单步距离：", font=('Arial', 9), bg='white').pack(side="left", padx=(5, 0))
        self.pln_speed_entry_2 = tk.Entry(self.pln_frame_31 , width=5, font=('Arial', 9), justify='center')
        self.pln_speed_entry_2.pack(side="left")
        self.pln_speed_entry_2.insert(0, "50")
        tk.Label(self.pln_frame_31 , text="1mm-500mm", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 40))

        tk.Label(self.pln_frame_31 , text="指定末端位置姿态：", font=('Arial', 9), bg='white').pack(side="left", padx=(5, 0))
        self.pln_speed_entry_3 = tk.Entry(self.pln_frame_31 , width=50, font=('Arial', 9), justify='center')
        self.pln_speed_entry_3.pack(side="left")
        self.pln_speed_entry_3.insert(0, "0,0,0,0,0,0")
        # tk.Label(self.pln_frame_31 , text="", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 10))

        self.pln_frame_311 = tk.Frame(pln_window, bg="white")
        self.pln_frame_311.pack(fill='x')
        tk.Label(self.pln_frame_311, text="单步", font=('Arial', 9), bg='white').grid(row=0, column=0, padx=(5, 5),pady=5)
        bt1_x_p = tk.Button(self.pln_frame_311, text="x+",command=lambda :self.pln_movla('x+'))
        bt1_x_p.grid(row=0, column=1, padx=(5, 5),pady=5)
        bt1_y_p = tk.Button(self.pln_frame_311, text="y+",command=lambda :self.pln_movla('y+'))
        bt1_y_p.grid(row=0, column=2, padx=(5, 5))
        bt1_z_p = tk.Button(self.pln_frame_311, text="z+",command=lambda :self.pln_movla('z+'))
        bt1_z_p.grid(row=0, column=3, padx=(5, 5))

        tk.Label(self.pln_frame_311, text="单步", font=('Arial', 9), bg='white').grid(row=1, column=0, padx=(5, 5),pady=5)
        bt1_x1_p = tk.Button(self.pln_frame_311, text="x+",command=lambda :self.pln_movla('x-'))
        bt1_x1_p.grid(row=1, column=1, padx=(5, 5))
        bt1_y1_p = tk.Button(self.pln_frame_311, text="y-",command=lambda :self.pln_movla('y-'))
        bt1_y1_p.grid(row=1, column=2, padx=(5, 5))
        bt1_z1_p = tk.Button(self.pln_frame_311, text="z-",command=lambda :self.pln_movla('z-'))
        bt1_z1_p.grid(row=1, column=3, padx=(5, 5))

        specific_pln = tk.Button(self.pln_frame_311, text="直线运行的到指定位置姿态",command=lambda :self.pln_movla('s'))
        specific_pln.grid(row=1, column=7, padx=(400, 5))


        self.pln_frame_4 = tk.Frame(pln_window, bg="white")
        self.pln_frame_4.pack(fill="x", padx=5, pady=(20, 10))
        #显示当前关节和当前xyzabc
        self.arm_text_2 = tk.Label(self.pln_frame_4, text="两个关节走直线", bg="#2196F3",
                                        fg="white", font=("Arial", 10, "bold"))
        self.arm_text_2.pack(fill='x')

        self.pln_frame_41 = tk.Frame(pln_window, bg="white")
        self.pln_frame_41.pack(fill="x", padx=5, pady=(10, 5))
        # 点动距离，速度
        tk.Label(self.pln_frame_41, text="规划速度：", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 0))
        self.pln_speed_entry_4 = tk.Entry(self.pln_frame_41, width=5, font=('Arial', 9), justify='center')
        self.pln_speed_entry_4.pack(side="left")
        self.pln_speed_entry_4.insert(0, "100")
        tk.Label(self.pln_frame_41, text="1mm/s-500mm/s", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 40))

        tk.Label(self.pln_frame_41, text="指定终点关节：", font=('Arial', 9), bg='white').pack(side="left",
                                                                                                  padx=(5, 0))
        self.pln_speed_entry_5 = tk.Entry(self.pln_frame_41, width=50, font=('Arial', 9), justify='center')
        self.pln_speed_entry_5.pack(side="left")
        self.pln_speed_entry_5.insert(0, "0,0,0,0,0,0,0")
        # tk.Label(self.pln_frame_41 , text="角度", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 10))

        tk.Button(self.pln_frame_41, text="直线运行的到指定关节构型", command=lambda: self.pln_movlkeepj).pack(side="left", padx=(0, 10))


    def update_pln_rt_data(self):
        joint_text=''
        cart_text=''
        arm = int(self.arm_select_combobox.get())
        robot_id='A'
        if arm == 0:
            joint_pos=self.result['outputs'][0]['fb_joint_pos']
            cart_pos=self.pose_6d_l
            result = messagebox.askokcancel('确认操作', f'手臂{arm} 将切换到笛卡尔阻抗模式')
            if result:
                '''设置阻抗参数'''
                robot.clear_set()
                robot.set_cart_kd_params(arm=robot_id, K=[12000, 12000, 12000, 600, 600, 600, 20],
                                         D=[0.4, 0.4, 0.17, 0.23, 0.23, 0.2, 0.2],
                                         type=2)
                robot.send_cmd()
                time.sleep(0.02)

                '''设置扭矩模式,笛卡尔阻抗模式,速度加速度百分比'''
                robot.clear_set()
                robot.set_state(arm=robot_id, state=3)  # state=3扭矩模式
                robot.set_impedance_type(arm=robot_id, type=2)  # type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
                robot.set_vel_acc(arm=robot_id, velRatio=100, AccRatio=100)
                robot.send_cmd()
                time.sleep(0.02)
        elif arm == 1:
            robot_id='B'
            joint_pos=self.result['outputs'][1]['fb_joint_pos']
            cart_pos = self.pose_6d_r
            result = messagebox.askokcancel('确认操作', f'手臂{arm} 将切换到笛卡尔阻抗模式')
            if result:
                '''设置阻抗参数'''
                robot.clear_set()
                robot.set_cart_kd_params(arm=robot_id, K=[12000, 12000, 12000, 600, 600, 600, 20],
                                         D=[0.4, 0.4, 0.17, 0.23, 0.23, 0.2, 0.2],
                                         type=2)
                robot.send_cmd()
                time.sleep(0.02)

                '''设置扭矩模式,笛卡尔阻抗模式,速度加速度百分比'''
                robot.clear_set()
                robot.set_state(arm=robot_id, state=3)  # state=3扭矩模式
                robot.set_impedance_type(arm=robot_id, type=2)  # type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
                robot.set_vel_acc(arm=robot_id, velRatio=100, AccRatio=100)
                robot.send_cmd()
                time.sleep(0.02)

        for i in range(7):
            joint_text += f"{joint_pos[i]:.3f}, "
        joint_text = joint_text.rstrip(", ")  # 移除最后一个逗号和空格
        for i in range(6):
            cart_text += f"{cart_pos[i]:.1f}, "
        cart_text = cart_text.rstrip(", ")  # 移除最后一个逗号和空格

        self.crt_jv_entry.delete(0, tk.END)
        self.crt_jv_entry.insert(0, joint_text)
        self.crt_cart_entry.delete(0, tk.END)
        self.crt_cart_entry.insert(0, cart_text)

    def pln_movlkeepj(self):
        end_jv=''
        speed = int(self.pln_speed_entry_4.get())
        robot_id = 'A'
        arm = int(self.arm_select_combobox.get())
        if arm == 1:
            robot_id = 'B'
        elif arm == 0:
            robot_id = 'A'
        ret, end_jv = self.validate_point(self.pln_speed_entry_5.get(), 7)
        if ret:
            values = end_jv.split(',')
            end_jv = [float(value.strip()) for value in values]

        if robot_id == 'A':
            points =kk1.movL_KeepJA(start_joints=self.result['outputs'][0]['fb_joint_pos'],
                        end_joints=end_jv,
                            vel=speed, acc=speed)
        elif robot_id == 'B':
            points =kk1.movL_KeepJA(start_joints=self.result['outputs'][1]['fb_joint_pos'],
                        end_joints=end_jv,
                            vel=speed, acc=speed)
        if not points:
            messagebox.showerror('error', f'规划失败，请检查当前构型和指定末端位置姿态是否正常')
        for idx, target_joints in enumerate(points):
            robot.clear_set()
            robot.set_joint_cmd_pose(arm=robot_id, joints=target_joints)
            robot.send_cmd()
            time.sleep(0.002)

    def pln_movla(self,type):
        '''
        获取指令类型来规划并用笛卡尔阻抗500HZ执行
        '''
        end_xyzabc=None
        speed=int(self.pln_speed_entry_1.get())
        robot_id='A'
        arm=int(self.arm_select_combobox.get())
        if arm==1:
            robot_id='B'
        elif arm==0:
            robot_id = 'A'

        if type=='s':
            ret, xyzabc = self.validate_point(self.pln_speed_entry_3.get(), 6)
            if ret:
                values = xyzabc.split(',')
                end_xyzabc = [float(value.strip()) for value in values]

        else:
            if robot_id=='A':
                end_xyzabc = self.pose_6d_l.copy()
            elif robot_id=='B':
                end_xyzabc = self.pose_6d_r.copy()
            if type=='x+':
                distance = int(self.pln_speed_entry_2.get())
                end_xyzabc[0] += distance
            elif type == 'x-':
                distance = int(self.pln_speed_entry_2.get())
                end_xyzabc[0] -= distance
            elif type == 'y+':
                distance = int(self.pln_speed_entry_2.get())
                end_xyzabc[1] += distance
            elif type == 'y-':
                distance = int(self.pln_speed_entry_2.get())
                end_xyzabc[1] -= distance
            elif type == 'z+':
                distance = int(self.pln_speed_entry_2.get())
                end_xyzabc[2] += distance
            elif type == 'z-':
                distance = int(self.pln_speed_entry_2.get())
                end_xyzabc[2] -= distance


        if robot_id == 'A':
            points = kk1.movLA(
                start_xyzabc=self.pose_6d_l,
                end_xyzabc=end_xyzabc,
                ref_joints=self.result['outputs'][0]['fb_joint_pos'],
                vel=speed,
                acc=speed
            )

        elif robot_id == 'B':
            points = kk2.movLA(
                start_xyzabc=self.pose_6d_r,
                end_xyzabc=end_xyzabc,
                ref_joints=self.result['outputs'][1]['fb_joint_pos'],
                vel=speed,
                acc=speed
            )
        if not points:
            messagebox.showerror('error', f'规划失败，请检查当前构型和指定末端位置姿态是否正常')
        for idx, target_joints in enumerate(points):
            robot.clear_set()
            robot.set_joint_cmd_pose(arm=robot_id, joints=target_joints)
            robot.send_cmd()
            time.sleep(0.002)


    def tool_identy_dialog(self):
        tools_dialog = tk.Toplevel(self.root)
        tools_dialog.title("Eef tool function")
        tools_dialog.geometry("1000x800")
        tools_dialog.resizable(True, True)
        tools_dialog.transient(self.root)
        tools_dialog.grab_set()

        # 设置对话框居中显示
        tools_dialog.update_idletasks()
        x = (tools_dialog.winfo_screenwidth() - tools_dialog.winfo_width()) // 2
        y = (tools_dialog.winfo_screenheight() - tools_dialog.winfo_height()) // 2
        tools_dialog.geometry(f"+{x}+{y}")

        # 创建主框架，使用pack布局
        main_frame = tk.Frame(tools_dialog, bg="white", padx=10, pady=10)
        main_frame.pack(fill="both", expand=True)

        title_identy_tool_frame = tk.Frame(main_frame, bg="white")
        title_identy_tool_frame.pack(fill="x", pady=(0, 10))
        title_label = tk.Label(title_identy_tool_frame, text="Tool dynamics identification", bg='white',font=("Arial", 14, "italic"))
        title_label.pack(fill='x',pady=(5,5))

        # ============ 第一行：标题和文件选择 ============
        identy_tool_frame = tk.Frame(main_frame, bg="white")
        identy_tool_frame.pack(fill="x", pady=(0, 10))
        # 机型选择标签
        robot_type_label = tk.Label(identy_tool_frame, text="Select model", bg='white', width=10)
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
        self.tool_trajectory_file = tk.Button(identy_tool_frame, text="Select track file",
                                         command=self.tool_trajectory)
        self.tool_trajectory_file.grid(row=0, column=3, padx=5)

        # 文件路径显示
        self.file_path_tool = tk.StringVar()
        self.path_tool = tk.Entry(identy_tool_frame, textvariable=self.file_path_tool, width=55,
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
        collect_tool_btn = tk.Button(identy_tool_frame2, text="L ARM no-load data acquisition",
                                     command=lambda: self.thread_collect_tool_data_no_load('A'))
        collect_tool_btn.grid(row=0, column=1, padx=5)

        collect_tool_btn2 = tk.Button(identy_tool_frame2, text="L ARM load data acquisition",
                                      command=lambda: self.thread_collect_tool_data_with_load('A'))
        collect_tool_btn2.grid(row=0, column=2, padx=5)

        # 中间空白
        tool_blank1 = tk.Label(identy_tool_frame2, text=" ", width=5, bg="white")
        tool_blank1.grid(row=0, column=3, padx=5)

        # 工具辨识按钮
        tool_dyn_identy_btn = tk.Button(identy_tool_frame2, text="ToolDyn", bg='#afb4db',
                                        command=self.tool_dyn_identy)
        tool_dyn_identy_btn.grid(row=0, column=4, padx=5)

        # 右侧空白
        tool_blank3 = tk.Label(identy_tool_frame2, text=" ", width=5, bg="white")
        tool_blank3.grid(row=0, column=5, padx=5)

        # 右臂按钮
        collect_tool_btn1 = tk.Button(identy_tool_frame2, text="R ARM no-load data acquisition",
                                      command=lambda: self.thread_collect_tool_data_no_load('B'))
        collect_tool_btn1.grid(row=0, column=6, padx=5)

        collect_tool_btn22 = tk.Button(identy_tool_frame2, text="R ARM load data acquisition",
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
                                      text="Tool Dyn Params[m,mx,my,mz,ixx,ixy,ixz,iyy,iyz,izz]",
                                      bg='white', width=40)
        robot_type_choose1.grid(row=0, column=1, padx=5, pady=5)

        # 参数输入框
        self.entry_tool_dyn  = tk.StringVar(value="0,0,0,0,0,0,0,0,0,0")
        tool_dyn_entry = tk.Entry(identy_tool_frame1, textvariable=self.entry_tool_dyn , width=60)
        tool_dyn_entry.grid(row=0, column=2, padx=5, sticky="ew")


        identy_tool_frame4 = tk.Frame(main_frame, bg="white")
        identy_tool_frame4.pack(fill="x", pady=(0, 10))

        # 机型选择标签
        which_tool_label = tk.Label(identy_tool_frame4, text="Select tool", bg='white')
        which_tool_label.grid(row=0, column=0, padx=5)

        # 机型选择下拉框
        self.tool_select_combobox_1 = ttk.Combobox(
            identy_tool_frame4,
            values=["tool-1", "tool-2","tool-3", "tool-4","tool-5"],
            width=10,
            state="readonly"
        )
        self.tool_select_combobox_1.current(0)
        self.tool_select_combobox_1.grid(row=0, column=1, padx=5)
        # 绑定选择事件
        self.tool_select_combobox_1.bind("<<ComboboxSelected>>",
                                         lambda e: self.update_tool_display('A','up_arm0'))

        close_button = tk.Button(identy_tool_frame4, text="Importing dynamic parameters into the left arm tool",bg='#E2F6FF',command=lambda :self.load_tool_dyn_to_tool_api('A','up_arm0'))
        close_button.grid(row=0, column=2, padx=(5,20))


        # 机型选择标签
        which_tool_label = tk.Label(identy_tool_frame4, text="Select tool", bg='white')
        which_tool_label.grid(row=0, column=3, padx=5)

        # 机型选择下拉框
        self.tool_select_combobox_11 = ttk.Combobox(
            identy_tool_frame4,
            values=["tool-1", "tool-2","tool-3", "tool-4","tool-5"],
            width=10,
            state="readonly"
        )
        self.tool_select_combobox_11.current(0)
        self.tool_select_combobox_11.grid(row=0, column=4, padx=5)
        # 绑定选择事件
        self.tool_select_combobox_11.bind("<<ComboboxSelected>>",
                                         lambda e: self.update_tool_display('B','up_arm1'))

        save_button = tk.Button(identy_tool_frame4, text="Importing dynamic parameters into the right arm tool",bg = '#F6DFF6', command=lambda :self.load_tool_dyn_to_tool_api('B','up_arm1'))
        save_button.grid(row=0, column=5, padx=5)


        # 添加横线
        horizontal_line1 = tk.Frame(main_frame, height=2, bg="#%02x%02x%02x" % (50, 150, 200))
        horizontal_line1.pack(fill="x", pady=(10, 10))


        title_tool_set_text_frame = tk.Frame(main_frame, bg="white")
        title_tool_set_text_frame.pack(fill="x", pady=(0, 10))

        title_tool_label = tk.Label(title_tool_set_text_frame, text="Tool parameter settings", bg='white',font=("Arial", 14, "italic"))
        title_tool_label.pack(fill='x',pady=(5,5))

        #right
        tool_set_frame = tk.Frame(main_frame, bg="white")
        tool_set_frame.pack(fill="x", pady=(0, 10))

        # 确保变量已初始化
        if not hasattr(self, 'tool_a_entry'):
            self.init_tool_variables()

        tool_a_label_1 = tk.Label(tool_set_frame, text="Tool Dyn Params(M~I_zz)", width=20, bg='white')
        tool_a_label_1.grid(row=0, column=0, padx=5, pady=5)

        # 改为下拉框 - 工具动力学参数 (arm0)
        self.tool_a_combobox = ttk.Combobox(
            tool_set_frame,
            textvariable=self.tool_a_entry,
            values=self.arm0_dyn_presets,
            width=70,
            state="readonly"
        )
        self.tool_a_combobox.grid(row=0, column=1, padx=5, sticky="ew")

        # 1设置工具运动学参数
        tool_a_label_2 = tk.Label(tool_set_frame, text="Tool Kine Params", width=20, bg='white')
        tool_a_label_2.grid(row=1, column=0, pady=5)

        # 改为下拉框 - 工具运动学参数 (arm0)
        self.tool_a1_combobox = ttk.Combobox(
            tool_set_frame,
            textvariable=self.tool_a1_entry,
            values=self.arm0_kine_presets,
            width=70,
            state="normal"
        )
        self.tool_a1_combobox.grid(row=1, column=1, padx=5, sticky="ew")

        # 1设置工具参数
        tool_set_frame1 = tk.Frame(main_frame, bg="white")
        tool_set_frame1.pack(fill="x", pady=(0, 10))
        which_tool_label = tk.Label(tool_set_frame1, text="Select tool", bg='white')
        which_tool_label.grid(row=0, column=0, padx=5,pady=(5,25))

        # 机型选择下拉框
        self.tool_select_combobox_2 = ttk.Combobox(
            tool_set_frame1,
            values=["tool-1", "tool-2", "tool-3", "tool-4", "tool-5"],
            width=10,
            state="readonly"
        )
        self.tool_select_combobox_2.current(0)
        self.tool_select_combobox_2.grid(row=0, column=1, padx=5,pady=(5,25),sticky="ew")
        # 绑定选择事件
        self.tool_select_combobox_2.bind("<<ComboboxSelected>>",
                                         lambda e: self.update_tool_display('A','down_arm0'))

        tool_a_button = tk.Button(tool_set_frame1, text="Set left arm tool parameters", width=30, bg='#7CCEF0',
                                  command=lambda: self.tool_set('A'))
        tool_a_button.grid(row=0, column=2, padx=5,pady=(5,25))


        #right
        tool_set_frame2 = tk.Frame(main_frame, bg="white")
        tool_set_frame2.pack(fill="x", pady=(0, 10))

        tool_b_label_1 = tk.Label(tool_set_frame2, text="Tool Dyn Params(M~I_zz)", width=25, bg='white')
        tool_b_label_1.grid(row=0, column=0, padx=5)

        # 改为下拉框 - 工具动力学参数 (arm1)
        self.tool_b_combobox = ttk.Combobox(
            tool_set_frame2,
            textvariable=self.tool_b_entry,
            values=self.arm1_dyn_presets,
            width=70,
            state="readonly"
        )
        self.tool_b_combobox.grid(row=0, column=1, padx=5, sticky="ew")

        # 1设置工具运动学参数
        tool_b_label_2 = tk.Label(tool_set_frame2, text="Tool Kine Params", width=25, bg='white')
        tool_b_label_2.grid(row=1, column=0, pady=5)

        # 改为下拉框 - 工具运动学参数 (arm1)
        self.tool_b1_combobox = ttk.Combobox(
            tool_set_frame2,
            textvariable=self.tool_b1_entry,
            values=self.arm1_kine_presets,
            width=70,
            state="normal"
        )
        self.tool_b1_combobox.grid(row=1, column=1, padx=5)

        # 1设置工具参数
        tool_set_frame3 = tk.Frame(main_frame, bg="white")
        tool_set_frame3.pack(fill="x", pady=(0, 10))
        which_tool_label1 = tk.Label(tool_set_frame3, text="Select tool", bg='white')
        which_tool_label1.grid(row=0, column=0, padx=5, pady=(5, 25))

        # 机型选择下拉框
        self.tool_select_combobox_3 = ttk.Combobox(
            tool_set_frame3,
            values=["tool-1", "tool-2", "tool-3", "tool-4", "tool-5"],
            width=10,
            state="readonly"
        )
        self.tool_select_combobox_3.current(0)
        # 绑定选择事件
        self.tool_select_combobox_3.bind("<<ComboboxSelected>>",
                                         lambda e: self.update_tool_display('B','down_arm1'))

        self.tool_select_combobox_3.grid(row=0, column=1, padx=5, pady=(5, 25), sticky="ew")


        tool_b_button = tk.Button(tool_set_frame3, text="Set right arm tool parameters", width=30, bg='#7CCEF0',
                                  command=lambda: self.tool_set('B'))
        tool_b_button.grid(row=0, column=2, padx=5, pady=(5,25))

        tool_set_frame33 = tk.Frame(main_frame, bg="white")
        tool_set_frame33.pack(fill="x", pady=(0, 10))
        tool_reset_button = tk.Button(tool_set_frame33, text="All tool information cleared", width=30, bg='#FFFF00',
                                  command=self.tool_reset)
        tool_reset_button.grid(row=0, column=0, padx=5, pady=(5,15))

    def tool_reset(self):
        result = messagebox.askokcancel('Confirm','Should I clear all tool information with one click？')
        if result:
            self.tools_cfg= {
                    "arm0": {
                        "tool-1": {"dyn": [0,0,0,0,0,0,0,0,0,0], "kine": [0,0,0,0,0,0]},
                        "tool-2": {"dyn": [0,0,0,0,0,0,0,0,0,0], "kine": [0,0,0,0,0,0]},
                        "tool-3": {"dyn": [0,0,0,0,0,0,0,0,0,0], "kine": [0,0,0,0,0,0]},
                        "tool-4": {"dyn": [0,0,0,0,0,0,0,0,0,0], "kine": [0,0,0,0,0,0]},
                        "tool-5": {"dyn": [0,0,0,0,0,0,0,0,0,0], "kine": [0,0,0,0,0,0]}
                    },
                    "arm1": {
                        "tool-1": {"dyn": [0,0,0,0,0,0,0,0,0,0], "kine": [0,0,0,0,0,0]},
                        "tool-2": {"dyn": [0,0,0,0,0,0,0,0,0,0], "kine": [0,0,0,0,0,0]},
                        "tool-3": {"dyn": [0,0,0,0,0,0,0,0,0,0], "kine": [0,0,0,0,0,0]},
                        "tool-4": {"dyn": [0,0,0,0,0,0,0,0,0,0], "kine": [0,0,0,0,0,0]},
                        "tool-5": {"dyn": [0,0,0,0,0,0,0,0,0,0], "kine": [0,0,0,0,0,0]}
                    },
                    "current_tool": {"arm0": None, "arm1": None}  # 保持为None/null
                }
            self.save_tools_config()
            self.update_tool_display('A','down_arm0')
            self.update_tool_display('B', 'down_arm1')

    def tool_set(self, robot_id):
        if self.connected:
            """设置工具参数"""
            if robot_id == 'A':
                selected_tool = self.tool_select_combobox_2.get() if hasattr(self,'tool_select_combobox_2') else "tool-1"

                # 获取当前显示的参数
                dyn_str = self.tool_a_entry.get()
                kine_str = self.tool_a1_entry.get()

                # 将字符串转换为列表
                dyn_list = [float(x) for x in dyn_str.split(',')]
                kine_list = [float(x) for x in kine_str.split(',')]

                # 更新配置文件
                self.tools_cfg["arm0"][selected_tool]["dyn"] = dyn_list
                self.tools_cfg["arm0"][selected_tool]["kine"] = kine_list
                self.tools_cfg["current_tool"]["arm0"]=selected_tool
                self.save_tools_config()
                print(f"set left arm tool: {selected_tool}: dyn={dyn_list}, kine={kine_list}")

            elif robot_id == 'B':
                selected_tool = self.tool_select_combobox_3.get() if hasattr(self, 'tool_select_combobox_3') else "tool-1"
                # 获取当前显示的参数
                dyn_str = self.tool_b_entry.get()
                kine_str = self.tool_b1_entry.get()
                # 将字符串转换为列表
                dyn_list = [float(x) for x in dyn_str.split(',')]
                kine_list = [float(x) for x in kine_str.split(',')]
                # 更新配置文件
                self.tools_cfg["arm1"][selected_tool]["dyn"] = dyn_list
                self.tools_cfg["arm1"][selected_tool]["kine"] = kine_list
                self.tools_cfg["current_tool"]["arm1"] = selected_tool
                self.save_tools_config()

                print(f"set right arm tool: {selected_tool}: dyn={dyn_list}, kine={kine_list}")

            robot.clear_set()
            robot.set_tool(arm=robot_id, kineParams=kine_list, dynamicParams=dyn_list)
            robot.send_cmd()

            tool_mat = kk1.xyzabc_to_mat4x4(xyzabc=kine_list)
            if robot_id == "A":
                kk1.set_tool_kine(tool_mat=tool_mat)
            elif robot_id == "B":
                kk2.set_tool_kine(tool_mat=tool_mat)

        else:
            messagebox.showerror('Error', 'Please connect robot')

    def tool_trajectory(self):
        file_path = filedialog.askopenfilename(
            defaultextension=".fmv",
            filetypes=[("fmv files", "*.fmv"), ("All files", "*.*")],
            title="Select the excitation trajectory file identified by the tool."
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
                title="Select the folder where the identification data is saved.",
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
                messagebox.showinfo('success', f'Successfully saved the {robot_id} arm no-load identification data')
        else:
            messagebox.showerror('error', 'Please connect robot')

    def thread_collect_tool_data_with_load(self, robot_id):
        """在新线程中执行collect_tool_data_with_load"""
        thread = threading.Thread(target=self.collect_tool_data_with_load, args=(robot_id))
        thread.daemon = True
        thread.start()

    def collect_tool_data_with_load(self, robot_id):
        if self.connected:
            folder_path = filedialog.askdirectory(
                title="Select the folder where the identification data is saved.",
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
                messagebox.showinfo('success', f'Successfully saved {robot_id} arm-mounted identification data')
        else:
            messagebox.showerror('error', 'Please connect robot')

    def load_tool_dyn_to_tool_api(self, robot_id, which_combox):
        try:
            # 获取工具动力学参数值
            if hasattr(self, 'entry_tool_dyn'):
                tool_dyn_value = self.entry_tool_dyn.get()
                print(f'******{tool_dyn_value}')

                # 验证格式 - 先检查 validate_point 的返回类型
                validation_result = self.validate_point(tool_dyn_value, 10)
                print(f'******{validation_result}')

                # 判断 validate_point 的返回类型
                if isinstance(validation_result, tuple) and len(validation_result) == 2:
                    # 如果返回的是元组 (success, result)
                    success, result = validation_result
                else:
                    # 如果返回的是布尔值或其他类型
                    success = validation_result
                    result = tool_dyn_value if success else "参数验证失败"
                if success:
                    # 根据传入的参数确定要更新哪个工具
                    selected_tool = None

                    if robot_id == 'A':
                        if which_combox == 'up_arm0':
                            selected_tool = self.tool_select_combobox_1.get()
                            # 将字符串转换为列表
                            dyn_list = [float(x) for x in result.split(',')]
                            print(f'*****{dyn_list}')
                            self.tools_cfg["arm0"][selected_tool]["dyn"] = dyn_list
                            self.save_tools_config()
                            # 更新界面显示
                            self.tool_a_entry.set(result)
                            if hasattr(self, 'tool_a_combobox'):
                                self.tool_a_combobox.set(result)

                    elif robot_id == 'B':
                        if which_combox == 'up_arm1':
                            selected_tool = self.tool_select_combobox_11.get()
                            # 将字符串转换为列表
                            dyn_list = [float(x) for x in result.split(',')]
                            self.tools_cfg["arm1"][selected_tool]["dyn"] = dyn_list
                            self.save_tools_config()
                            # 更新界面显示
                            self.tool_b_entry.set(result)
                            if hasattr(self, 'tool_b_combobox'):
                                self.tool_b_combobox.set(result)

                    print(f"成功导入工具{robot_id}参数到{selected_tool}: {result}")
                    messagebox.showinfo("Success", f"Tool dynamics parameters have been imported to {robot_id},tool:{selected_tool}")
                else:
                    print(f"参数格式错误: {result}")
                    messagebox.showerror("Error", f"Parameter format error: {result}")
            else:
                print("找不到工具动力学参数输入框")
                messagebox.showerror("Error", "Tool dynamics parameter input box not found")
        except Exception as e:
            print(f"导入失败: {e}")
            messagebox.showerror("Error", f"Import failed: {str(e)}")

    def tool_dyn_identy(self):
        print(f"ccs srs:{self.type_select_combobox_1.get()}")
        print(f"tool data:{self.save_tool_data_path}")
        if self.type_select_combobox_1.get() == 'CCS':
            identy_results = kk1.identify_tool_dyn(robot_type=1, ipath=self.save_tool_data_path)
            if type(identy_results) == str:
                print('error:', identy_results)
                messagebox.showerror('wrong', f'Tool dynamics parameter identification error message:{identy_results}')
            else:
                print(f' identy_results:{identy_results}')
                tool_dyn_text = ""
                for i in range(10):
                    tool_dyn_text += f"{identy_results[i]:.3f}, "
                tool_dyn_text = tool_dyn_text.rstrip(", ")
                self.entry_tool_dyn.set(tool_dyn_text)
                messagebox.showinfo('success', 'Tool dynamics parameter identification completed')

        else:
            identy_results = kk1.identify_tool_dyn(robot_type=2, ipath=self.save_tool_data_path)
            if type(identy_results) == str:
                print('error:', identy_results)
                messagebox.showerror('wrong', f'Tool dynamics parameter identification error message:{identy_results}')
            else:
                print(f' identy_results:{identy_results}')
                tool_dyn_text = ""
                for i in range(10):
                    tool_dyn_text += f"{identy_results[i]:.3f}, "
                tool_dyn_text = tool_dyn_text.rstrip(", ")  # 移除最后一个逗号和空格
                self.entry_tool_dyn.set(tool_dyn_text)
                messagebox.showinfo('success', 'Tool dynamics parameter identification completed')

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
                self.status_label.config(text="⚠️ Emergency stop has been triggered", foreground='red')
                return True  # 返回成功
            except Exception as e:
                print(f"急停操作失败: {e}")
                messagebox.showerror('Error', f'Emergency stop operation failed: {e}')
                return False  # 返回失败
        else:
            messagebox.showerror('Error', 'Please connect robot')
            return False  # 返回失败

    def toggle_connection(self):
        global_robot_ip = self.arm_ip_entry.get()
        if global_robot_ip:
            init = robot.connect(global_robot_ip)
            print(f'\nrobot connect ({global_robot_ip}), return:{init}')
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
                self.connect_btn.config(text="Disconnect", bg="#F44336")
                self.status_label.config(text="Connected")
                self.status_light.config(fg="green")
                self.mode_btn.config(state="normal")
                # get controller version
                ret, version = robot.get_param('int', 'VERSION')
                self.version = version

                '''启动读485数据'''
                self.data_subscriber = DataSubscriber(self.update_data)

                '''tool '''
                robot.receive_file(self.tools_cfg_path, '/home/fusion/tools_cfg.json')
                time.sleep(0.5)
                self.tools_cfg=load_or_create_tools_config(self.tools_cfg_path)
                last_arm0_tool = None
                last_arm1_tool = None
                if self.tools_cfg:
                    if self.tools_cfg["current_tool"].get("arm0") is None:
                        last_arm0_tool=None
                    elif self.tools_cfg["current_tool"].get("arm1") is None:
                        last_arm1_tool=None
                    else:
                        last_arm0_tool=self.tools_cfg["current_tool"]["arm0"]
                        last_arm1_tool=self.tools_cfg["current_tool"]["arm1"]
                    print(f'last arm0 tool:{last_arm0_tool},last arm1 tool:{last_arm1_tool}')

                    if last_arm0_tool!=None and last_arm1_tool!=None:
                        messagebox.showinfo('Success', f'Robot connection successful. \nRobot history tool information is as follows: \nleft：{last_arm0_tool}，right：{last_arm1_tool}.\n 如需修改请重新设置工具参数')
                        # 设置历史数据
                        robot.clear_set()
                        robot.set_tool(arm='A', dynamicParams=self.tools_cfg["arm0"][last_arm0_tool]['dyn'], kineParams=self.tools_cfg["arm0"][last_arm0_tool]['kine'])
                        robot.set_tool(arm='B',  dynamicParams=self.tools_cfg["arm1"][last_arm1_tool]['dyn'], kineParams=self.tools_cfg["arm1"][last_arm1_tool]['kine'])
                        robot.send_cmd()
                        tool_mat = kk1.xyzabc_to_mat4x4(self.tools_cfg["arm0"][last_arm0_tool]['kine'])
                        tool_mat1 = kk2.xyzabc_to_mat4x4(self.tools_cfg["arm1"][last_arm1_tool]['kine'])
                        kk1.set_tool_kine(tool_mat=tool_mat)
                        kk2.set_tool_kine(tool_mat=tool_mat1)

                    elif last_arm0_tool!=None and last_arm1_tool==None:
                        messagebox.showinfo('Success',
                                            f'Robot connection successful. \nLeft arm tool:{last_arm0_tool}，Right arm not set.')
                        # 设置历史数据
                        robot.clear_set()
                        robot.set_tool(arm='A', dynamicParams=self.tools_cfg["arm0"][last_arm0_tool]['dyn'], kineParams=self.tools_cfg["arm0"][last_arm0_tool]['kine'])
                        robot.send_cmd()
                        tool_mat = kk1.xyzabc_to_mat4x4(self.tools_cfg["arm0"][last_arm0_tool]['kine'])
                        kk1.set_tool_kine(tool_mat=tool_mat)

                    elif last_arm0_tool==None and last_arm1_tool!=None:
                        messagebox.showinfo('Success', f'Robot connection successful. \nRight arm tool:{last_arm1_tool}，Left arm not set.')
                        # 设置历史数据
                        robot.clear_set()
                        robot.set_tool(arm='B',  dynamicParams=self.tools_cfg["arm1"][last_arm1_tool]['dyn'], kineParams=self.tools_cfg["arm1"][last_arm1_tool]['kine'])
                        robot.send_cmd()
                        tool_mat1 = kk2.xyzabc_to_mat4x4(self.tools_cfg["arm1"][last_arm1_tool]['kine'])
                        kk2.set_tool_kine(tool_mat=tool_mat1)

                    elif last_arm0_tool==None and last_arm1_tool==None:
                        messagebox.showinfo('Success',f'Robot connection successful. \nTool information not set for robot.')

            if motion_tag == 0:
                messagebox.showerror('Failed!', "Robot connection failed, please reconnect.")
                self.status_label.config(text="Disconnected")
                self.status_light.config(fg="red")
                self.mode_btn.config(state="disabled")

        else:
            self.connect_btn.config(text="Connect Robot", bg="#4CAF50")
            self.status_label.config(text="Disconnected")
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
        if str(arm_error) in arm_err_code_EN:
            type = arm_err_code[str(arm_error)]
            self.left_arm_error.config(text=f"arm error {arm_error} is:"+f"{type}")
        else:
            self.left_arm_error.config(text="")
        type1 = ''
        self.right_state_main.config(text=f"{self.result['states'][1]['cur_state']}")
        self.right_state_1.config(text=f"{self.result['outputs'][1]['tip_di'][0]}")
        self.right_state_2.config(text=f"{self.result['outputs'][1]['low_speed_flag'][0]}")
        self.right_state_3.config(text=f"{self.result['states'][1]['err_code']}")
        arm_error1 = self.result['states'][1]['err_code']
        if str(arm_error1) in arm_err_code_EN:
            type1 = arm_err_code[str(arm_error1)]
            self.right_arm_error.config(text=f"arm error {arm_error} is"+f"{type1}")
        else:
            self.right_arm_error.config(text="")

    def update_6d(self):
        """更新机械臂数据"""
        key = self.data_keys[self.display_mode]

        '''left'''
        joint_pos_l = self.result['outputs'][0][key]
        '''xyzabc'''
        fk_mat_l = kk1.fk(joints=joint_pos_l)
        self.pose_6d_l = kk1.mat4x4_to_xyzabc(pose_mat=fk_mat_l)
        cartesian_text_l = f"{self.pose_6d_l[0]:.3f},{self.pose_6d_l[1]:.3f},{self.pose_6d_l[2]:.3f},{self.pose_6d_l[3]:.3f}, {self.pose_6d_l[4]:.3f}, {self.pose_6d_l[5]:.3f}"
        joint_text_l = ""
        for i in range(7):
            joint_text_l += f"{joint_pos_l[i]:.3f}, "
        joint_text_l = joint_text_l.rstrip(", ")
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
        self.pose_6d_r = kk1.mat4x4_to_xyzabc(pose_mat=fk_mat_r)

        cartesian_text_r = f"{self.pose_6d_r[0]:.3f},{self.pose_6d_r[1]:.3f},{self.pose_6d_r[2]:.3f},{self.pose_6d_r[3]:.3f}, {self.pose_6d_r[4]:.3f}, {self.pose_6d_r[5]:.3f}"
        joint_text_r = ""
        for i in range(7):
            joint_text_r += f"{joint_pos_r[i]:.3f}, "
        joint_text_r = joint_text_r.rstrip(", ")
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


def load_or_create_tools_config(file_path):
    tools_cfg= {
        "arm0": {
            "tool-1": {"dyn": [0,0,0,0,0,0,0,0,0,0], "kine": [0,0,0,0,0,0]},
            "tool-2": {"dyn": [0,0,0,0,0,0,0,0,0,0], "kine": [0,0,0,0,0,0]},
            "tool-3": {"dyn": [0,0,0,0,0,0,0,0,0,0], "kine": [0,0,0,0,0,0]},
            "tool-4": {"dyn": [0,0,0,0,0,0,0,0,0,0], "kine": [0,0,0,0,0,0]},
            "tool-5": {"dyn": [0,0,0,0,0,0,0,0,0,0], "kine": [0,0,0,0,0,0]}
        },
        "arm1": {
            "tool-1": {"dyn": [0,0,0,0,0,0,0,0,0,0], "kine": [0,0,0,0,0,0]},
            "tool-2": {"dyn": [0,0,0,0,0,0,0,0,0,0], "kine": [0,0,0,0,0,0]},
            "tool-3": {"dyn": [0,0,0,0,0,0,0,0,0,0], "kine": [0,0,0,0,0,0]},
            "tool-4": {"dyn": [0,0,0,0,0,0,0,0,0,0], "kine": [0,0,0,0,0,0]},
            "tool-5": {"dyn": [0,0,0,0,0,0,0,0,0,0], "kine": [0,0,0,0,0,0]}
        },
        "current_tool": {"arm0": None, "arm1": None}  # 保持为None/null
    }
    try:
        # 检查文件是否存在
        if not os.path.exists(file_path):
            # 创建目录（如果不存在）
            directory = os.path.dirname(file_path)
            if directory and not os.path.exists(directory):
                os.makedirs(directory, exist_ok=True)

            # 写入默认配置
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(tools_cfg, f, indent=4, ensure_ascii=False)
            print(f"文件不存在，已创建默认配置文件: {file_path}")
            return False

        # 检查文件是否为空
        if os.path.getsize(file_path) == 0:
            # 文件存在但为空，写入默认配置
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(tools_cfg, f, indent=4, ensure_ascii=False)
            print(f"文件为空，已写入默认配置: {file_path}")
            return False

        # 尝试读取文件内容
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read().strip()
            if not content:  # 文件内容全是空白字符
                # 写入默认配置
                with open(file_path, 'w', encoding='utf-8') as f:
                    json.dump(tools_cfg, f, indent=4, ensure_ascii=False)
                print(f"文件内容为空，已写入默认配置: {file_path}")
                return False

            # 解析JSON
            config = json.loads(content)

            # 检查必需的结构
            required_keys = ["arm0", "arm1", "current_tool"]
            for key in required_keys:
                if key not in config:
                    print(f"配置文件缺少必需键 '{key}'，使用默认配置替换")
                    with open(file_path, 'w', encoding='utf-8') as f:
                        json.dump(tools_cfg, f, indent=4, ensure_ascii=False)
                    return False

            # 检查tool结构
            tool_names = ["tool-1", "tool-2", "tool-3", "tool-4", "tool-5"]
            for arm_key in ["arm0", "arm1"]:
                if arm_key in config:
                    for tool_name in tool_names:
                        if tool_name not in config[arm_key]:
                            print(f"配置中 {arm_key} 缺少工具 {tool_name}，使用默认配置替换")
                            with open(file_path, 'w', encoding='utf-8') as f:
                                json.dump(tools_cfg, f, indent=4, ensure_ascii=False)
                            return False

            # 所有检查通过，返回配置
            print(f"成功加载配置文件: {file_path}")
            return config

    except json.JSONDecodeError :
        # JSON解析错误，使用默认配置替换
        print(f"JSON解析错误，文件可能损坏，使用默认配置替换: {file_path}")
        with open(file_path, 'w', encoding='utf-8') as f:
            json.dump(tools_cfg, f, indent=4, ensure_ascii=False)
        return False
    except Exception as e:
        # 其他错误
        print(f"加载配置文件时发生错误: {e}")
        print(f"使用默认配置替换: {file_path}")
        try:
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(tools_cfg, f, indent=4, ensure_ascii=False)
        except Exception as write_error:
            print(f"写入默认配置失败: {write_error}")
        return False


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
    messagebox.showinfo("Data Acquisition ID Description", f"Data Acquisition ID Sequence Number:\n"

                            "Left Arm\n"
                            
                            "0-6: Left Arm Joint Position\n"
                            
                            "10-16: Left Arm Joint Velocity\n"
                            
                            "20-26: Left Arm External Position\n"
                            
                            "30-36: Left Arm Joint Command Position\n"
                            
                            "40-46: Left Arm Joint Current (per mille)\n"
                            
                            "50-56: Left Arm Joint Sensor Torque (Nm)\n"
                            
                            "60-66: Estimated Friction Force of Left Arm\n"
                            
                            "70-76: Estimated Friction Force Velocity of Left Arm\n"
                            
                            "80-85: Estimated External Force of Left Arm Joint\n"
                            
                            "90-95: Estimated External Force at the End of Left Arm\n\n"
                            
                            "\nRight Arm\n"
                            
                            "100-106: Right Arm Joint Position\n"
                            
                            "110-116: Right Arm Joint Velocity\n" "120-126: Right arm external position\n"
                            
                            "130-136: Right arm joint command position\n"
                            
                            "140-146: Right arm joint current (per mille)\n"
                            
                            "150-156: Right arm joint sensor torque (Nm)\n"
                            
                            "160-166: Estimated right arm friction force\n"
                            
                            "170-176: Estimated right arm friction force velocity\n"
                            
                            "180-185: Estimated right arm joint external force\n"
                            
                            "190-195: Estimated right arm distal end external force\n\n"
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
    m_S += "Matrix form (column vectors are coordinate direction vectors):\n"
    m_S += f"{m_mat[0][0]:.2f}\t{m_mat[0][1]:.2f}\t{m_mat[0][2]:.2f}\n"
    m_S += f"{m_mat[1][0]:.2f}\t{m_mat[1][1]:.2f}\t{m_mat[1][2]:.2f}\n"
    m_S += f"{m_mat[2][0]:.2f}\t{m_mat[2][1]:.2f}\t{m_mat[2][2]:.2f}\n\n"
    # 计算ABC角度
    m_abc = [0.0] * 3
    Matrix2ABC(m_mat, m_abc)
    m_S += f"ABC angles：[{m_abc[0]:.5f}, {m_abc[1]:.5f}, {m_abc[2]:.5f}]\n"
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
            first_part = re.sub(r'(=)\d+(@?)', r'\g<1>9\2', parts[0])
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
    new_letters = ['X', 'Y', 'Z', 'A', 'B', 'C', 'U','V','W']
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

                        new_features.append(f"{new_letters[7]} {0.000000}")
                        new_features.append(f"{new_letters[8]} {0.000000}")

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

                        new_features.append(f"{new_letters[7]} {0.000000}")
                        new_features.append(f"{new_letters[8]} {0.000000}")

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

    print(f"downsample：from {original_data_lines} to {processed_data_lines} ")
    if '/'in file_path:
        file_path=file_path.split('/')[-1]
    file_path_save='processed_'+file_path
    with open(file_path_save, 'w', encoding='utf-8', newline='\n') as f:
        f.writelines(processed_lines)

    print(f"\nfile '{file_path}' saved as  {file_path_save}")
    print(f"freq：1000Hz -> 500Hz")
    print(f"lines：{original_data_lines} -> {processed_data_lines}")
    print(f"format unify：{'yes' if format_unify else 'no'}")

if __name__ == "__main__":
    # 定义常量
    DBL_EPSILON = sys.float_info.epsilon
    arm_main_state_with = 130
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
    kk1.log_switch(0)
    kk2.log_switch(0)
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