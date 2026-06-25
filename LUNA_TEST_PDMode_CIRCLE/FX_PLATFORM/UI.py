import tkinter as tk
from tkinter import messagebox, ttk, scrolledtext, filedialog, simpledialog
import threading
import time
import queue
import os
import math
import sys
import ast
import difflib
import re
from pathlib import Path

if getattr(sys, 'frozen', False):
    base_dir = Path(sys._MEIPASS)
    root_dir = Path(sys.executable).parent
else:
    base_dir = Path(__file__).parent
    root_dir = base_dir.parent

for p in [str(base_dir), str(root_dir)]:
    if p not in sys.path:
        sys.path.insert(0, p)

from PYTHON_SDK.GentoRobot import GentoRobot, RobotDataManager, ArmsSynchronousPlanningParams, error_dict, FXObjType, \
    FXLogMask, FXObjMask, FXTerminalType, robot_type_map, state_map, FXHandType,FXHandAction,FXHandState


class App:
    def __init__(self, root):
        self.root = root
        self.root.title("FXPlatform")
        self.root.geometry("1350x800")
        self.root.configure(bg="#f0f0f0")

        self.data_manager = None
        self.rt = None
        self.sg = None

        self.servo_versions = {}
        self.sensor_versions = {}
        self.sensor_serials = {}
        self.sys_version = "Unknown"
        self.sdk_version = sdk_version
        self.drag_mode = False
        self.ini_file_path = ""
        self.system_file_path = ""

        self.params = []
        self.init_kd_variables()
        self.points1 = []
        self.points2 = []
        self.body_points = []
        self.head_points = []
        self.lift_points = []
        self.hand_points1 = []
        self.hand_points2 = []

        self.command1 = []
        self.command2 = []

        self.display_mode = 0
        self.mode_names = ["Position", "CmdPosition", "Velocity", "SensorTorque", "TorqueExt", "MotorTorque",
                           "ExtPosition", "BaseGyro", "FlangForce"]
        self.data_keys = [('fb_pos'), ('cmd_pos'), ('fb_vel'), ('fb_sensor'), ('fb_ext_torque'), ('joint_torque'),
                          ('ext_pos'), ('base_gyro'), ('flange_force')]
        self.arm_rt_key = [('fb_pos'), ('cmd_pos'), ('fb_vel'), ('fb_sensor'), ('fb_ext_torque'), ('base_gyro'),
                           ('flange_force')]
        self.arm_sg_key = [('joint_torque'), ('ext_pos')]
        self.body_rt_key = [('fb_pos'), ('cmd_pos'), ('fb_vel'), ('fb_sensor')]
        self.body_sg_key = [('joint_torque'), ('ext_pos')]
        self.head_rt_key = [('fb_pos'), ('cmd_pos')]
        self.head_sg_key = [('ext_pos')]
        self.lift_rt_key = [('fb_pos'), ('cmd_pos')]
        self.lift_sg_key = [('joint_torque')]

        self.hand_rt_key=[('fb_pos'),('vel'),('cmd_pos')]
        self.hand_sg_key = [('joint_torque')]

        self.widgets = {}

        # Create control panel
        self.create_control_components()

        # Create main content area
        self.create_main_content()

        # Create component frames
        self.create_left_arm_components()
        self.create_separator()
        self.create_right_arm_components()
        self.create_separator()
        self.create_body_components()
        self.create_separator()
        self.create_head_components()
        self.create_separator()
        self.create_lift_components()
        self.create_separator()
        self.create_hand0_components()
        self.create_separator()
        self.create_hand1_components()

        # Create status bar
        self.create_status_bar()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.connected = False
        self.data_subscriber = None
        self.stop_event = threading.Event()
        self.thread = None

    def create_main_content(self):
        self.main_container = tk.Frame(self.root, bg="white", padx=5, pady=10)
        self.main_container.pack(fill="both", expand=True)
        self.main_canvas = tk.Canvas(self.main_container, bg="white", highlightthickness=0)
        self.main_scrollbar = ttk.Scrollbar(self.main_container, orient="vertical", command=self.main_canvas.yview)
        self.scrollable_frame = tk.Frame(self.main_canvas, bg="white")
        self.stop_frame = tk.Frame(self.main_canvas, bg='white')
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
        separator.pack(fill="x", pady=(5, 10))

    def create_left_arm_components(self):
        container = tk.Frame(self.scrollable_frame, bg="white", pady=5)
        container.pack(fill="x", pady=(0, 5))

        content = tk.Frame(container, bg="white")
        content.pack(fill="x")

        # ---------------------------- First column: status info ----------------------------
        left_status_frame = tk.Frame(content, bg="white", width=arm_main_state_with)
        left_status_frame.pack(side="left", fill="y", padx=(0, 10))
        left_status_frame.pack_propagate(False)

        status_title_frame = tk.Frame(left_status_frame, bg="white")
        status_title_frame.pack(fill="x", pady=(0, 10))
        tk.Label(status_title_frame, text="ARM0", font=('Arial', 11, 'bold'),
                 fg='#2c3e50', bg="white").pack(anchor="w", padx=40, pady=(0, 5))

        status_info_frame = tk.Frame(left_status_frame, bg="white")
        status_info_frame.pack(fill="both", expand=True, anchor="nw")

        # Status row
        row1 = tk.Frame(status_info_frame, bg="white")
        row1.pack(anchor="w", pady=(0, 5))
        tk.Label(row1, text="Status:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.left_state_main = tk.Label(row1, text='IDLE', font=('Arial', 9),
                                        fg='#34495e', bg='white', width=12, pady=3,
                                        relief=tk.SUNKEN, bd=1)
        self.left_state_main.pack(side="left")

        # Drag flag
        row2 = tk.Frame(status_info_frame, bg="white")
        row2.pack(anchor="w", pady=(0, 5))
        tk.Label(row2, text="Drag:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.left_state_1 = tk.Label(row2, text='Drag off', font=('Arial', 9),
                                     fg='#34495e', bg='white', width=12, pady=3,
                                     relief=tk.SUNKEN, bd=1)
        self.left_state_1.pack(side="left")

        # Low speed flag
        row3 = tk.Frame(status_info_frame, bg="white")
        row3.pack(anchor="w", pady=(0, 5))
        tk.Label(row3, text="Motion:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.left_state_2 = tk.Label(row3, text='Stopped', font=('Arial', 9),
                                     fg='#34495e', bg='white', width=12, pady=3,
                                     relief=tk.SUNKEN, bd=1)
        self.left_state_2.pack(side="left")

        # Error code
        row4 = tk.Frame(status_info_frame, bg="white")
        row4.pack(anchor="w", pady=(0, 5))
        tk.Label(row4, text="Error:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.left_state_3 = tk.Label(row4, text='None', font=('Arial', 9),
                                     fg='#34495e', bg='white', width=12, pady=3,
                                     relief=tk.SUNKEN, bd=1)
        self.left_state_3.pack(side="left")

        # Error detail (wraps to multiple lines, keep fill)
        row5 = tk.Frame(status_info_frame, bg="white")
        row5.pack(fill="x", pady=(0, 5))
        self.left_arm_error = tk.Label(row5, text="", font=('Arial', 9),
                                       fg='#2c3e50', bg='white', pady=5,
                                       anchor='w', wraplength=100, justify='left')
        self.left_arm_error.pack(fill="x", padx=5)

        # ---------------------------- Second column: control functions ----------------------------
        middle_frame = tk.Frame(content, bg="white", width=300)
        middle_frame.pack(side="left", fill="y", expand=True, padx=(0, 15))

        # Parameter settings area
        param_frame = ttk.LabelFrame(middle_frame, text="Parameters", padding=10,
                                     relief=tk.GROOVE, borderwidth=2,
                                     style="MyCustom.TLabelframe")
        param_frame.pack(fill="x", pady=(0, 10))

        param_row = tk.Frame(param_frame, bg="white")
        param_row.pack(fill="x")

        tk.Label(param_row, text="Speed:", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        self.left_speed_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.left_speed_entry.pack(side="left")
        self.left_speed_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 5))

        tk.Label(param_row, text="Accel:", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        self.left_accel_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.left_accel_entry.pack(side="left")
        self.left_accel_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 5))

        speed_btn1 = tk.Button(param_row, text="Confirm Speed", width=15,
                               command=lambda: self.vel_acc_set('Arm0'),
                               bg="#58C3EE", font=("Arial", 9, "bold"))
        speed_btn1.pack(side="left", padx=(0, 20))

        self.left_impedance_btn = tk.Button(param_row, text="Impedance Params", width=15,
                                            command=lambda: self.show_impedance_dialog('Arm0'),
                                            bg="#9C27B0", fg="white", font=("Arial", 9, "bold"))
        self.left_impedance_btn.pack(side="left")

        # Status switching + error handling (horizontal layout)
        top_mid = tk.Frame(middle_frame, bg="white")
        top_mid.pack(fill="x", pady=(0, 5))

        # Status switching area
        state_switch_frame = ttk.LabelFrame(top_mid, text="Status switching", padding=10,
                                            relief=tk.GROOVE, borderwidth=2,
                                            style="MyCustom.TLabelframe")
        state_switch_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))

        state_row1 = tk.Frame(state_switch_frame, bg="white")
        state_row1.pack(fill="x", pady=(0, 5))

        self.reset_button = tk.Button(state_row1, text="IDLE", width=10,
                                      command=lambda: self.idle_state('Arm0'),
                                      bg="#2196F3", fg="white", font=("Arial", 10, "bold"))
        self.reset_button.pack(side="left", pady=(0, 5), padx=(0, 5))

        self.position_button = tk.Button(state_row1, text="Position", width=10,
                                         command=lambda: self.position_state('Arm0'), bg="#9fd4cf", fg="black",
                                         font=("Arial", 10, "bold"))
        self.position_button.pack(side="left", pady=(0, 5), padx=(0, 5))

        self.jointimp_button = tk.Button(state_row1, text="JointImp", width=10,
                                         command=lambda: self.jointImp_state('Arm0'), bg="#d9d0ca", fg="black",
                                         font=("Arial", 10, "bold"))
        self.jointimp_button.pack(side="left", pady=(0, 5), padx=(0, 5))

        state_row2 = tk.Frame(state_switch_frame, bg="white")
        state_row2.pack(fill="x", pady=(0, 5))

        self.cartimp_button = tk.Button(state_row2, text="CartImp", width=10,
                                        command=lambda: self.cartImp_state('Arm0'),
                                        bg="#dbd0db", fg="black", font=("Arial", 10, "bold"))
        self.cartimp_button.pack(side="left", pady=(0, 5), padx=(0, 5))

        self.forceimp_button = tk.Button(state_row2, text="ForceImp", width=10,
                                         command=lambda: self.forceImp_state('Arm0'), bg="#dfdcf2", fg="black",
                                         font=("Arial", 10, "bold"))
        self.forceimp_button.pack(side="left", pady=(0, 5), padx=(0, 5))

        # Add drag mode selection (combobox left of button)
        drag_frame = tk.Frame(state_row2, bg="white")
        drag_frame.pack(side="left", padx=(0, 0))
        self.drag_combo = ttk.Combobox(drag_frame, values=["joint", "cartX", "cartY", "cartZ", "cartR"],
                                       state="readonly", width=4)
        self.drag_combo.current(0)
        self.drag_combo.pack(side="left", padx=(0, 0))
        self.drag_btn = tk.Button(drag_frame, text="Drag", width=5,
                                  command=lambda: self.drag_state('Arm0'),
                                  bg="#FFA07A", fg="black", font=("Arial", 9, "bold"))
        self.drag_btn.pack(side="left")

        # Error handling area
        error_handle_frame = ttk.LabelFrame(top_mid, text="Error Handling", padding=10,
                                            relief=tk.GROOVE, borderwidth=2,
                                            style="MyCustom.TLabelframe")
        error_handle_frame.pack(side="left", fill="both", expand=True)

        servo_frame = tk.Frame(error_handle_frame, bg="white")
        servo_frame.pack(fill="x", pady=(0, 10))

        self.reset_btn_arm0 = tk.Button(servo_frame, text="Reset", width=10,
                                        command=lambda: self.reset_error('Arm0'),
                                        bg="#a0ebc8", fg="black", font=("Arial", 10, "bold"),
                                        relief=tk.RAISED, bd=2)
        self.reset_btn_arm0.pack(side="left", padx=(0, 5))

        self.get_servo_error_left_btn = tk.Button(servo_frame, text="GetSroErr", width=10,
                                                  command=lambda: self.error_get('Arm0'),
                                                  font=("Arial", 10, "bold"))
        self.get_servo_error_left_btn.pack(side="left", padx=(0, 20))

        control_frame = tk.Frame(error_handle_frame, bg='white')
        control_frame.pack(fill="x")

        self.release_collab_left_btn = tk.Button(control_frame, text="CR", width=5,
                                                 command=lambda: self.cr_state('Arm0'),
                                                 bg="#4CAF50", fg="white", font=("Arial", 10, "bold"))
        self.release_collab_left_btn.pack(side="left", padx=(0, 5))

        self.release_brake_left_btn = tk.Button(control_frame, text="Brake", width=10,
                                                command=lambda: self.brake('Arm0'),
                                                font=("Arial", 10, "bold"))
        self.release_brake_left_btn.pack(side="left", padx=(0, 5))

        self.hold_brake_left_btn = tk.Button(control_frame, text="UnBrake", width=10,
                                             command=lambda: self.release_brake('Arm0'),
                                             font=("Arial", 10, "bold"))
        self.hold_brake_left_btn.pack(side="left")

        # ---------------------------- Third column: realtime data + position command ----------------------------
        right_frame = tk.Frame(content, bg="white", width=650)
        right_frame.pack(side="left", fill="y")
        right_frame.pack_propagate(False)

        # Realtime data area
        data_frame = ttk.LabelFrame(right_frame, text="Realtime Data", padding=1,
                                    relief=tk.GROOVE, borderwidth=2,
                                    style="MyCustom.TLabelframe")
        data_frame.pack(fill="x", pady=(0, 5))

        # Joint positions row
        joint_pos_frame = tk.Frame(data_frame, bg="white")
        joint_pos_frame.pack(fill="x", pady=(0, 5))
        tk.Label(joint_pos_frame, text="J1~J7:", font=('Arial', 10, 'bold'), width=8,
                 bg='white').pack(side="left", padx=(0, 2))
        self.left_joint_text = tk.Text(joint_pos_frame, width=55, height=1,
                                       font=('Arial', 9), bg='white',
                                       relief=tk.SUNKEN, bd=1, wrap=tk.NONE)
        self.left_joint_text.tag_configure("center", justify='center')
        self.left_joint_text.pack(side="left")
        self.left_joint_text.insert("1.0", "0.000,0.000,0.000,0.000,0.000,0.000,0.000")
        self.left_joint_text.tag_add("center", "1.0", "end")
        self.left_joint_text.config(state="disabled")

        pose_frame = tk.Frame(data_frame, bg="white")
        pose_frame.pack(fill="x", pady=(0, 5))
        tk.Label(pose_frame, text="XYZABC:", font=('Arial', 10, 'bold'), width=8,
                 bg='white').pack(side="left", padx=(0, 2))
        self.left_pose_text = tk.Text(pose_frame, width=55, height=1,
                                      font=('Arial', 9), bg='white',
                                      relief=tk.SUNKEN, bd=1, wrap=tk.NONE)
        self.left_pose_text.tag_configure("center", justify='center')
        self.left_pose_text.pack(side="left")
        self.left_pose_text.insert("1.0", "0.000,0.000,0.000,0.000,0.000,0.000")
        self.left_pose_text.tag_add("center", "1.0", "end")
        self.left_pose_text.config(state="disabled")

        # Position command area
        joint_cmd_frame = ttk.LabelFrame(right_frame, text="Position Cmd", padding=10,
                                         relief=tk.GROOVE, borderwidth=2,
                                         style="MyCustom.TLabelframe")
        joint_cmd_frame.pack(fill="x")

        # First row: Get current position + input + Add
        row_cmd1 = tk.Frame(joint_cmd_frame, bg='white')
        row_cmd1.pack(fill="x", pady=(0, 5))

        self.btn_add3 = tk.Button(row_cmd1, text="GetCurPos", width=8, command=lambda: self.get_current_pos('Arm0'))
        self.btn_add3.pack(side="left", padx=(0, 5))

        self.entry_var = tk.StringVar(value="0,0,0,0,0,0,0")
        self.entry = tk.Entry(row_cmd1, textvariable=self.entry_var, width=45)
        self.entry.pack(side="left", padx=(5, 5))

        self.btn_add1 = tk.Button(row_cmd1, text="Add", width=8, command=lambda: self.add_pos('Arm0'))
        self.btn_add1.pack(side="left", padx=(20, 5))

        # Second row: Delete + point selection + Run
        row_cmd2 = tk.Frame(joint_cmd_frame, bg='white')
        row_cmd2.pack(fill="x", pady=(0, 5))

        self.btn_del1 = tk.Button(row_cmd2, text="Delete", width=8, command=lambda: self.delete_pos('Arm0'))
        self.btn_del1.pack(side="left", padx=(0, 5))

        self.combo1 = ttk.Combobox(row_cmd2, state="readonly", width=45)
        self.combo1.pack(side="left", padx=(0, 5))

        self.btn_run1 = tk.Button(row_cmd2, text="Run", width=8, command=lambda: self.run_pos('Arm0'),
                                  font=("Arial", 11, "bold"), fg='white',
                                  bg='#EC2A23', border=5)
        self.btn_run1.pack(side="left", padx=(0, 5))

    def create_right_arm_components(self):
        container = tk.Frame(self.scrollable_frame, bg="white", pady=5)
        container.pack(fill="x", pady=(0, 5))

        content = tk.Frame(container, bg="white")
        content.pack(fill="x")

        # ---------------------------- First column: status info ----------------------------
        left_status_frame = tk.Frame(content, bg="white", width=arm_main_state_with)
        left_status_frame.pack(side="left", fill="y", padx=(0, 10))
        left_status_frame.pack_propagate(False)

        status_title_frame = tk.Frame(left_status_frame, bg="white")
        status_title_frame.pack(fill="x", pady=(0, 10))
        tk.Label(status_title_frame, text="ARM1", font=('Arial', 11, 'bold'),
                 fg='#2c3e50', bg="white").pack(anchor="w", padx=40, pady=(0, 5))

        status_info_frame = tk.Frame(left_status_frame, bg="white")
        status_info_frame.pack(fill="both", expand=True, anchor="nw")

        row1 = tk.Frame(status_info_frame, bg="white")
        row1.pack(anchor="w", pady=(0, 5))
        tk.Label(row1, text="Status:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.right_state_main = tk.Label(row1, text='IDLE', font=('Arial', 9),
                                         fg='#34495e', bg='white', width=15, pady=3,
                                         relief=tk.SUNKEN, bd=1)
        self.right_state_main.pack(side="left")

        row2 = tk.Frame(status_info_frame, bg="white")
        row2.pack(anchor="w", pady=(0, 5))
        tk.Label(row2, text="Drag:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.right_state_1 = tk.Label(row2, text='Drag off', font=('Arial', 9),
                                      fg='#34495e', bg='white', width=15, pady=3,
                                      relief=tk.SUNKEN, bd=1)
        self.right_state_1.pack(side="left")

        row3 = tk.Frame(status_info_frame, bg="white")
        row3.pack(anchor="w", pady=(0, 5))
        tk.Label(row3, text="Motion:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.right_state_2 = tk.Label(row3, text='Stopped', font=('Arial', 9),
                                      fg='#34495e', bg='white', width=15, pady=3,
                                      relief=tk.SUNKEN, bd=1)
        self.right_state_2.pack(side="left")

        row4 = tk.Frame(status_info_frame, bg="white")
        row4.pack(anchor="w", pady=(0, 5))
        tk.Label(row4, text="Error:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.right_state_3 = tk.Label(row4, text='None', font=('Arial', 9),
                                      fg='#34495e', bg='white', width=15, pady=3,
                                      relief=tk.SUNKEN, bd=1)
        self.right_state_3.pack(side="left")

        row5 = tk.Frame(status_info_frame, bg="white")
        row5.pack(fill="x", pady=(0, 5))
        self.right_arm_error = tk.Label(row5, text="", font=('Arial', 9),
                                        fg='#2c3e50', bg='white', pady=5,
                                        anchor='w', wraplength=100, justify='left')
        self.right_arm_error.pack(fill="x", padx=5)

        # ---------------------------- Second column: control functions ----------------------------
        middle_frame = tk.Frame(content, bg="white", width=300)
        middle_frame.pack(side="left", fill="y", expand=True, padx=(0, 15))

        # Parameter settings area
        param_frame = ttk.LabelFrame(middle_frame, text="Parameters", padding=10,
                                     relief=tk.GROOVE, borderwidth=2,
                                     style="MyCustom.TLabelframe")
        param_frame.pack(fill="x", pady=(0, 10))

        param_row = tk.Frame(param_frame, bg="white")
        param_row.pack(fill="x")

        tk.Label(param_row, text="Speed:", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        self.right_speed_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.right_speed_entry.pack(side="left")
        self.right_speed_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 5))

        tk.Label(param_row, text="Accel:", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        self.right_accel_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.right_accel_entry.pack(side="left")
        self.right_accel_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 5))

        speed_btn = tk.Button(param_row, text="Confirm Speed", width=15,
                              command=lambda: self.vel_acc_set('Arm1'),
                              bg="#58C3EE", font=("Arial", 9, "bold"))
        speed_btn.pack(side="left", padx=(0, 20))

        self.right_impedance_btn = tk.Button(param_row, text="Impedance Params", width=15,
                                             command=lambda: self.show_impedance_dialog('Arm1'),
                                             bg="#9C27B0", fg="white", font=("Arial", 9, "bold"))
        self.right_impedance_btn.pack(side="left")

        # Status switching + error handling (horizontal layout)
        top_mid = tk.Frame(middle_frame, bg="white")
        top_mid.pack(fill="x", pady=(0, 5))

        # Status switching area
        state_switch_frame = ttk.LabelFrame(top_mid, text="Status switching", padding=10,
                                            relief=tk.GROOVE, borderwidth=2,
                                            style="MyCustom.TLabelframe")
        state_switch_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))

        state_row1 = tk.Frame(state_switch_frame, bg="white")
        state_row1.pack(fill="x", pady=(0, 5))

        self.reset_button_r = tk.Button(state_row1, text="IDLE", width=10,
                                        command=lambda: self.idle_state('Arm1'),
                                        bg="#2196F3", fg="white", font=("Arial", 10, "bold"))
        self.reset_button_r.pack(side="left", pady=(0, 5), padx=(0, 5))

        self.position_button_r = tk.Button(state_row1, text="Position", width=10,
                                           command=lambda: self.position_state('Arm1'), bg="#9fd4cf", fg="black",
                                           font=("Arial", 10, "bold"))
        self.position_button_r.pack(side="left", pady=(0, 5), padx=(0, 5))

        self.jointimp_button_r = tk.Button(state_row1, text="JointImp", width=10,
                                           command=lambda: self.jointImp_state('Arm1'), bg="#d9d0ca", fg="black",
                                           font=("Arial", 10, "bold"))
        self.jointimp_button_r.pack(side="left", pady=(0, 5), padx=(0, 5))

        state_row2 = tk.Frame(state_switch_frame, bg="white")
        state_row2.pack(fill="x", pady=(0, 5))

        self.cartimp_button_r = tk.Button(state_row2, text="CartImp", width=10,
                                          command=lambda: self.cartImp_state('Arm1'),
                                          bg="#dbd0db", fg="black", font=("Arial", 10, "bold"))
        self.cartimp_button_r.pack(side="left", pady=(0, 5), padx=(0, 5))

        self.forceimp_button_r = tk.Button(state_row2, text="ForceImp", width=10,
                                           command=lambda: self.forceImp_state('Arm1'), bg="#dfdcf2", fg="black",
                                           font=("Arial", 10, "bold"))
        self.forceimp_button_r.pack(side="left", pady=(0, 5), padx=(0, 5))

        # Add drag mode selection (combobox left of button)
        drag_frame = tk.Frame(state_row2, bg="white")
        drag_frame.pack(side="left", padx=(0, 0))
        self.drag_combo_r = ttk.Combobox(drag_frame, values=["joint", "cartX", "cartY", "cartZ", "cartR"],
                                         state="readonly", width=4)
        self.drag_combo_r.current(0)
        self.drag_combo_r.pack(side="left", padx=(0, 0))
        self.drag_btn_r = tk.Button(drag_frame, text="Drag", width=5,
                                    command=lambda: self.drag_state('Arm1'),
                                    bg="#FFA07A", fg="black", font=("Arial", 9, "bold"))
        self.drag_btn_r.pack(side="left")

        # Error handling area
        error_handle_frame = ttk.LabelFrame(top_mid, text="Error Handling", padding=10,
                                            relief=tk.GROOVE, borderwidth=2,
                                            style="MyCustom.TLabelframe")
        error_handle_frame.pack(side="left", fill="both", expand=True)

        servo_frame = tk.Frame(error_handle_frame, bg="white")
        servo_frame.pack(fill="x", pady=(0, 10))

        self.reset_btn_arm1 = tk.Button(servo_frame, text="Reset", width=10,
                                        command=lambda: self.reset_error('Arm1'),
                                        bg="#a0ebc8", fg="black", font=("Arial", 10, "bold"),
                                        relief=tk.RAISED, bd=2)
        self.reset_btn_arm1.pack(side="left", padx=(0, 5))

        self.get_servo_error_right_btn = tk.Button(servo_frame, text="GetSroErr", width=10,
                                                   command=lambda: self.error_get('Arm1'),
                                                   font=("Arial", 10, "bold"))
        self.get_servo_error_right_btn.pack(side="left", padx=(0, 20))

        control_frame = tk.Frame(error_handle_frame, bg='white')
        control_frame.pack(fill="x")

        self.release_collab_right_btn = tk.Button(control_frame, text="CR", width=5,
                                                  command=lambda: self.cr_state('Arm1'),
                                                  bg="#4CAF50", fg="white", font=("Arial", 10, "bold"))
        self.release_collab_right_btn.pack(side="left", padx=(0, 5))

        self.release_brake_right_btn = tk.Button(control_frame, text="Brake", width=10,
                                                 command=lambda: self.brake('Arm1'),

                                                 font=("Arial", 10, "bold"))
        self.release_brake_right_btn.pack(side="left", padx=(0, 5))

        self.hold_brake_right_btn = tk.Button(control_frame, text="UnBrake", width=10,
                                              command=lambda: self.release_brake('Arm1'),
                                              font=("Arial", 10, "bold"))
        self.hold_brake_right_btn.pack(side="left")

        # ---------------------------- Third column: realtime data + position command ----------------------------
        right_frame = tk.Frame(content, bg="white", width=650)
        right_frame.pack(side="left", fill="y")
        right_frame.pack_propagate(False)

        # Realtime data area
        data_frame = ttk.LabelFrame(right_frame, text="Realtime Data", padding=1,
                                    relief=tk.GROOVE, borderwidth=2,
                                    style="MyCustom.TLabelframe")
        data_frame.pack(fill="x", pady=(0, 5))

        # Joint positions row (right arm)
        joint_frame = tk.Frame(data_frame, bg="white")
        joint_frame.pack(fill="x", pady=(0, 5))
        tk.Label(joint_frame, text="J1~J7:", font=('Arial', 10, 'bold'), width=8,
                 bg='white').pack(side="left", padx=(0, 2))
        self.r_joint_text = tk.Text(joint_frame, width=55, height=1,
                                    font=('Arial', 9), bg='white',
                                    relief=tk.SUNKEN, bd=1, wrap=tk.NONE)
        self.r_joint_text.tag_configure("center", justify='center')
        self.r_joint_text.pack(side="left", fill="x")
        self.r_joint_text.insert("1.0", "0.000,0.000,0.000,0.000,0.000,0.000,0.000")
        self.r_joint_text.tag_add("center", "1.0", "end")
        self.r_joint_text.config(state="disabled")

        pose_frame = tk.Frame(data_frame, bg="white")
        pose_frame.pack(fill="x", pady=(0, 5))
        tk.Label(pose_frame, text="XYZABC:", font=('Arial', 10, 'bold'), width=8,
                 bg='white').pack(side="left", padx=(0, 2))
        self.right_pose_text = tk.Text(pose_frame, width=55, height=1,
                                       font=('Arial', 9), bg='white',
                                       relief=tk.SUNKEN, bd=1, wrap=tk.NONE)
        self.right_pose_text.tag_configure("center", justify='center')
        self.right_pose_text.pack(side="left")
        self.right_pose_text.insert("1.0", "0.000,0.000,0.000,0.000,0.000,0.000")
        self.right_pose_text.tag_add("center", "1.0", "end")
        self.right_pose_text.config(state="disabled")

        # Position command area
        joint_cmd_frame = ttk.LabelFrame(right_frame, text="Position Cmd", padding=10,
                                         relief=tk.GROOVE, borderwidth=2,
                                         style="MyCustom.TLabelframe")
        joint_cmd_frame.pack(fill="x")

        # Realtime data area
        data_frame = ttk.LabelFrame(right_frame, text="Realtime Data", padding=10,
                                    relief=tk.GROOVE, borderwidth=2,
                                    style="MyCustom.TLabelframe")
        data_frame.pack(fill="x", pady=(0, 5))

        # First row: Get current position + input + Add
        row_cmd1 = tk.Frame(joint_cmd_frame, bg='white')
        row_cmd1.pack(fill="x", pady=(0, 5))

        self.btn_get_cur_r = tk.Button(row_cmd1, text="GetCurPos", width=8,
                                       command=lambda: self.get_current_pos('Arm1'))
        self.btn_get_cur_r.pack(side="left", padx=(0, 5))

        self.entry_var1 = tk.StringVar(value="0,0,0,0,0,0,0")
        self.entry1 = tk.Entry(row_cmd1, textvariable=self.entry_var1, width=45)
        self.entry1.pack(side="left", padx=(5, 5))

        self.btn_add_r = tk.Button(row_cmd1, text="Add", width=8, command=lambda: self.add_pos('Arm1'))
        self.btn_add_r.pack(side="left", padx=(20, 5))

        # Second row: Delete + point selection + Run
        row_cmd2 = tk.Frame(joint_cmd_frame, bg='white')
        row_cmd2.pack(fill="x", pady=(0, 5))

        self.btn_del_r = tk.Button(row_cmd2, text="Delete", width=8, command=lambda: self.delete_pos('Arm1'))
        self.btn_del_r.pack(side="left", padx=(0, 5))

        self.combo2 = ttk.Combobox(row_cmd2, state="readonly", width=45)
        self.combo2.pack(side="left", padx=(5, 5))

        self.btn_run_r = tk.Button(row_cmd2, text="Run", width=8, command=lambda: self.run_pos('Arm1'),
                                   font=("Arial", 11, "bold"), fg='white',
                                   bg='#EC2A23', border=5)
        self.btn_run_r.pack(side="left", padx=(0, 5))

    # ==================== Body Component ====================
    def create_body_components(self):
        container = tk.Frame(self.scrollable_frame, bg="white", pady=5)
        container.pack(fill="x", pady=(0, 5))

        content = tk.Frame(container, bg="white")
        content.pack(fill="x")

        # Left side: status info
        left_status_frame = tk.Frame(content, bg="white", width=arm_main_state_with)
        left_status_frame.pack(side="left", fill="y", padx=(0, 10))
        left_status_frame.pack_propagate(False)

        status_title_frame = tk.Frame(left_status_frame, bg="white")
        status_title_frame.pack(fill="x", pady=(0, 10))
        tk.Label(status_title_frame, text="BODY", font=('Arial', 11, 'bold'), fg='#2c3e50', bg="white").pack(anchor="w",
                                                                                                             padx=40,
                                                                                                             pady=(0,
                                                                                                                   5))

        status_info_frame = tk.Frame(left_status_frame, bg="white")
        status_info_frame.pack(fill="both", expand=True, anchor="nw")

        row1 = tk.Frame(status_info_frame, bg="white")
        row1.pack(anchor="w", pady=(0, 5))
        tk.Label(row1, text="Status:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.body_state_main = tk.Label(row1, text='IDLE', font=('Arial', 9),
                                        fg='#34495e', bg='white', width=15, pady=3,
                                        relief=tk.SUNKEN, bd=1)
        self.body_state_main.pack(side="left")

        row2 = tk.Frame(status_info_frame, bg="white")
        row2.pack(anchor="w", pady=(0, 5))
        tk.Label(row2, text="Error:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.body_error_code = tk.Label(row2, text='None', font=('Arial', 9),
                                        fg='#34495e', bg='white', width=15, pady=3,
                                        relief=tk.SUNKEN, bd=1)
        self.body_error_code.pack(side="left")

        row3 = tk.Frame(status_info_frame, bg="white")
        row3.pack(fill="x", pady=(0, 5))
        self.body_error_detail = tk.Label(row3, text="", font=('Arial', 9),
                                          fg='#2c3e50', bg='white', pady=5,
                                          anchor='w', wraplength=100, justify='left')
        self.body_error_detail.pack(fill="x", padx=5)

        # Middle area: parameters, status switching, error handling
        middle_frame = tk.Frame(content, bg="white")
        middle_frame.pack(side="left", fill="y", expand=True, padx=(0, 15))

        # Parameter settings
        param_frame = ttk.LabelFrame(middle_frame, text="Parameters", padding=10, relief=tk.GROOVE, borderwidth=2,
                                     style="MyCustom.TLabelframe")
        param_frame.pack(fill="x", pady=(0, 10))
        param_row = tk.Frame(param_frame, bg="white")
        param_row.pack(fill="x")
        tk.Label(param_row, text="Speed:", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        self.body_speed_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.body_speed_entry.pack(side="left")
        self.body_speed_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 5))
        tk.Label(param_row, text="Accel:", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        self.body_accel_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.body_accel_entry.pack(side="left")
        self.body_accel_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 5))
        speed_btn = tk.Button(param_row, text="Confirm Speed", width=15, command=lambda: self.vel_acc_set('Body'),
                              bg="#58C3EE", font=("Arial", 9, "bold"))
        speed_btn.pack(side="left", padx=(0, 20))
        self.body_impedance_btn = tk.Button(param_row, text="PD Params", width=15,
                                            command=lambda: self.show_impedance_dialog('Body'), bg="#9C27B0",
                                            fg="white", font=("Arial", 9, "bold"))
        self.body_impedance_btn.pack(side="left")

        # Status switching
        top_mid = tk.Frame(middle_frame, bg="white")
        top_mid.pack(fill="x", pady=(0, 5))
        state_switch_frame = ttk.LabelFrame(top_mid, text="Status switching", padding=10, relief=tk.GROOVE,
                                            borderwidth=2, style="MyCustom.TLabelframe")
        state_switch_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))

        state_row1 = tk.Frame(state_switch_frame, bg="white")
        state_row1.pack(fill="x", pady=(0, 5))
        self.body_idle_btn = tk.Button(state_row1, text="IDLE", width=10, command=lambda: self.idle_state('Body'),
                                       bg="#2196F3", fg="white", font=("Arial", 10, "bold"))
        self.body_idle_btn.pack(side="left", pady=(0, 5), padx=(0, 5))
        self.body_pos_btn = tk.Button(state_row1, text="Position", width=10,
                                      command=lambda: self.position_state('Body'), bg="#9fd4cf", fg="black",
                                      font=("Arial", 10, "bold"))
        self.body_pos_btn.pack(side="left", pady=(0, 5), padx=(0, 5))
        self.body_jointimp_btn = tk.Button(state_row1, text="PD", width=10,
                                           command=lambda: self.pd_state('Body'), bg="#d9d0ca", fg="black",
                                           font=("Arial", 10, "bold"))
        self.body_jointimp_btn.pack(side="left", pady=(0, 5), padx=(0, 5))

        # Error handling
        error_handle_frame = ttk.LabelFrame(top_mid, text="Error Handling", padding=10, relief=tk.GROOVE, borderwidth=2,
                                            style="MyCustom.TLabelframe")
        error_handle_frame.pack(side="left", fill="both", expand=True)

        servo_frame = tk.Frame(error_handle_frame, bg="white")
        servo_frame.pack(fill="x", pady=(0, 10))
        self.body_reset_btn = tk.Button(servo_frame, text="Reset", width=10, command=lambda: self.reset_error('Body'),
                                        bg="#a0ebc8", fg="black", font=("Arial", 10, "bold"), relief=tk.RAISED, bd=2)
        self.body_reset_btn.pack(side="left", padx=(0, 5))
        self.body_get_error_btn = tk.Button(servo_frame, text="GetSroErr", width=10,
                                            command=lambda: self.error_get('Body'), font=("Arial", 10, "bold"))
        self.body_get_error_btn.pack(side="left", padx=(0, 20))

        control_frame = tk.Frame(error_handle_frame, bg='white')
        control_frame.pack(fill="x")
        self.body_brake_btn = tk.Button(control_frame, text="Brake", width=10, command=lambda :self.brake('Body'),
                                        font=("Arial", 10, "bold"))
        self.body_brake_btn.pack(side="left", padx=(0, 5))
        self.body_unbrake_btn = tk.Button(control_frame, text="UnBrake", width=10, command=lambda :self.release_brake('Body'),
                                          font=("Arial", 10, "bold"))
        self.body_unbrake_btn.pack(side="left")

        # Right side: realtime data and position command
        right_frame = tk.Frame(content, bg="white", width=650)
        right_frame.pack(side="left", fill="y")
        right_frame.pack_propagate(False)

        # Realtime data
        data_frame = ttk.LabelFrame(right_frame, text="Realtime Data", padding=10, relief=tk.GROOVE, borderwidth=2,
                                    style="MyCustom.TLabelframe")
        data_frame.pack(fill="x", pady=(0, 5))
        tk.Label(data_frame, text="Pos(J1~J6):", font=('Arial', 10, 'bold'), bg='white').pack(side="left", padx=(0, 2))
        self.body_pos_text = tk.Text(data_frame, width=55, height=1, font=('Arial', 9), bg='white', relief=tk.SUNKEN,
                                     bd=1, wrap=tk.NONE)
        self.body_pos_text.tag_configure("center", justify='center')
        self.body_pos_text.pack(side="left")
        self.body_pos_text.insert("1.0", "0.000,0.000,0.000,0.000,0.000,0.000")
        self.body_pos_text.tag_add("center", "1.0", "end")
        self.body_pos_text.config(state="disabled")

        # Position command
        cmd_frame = ttk.LabelFrame(right_frame, text="Position Cmd", padding=10, relief=tk.GROOVE, borderwidth=2,
                                   style="MyCustom.TLabelframe")
        cmd_frame.pack(fill="x")

        row_cmd1 = tk.Frame(cmd_frame, bg='white')
        row_cmd1.pack(fill="x", pady=(0, 5))
        self.body_get_btn = tk.Button(row_cmd1, text="GetCurPos", width=8, command=lambda: self.get_current_pos('Body'))
        self.body_get_btn.pack(side="left", padx=(0, 5))
        self.body_cmd_entry = tk.Entry(row_cmd1, width=45)
        self.body_cmd_entry.insert(0, "0,0,0,0,0,0")
        self.body_cmd_entry.pack(side="left", padx=(5, 5))
        self.body_add_btn = tk.Button(row_cmd1, text="Add", width=8, command=lambda: self.add_pos('Body'))
        self.body_add_btn.pack(side="left", padx=(20, 5))

        row_cmd2 = tk.Frame(cmd_frame, bg='white')
        row_cmd2.pack(fill="x", pady=(0, 5))
        self.body_del_btn = tk.Button(row_cmd2, text="Delete", width=8, command=lambda: self.delete_pos('Body'))
        self.body_del_btn.pack(side="left", padx=(0, 5))
        self.body_combo = ttk.Combobox(row_cmd2, state="readonly", width=45)
        self.body_combo.pack(side="left", padx=(0, 5))
        self.body_run_btn = tk.Button(row_cmd2, text="Run", width=8, command=lambda: self.run_pos('Body'),
                                      font=("Arial", 11, "bold"), fg='white', bg='#EC2A23', border=5)
        self.body_run_btn.pack(side="left", padx=(0, 5))

        self.body_points = []
        self.body_combo['values'] = []

    # ==================== Head Component ====================
    def create_head_components(self):
        container = tk.Frame(self.scrollable_frame, bg="white", pady=5)
        container.pack(fill="x", pady=(0, 5))

        content = tk.Frame(container, bg="white")
        content.pack(fill="x")

        # Left status
        left_status_frame = tk.Frame(content, bg="white", width=arm_main_state_with)
        left_status_frame.pack(side="left", fill="y", padx=(0, 10))
        left_status_frame.pack_propagate(False)

        status_title_frame = tk.Frame(left_status_frame, bg="white")
        status_title_frame.pack(fill="x", pady=(0, 10))
        tk.Label(status_title_frame, text="HEAD", font=('Arial', 11, 'bold'), fg='#2c3e50', bg="white").pack(anchor="w",
                                                                                                             padx=40,
                                                                                                             pady=(0,
                                                                                                                   5))

        status_info_frame = tk.Frame(left_status_frame, bg="white")
        status_info_frame.pack(fill="both", expand=True, anchor="nw")

        row1 = tk.Frame(status_info_frame, bg="white")
        row1.pack(anchor="w", pady=(0, 5))
        tk.Label(row1, text="Status:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.head_state_main = tk.Label(row1, text='IDLE', font=('Arial', 9),
                                        fg='#34495e', bg='white', width=15, pady=3,
                                        relief=tk.SUNKEN, bd=1)
        self.head_state_main.pack(side="left")

        row2 = tk.Frame(status_info_frame, bg="white")
        row2.pack(anchor="w", pady=(0, 5))
        tk.Label(row2, text="Error:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.head_error_code = tk.Label(row2, text='None', font=('Arial', 9),
                                        fg='#34495e', bg='white', width=15, pady=3,
                                        relief=tk.SUNKEN, bd=1)
        self.head_error_code.pack(side="left")

        row3 = tk.Frame(status_info_frame, bg="white")
        row3.pack(fill="x", pady=(0, 5))
        self.head_error_detail = tk.Label(row3, text="", font=('Arial', 9),
                                          fg='#2c3e50', bg='white', pady=5,
                                          anchor='w', wraplength=100, justify='left')
        self.head_error_detail.pack(fill="x", padx=5)

        # Middle area
        middle_frame = tk.Frame(content, bg="white", width=300)
        middle_frame.pack(side="left", fill="both", expand=True, padx=(0, 15))

        param_frame = ttk.LabelFrame(middle_frame, text="Parameters", padding=10, relief=tk.GROOVE, borderwidth=2,
                                     style="MyCustom.TLabelframe")
        param_frame.pack(fill="x", pady=(0, 10))
        param_row = tk.Frame(param_frame, bg="white")
        param_row.pack(fill="x")
        tk.Label(param_row, text="Speed:", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        self.head_speed_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.head_speed_entry.pack(side="left")
        self.head_speed_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 5))
        tk.Label(param_row, text="Accel:", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        self.head_accel_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.head_accel_entry.pack(side="left")
        self.head_accel_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 5))
        speed_btn = tk.Button(param_row, text="Confirm Speed", width=15, command=lambda: self.vel_acc_set('Head'),
                              bg="#58C3EE", font=("Arial", 9, "bold"))
        speed_btn.pack(side="left", padx=(0, 20))

        top_mid = tk.Frame(middle_frame, bg="white")
        top_mid.pack(fill="x", pady=(0, 5))
        state_switch_frame = ttk.LabelFrame(top_mid, text="Status switching", padding=10, relief=tk.GROOVE,
                                            borderwidth=2, style="MyCustom.TLabelframe")
        state_switch_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))

        state_row1 = tk.Frame(state_switch_frame, bg="white")
        state_row1.pack(fill="x", pady=(0, 5))
        self.head_idle_btn = tk.Button(state_row1, text="IDLE", width=10, command=lambda: self.idle_state('Head'),
                                       bg="#2196F3", fg="white", font=("Arial", 10, "bold"))
        self.head_idle_btn.pack(side="left", pady=(0, 5), padx=(0, 5))
        self.head_pos_btn = tk.Button(state_row1, text="Position", width=10,
                                      command=lambda: self.position_state('Head'), bg="#9fd4cf", fg="black",
                                      font=("Arial", 10, "bold"))
        self.head_pos_btn.pack(side="left", pady=(0, 5), padx=(0, 5))

        error_handle_frame = ttk.LabelFrame(top_mid, text="Error Handling", padding=10, relief=tk.GROOVE, borderwidth=2,
                                            style="MyCustom.TLabelframe")
        error_handle_frame.pack(side="left", fill="both", expand=True)

        servo_frame = tk.Frame(error_handle_frame, bg="white")
        servo_frame.pack(fill="x", pady=(0, 10))
        self.head_reset_btn = tk.Button(servo_frame, text="Reset", width=10, command=lambda: self.reset_error('Head'),
                                        bg="#a0ebc8", fg="black", font=("Arial", 10, "bold"), relief=tk.RAISED, bd=2)
        self.head_reset_btn.pack(side="left", padx=(0, 5))
        self.head_get_error_btn = tk.Button(servo_frame, text="GetSroErr", width=10,
                                            command=lambda: self.error_get('Head'), font=("Arial", 10, "bold"))
        self.head_get_error_btn.pack(side="left", padx=(0, 20))

        control_frame = tk.Frame(error_handle_frame, bg='white')
        control_frame.pack(fill="x")
        self.head_brake_btn = tk.Button(control_frame, text="Brake", width=10, command=lambda :self.brake('Head'), font=("Arial", 10, "bold"))
        self.head_brake_btn.pack(side="left", padx=(0, 5))
        self.head_unbrake_btn = tk.Button(control_frame, text="UnBrake", width=10, command=lambda :self.release_brake('Head'),
                                          font=("Arial", 10, "bold"))
        self.head_unbrake_btn.pack(side="left")

        # Right area
        right_frame = tk.Frame(content, bg="white", width=650)
        right_frame.pack(side="left", fill="y")
        right_frame.pack_propagate(False)

        data_frame = ttk.LabelFrame(right_frame, text="Realtime Data", padding=10, relief=tk.GROOVE, borderwidth=2,
                                    style="MyCustom.TLabelframe")
        data_frame.pack(fill="x", pady=(0, 5))
        tk.Label(data_frame, text="Pos(J1~J3):", font=('Arial', 10, 'bold'), bg='white').pack(side="left", padx=(0, 2))
        self.head_pos_text = tk.Text(data_frame, width=55, height=1, font=('Arial', 9), bg='white', relief=tk.SUNKEN,
                                     bd=1, wrap=tk.NONE)
        self.head_pos_text.tag_configure("center", justify='center')
        self.head_pos_text.pack(side="left")
        self.head_pos_text.insert("1.0", "0.000,0.000,0.000")
        self.head_pos_text.tag_add("center", "1.0", "end")
        self.head_pos_text.config(state="disabled")

        cmd_frame = ttk.LabelFrame(right_frame, text="Position Cmd", padding=10, relief=tk.GROOVE, borderwidth=2,
                                   style="MyCustom.TLabelframe")
        cmd_frame.pack(fill="x")

        row_cmd1 = tk.Frame(cmd_frame, bg='white')
        row_cmd1.pack(fill="x", pady=(0, 5))
        self.head_get_btn = tk.Button(row_cmd1, text="GetCurPos", width=8, command=lambda: self.get_current_pos('Head'))
        self.head_get_btn.pack(side="left", padx=(0, 5))
        self.head_cmd_entry = tk.Entry(row_cmd1, width=45)
        self.head_cmd_entry.insert(0, "0,0,0")
        self.head_cmd_entry.pack(side="left", padx=(5, 5))
        self.head_add_btn = tk.Button(row_cmd1, text="Add", width=8, command=lambda: self.add_pos('Head'))
        self.head_add_btn.pack(side="left", padx=(20, 5))

        row_cmd2 = tk.Frame(cmd_frame, bg='white')
        row_cmd2.pack(fill="x", pady=(0, 5))
        self.head_del_btn = tk.Button(row_cmd2, text="Delete", width=8, command=lambda: self.delete_pos('Head'))
        self.head_del_btn.pack(side="left", padx=(0, 5))
        self.head_combo = ttk.Combobox(row_cmd2, state="readonly", width=45)
        self.head_combo.pack(side="left", padx=(0, 5))
        self.head_run_btn = tk.Button(row_cmd2, text="Run", width=8, command=lambda: self.run_pos('Head'),
                                      font=("Arial", 11, "bold"), fg='white', bg='#EC2A23', border=5)
        self.head_run_btn.pack(side="left", padx=(0, 5))

        self.head_points = []
        self.head_combo['values'] = []

    # ==================== Lift Component ====================
    def create_lift_components(self):
        container = tk.Frame(self.scrollable_frame, bg="white", pady=5)
        container.pack(fill="x", pady=(0, 5))

        content = tk.Frame(container, bg="white")
        content.pack(fill="x")

        # Left status
        left_status_frame = tk.Frame(content, bg="white", width=arm_main_state_with)
        left_status_frame.pack(side="left", fill="y", padx=(0, 10))
        left_status_frame.pack_propagate(False)

        status_title_frame = tk.Frame(left_status_frame, bg="white")
        status_title_frame.pack(fill="x", pady=(0, 10))
        tk.Label(status_title_frame, text="LIFT", font=('Arial', 11, 'bold'), fg='#2c3e50', bg="white").pack(anchor="w",
                                                                                                             padx=40,
                                                                                                             pady=(0,
                                                                                                                   5))

        status_info_frame = tk.Frame(left_status_frame, bg="white")
        status_info_frame.pack(fill="both", expand=True, anchor="nw")

        row1 = tk.Frame(status_info_frame, bg="white")
        row1.pack(anchor="w", pady=(0, 5))
        tk.Label(row1, text="Status:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.lift_state_main = tk.Label(row1, text='IDLE', font=('Arial', 9),
                                        fg='#34495e', bg='white', width=15, pady=3,
                                        relief=tk.SUNKEN, bd=1)
        self.lift_state_main.pack(side="left")

        row2 = tk.Frame(status_info_frame, bg="white")
        row2.pack(anchor="w", pady=(0, 5))
        tk.Label(row2, text="Error:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.lift_error_code = tk.Label(row2, text='None', font=('Arial', 9),
                                        fg='#34495e', bg='white', width=15, pady=3,
                                        relief=tk.SUNKEN, bd=1)
        self.lift_error_code.pack(side="left")

        row3 = tk.Frame(status_info_frame, bg="white")
        row3.pack(fill="x", pady=(0, 5))
        self.lift_error_detail = tk.Label(row3, text="", font=('Arial', 9),
                                          fg='#2c3e50', bg='white', pady=5,
                                          anchor='w', wraplength=100, justify='left')
        self.lift_error_detail.pack(fill="x", padx=5)

        # Middle area
        middle_frame = tk.Frame(content, bg="white", width=300)
        middle_frame.pack(side="left", fill="both", expand=True, padx=(0, 15))

        param_frame = ttk.LabelFrame(middle_frame, text="Parameters", padding=10, relief=tk.GROOVE, borderwidth=2,
                                     style="MyCustom.TLabelframe")
        param_frame.pack(fill="x", pady=(0, 10))
        param_row = tk.Frame(param_frame, bg="white")
        param_row.pack(fill="x")
        tk.Label(param_row, text="Speed:", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        self.lift_speed_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.lift_speed_entry.pack(side="left")
        self.lift_speed_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 5))
        tk.Label(param_row, text="Accel:", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        self.lift_accel_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.lift_accel_entry.pack(side="left")
        self.lift_accel_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 5))
        speed_btn = tk.Button(param_row, text="Confirm Speed", width=15, command=lambda: self.vel_acc_set('Lift'),
                              bg="#58C3EE", font=("Arial", 9, "bold"))
        speed_btn.pack(side="left", padx=(0, 20))

        top_mid = tk.Frame(middle_frame, bg="white")
        top_mid.pack(fill="x", pady=(0, 5))
        state_switch_frame = ttk.LabelFrame(top_mid, text="Status switching", padding=10, relief=tk.GROOVE,
                                            borderwidth=2, style="MyCustom.TLabelframe")
        state_switch_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))

        state_row1 = tk.Frame(state_switch_frame, bg="white")
        state_row1.pack(fill="x", pady=(0, 5))
        self.lift_idle_btn = tk.Button(state_row1, text="IDLE", width=10, command=lambda: self.idle_state('Lift'),
                                       bg="#2196F3", fg="white", font=("Arial", 10, "bold"))
        self.lift_idle_btn.pack(side="left", pady=(0, 5), padx=(0, 5))
        self.lift_pos_btn = tk.Button(state_row1, text="Position", width=10,
                                      command=lambda: self.position_state('Lift'), bg="#9fd4cf", fg="black",
                                      font=("Arial", 10, "bold"))
        self.lift_pos_btn.pack(side="left", pady=(0, 5), padx=(0, 5))

        error_handle_frame = ttk.LabelFrame(top_mid, text="Error Handling", padding=10, relief=tk.GROOVE, borderwidth=2,
                                            style="MyCustom.TLabelframe")
        error_handle_frame.pack(side="left", fill="both", expand=True)

        servo_frame = tk.Frame(error_handle_frame, bg="white")
        servo_frame.pack(fill="x", pady=(0, 10))
        self.lift_reset_btn = tk.Button(servo_frame, text="Reset", width=10, command=lambda: self.reset_error('Lift'),
                                        bg="#a0ebc8", fg="black", font=("Arial", 10, "bold"), relief=tk.RAISED, bd=2)
        self.lift_reset_btn.pack(side="left", padx=(0, 5))
        self.lift_get_error_btn = tk.Button(servo_frame, text="GetSroErr", width=10,
                                            command=lambda: self.error_get('Lift'), font=("Arial", 10, "bold"))
        self.lift_get_error_btn.pack(side="left", padx=(0, 20))

        # Right area
        right_frame = tk.Frame(content, bg="white", width=650)
        right_frame.pack(side="left", fill="both", expand=True)

        data_frame = ttk.LabelFrame(right_frame, text="Realtime Data", padding=10, relief=tk.GROOVE, borderwidth=2,
                                    style="MyCustom.TLabelframe")
        data_frame.pack(fill="x", pady=(0, 5))
        tk.Label(data_frame, text="Pos(J1):", font=('Arial', 10, 'bold'), bg='white').pack(side="left", padx=(0, 2))
        self.lift_pos_text = tk.Text(data_frame, width=55, height=1, font=('Arial', 9), bg='white', relief=tk.SUNKEN,
                                     bd=1, wrap=tk.NONE)
        self.lift_pos_text.tag_configure("center", justify='center')
        self.lift_pos_text.pack(side="left")
        self.lift_pos_text.insert("1.0", "0.000,0.000")
        self.lift_pos_text.tag_add("center", "1.0", "end")
        self.lift_pos_text.config(state="disabled")

        cmd_frame = ttk.LabelFrame(right_frame, text="Position Cmd", padding=10, relief=tk.GROOVE, borderwidth=2,
                                   style="MyCustom.TLabelframe")
        cmd_frame.pack(fill="x")

        row_cmd1 = tk.Frame(cmd_frame, bg='white')
        row_cmd1.pack(fill="x", pady=(0, 5))
        self.lift_get_btn = tk.Button(row_cmd1, text="GetCurPos", width=8, command=lambda: self.get_current_pos('Lift'))
        self.lift_get_btn.pack(side="left", padx=(0, 5))
        self.lift_cmd_entry = tk.Entry(row_cmd1, width=45)
        self.lift_cmd_entry.insert(0, "0,0")
        self.lift_cmd_entry.pack(side="left", padx=(5, 5))
        self.lift_add_btn = tk.Button(row_cmd1, text="Add", width=8, command=lambda: self.add_pos('Lift'))
        self.lift_add_btn.pack(side="left", padx=(20, 5))

        row_cmd2 = tk.Frame(cmd_frame, bg='white')
        row_cmd2.pack(fill="x", pady=(0, 5))
        self.lift_del_btn = tk.Button(row_cmd2, text="Delete", width=8, command=lambda: self.delete_pos('Lift'))
        self.lift_del_btn.pack(side="left", padx=(0, 5))
        self.lift_combo = ttk.Combobox(row_cmd2, state="readonly", width=45)
        self.lift_combo.pack(side="left", padx=(0, 5))
        self.lift_run_btn = tk.Button(row_cmd2, text="Run", width=8, command=lambda: self.run_pos('Lift'),
                                      font=("Arial", 11, "bold"), fg='white', bg='#EC2A23', border=5)
        self.lift_run_btn.pack(side="left", padx=(0, 5))

        self.lift_points = []
        self.lift_combo['values'] = []

    # ==================== HAND0 Component ====================
    def create_hand0_components(self):
        container = tk.Frame(self.scrollable_frame, bg="white", pady=5)
        container.pack(fill="x", pady=(0, 5))

        content = tk.Frame(container, bg="white")
        content.pack(fill="x")

        # Left status
        left_status_frame = tk.Frame(content, bg="white", width=arm_main_state_with)
        left_status_frame.pack(side="left", fill="y", padx=(0, 10))
        left_status_frame.pack_propagate(False)

        status_title_frame = tk.Frame(left_status_frame, bg="white")
        status_title_frame.pack(fill="x", pady=(0, 10))
        tk.Label(status_title_frame, text="Hand0", font=('Arial', 11, 'bold'), fg='#2c3e50', bg="white").pack(anchor="w",
                                                                                                             padx=40,
                                                                                                             pady=(0,
                                                                                                                   5))

        status_info_frame = tk.Frame(left_status_frame, bg="white")
        status_info_frame.pack(fill="both", expand=True, anchor="nw")

        row1 = tk.Frame(status_info_frame, bg="white")
        row1.pack(anchor="w", pady=(0, 5))
        tk.Label(row1, text="Status:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.hand0_state_main = tk.Label(row1, text='DISABLE', font=('Arial', 9),
                                        fg='#34495e', bg='white', width=15, pady=3,
                                        relief=tk.SUNKEN, bd=1)
        self.hand0_state_main.pack(side="left")
        row2 = tk.Frame(status_info_frame, bg="white")
        row2.pack(anchor="w", pady=(0, 5))
        tk.Label(row2, text="Error:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.hand0_error_code = tk.Label(row2, text='None', font=('Arial', 9),
                                        fg='#34495e', bg='white', width=15, pady=3,
                                        relief=tk.SUNKEN, bd=1)
        self.hand0_error_code.pack(side="left")

        row3 = tk.Frame(status_info_frame, bg="white")
        row3.pack(fill="x", pady=(0, 5))
        self.hand0_error_detail = tk.Label(row3, text="", font=('Arial', 9),
                                          fg='#2c3e50', bg='white', pady=5,
                                          anchor='w', wraplength=100, justify='left')
        self.hand0_error_detail.pack(fill="x", padx=5)

        # Middle area
        middle_frame = tk.Frame(content, bg="white", width=300)
        middle_frame.pack(side="left", fill="both", expand=True, padx=(0, 15))

        param_frame = ttk.LabelFrame(middle_frame, text="Parameters", padding=10, relief=tk.GROOVE, borderwidth=2,
                                     style="MyCustom.TLabelframe")
        param_frame.pack(fill="x", pady=(0, 10))
        param_row = tk.Frame(param_frame, bg="white")
        param_row.pack(fill="x")
        tk.Label(param_row, text="P:", width=5, font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        hand0_kp_entry = tk.Entry(param_row, textvariable=self.hand0_kp_entry,width=65, font=('Arial', 9), justify='center')
        hand0_kp_entry.pack(side="left")
        param_row0 = tk.Frame(param_frame, bg="white")
        param_row0.pack(fill="x")
        tk.Label(param_row0, text="D:", width=5,font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        hand0_kd_entry = tk.Entry(param_row0, textvariable=self.hand0_kd_entry,width=65, font=('Arial', 9), justify='center')
        hand0_kd_entry.pack(side="left")
        param_row1 = tk.Frame(param_frame, bg="white")
        param_row1.pack(fill="x")
        tk.Label(param_row1, text="Torque:", width=5,font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        hand0_tor_entry = tk.Entry(param_row1, textvariable=self.hand0_tor_entry,width=65, font=('Arial', 9), justify='center')
        hand0_tor_entry.pack(side="left")
        param_row2 = tk.Frame(param_frame, bg="white")
        param_row2.pack(fill="x")
        speed_btn = tk.Button(param_row2, text="Confirm parameters", width=20, command=lambda: self.hand_p_d_torq_set('Hand0'),
                              bg="#58C3EE", font=("Arial", 9, "bold"))
        speed_btn.pack(side="left", padx=(100, 0))

        top_mid = tk.Frame(middle_frame, bg="white")
        top_mid.pack(fill="x", pady=(0, 5))
        state_switch_frame = ttk.LabelFrame(top_mid, text="Status switching", padding=10, relief=tk.GROOVE,
                                            borderwidth=2, style="MyCustom.TLabelframe")
        state_switch_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))

        state_row1 = tk.Frame(state_switch_frame, bg="white")
        state_row1.pack(fill="x")
        hand0_idle_btn = tk.Button(state_row1, text="Disable", width=10, command=lambda: self.hand_disable('Hand0'),
                                       bg="#2196F3", fg="white", font=("Arial", 10, "bold"))
        hand0_idle_btn.pack(side="left", padx=(0, 5))
        hand0_pos_btn = tk.Button(state_row1, text="Enable", width=10,
                                      command=lambda: self.hand_enable('Hand0'), bg="#9fd4cf", fg="black",
                                      font=("Arial", 10, "bold"))
        hand0_pos_btn.pack(side="left", padx=(0, 5))

        hand0_reset_btn = tk.Button(state_row1, text="Reset", width=10,
                                      command=lambda: self.hand_reset('Hand0'),  bg = "#a0ebc8",  fg="black",
                                      font=("Arial", 10, "bold"))
        hand0_reset_btn.pack(side="left", padx=(0, 5))

        # Right area
        right_frame = tk.Frame(content, bg="white", width=650)
        right_frame.pack(side="left", fill="both", expand=True)
        data_frame = ttk.LabelFrame(right_frame, text="Realtime Data", padding=10, relief=tk.GROOVE, borderwidth=2,
                                    style="MyCustom.TLabelframe")
        data_frame.pack(fill="x", pady=(0, 5))
        tk.Label(data_frame, text="j0~j23:", font=('Arial', 10, 'bold'), bg='white').pack(side="left", padx=(0, 2))
        self.hand0_pos_text = tk.Text(data_frame, width=70, height=1, font=('Arial', 9), bg='white', relief=tk.SUNKEN,
                                     bd=1, wrap=tk.NONE)
        self.hand0_pos_text.tag_configure("center", justify='center')
        self.hand0_pos_text.pack(side="left")
        self.hand0_pos_text.insert("1.0", "0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        self.hand0_pos_text.tag_add("center", "1.0", "end")
        self.hand0_pos_text.config(state="disabled")

        cmd_frame = ttk.LabelFrame(right_frame, text="Position Cmd", padding=10, relief=tk.GROOVE, borderwidth=2,
                                   style="MyCustom.TLabelframe")
        cmd_frame.pack(fill="x")

        row_cmd1 = tk.Frame(cmd_frame, bg='white')
        row_cmd1.pack(fill="x", pady=(0, 5))
        self.hand0_get_btn = tk.Button(row_cmd1, text="GetCurPos", width=8, command=lambda: self.hand_get_current_pos('Hand0'))
        self.hand0_get_btn.pack(side="left", padx=(0, 5))
        self.hand0_cmd_entry = tk.Entry(row_cmd1, width=65)
        self.hand0_cmd_entry.insert(0, "0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        self.hand0_cmd_entry.pack(side="left", padx=(5, 5))
        self.hand0_add_btn = tk.Button(row_cmd1, text="Add", width=8, command=lambda: self.hand_add_pos('Hand0'))
        self.hand0_add_btn.pack(side="left", padx=(20, 5))

        row_cmd2 = tk.Frame(cmd_frame, bg='white')
        row_cmd2.pack(fill="x", pady=(0, 5))
        self.hand0_del_btn = tk.Button(row_cmd2, text="Delete", width=8, command=lambda: self.hand_delete_pos('Hand0'))
        self.hand0_del_btn.pack(side="left", padx=(0, 5))
        self.hand0_combo = ttk.Combobox(row_cmd2, state="readonly", width=65)
        self.hand0_combo.pack(side="left", padx=(0, 5))
        self.hand0_run_btn = tk.Button(row_cmd2, text="Run", width=8, command=lambda: self.hand_run_pos('Hand0'),
                                      font=("Arial", 11, "bold"), fg='white', bg='#EC2A23', border=5)
        self.hand0_run_btn.pack(side="left", padx=(0, 5))

        self.hand0_points = []
        self.hand0_combo['values'] = []

    # ==================== HAND1 Component ====================
    def create_hand1_components(self):
        container = tk.Frame(self.scrollable_frame, bg="white", pady=5)
        container.pack(fill="x", pady=(0, 5))

        content = tk.Frame(container, bg="white")
        content.pack(fill="x")

        # Left status
        left_status_frame = tk.Frame(content, bg="white", width=arm_main_state_with)
        left_status_frame.pack(side="left", fill="y", padx=(0, 10))
        left_status_frame.pack_propagate(False)

        status_title_frame = tk.Frame(left_status_frame, bg="white")
        status_title_frame.pack(fill="x", pady=(0, 10))
        tk.Label(status_title_frame, text="Hand1", font=('Arial', 11, 'bold'), fg='#2c3e50', bg="white").pack(anchor="w",
                                                                                                             padx=40,
                                                                                                             pady=(0,
                                                                                                                   5))

        status_info_frame = tk.Frame(left_status_frame, bg="white")
        status_info_frame.pack(fill="both", expand=True, anchor="nw")

        row1 = tk.Frame(status_info_frame, bg="white")
        row1.pack(anchor="w", pady=(0, 5))
        tk.Label(row1, text="Status:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.hand1_state_main = tk.Label(row1, text='DISABLE', font=('Arial', 9),
                                        fg='#34495e', bg='white', width=15, pady=3,
                                        relief=tk.SUNKEN, bd=1)
        self.hand1_state_main.pack(side="left")
        row2 = tk.Frame(status_info_frame, bg="white")
        row2.pack(anchor="w", pady=(0, 5))
        tk.Label(row2, text="Error:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.hand1_error_code = tk.Label(row2, text='None', font=('Arial', 9),
                                        fg='#34495e', bg='white', width=15, pady=3,
                                        relief=tk.SUNKEN, bd=1)
        self.hand1_error_code.pack(side="left")

        row3 = tk.Frame(status_info_frame, bg="white")
        row3.pack(fill="x", pady=(0, 5))
        self.hand1_error_detail = tk.Label(row3, text="", font=('Arial', 9),
                                          fg='#2c3e50', bg='white', pady=5,
                                          anchor='w', wraplength=100, justify='left')
        self.hand1_error_detail.pack(fill="x", padx=5)

        # Middle area
        middle_frame = tk.Frame(content, bg="white", width=300)
        middle_frame.pack(side="left", fill="both", expand=True, padx=(0, 15))

        param_frame = ttk.LabelFrame(middle_frame, text="Parameters", padding=10, relief=tk.GROOVE, borderwidth=2,
                                     style="MyCustom.TLabelframe")
        param_frame.pack(fill="x", pady=(0, 10))
        param_row = tk.Frame(param_frame, bg="white")
        param_row.pack(fill="x")
        tk.Label(param_row, text="P:", width=5, font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        hand1_kp_entry = tk.Entry(param_row,textvariable=self.hand1_kp_entry, width=65, font=('Arial', 9), justify='center')
        hand1_kp_entry.pack(side="left")
        param_row0 = tk.Frame(param_frame, bg="white")
        param_row0.pack(fill="x")
        tk.Label(param_row0, text="D:", width=5,font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        hand1_kd_entry = tk.Entry(param_row0, textvariable=self.hand1_kp_entry, width=65, font=('Arial', 9), justify='center')
        hand1_kd_entry.pack(side="left")
        param_row1 = tk.Frame(param_frame, bg="white")
        param_row1.pack(fill="x")
        tk.Label(param_row1, text="Torque:", width=5,font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        hand1_tor_entry = tk.Entry(param_row1, textvariable=self.hand1_kp_entry, width=65, font=('Arial', 9), justify='center')
        hand1_tor_entry.pack(side="left")
        param_row2 = tk.Frame(param_frame, bg="white")
        param_row2.pack(fill="x")
        speed_btn = tk.Button(param_row2, text="Confirm parameters", width=20, command=lambda: self.hand_p_d_torq_set('Hand1'),
                              bg="#58C3EE", font=("Arial", 9, "bold"))
        speed_btn.pack(side="left", padx=(100, 0))

        top_mid = tk.Frame(middle_frame, bg="white")
        top_mid.pack(fill="x", pady=(0, 5))
        state_switch_frame = ttk.LabelFrame(top_mid, text="Status switching", padding=10, relief=tk.GROOVE,
                                            borderwidth=2, style="MyCustom.TLabelframe")
        state_switch_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))

        state_row1 = tk.Frame(state_switch_frame, bg="white")
        state_row1.pack(fill="x")
        hand0_idle_btn = tk.Button(state_row1, text="Disable", width=10, command=lambda: self.hand_disable('Hand1'),
                                       bg="#2196F3", fg="white", font=("Arial", 10, "bold"))
        hand0_idle_btn.pack(side="left", padx=(0, 5))
        hand0_pos_btn = tk.Button(state_row1, text="Enable", width=10,
                                      command=lambda: self.hand_enable('Hand1'), bg="#9fd4cf", fg="black",
                                      font=("Arial", 10, "bold"))
        hand0_pos_btn.pack(side="left", padx=(0, 5))

        hand0_reset_btn = tk.Button(state_row1, text="Reset", width=10,
                                      command=lambda: self.hand_reset('Hand1'),  bg = "#a0ebc8",  fg="black",
                                      font=("Arial", 10, "bold"))
        hand0_reset_btn.pack(side="left", padx=(0, 5))

        # Right area
        right_frame = tk.Frame(content, bg="white", width=650)
        right_frame.pack(side="left", fill="both", expand=True)
        data_frame = ttk.LabelFrame(right_frame, text="Realtime Data", padding=10, relief=tk.GROOVE, borderwidth=2,
                                    style="MyCustom.TLabelframe")
        data_frame.pack(fill="x", pady=(0, 5))
        tk.Label(data_frame, text="j0~j23:", font=('Arial', 10, 'bold'), bg='white').pack(side="left", padx=(0, 2))
        self.hand1_pos_text = tk.Text(data_frame, width=70, height=1, font=('Arial', 9), bg='white', relief=tk.SUNKEN,
                                     bd=1, wrap=tk.NONE)
        self.hand1_pos_text.tag_configure("center", justify='center')
        self.hand1_pos_text.pack(side="left")
        self.hand1_pos_text.insert("1.0", "0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        self.hand1_pos_text.tag_add("center", "1.0", "end")
        self.hand1_pos_text.config(state="disabled")

        cmd_frame = ttk.LabelFrame(right_frame, text="Position Cmd", padding=10, relief=tk.GROOVE, borderwidth=2,
                                   style="MyCustom.TLabelframe")
        cmd_frame.pack(fill="x")

        row_cmd1 = tk.Frame(cmd_frame, bg='white')
        row_cmd1.pack(fill="x", pady=(0, 5))
        self.hand1_get_btn = tk.Button(row_cmd1, text="GetCurPos", width=8, command=lambda: self.hand_get_current_pos('Hand1'))
        self.hand1_get_btn.pack(side="left", padx=(0, 5))
        self.hand1_cmd_entry = tk.Entry(row_cmd1, width=65)
        self.hand1_cmd_entry.insert(0, "0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        self.hand1_cmd_entry.pack(side="left", padx=(5, 5))
        self.hand1_add_btn = tk.Button(row_cmd1, text="Add", width=8, command=lambda: self.hand_add_pos('Hand1'))
        self.hand1_add_btn.pack(side="left", padx=(20, 5))

        row_cmd2 = tk.Frame(cmd_frame, bg='white')
        row_cmd2.pack(fill="x", pady=(0, 5))
        self.hand1_del_btn = tk.Button(row_cmd2, text="Delete", width=8, command=lambda: self.hand_delete_pos('Hand1'))
        self.hand1_del_btn.pack(side="left", padx=(0, 5))
        self.hand1_combo = ttk.Combobox(row_cmd2, state="readonly", width=65)
        self.hand1_combo.pack(side="left", padx=(0, 5))
        self.hand1_run_btn = tk.Button(row_cmd2, text="Run", width=8, command=lambda: self.hand_run_pos('Hand1'),
                                      font=("Arial", 11, "bold"), fg='white', bg='#EC2A23', border=5)
        self.hand1_run_btn.pack(side="left", padx=(0, 5))

        self.hand1_points = []
        self.hand1_combo['values'] = []

    # ==================== Control Panel ====================
    def create_control_components(self):
        """Create the top control panel"""
        self.control_frame = tk.Frame(self.root, bg="#e0e0e0", pady=5)
        self.control_frame.pack(fill="x")

        # Connect button
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
        self.arm_ip_entry.insert(0, "6.6.7.190")
        self.arm_ip_entry.pack(side="left", padx=5)

        # more func
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

        # Estop
        self.mode_btn = tk.Button(
            self.control_frame,
            text="EmergencyStop",
            width=20,
            command=self.Estop,
            bg="#ebfa1e",
            fg="black",
            font=("Arial", 14, "bold"))
        self.mode_btn.pack(side="right", padx=5)

        self.mode_combo = ttk.Combobox(
            self.control_frame,
            values=self.mode_names,
            state="readonly",
            width=13,
            font=("Arial", 10)
        )
        self.mode_combo.current(self.display_mode)
        self.mode_combo.bind("<<ComboboxSelected>>", self.on_mode_selected)
        self.mode_combo.pack(side="right", padx=5)

        # Status indicator
        status_frame = tk.Frame(self.control_frame, bg="#e0e0e0")
        status_frame.pack(side="right", padx=5)
        self.status_light = tk.Label(status_frame, text="●", font=("Arial", 16), fg="red")
        self.status_light.pack(side="left", padx=5)
        self.status_label = tk.Label(status_frame, text="disconnected", bg="#e0e0e0", font=("Arial", 9))
        self.status_label.pack(side="left")
        tk.Label(status_frame, text="Realtime data:", bg="#3BA4FD", fg="white", font=("Arial", 13, 'bold')).pack(
            side="right", padx=(20, 0))

    def create_status_bar(self):
        """Create the bottom status bar"""
        self.status_bar = tk.Frame(self.root, height=20)
        self.status_bar.pack(side="bottom", fill="x")

        self.which_robot_label = tk.Label(
            self.status_bar, text=f"Robot type:", fg="black", font=("Arial", 9))
        self.which_robot_label.pack(side="left", padx=(15, 150))

        self.version_label = tk.Label(
            self.status_bar, text=f"Controller version:", fg="black", font=("Arial", 9))
        self.version_label.pack(side="left", padx=(0, 150))

        self.sdk_version_label = tk.Label(
            self.status_bar, text=f"SDK version:{self.sdk_version}", fg="black", font=("Arial", 9))
        self.sdk_version_label.pack(side="left", padx=(0, 50))

        self.time_label = tk.Label(
            self.status_bar, text="", fg="black", font=("Arial", 9))
        self.time_label.pack(side="right", padx=15)
        self.update_time()

    # ==================== Core Control Methods (Adapted to new MarvinRobot API) ====================
    def init_kd_variables(self):
        self.cart_k_b_entry = tk.StringVar(value="3000,3000,3000,100,100,100,50")
        self.cart_k_a_entry = tk.StringVar(value="3000,3000,3000,100,100,100,50")
        self.cart_d_a_entry = tk.StringVar(value="0.1,0.1,0.1,0.1,0.1,0.1,0.11")
        self.cart_d_b_entry = tk.StringVar(value="0.1,0.1,0.1,0.1,0.1,0.1,0.11")

        self.k_a_entry = tk.StringVar(value="3,3,3,2,1,1,1")
        self.k_b_entry = tk.StringVar(value="3,3,3,2,1,1,1")
        self.d_a_entry = tk.StringVar(value="0.2,0.2,0.2,0.2,0.2,0.2,0.2")
        self.d_b_entry = tk.StringVar(value="0.2,0.2,0.2,0.2,0.2,0.2,0.2")

        # self.pdp_entry = tk.StringVar(value="12,15,15,12,2,2")
        # self.pdd_entry = tk.StringVar(value="3,3,3,2,1,1")
        # self.pdp_entry = tk.StringVar(value="28,26,28,14,6,4")
        # self.pdd_entry = tk.StringVar(value="5.5,3.7,2.0,2.2,2.0,0.6")
        self.pdp_entry = tk.StringVar(value="28,26,28,14,6,4")
        self.pdd_entry = tk.StringVar(value="5.5,3.7,2.0,2.2,2.0,0.6")

        self.hand0_kp_entry=tk.StringVar(value='0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
        self.hand0_kd_entry = tk.StringVar(value='0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
        self.hand0_tor_entry = tk.StringVar(value='0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
        self.hand1_kp_entry=tk.StringVar(value='0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
        self.hand1_kd_entry = tk.StringVar(value='0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
        self.hand1_tor_entry = tk.StringVar(value='0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')

        self.force_a_entry = tk.StringVar(value="0,1,0,25,25")
        self.torque_a_entry = tk.StringVar(value="0,1,0,5,10")
        self.force_b_entry = tk.StringVar(value="0,1,0,25,25")
        self.torque_b_entry = tk.StringVar(value="0,1,0,5,10")

        self.arm0_tool_dyn_entry = tk.StringVar(value="0,0,0,0,0,0,0,0,0,0")
        self.arm1_tool_dyn_entry = tk.StringVar(value="0,0,0,0,0,0,0,0,0,0")

        self.arm0_tool_kine_entry = tk.StringVar(value="0,0,0,0,0,0")
        self.arm1_tool_kine_entry = tk.StringVar(value="0,0,0,0,0,0")

    def kine_initial(self):
        if robot.init_single_arm_config(0) !=0:
            print("[ERROR] Failed to initialize arm0 kinematics")
            return -1
        if robot.init_single_arm_config(1) !=0:
            print("[ERROR] Failed to initialize arm1 kinematics")
            return -1
        print("\narms kinematics initialized")
        robot.kine_log_level(FXLogMask.FX_LOG_DEBG_FLAG)
        return 0

    def toggle_connection(self):
        def validate_and_parse_ip(ip_str):
            ip_str = ip_str.strip()
            if not ip_str:
                return None
            pattern = re.compile(
                r'^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$')
            if not pattern.match(ip_str):
                return None
            ip_parts = [int(x) for x in ip_str.split('.')]
            return ip_parts
        if not self.connected:
            try:
                global_robot_ip = self.arm_ip_entry.get()
                ip_parts = validate_and_parse_ip(global_robot_ip)
                if ip_parts is None or len(ip_parts) != 4:
                    messagebox.showerror("IP error", f"Your input ip: {global_robot_ip}\nplease enter IPv4 address, eg: 6.6.7.190")
                    return
                # if len(ip_parts) != 4:
                #     messagebox.showerror("IP error", "please enter IPv4 address, eg: 6.6.7.190")
                #     return
                self.connect_btn.config(state="disabled")
                self.status_label.config(text="Connecting...")
                self.status_light.config(fg="blue")
                self.root.update_idletasks()
                ret = robot.link(ip_parts[0], ip_parts[1], ip_parts[2], ip_parts[3])
                if ret <0:
                    messagebox.showerror('Failed!', f"Robot connection failed, error msg: {robot._get_operate_error_msg(ret)}")
                    self._reset_ui_after_disconnect()
                    return
                else:
                    ret_link=robot.check_link_state()
                    if ret_link==1:
                        self.kine_initial()
                        self.connected = True
                        self.connect_btn.config(text="Disconnect", bg="#F44336", state="normal")
                        self.status_label.config(text="Connected")
                        self.status_light.config(fg="green")
                        self.mode_btn.config(state="normal")
                        self.update_version_control()
                        self.data_manager = RobotDataManager(robot)
                        self.update_data()
                        self.refresh_params_from_realtime_data()

                    elif ret_link==-1:
                        messagebox.showerror('Linked', f"Link is established, but no data arrived in 100ms,\n please check the cables and firewall ")
                        self._reset_ui_after_disconnect()
                        return
                    elif ret_link==0:
                        messagebox.showerror('Failed!', f"Link is not established")
                        self._reset_ui_after_disconnect()
                        return

            except Exception as e:
                messagebox.showerror('Error', f"Connection failed: {e}")
        else:
            try:
                robot.unlink()
                self.connect_btn.config(state="disabled")
                if hasattr(self, 'data_manager') and self.data_manager:
                    self.data_manager.stop()
                    self.data_manager = None

                self.connected = False
                self._reset_ui_after_disconnect()
                self.mode_btn.config(state="disabled")
            except Exception as e:
                messagebox.showerror('Error', f"Disconnect failed: {e}")
                self.connect_btn.config(state="normal")

    def _reset_ui_after_disconnect(self):
        self.connect_btn.config(text="Connect Robot", bg="#4CAF50", state="normal")
        self.status_label.config(text="Disconnected")
        self.status_light.config(fg="red")

    def update_version_control(self):
            self.sys_version = robot.get_controller_version()
            self.version_label.config(text=f"System version: {self.sys_version}")
            self.sdk_version_label.config(text=f"SDK version: {self.sdk_version}")
            self.which_robot_label.config(text=f"Robot type: {robot.get_robot_type()}")

            devices_phys = ["Arm0", "Arm1", "Body", "Head", "Lift"]
            obj_types_phys = [FXObjType.OBJ_ARM0, FXObjType.OBJ_ARM1,
                              FXObjType.OBJ_BODY, FXObjType.OBJ_HEAD, FXObjType.OBJ_LIFT]
            if not hasattr(self, 'physical_states'):
                self.physical_states = {}
            state_map = {
                0: "not used",
                1: "virtual",
                2: "real",
            }
            for device, obj_type in zip(devices_phys, obj_types_phys):
                try:
                    ret, state = robot.get_ctrl_obj_physical_state(obj_type)
                    if ret == 0:
                        state_text = state_map.get(state, f"State {state}")
                    else:
                        state_text = f"Failed (ret={robot._get_operate_error_msg(ret)}, details: {robot._get_operate_error_msg(ret)})"

                    self.physical_states[device] = state_text
                except Exception as e:
                    self.physical_states[device] = f"Error: {e}"
                if hasattr(self, 'physical_state_labels') and device in self.physical_state_labels:
                    self.physical_state_labels[device].config(text=self.physical_states[device])

            devices_servo = ["Arm0", "Arm1", "Body", "Head"]
            obj_types_servo = [FXObjType.OBJ_ARM0, FXObjType.OBJ_ARM1,
                               FXObjType.OBJ_BODY, FXObjType.OBJ_HEAD]
            for device, obj_type in zip(devices_servo, obj_types_servo):
                try:
                    ret,val = robot.get_ctrl_obj_servo_version(obj_type)
                    if ret<0:
                        self.servo_versions[device] =  f"Error: {robot._get_operate_error_msg(ret)}"
                        return
                    if isinstance(val, (list, tuple)):
                        val = ", ".join(str(v) for v in val)
                    self.servo_versions[device] = str(val)
                except Exception as e:
                    self.servo_versions[device] = f"Error: {e}"
                if hasattr(self, 'servo_version_labels') and device in self.servo_version_labels:
                    self.servo_version_labels[device].config(text=self.servo_versions[device])

            devices_sensor = ["Arm0", "Arm1", "Body"]
            obj_types_sensor = [FXObjType.OBJ_ARM0, FXObjType.OBJ_ARM1, FXObjType.OBJ_BODY]
            for device, obj_type in zip(devices_sensor, obj_types_sensor):
                try:
                    ret, ver, serial = robot.get_ctrl_obj_sensor_version_and_serial(obj_type)
                    if ret < 0:
                        self.sensor_versions[device] = f"Error: {robot._get_operate_error_msg(ret)}"
                        self.sensor_serials[device]= f"Error: {robot._get_operate_error_msg(ret)}"
                        return
                    if isinstance(ver, (list, tuple)):
                        ver_str = ", ".join(str(v) for v in ver)
                    else:
                        ver_str = str(ver)
                    if isinstance(serial, (list, tuple)):
                        serial_str = ", ".join(str(s) for s in serial)
                    else:
                        serial_str = str(serial)

                    self.sensor_versions[device] = ver_str
                    self.sensor_serials[device] = serial_str
                except Exception as e:
                    self.sensor_versions[device] = f"Error: {e}"
                    self.sensor_serials[device] = f"Error: {e}"
                if hasattr(self, 'sensor_version_labels') and device in self.sensor_version_labels:
                    self.sensor_version_labels[device].config(text=self.sensor_versions[device])
                if hasattr(self, 'sensor_serial_labels') and device in self.sensor_serial_labels:
                    self.sensor_serial_labels[device].config(text=self.sensor_serials[device])

    def refresh_params_from_realtime_data(self):
        if not self.connected or not self.data_manager:
            return
        sg_dict = self.data_manager.latest_sg
        rt_dict = self.data_manager.latest_rt
        if not sg_dict or "error" in sg_dict:
            try:
                sg_dict = robot.get_sg_dict()
            except:
                return
        if not rt_dict or "error" in rt_dict:
            try:
                rt_dict = robot.get_rt_dict()
            except:
                return

        def has_nonzero(lst, tol=1e-6):
            return any(abs(v) > tol for v in lst)

        def update_if_valid(var, new_list, default_list):
            if has_nonzero(new_list):
                var.set(','.join(str(v) for v in new_list))
            else:
                pass

        # --- Joint K/D tools info for Arm0 and Arm1 ---
        for arm_idx, arm_prefix in enumerate(['a', 'b']):
            arm_data = sg_dict['arms'][arm_idx]['set']
            joint_k = arm_data.get('joint_k', [])
            if joint_k:
                update_if_valid(getattr(self, f'k_{arm_prefix}_entry'), joint_k, None)
            joint_d = arm_data.get('joint_d', [])
            if joint_d:
                update_if_valid(getattr(self, f'd_{arm_prefix}_entry'), joint_d, None)
            cart_k = arm_data.get('cart_k', [])
            if cart_k:
                update_if_valid(getattr(self, f'cart_k_{arm_prefix}_entry'), cart_k, None)
            cart_d = arm_data.get('cart_d', [])
            if cart_d:
                update_if_valid(getattr(self, f'cart_d_{arm_prefix}_entry'), cart_d, None)

            # --- Tool kinematics (6 values) and dynamics (10 values) ---
            tool_kine = arm_data.get('tool_kine', [])
            if tool_kine:
                update_if_valid(getattr(self, f'arm{arm_idx}_tool_kine_entry'), tool_kine, None)
            tool_dyna = arm_data.get('tool_dyna', [])
            if tool_dyna:
                update_if_valid(getattr(self, f'arm{arm_idx}_tool_dyn_entry'), tool_dyna, None)

        # --- Body PD ---
        body_set = sg_dict.get('body', {}).get('set', {})
        body_pdk = body_set.get('pdk', [])
        if body_pdk:
            update_if_valid(self.pdp_entry, body_pdk, None)
        body_pdd = body_set.get('pdd', [])
        if body_pdd:
            update_if_valid(self.pdd_entry, body_pdd, None)

        # vel & acc
        vel_arm0 = sg_dict['arms'][0]['set']['vel_ratio']
        if vel_arm0 is not None and int(vel_arm0) != 0:
            self.left_speed_entry.delete(0, tk.END)
            self.left_speed_entry.insert(0, str(int(vel_arm0)))

        acc_arm0 = sg_dict['arms'][0]['set']['acc_ratio']
        if acc_arm0 is not None and int(acc_arm0) != 0:
            self.left_accel_entry.delete(0, tk.END)
            self.left_accel_entry.insert(0, str(int(acc_arm0)))

        vel_arm1 = sg_dict['arms'][1]['set']['vel_ratio']
        if vel_arm1 is not None and int(vel_arm1) != 0:
            self.right_speed_entry.delete(0, tk.END)
            self.right_speed_entry.insert(0, str(int(vel_arm1)))

        acc_arm1 = sg_dict['arms'][1]['set']['acc_ratio']
        if acc_arm1 is not None and int(acc_arm1) != 0:
            self.right_accel_entry.delete(0, tk.END)
            self.right_accel_entry.insert(0, str(int(acc_arm1)))

        # force & torque
        fb_force_arm0 = rt_dict['arms'][0]['cmd']['force_dir']
        if has_nonzero(fb_force_arm0):
            self.force_a_entry.set(','.join(str(v) for v in fb_force_arm0))
        else:
            pass
        fb_torque_arm0 = rt_dict['arms'][0]['cmd']['torque_dir']
        if has_nonzero(fb_torque_arm0):
            self.torque_a_entry.set(','.join(str(v) for v in fb_torque_arm0))
        else:
            pass

        fb_force_arm1 = rt_dict['arms'][1]['cmd']['force_dir']
        if has_nonzero(fb_force_arm1):
            self.force_b_entry.set(','.join(str(v) for v in fb_force_arm1))
        else:
            pass
        fb_torque_arm1 = rt_dict['arms'][1]['cmd']['torque_dir']
        if has_nonzero(fb_torque_arm1):
            self.torque_b_entry.set(','.join(str(v) for v in fb_torque_arm1))
        else:
            pass

        #hand kp kd torque
        fb_kp_hand0=sg_dict['hands'][0]['set']['kp']
        if has_nonzero(fb_kp_hand0):
            self.hand0_kp_entry.set(','.join(str(v) for v in fb_kp_hand0))
        else:
            pass
        fb_kp_hand1=sg_dict['hands'][1]['set']['kp']
        if has_nonzero(fb_kp_hand1):
            self.hand1_kp_entry.set(','.join(str(v) for v in fb_kp_hand1))
        else:
            pass
        fb_kd_hand0=sg_dict['hands'][0]['set']['kd']
        if has_nonzero(fb_kd_hand0):
            self.hand0_kd_entry.set(','.join(str(v) for v in fb_kd_hand0))
        else:
            pass
        fb_kd_hand1=sg_dict['hands'][1]['set']['kd']
        if has_nonzero(fb_kd_hand1):
            self.hand1_kd_entry.set(','.join(str(v) for v in fb_kd_hand1))
        else:
            pass
        fb_tor_hand0=sg_dict['hands'][0]['set']['max_torque']
        if has_nonzero(fb_tor_hand0):
            self.hand0_tor_entry.set(','.join(str(v) for v in fb_tor_hand0))
        else:
            pass
        fb_tor_hand1=sg_dict['hands'][1]['set']['max_torque']
        if has_nonzero(fb_tor_hand1):
            self.hand1_tor_entry.set(','.join(str(v) for v in fb_tor_hand1))
        else:
            pass

    def update_data(self):
        if not self.connected:
            return
        if self.data_manager:
            self.rt = self.data_manager.latest_rt
            self.sg = self.data_manager.latest_sg
            self.update_ui()
        self.root.after(200, self.update_data)

    def update_ui(self):
        if self.rt is None or self.sg is None:
            return
        joint_pos_l = ''
        joint_pos_r = ''
        body_pos = ''
        head_pos = ''
        lift_pos = ''
        hand0_pos=''
        hand1_pos=''
        key = self.data_keys[self.display_mode]

        # ==================== HANDS ====================
        hand0_state = self.rt["hands"][0]["fb"]["state"]
        if hand0_state==FXHandState.FX_HAND_STATE_ERROR:
            self.hand0_state_main.config(text=f"Error")
        elif hand0_state==FXHandState.FX_HAND_STATE_ENABLED:
            self.hand0_state_main.config(text=f"Enable")
        elif hand0_state==FXHandState.FX_HAND_STATE_DISABLED:
            self.hand0_state_main.config(text=f"Disable")
        hand1_state = self.rt["hands"][1]["fb"]["state"]
        if hand1_state == FXHandState.FX_HAND_STATE_ERROR:
            self.hand1_state_main.config(text=f"Error")
        elif hand1_state==FXHandState.FX_HAND_STATE_ENABLED:
            self.hand1_state_main.config(text=f"Enable")
        elif hand1_state==FXHandState.FX_HAND_STATE_DISABLED:
            self.hand1_state_main.config(text=f"Disable")

        if key in self.hand_rt_key:
            hand0_pos = self.rt["hands"][0]["fb"][key]
            hand1_pos = self.rt["hands"][1]["fb"][key]
        if key in self.hand_sg_key:
            hand0_pos = self.sg["hands"][0]["get"][key]
            hand1_pos = self.sg["hands"][0]["get"][key]
        pos_text_hand0 = ", ".join(f"{v:.2f}" for v in hand0_pos)
        self.hand0_pos_text.config(state="normal")
        self.hand0_pos_text.delete("1.0", tk.END)
        self.hand0_pos_text.insert("1.0", pos_text_hand0)
        self.hand0_pos_text.tag_add("center", "1.0", "end")
        self.hand0_pos_text.config(state="disabled")

        pos_text_hand1 = ", ".join(f"{v:.2f}" for v in hand1_pos)
        self.hand1_pos_text.config(state="normal")
        self.hand1_pos_text.delete("1.0", tk.END)
        self.hand1_pos_text.insert("1.0", pos_text_hand1)
        self.hand1_pos_text.tag_add("center", "1.0", "end")
        self.hand1_pos_text.config(state="disabled")

        # ==================== ARM0 ====================
        cur_state = robot.current_state(FXObjType.OBJ_ARM0)
        self.left_state_main.config(text=state_map[cur_state])

        if self.sg['arms'][0]['get']['tip_di'] == 1:
            self.left_state_1.config(text=f"Dragging")
        else:
            self.left_state_1.config(text=f"Drag off")
        if self.sg['arms'][0]['get']['low_speed_flag'] == 0:
            self.left_state_2.config(text=f"Moving")
        else:
            self.left_state_2.config(text=f"Stopped")

        arm_err = self.rt['arms'][0]['state']['err']
        self.left_state_3.config(text=f"{arm_err}")
        # Display error description using error_dict
        if arm_err != 0 and arm_err in error_dict:
            self.left_arm_error.config(text=f"Error {arm_err}: {error_dict[arm_err]}")
        else:
            self.left_arm_error.config(text="")

        if key in self.arm_rt_key:
            joint_pos_l = self.rt["arms"][0]["fb"][key]
        if key in self.arm_sg_key:
            joint_pos_l = self.sg["arms"][0]["get"][key]
        joint_text_l = ", ".join(f"{v:.3f}" for v in joint_pos_l)
        self.left_joint_text.config(state="normal")
        self.left_joint_text.delete("1.0", tk.END)
        self.left_joint_text.insert("1.0", joint_text_l)
        self.left_joint_text.tag_add("center", "1.0", "end")
        self.left_joint_text.config(state="disabled")

        arm0_joints = robot.forward_kinematics(0, self.rt["arms"][0]["fb"]['fb_pos'])
        arm0_xyzabc = robot.matrix2xyzabc(arm0_joints)
        arm0_xyzabc_text = ", ".join(f"{v:.3f}" for v in arm0_xyzabc)
        self.left_pose_text.config(state="normal")
        self.left_pose_text.delete("1.0", tk.END)
        self.left_pose_text.insert("1.0", f"{arm0_xyzabc_text}")
        self.left_pose_text.tag_add("center", "1.0", "end")
        self.left_pose_text.config(state="disabled")

        # ==================== ARM1 ====================
        cur_state = robot.current_state(FXObjType.OBJ_ARM1)
        self.right_state_main.config(text=state_map[cur_state])

        if self.sg['arms'][1]['get']['tip_di'] == 1:
            self.right_state_1.config(text=f"Dragging")
        else:
            self.right_state_1.config(text=f"Drag off")

        if self.sg['arms'][1]['get']['low_speed_flag'] == 0:
            self.right_state_2.config(text=f"Moving")
        else:
            self.right_state_2.config(text=f"Stopped")
        arm_err_r = self.rt['arms'][1]['state']['err']
        self.right_state_3.config(text=f"{arm_err_r}")
        if arm_err_r != 0 and arm_err_r in error_dict:
            self.right_arm_error.config(text=f"Error {arm_err_r}: {error_dict[arm_err_r]}")
        else:
            self.right_arm_error.config(text="")

        if key in self.arm_rt_key:
            joint_pos_r = self.rt["arms"][1]["fb"][key]
        if key in self.arm_sg_key:
            joint_pos_r = self.sg["arms"][1]["get"][key]
        joint_text_r = ", ".join(f"{v:.3f}" for v in joint_pos_r)

        self.r_joint_text.config(state="normal")
        self.r_joint_text.delete("1.0", tk.END)
        self.r_joint_text.insert("1.0", joint_text_r)
        self.r_joint_text.tag_add("center", "1.0", "end")
        self.r_joint_text.config(state="disabled")

        arm1_joints = robot.forward_kinematics(1, self.rt["arms"][1]["fb"]['fb_pos'])
        arm1_xyzabc = robot.matrix2xyzabc(arm1_joints)
        arm1_xyzabc_text = ", ".join(f"{v:.3f}" for v in arm1_xyzabc)
        self.right_pose_text.config(state="normal")
        self.right_pose_text.delete("1.0", tk.END)
        self.right_pose_text.insert("1.0", f"{arm1_xyzabc_text}")
        self.right_pose_text.tag_add("center", "1.0", "end")
        self.right_pose_text.config(state="disabled")

        # ==================== BODY ====================
        cur_state = robot.current_state(FXObjType.OBJ_BODY)
        self.body_state_main.config(text=state_map.get(cur_state, str(cur_state)))

        body_err = self.rt['body']['state']['err']
        self.body_error_code.config(text=f"{body_err}")
        if body_err != 0 and body_err in error_dict:
            self.body_error_detail.config(text=f"Error {body_err}: {error_dict[body_err]}")
        else:
            self.body_error_detail.config(text="")

        if key in self.body_rt_key:
            body_pos = self.rt["body"][key]
        if key in self.body_sg_key:
            body_pos = self.sg["body"]["get"][key]
        body_text = ", ".join(f"{v:.3f}" for v in body_pos)
        self.body_pos_text.config(state="normal")
        self.body_pos_text.delete("1.0", tk.END)
        self.body_pos_text.insert("1.0", body_text)
        self.body_pos_text.tag_add("center", "1.0", "end")
        self.body_pos_text.config(state="disabled")

        # ==================== HEAD ====================
        cur_state = robot.current_state(FXObjType.OBJ_HEAD)
        self.head_state_main.config(text=state_map.get(cur_state, str(cur_state)))

        head_err = self.rt['head']['state']['err']
        self.head_error_code.config(text=f"{head_err}")
        if head_err != 0 and head_err in error_dict:
            self.head_error_detail.config(text=f"Error {head_err}: {error_dict[head_err]}")
        else:
            self.head_error_detail.config(text="")

        if key in self.head_rt_key:
            head_pos = self.rt["head"][key]
        if key in self.head_sg_key:
            head_pos = self.sg["head"]["get"][key]
        head_text = ", ".join(f"{v:.3f}" for v in head_pos)
        self.head_pos_text.config(state="normal")
        self.head_pos_text.delete("1.0", tk.END)
        self.head_pos_text.insert("1.0", head_text)
        self.head_pos_text.tag_add("center", "1.0", "end")
        self.head_pos_text.config(state="disabled")

        # ==================== LIFT ====================
        cur_state = robot.current_state(FXObjType.OBJ_LIFT)
        self.lift_state_main.config(text=state_map.get(cur_state, str(cur_state)))

        lift_err = self.rt['lift']['state']['err']
        self.lift_error_code.config(text=f"{lift_err}")
        if lift_err != 0 and lift_err in error_dict:
            self.lift_error_detail.config(text=f"Error {lift_err}: {error_dict[lift_err]}")
        else:
            self.lift_error_detail.config(text="")

        if key in self.lift_rt_key:
            lift_pos = self.rt["lift"][key]
        if key in self.lift_sg_key:
            lift_pos = self.sg["lift"]["get"][key]
        lift_text = ", ".join(f"{v:.3f}" for v in lift_pos)
        self.lift_pos_text.config(state="normal")
        self.lift_pos_text.delete("1.0", tk.END)
        self.lift_pos_text.insert("1.0", lift_text)
        self.lift_pos_text.tag_add("center", "1.0", "end")
        self.lift_pos_text.config(state="disabled")

    def on_mode_selected(self, event=None):
        selected = self.mode_combo.get()
        self.display_mode = self.mode_names.index(selected)
        self.update_ui()

    def update_time(self):
        current_time = time.strftime("%Y-%m-%d %H:%M:%S")
        self.time_label.config(text=current_time)
        self.root.after(1000, self.update_time)

    def on_mousewheel(self, event):
        self.main_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    def on_close(self):
        if messagebox.askokcancel("Exit", "Are you sure you want to exit the application?"):
            self.root.destroy()
            robot.unlink()

    def hand_p_d_torq_set(self,obj):
        try:
            if obj == 'Hand0':
                kp = float(self.hand0_kp_entry.get())
                kd = float(self.hand0_kd_entry.get())
                tor = float(self.hand0_tor_entry.get())
                ret = robot.runtime_set_hand_p(FXHandType.FX_HAND_LEFT, kp)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set kp failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
                ret = robot.runtime_set_hand_d(FXHandType.FX_HAND_LEFT, kd)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set kd failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
                ret = robot.runtime_set_hand_max_tor(FXHandType.FX_HAND_LEFT, tor)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set max torque failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
            elif obj == 'Hand1':
                kp = float(self.hand1_kp_entry.get())
                kd = float(self.hand1_kd_entry.get())
                tor= float(self.hand1_tor_entry.get())
                ret = robot.runtime_set_hand_p(FXHandType.FX_HAND_RIGHT, kp)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set kp failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
                ret = robot.runtime_set_hand_d(FXHandType.FX_HAND_RIGHT, kd)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set kd failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
                ret = robot.runtime_set_hand_max_tor(FXHandType.FX_HAND_RIGHT, tor)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set max torque failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
            else:
                raise ValueError(f"Unknown obj: {obj}")
        except Exception as e:
            messagebox.showerror('Error', f"Operation failed: {e}")

    def hand_disable(self,obj):
        try:
            if obj=='Hand0':
                ret = robot.runtime_set_hand_action(FXHandType.FX_HAND_LEFT, FXHandAction.FX_HAND_ACTION_DISABLE)
                if ret != 0:
                    messagebox.showerror('Error', f"Set hand0 disable failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
            elif obj=="Hand1":
                ret = robot.runtime_set_hand_action(FXHandType.FX_HAND_RIGHT, FXHandAction.FX_HAND_ACTION_DISABLE)
                if ret != 0:
                    messagebox.showerror('Error', f"Set hand1 disable failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
        except Exception as e:
            messagebox.showerror('Error', f"Set idle failed: {e}")

    def hand_enable(self, obj):
        try:
            if obj == 'Hand0':
                ret = robot.runtime_set_hand_action(FXHandType.FX_HAND_LEFT, FXHandAction.FX_HAND_ACTION_ENABLE)
                if ret != 0:
                    messagebox.showerror('Error', f"Set hand0 enable failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
            elif obj == "Hand1":
                ret = robot.runtime_set_hand_action(FXHandType.FX_HAND_RIGHT, FXHandAction.FX_HAND_ACTION_ENABLE)
                if ret != 0:
                    messagebox.showerror('Error', f"Set hand1 enable failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
        except Exception as e:
            messagebox.showerror('Error', f"Set idle failed: {e}")

    def hand_reset(self, obj):
        try:
            if obj == 'Hand0':
                ret = robot.runtime_set_hand_action(FXHandType.FX_HAND_LEFT, FXHandAction.FX_HAND_ACTION_RESET)
                if ret != 0:
                    messagebox.showerror('Error', f"Set hand0 reset failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
            elif obj == "Hand1":
                ret = robot.runtime_set_hand_action(FXHandType.FX_HAND_RIGHT, FXHandAction.FX_HAND_ACTION_RESET)
                if ret != 0:
                    messagebox.showerror('Error', f"Set hand1 reset failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
        except Exception as e:
            messagebox.showerror('Error', f"Set idle failed: {e}")

    def hand_get_current_pos(self,obj):
        try:
            pose = None
            if obj == 'Hand0':
                pose = self.rt["hands"][0]["fb"]["fb_pos"]
                if pose and len(pose) == 24:
                    pose_text = ", ".join(f"{v}" for v in pose)
                    self.hand0_cmd_entry.delete(0, tk.END)
                    self.hand0_cmd_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid pos for hand0')
                    return
            elif obj == 'Hand1':
                pose = self.rt["hands"][1]["fb"]["fb_pos"]
                if pose and len(pose) == 24:
                    pose_text = ", ".join(f"{v}" for v in pose)
                    self.hand1_cmd_entry.delete(0, tk.END)
                    self.hand1_cmd_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid pos for hand1')

            else:
                messagebox.showerror('Error', f'Unknown object: {obj}')
        except (KeyError, IndexError, TypeError) as e:
            messagebox.showerror('Error', f'Failed to get {obj} pos: {e}')

    def hand_add_pos(self, obj):
        num_points = 24
        if obj == 'Hand0':
            point_str = self.hand0_cmd_entry.get()
            points_list = self.hand_points1
            combo = self.hand0_combo
        elif obj == 'Hand1':
            point_str = self.hand1_cmd_entry.get()
            points_list = self.hand_points2
            combo = self.hand1_combo
        else:
            messagebox.showerror("Error", f"Unknown object: {obj}")
            return
        is_valid, result = self.validate_point(point_str, num_points)
        if not is_valid:
            messagebox.showwarning("Wrong inputs", result)
            return
        if self.is_duplicate_command(result, points_list):
            messagebox.showwarning("Duplicate point", f"This point already exists in {obj} list")
            return
        points_list.insert(0, result)
        self.update_comboboxes()

    def hand_delete_pos(self, obj):
        if obj == 'Hand0':
            combo = self.hand0_combo
            points_list = self.hand_points1
        elif obj == 'Hand1':
            combo = self.hand1_combo
            points_list = self.hand_points2
        else:
            messagebox.showerror("Error", f"Unknown object: {obj}")
            return

        selected_index = combo.current()
        if selected_index != -1 and selected_index < len(points_list):
            points_list.pop(selected_index)
            self.update_comboboxes()
        else:
            messagebox.showwarning("Warning", f"Please select a point to delete in {obj}")

    def hand_run_pos(self, obj):
        try:
            if obj == 'Hand0':
                selected = self.hand0_combo.get()
                if selected:
                    is_valid, value_str = self.validate_point(selected, 24)
                    if is_valid:
                        values = value_str.split(',')
                        point_list = [int(value.strip()) for value in values]
                        ret=robot.runtime_set_hand_pos(FXHandType.FX_HAND_LEFT, point_list)
                        if ret!= 0:
                            messagebox.showerror('Failed!', f"{obj} set run pose failed. Error msg: {robot._get_operate_error_msg(ret)}")
                            return
                    else:
                        messagebox.showerror("Error", f"Invalid format: {selected}")
                        return
                else:
                    messagebox.showwarning("Warning", f"No point selected for {obj}")
            elif obj == 'Hand1':
                selected = self.hand1_combo.get()
                if selected:
                    is_valid, value_str = self.validate_point(selected, 24)
                    if is_valid:
                        values = value_str.split(',')
                        point_list = [int(value.strip()) for value in values]
                        ret = robot.runtime_set_hand_pos(FXHandType.FX_HAND_RIGHT, point_list)
                        if ret != 0:
                            messagebox.showerror('Failed!', f"set {obj} run pose failed. Error msg: {robot._get_operate_error_msg(ret)}")
                            return
                    else:
                        messagebox.showerror("Error", f"Invalid format: {selected}")
                        return
                else:
                    messagebox.showwarning("Warning", f"No point selected for {obj}")
            else:
                messagebox.showwarning("Warning", f"Unknown object: {obj}")
        except Exception as e:
            messagebox.showerror('Error', f"Operation failed: {e}")

    def vel_acc_set(self, obj):
        try:
            if obj == 'Arm0':
                vel = int(self.left_speed_entry.get())
                acc = int(self.left_accel_entry.get())
                ret = robot.runtime_set_vel_ratio(FXObjType.OBJ_ARM0, vel)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set vel failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
                ret = robot.runtime_set_acc_ratio(FXObjType.OBJ_ARM0, acc)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set acc failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
            elif obj == 'Arm1':
                vel = int(self.right_speed_entry.get())
                acc = int(self.right_accel_entry.get())
                ret = robot.runtime_set_vel_ratio(FXObjType.OBJ_ARM1, vel)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set vel failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
                ret = robot.runtime_set_acc_ratio(FXObjType.OBJ_ARM1, acc)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set acc failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
            elif obj == 'Body':
                vel = int(self.body_speed_entry.get())
                acc = int(self.body_accel_entry.get())
                ret = robot.runtime_set_vel_ratio(FXObjType.OBJ_BODY, vel)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set vel failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
                ret = robot.runtime_set_acc_ratio(FXObjType.OBJ_BODY, acc)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set acc failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
            elif obj == 'Head':
                vel = int(self.head_speed_entry.get())
                acc = int(self.head_accel_entry.get())
                ret = robot.runtime_set_vel_ratio(FXObjType.OBJ_HEAD, vel)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set vel failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
                ret = robot.runtime_set_acc_ratio(FXObjType.OBJ_HEAD, acc)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set acc failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
            elif obj == 'Lift':
                vel = int(self.lift_speed_entry.get())
                acc = int(self.lift_accel_entry.get())
                ret = robot.runtime_set_vel_ratio(FXObjType.OBJ_LIFT, vel)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set vel failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
                ret = robot.runtime_set_acc_ratio(FXObjType.OBJ_LIFT, acc)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set acc failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
            else:
                raise ValueError(f"Unknown obj: {obj}")
        except Exception as e:
            messagebox.showerror('Error', f"Operation failed: {e}")

    def reset_error(self, obj):
        try:
            obj_ = self._obj_name_to_type(obj)
            ret, system_errorcode = robot.reset_error(obj_, 1000)
            if ret != 0:
                messagebox.showerror('Error', f"Reset {obj} failed: {system_errorcode}")
        except Exception as e:
            messagebox.showerror('Error', f"Reset failed: {e}")

    def idle_state(self, obj):
        try:
            obj_type = self._obj_name_to_type(obj)
            ret =robot.switch_to_idle(obj_type, 1000)
            if ret !=0:
                messagebox.showerror('Error', f"Set idle failed: {robot._get_operate_error_msg(ret)}")
        except Exception as e:
            messagebox.showerror('Error', f"Set idle failed: {e}")

    def cr_state(self, obj):
        if obj not in ('Arm0', 'Arm1'):
            messagebox.showerror('Error', f'Invalid obj: {obj}')
            return
        try:
            arm_idx = 0 if obj == 'Arm0' else 1
            ret=robot.switch_to_collab_release(arm_idx, 1000)
            if ret!=0:
                messagebox.showerror('Failed!', f'{obj} switch to collaborative release failed: {robot._get_operate_error_msg(ret)}')
        except Exception as e:
            messagebox.showerror('Error', f"CR failed: {e}")

    def position_state(self, obj):
        try:
            obj_type = self._obj_name_to_type(obj)
            if obj == 'Arm0':
                vel = int(self.left_speed_entry.get())
                acc = int(self.left_accel_entry.get())
            elif obj == 'Arm1':
                vel = int(self.right_speed_entry.get())
                acc = int(self.right_accel_entry.get())
            elif obj == 'Body':
                vel = int(self.body_speed_entry.get())
                acc = int(self.body_accel_entry.get())
            elif obj == 'Head':
                vel = int(self.head_speed_entry.get())
                acc = int(self.head_accel_entry.get())
            elif obj == 'Lift':
                vel = int(self.lift_speed_entry.get())
                acc = int(self.lift_accel_entry.get())
            else:
                vel = acc = 20
            ret=robot.switch_to_position_mode(obj_type, 1000, vel, acc)
            if ret!=0:
                messagebox.showerror('Error', f"{obj} switch to position state failed:\n {robot._get_operate_error_msg(ret)}")
        except Exception as e:
            messagebox.showerror('Error', f"Set position state failed: {e}")

    def get_current_pos(self, obj):
        try:
            pose = None
            if obj == 'Arm0':
                pose = self.rt["arms"][0]["fb"]["fb_pos"]
                if pose and len(pose) == 7:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.entry.delete(0, tk.END)
                    self.entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Arm0')
                    return
            elif obj == 'Arm1':
                pose = self.rt["arms"][1]["fb"]["fb_pos"]
                if pose and len(pose) == 7:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.entry1.delete(0, tk.END)
                    self.entry1.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Arm1')
            elif obj == 'Body':
                pose = self.rt["body"]["fb_pos"]
                if pose and len(pose) == 6:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.body_cmd_entry.delete(0, tk.END)
                    self.body_cmd_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Body')
                    return
            elif obj == 'Head':
                pose = self.rt["head"]["fb_pos"]
                if pose and len(pose) == 3:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.head_cmd_entry.delete(0, tk.END)
                    self.head_cmd_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Head')
                    return
            elif obj == 'Lift':
                pose = self.rt["lift"]["fb_pos"]
                if pose and len(pose) == 2:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.lift_cmd_entry.delete(0, tk.END)
                    self.lift_cmd_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Lift')
                    return
            else:
                messagebox.showerror('Error', f'Unknown object: {obj}')
        except (KeyError, IndexError, TypeError) as e:
            messagebox.showerror('Error', f'Failed to get joint positions: {e}')

    def add_pos(self, obj):
        if obj == 'Arm0':
            point_str = self.entry_var.get()
            num_points = 7
            points_list = self.points1
            combo = self.combo1
        elif obj == 'Arm1':
            point_str = self.entry1.get()
            num_points = 7
            points_list = self.points2
            combo = self.combo2
        elif obj == 'Body':
            point_str = self.body_cmd_entry.get()
            num_points = 6
            points_list = self.body_points
            combo = self.body_combo
        elif obj == 'Head':
            point_str = self.head_cmd_entry.get()
            num_points = 3
            points_list = self.head_points
            combo = self.head_combo
        elif obj == 'Lift':
            point_str = self.lift_cmd_entry.get()
            num_points = 2
            points_list = self.lift_points
            combo = self.lift_combo
        else:
            messagebox.showerror("Error", f"Unknown object: {obj}")
            return

        is_valid, result = self.validate_point(point_str, num_points)
        if not is_valid:
            messagebox.showwarning("Wrong inputs", result)
            return
        if self.is_duplicate_command(result, points_list):
            messagebox.showwarning("Duplicate point", f"This point already exists in {obj} list")
            return

        points_list.insert(0, result)
        self.update_comboboxes()

    def delete_pos(self, obj):
        if obj == 'Arm0':
            combo = self.combo1
            points_list = self.points1
        elif obj == 'Arm1':
            combo = self.combo2
            points_list = self.points2
        elif obj == 'Body':
            combo = self.body_combo
            points_list = self.body_points
        elif obj == 'Head':
            combo = self.head_combo
            points_list = self.head_points
        elif obj == 'Lift':
            combo = self.lift_combo
            points_list = self.lift_points
        else:
            messagebox.showerror("Error", f"Unknown object: {obj}")
            return

        selected_index = combo.current()
        if selected_index != -1 and selected_index < len(points_list):
            points_list.pop(selected_index)
            self.update_comboboxes()
        else:
            messagebox.showwarning("Warning", f"Please select a point to delete in {obj}")

    def update_comboboxes(self):
        self.combo1['values'] = self.points1
        self.combo2['values'] = self.points2
        self.body_combo['values'] = self.body_points
        self.head_combo['values'] = self.head_points
        self.lift_combo['values'] = self.lift_points
        self.hand0_combo['values']=self.hand_points1
        self.hand1_combo['values'] = self.hand_points2

        self._set_combo_selection(self.combo1, self.points1)
        self._set_combo_selection(self.combo2, self.points2)
        self._set_combo_selection(self.body_combo, self.body_points)
        self._set_combo_selection(self.head_combo, self.head_points)
        self._set_combo_selection(self.lift_combo, self.lift_points)
        self._set_combo_selection(self.hand0_combo, self.hand_points1)
        self._set_combo_selection(self.hand1_combo, self.hand_points2)

    def _set_combo_selection(self, combo, points_list):
        if points_list:
            combo.current(0)
        else:
            combo.set('')
        combo.update_idletasks()

    def run_pos(self, obj):
        try:
            if obj == 'Arm0':
                selected = self.combo1.get()
                if selected:
                    is_valid, value_str = self.validate_point(selected, 7)
                    if is_valid:
                        values = value_str.split(',')
                        point_list = [float(value.strip()) for value in values]
                        ret=robot.runtime_set_joint_pos_cmd(FXObjType.OBJ_ARM0, point_list)
                        if ret!= 0:
                            messagebox.showerror('Failed!', f"{obj} set run pose failed. Error msg: {robot._get_operate_error_msg(ret)}")
                            return
                        time.sleep(0.1)
                    else:
                        messagebox.showerror("Error", f"Invalid format: {selected}")
                        return
                else:
                    messagebox.showwarning("Warning", "No point selected for Arm0")
            elif obj == 'Arm1':
                selected = self.combo2.get()
                if selected:
                    is_valid, value_str = self.validate_point(selected, 7)
                    if is_valid:
                        values = value_str.split(',')
                        point_list = [float(value.strip()) for value in values]
                        ret = robot.runtime_set_joint_pos_cmd(FXObjType.OBJ_ARM1, point_list)
                        if ret != 0:
                            messagebox.showerror('Failed!', f"set {obj} run pose failed. Error msg: {robot._get_operate_error_msg(ret)}")
                            return
                        time.sleep(0.1)
                    else:
                        messagebox.showerror("Error", f"Invalid format: {selected}")
                        return
                else:
                    messagebox.showwarning("Warning", "No point selected for Arm1")
            elif obj == 'Body':
                selected = self.body_combo.get()
                if selected:
                    is_valid, value_str = self.validate_point(selected, 6)
                    if is_valid:
                        values = value_str.split(',')
                        point_list = [float(value.strip()) for value in values]
                        ret = robot.runtime_set_joint_pos_cmd(FXObjType.OBJ_BODY, point_list)
                        if ret != 0:
                            messagebox.showerror('Failed!', f"set {obj} run pose failed. Error msg: {robot._get_operate_error_msg(ret)}")
                            return
                        time.sleep(0.1)
                    else:
                        messagebox.showerror("Error", f"Invalid format: {selected}")
                        return
                else:
                    messagebox.showwarning("Warning", "No point selected for Body")
            elif obj == 'Head':
                selected = self.head_combo.get()
                if selected:
                    is_valid, value_str = self.validate_point(selected, 3)
                    if is_valid:
                        values = value_str.split(',')
                        point_list = [float(value.strip()) for value in values]
                        ret = robot.runtime_set_joint_pos_cmd(FXObjType.OBJ_HEAD, point_list)
                        if ret != 0:
                            messagebox.showerror('Failed!', f"set {obj} run pose failed. Error msg: {robot._get_operate_error_msg(ret)}")
                            return
                        time.sleep(0.1)
                    else:
                        messagebox.showerror("Error", f"Invalid format: {selected}")
                        return
                else:
                    messagebox.showwarning("Warning", "No point selected for Head")
            elif obj == 'Lift':
                selected = self.lift_combo.get()
                if selected:
                    is_valid, value_str = self.validate_point(selected, 2)
                    if is_valid:
                        values = value_str.split(',')
                        point_list = [float(value.strip()) for value in values]
                        ret = robot.runtime_set_joint_pos_cmd(FXObjType.OBJ_LIFT, point_list)
                        if ret != 0:
                            messagebox.showerror('Failed!', f"set {obj} run pose failed. Error msg: {robot._get_operate_error_msg(ret)}")
                            return
                        time.sleep(0.1)
                    else:
                        messagebox.showerror("Error", f"Invalid format: {selected}")
                        return
                else:
                    messagebox.showwarning("Warning", "No point selected for Lift")
            else:
                messagebox.showwarning("Warning", f"Unknown object: {obj}")
        except Exception as e:
            messagebox.showerror('Error', f"Operation failed: {e}")

    def validate_point(self, point_str, nums):
        try:
            point_str = point_str.strip()
            if not point_str:
                return False, "The input cannot be empty."
            values = point_str.split(',')
            if len(values) != nums:
                return False, f"Please enter {nums} comma-separated numbers"
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

    def tools_dialog(self):
        if not self.connected:
            messagebox.showerror('Error', "Please connect robot first!")
            return

        set_tools_dialog = tk.Toplevel(self.root)
        set_tools_dialog.title("Tools dynamics and kinematics parameter setting")
        set_tools_dialog.geometry("800x200")  # Reduced height to avoid extra space
        set_tools_dialog.configure(bg="white")
        set_tools_dialog.transient(self.root)
        set_tools_dialog.resizable(True, True)
        set_tools_dialog.grab_set()

        # Main container frame anchored to the top
        main_frame = tk.Frame(set_tools_dialog, bg='white')
        main_frame.pack(fill="both", expand=True, anchor='n')

        # Arm0 row
        arm0_row1 = tk.Frame(main_frame, bg="white")
        arm0_row1.pack(fill="x", pady=5, anchor='n')
        tk.Label(arm0_row1, text="Arm0", bg="#D8F4F3", width=5).pack(side="left", padx=(5, 5))
        tk.Label(arm0_row1, text="Dynamics:", width=10).pack(side="left", padx=(5, 0))
        tk.Entry(arm0_row1, textvariable=self.arm0_tool_dyn_entry, width=50).pack(side="left", padx=(5, 5))
        tk.Label(arm0_row1, text="Kinematics:", width=10).pack(side="left", padx=(5, 0))
        tk.Entry(arm0_row1, textvariable=self.arm0_tool_kine_entry, width=30).pack(side="left", padx=(5, 5))

        # Arm1 row
        arm1_row1 = tk.Frame(main_frame, bg="white")
        arm1_row1.pack(fill="x", pady=5, anchor='n')
        tk.Label(arm1_row1, text="Arm1", bg="#F4E4D8", width=5).pack(side="left", padx=(5, 5))
        tk.Label(arm1_row1, text="Dynamics:", width=10).pack(side="left", padx=(5, 0))
        tk.Entry(arm1_row1, textvariable=self.arm1_tool_dyn_entry, width=50).pack(side="left", padx=(5, 5))
        tk.Label(arm1_row1, text="Kinematics:", width=10).pack(side="left", padx=(5, 0))
        tk.Entry(arm1_row1, textvariable=self.arm1_tool_kine_entry, width=30).pack(side="left", padx=(5, 5))

        # Button row
        btn_row1 = tk.Frame(main_frame, bg="white")
        btn_row1.pack(pady=10, anchor='center')
        tk.Button(btn_row1, text="Set tools", width=10, font=("Arial", 11, "bold"), bg="#A2CD5A",
                  command=self.tools_set).pack(side="left", padx=10)

    def tools_set(self):
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        try:

            k0 = self.arm0_tool_kine_entry.get().strip()
            if not k0:
                messagebox.showerror("Error", "arm0 tools kinematics parameter cannot be empty!")
                return
            is_valid, result = self.validate_point(k0, 6)
            if not is_valid:
                messagebox.showerror("Error", f"arm0 tools Invalid kinematics format: {result}")
                return
            k0_list = [float(x) for x in result.split(',')]

            d0 = self.arm0_tool_dyn_entry.get().strip()
            if not d0:
                messagebox.showerror("Error", "arm0 tools dynamics parameter cannot be empty!")
                return
            is_valid, result = self.validate_point(d0, 10)
            if not is_valid:
                messagebox.showerror("Error", f"arm0 tools Invalid dynamics format: {result}")
                return
            d0_list = [float(x) for x in result.split(',')]

            k1 = self.arm1_tool_kine_entry.get().strip()
            if not k1:
                messagebox.showerror("Error", "arm1 tools kinematics parameter cannot be empty!")
                return
            is_valid, result = self.validate_point(k1, 6)
            if not is_valid:
                messagebox.showerror("Error", f"arm1 tools Invalid kinematics format: {result}")
                return
            k1_list = [float(x) for x in result.split(',')]

            d1 = self.arm1_tool_dyn_entry.get().strip()
            if not d1:
                messagebox.showerror("Error", "arm1 tools dynamics parameter cannot be empty!")
                return
            is_valid, result = self.validate_point(d1, 10)
            if not is_valid:
                messagebox.showerror("Error", f"arm1 tools Invalid dynamics format: {result}")
                return
            d1_list = [float(x) for x in result.split(',')]

            ret = robot.runtime_set_tool_kd(FXObjType.OBJ_ARM0, k0_list, d0_list)
            if ret != 0:
                messagebox.showerror('Failed!', f"set tools failed for arm0. Error msg: {robot._get_operate_error_msg(ret)}")
                return
            ret = robot.runtime_set_tool_kd(FXObjType.OBJ_ARM1, k1_list, d1_list)
            if ret != 0:
                messagebox.showerror('Failed!', f"set tools failed for arm1. Error msg: {robot._get_operate_error_msg(ret)}")
                return
        except Exception as e:
            messagebox.showerror('Error', str(e))

    def eef_dialog(self):
        if not self.connected:
            messagebox.showerror('Error', "Please connect robot first!")
            return

        drag_dialog = tk.Toplevel(self.root)
        drag_dialog.title("End-Effector Communication")
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
        self.eef_text_1 = tk.Button(self.eef_frame_1, text="Arm0 send", command=lambda: self.send_data_eef('Arm0'))
        self.eef_text_1.grid(row=0, column=0, padx=5, pady=5)

        self.com_text_1 = tk.Label(self.eef_frame_1, text="Channel", bg="white", width=5)
        self.com_text_1.grid(row=0, column=1, padx=5)

        self.com_select_combobox_1 = ttk.Combobox(
            self.eef_frame_1,
            values=["CAN", "COM1", "COM2"],
            width=5,
            state="readonly"
        )
        self.com_select_combobox_1.current(0)
        self.com_select_combobox_1.grid(row=0, column=2, padx=5)

        self.com_entry_1 = tk.Entry(self.eef_frame_1, width=120)
        self.com_entry_1.insert(0, "01 00 00 00 FF FF FF FF FF FF FF FC")
        self.com_entry_1.grid(row=0, column=4, padx=5, sticky="ew")

        self.eef_delet_1 = tk.Button(self.eef_frame_1, text="Delete", command=lambda: self.delete_eef_command('Arm0'))
        self.eef_delet_1.grid(row=0, column=3, padx=5, pady=5)

        self.eef_combo1 = ttk.Combobox(self.eef_frame_1, state="readonly", width=120)
        self.eef_combo1.grid(row=0, column=4, padx=5)

        self.eef_bt_1 = tk.Button(self.eef_frame_1, text="Arm0 receive", command=lambda: self.receive_data_eef('Arm0'))
        self.eef_bt_1.grid(row=0, column=5, padx=5)

        self.eef_frame_1_2 = tk.Frame(parent, bg="white")
        self.eef_frame_1_2.pack(fill="x")

        self.eef1_2_b1 = tk.Label(self.eef_frame_1_2, text="", bg="white", width=7)
        self.eef1_2_b1.grid(row=0, column=0, padx=5)

        self.eef1_2_b2 = tk.Label(self.eef_frame_1_2, text="", bg="white", width=7)
        self.eef1_2_b2.grid(row=0, column=1, padx=5)

        self.eef1_2_b3 = tk.Label(self.eef_frame_1_2, text="", bg="white", width=7)
        self.eef1_2_b3.grid(row=0, column=2, padx=5)

        self.eef_add_1 = tk.Button(self.eef_frame_1_2, text='Arm0 add', command=lambda: self.add_eef_command('Arm0'))
        self.eef_add_1.grid(row=0, column=3, padx=5)

        self.eef_entry = tk.Entry(self.eef_frame_1_2, width=120)
        self.eef_entry.insert(0, "01 06 00 00 00 01 48 0A")
        self.eef_entry.grid(row=0, column=4, padx=5, sticky="ew")

        self.eef_add_2 = tk.Button(self.eef_frame_1_2, text='Arm1 add', command=lambda: self.add_eef_command('Arm1'))
        self.eef_add_2.grid(row=0, column=5, padx=5)

        self.eef_frame_2 = tk.Frame(parent, bg="white")
        self.eef_frame_2.pack(fill="x")
        self.eef_bt_2 = tk.Button(self.eef_frame_2, text="Arm1 send", command=lambda: self.send_data_eef('Arm1'))
        self.eef_bt_2.grid(row=0, column=0, padx=5)

        self.com_text_2 = tk.Label(self.eef_frame_2, text="Channel", bg="white", width=5)
        self.com_text_2.grid(row=0, column=1, padx=5)

        self.com_select_combobox_2 = ttk.Combobox(
            self.eef_frame_2,
            values=["CAN", "COM1", "COM2"],
            width=5,
            state="readonly"
        )
        self.com_select_combobox_2.current(0)
        self.com_select_combobox_2.grid(row=0, column=2, padx=5)

        self.eef_delet_2 = tk.Button(self.eef_frame_2, text="Delete", command=lambda: self.delete_eef_command('Arm1'))
        self.eef_delet_2.grid(row=0, column=3, padx=5, pady=5)

        self.eef_combo2 = ttk.Combobox(self.eef_frame_2, state="readonly", width=120)
        self.eef_combo2.grid(row=0, column=4, padx=5)

        self.eef_bt_4 = tk.Button(self.eef_frame_2, text="Arm1 receive", command=lambda: self.receive_data_eef('Arm1'))
        self.eef_bt_4.grid(row=0, column=5, padx=5, pady=5)

        self.eef_frame_3 = tk.Frame(parent, bg="white")
        self.eef_frame_3.pack(fill="x")

        recv_label1 = tk.Label(self.eef_frame_3, text="Arm0 received:")
        recv_label1.grid(row=0, column=0, padx=5)

        spacer = tk.Label(self.eef_frame_3, text="   ", bg='white')
        spacer.grid(row=0, column=1, padx=5)

        self.recv_text1 = scrolledtext.ScrolledText(self.eef_frame_3, width=70, height=8, wrap=tk.WORD)
        self.recv_text1.grid(row=1, column=0, padx=5)
        self.recv_text1.insert(tk.END,
                               'Usage tips:\nFirst select the port: CAN/COM1/COM2,\nClick Arm0 receive button,\nEnter data to send, click Arm0 send button,\nReceived end-effector data is refreshed at 1kHz')

        spacer1 = tk.Label(self.eef_frame_3, text="   ", bg='white')
        spacer1.grid(row=1, column=1, padx=5)

        recv_label2 = tk.Label(self.eef_frame_3, text="Arm1 received:")
        recv_label2.grid(row=0, column=2, padx=5)

        self.recv_text2 = scrolledtext.ScrolledText(self.eef_frame_3, width=70, height=8, wrap=tk.WORD)
        self.recv_text2.grid(row=1, column=2, padx=5)
        self.recv_text2.insert(tk.END,
                               'Usage tips:\nFirst select the port: CAN/COM1/COM2,\nClick Arm1 receive button,\nEnter data to send, click Arm1 send button,\nReceived end-effector data is refreshed at 1kHz')

        status_display_frame_7 = tk.Frame(parent, bg="white", padx=10, pady=5)
        status_display_frame_7.pack(fill="x", pady=5)

    def is_duplicate_command(self, point_list, target_list):
        for existing_point_str in target_list:
            if existing_point_str == point_list:
                return True
        return False

    def add_eef_command(self, obj):
        command_str = self.eef_entry.get()
        if obj == 'Arm0':
            if self.is_duplicate_command(command_str, self.command1):
                messagebox.showwarning("Duplicate instruction", "This instruction already exists in left arm list.")
                return
            else:
                self.command1.insert(0, command_str)
        elif obj == 'Arm1':
            if self.is_duplicate_command(command_str, self.command2):
                messagebox.showwarning("Duplicate instruction", "This instruction already exists in right arm list.")
                return
            else:
                self.command2.insert(0, command_str)
        self.update_combo_eef()

    def update_combo_eef(self):
        self.eef_combo1['values'] = self.command1
        self.eef_combo2['values'] = self.command2
        if self.command1:
            self.eef_combo1.current(0)
        else:
            self.eef_combo1.set('')
        if self.command2:
            self.eef_combo2.current(0)
        else:
            self.eef_combo2.set('')

    def send_data_eef(self, obj):
        if not self.connected:
            messagebox.showerror('Error', 'Please connect robot')
            return
        try:
            com = 0
            com_str = ''
            sample_data = None
            if obj == 'Arm0':
                sample_data = self.eef_combo1.get()
                com_str = self.com_select_combobox_1.get()
                terminal = FXTerminalType.TERMINAL_ARM0
            elif obj == 'Arm1':
                sample_data = self.eef_combo2.get()
                com_str = self.com_select_combobox_2.get()
                terminal = FXTerminalType.TERMINAL_ARM1
            else:
                return

            if com_str == 'CAN':
                com = 1
            elif com_str == 'COM1':
                com = 2
            elif com_str == 'COM2':
                com = 3

            ret=robot.terminal_clear(terminal)
            if ret!= 0:
                messagebox.showerror('Failed', f'{obj} can not clear {com_str} buffer: {robot._get_operate_error_msg(ret)}')
                return
            time.sleep(0.01)
            ret=robot.terminal_set(terminal, com, sample_data)
            if ret!= 0:
                messagebox.showerror('Failed', f'{obj} set data to {com_str} failed: {robot._get_operate_error_msg(ret)}')
                return

            for _ in range(200):
                chn, data = robot.terminal_get(terminal)
                if chn > 0:
                    hex_str = data.hex().upper()
                    formatted_hex = ' '.join(hex_str[i:i + 2] for i in range(0, len(hex_str), 2))
                    if obj == 'Arm0':
                        self.recv_text1.delete('1.0', tk.END)
                        self.recv_text1.insert(tk.END, formatted_hex)
                    else:
                        self.recv_text2.delete('1.0', tk.END)
                        self.recv_text2.insert(tk.END, formatted_hex)
                    break
                time.sleep(0.005)
            else:
                if obj == 'Arm0':
                    self.recv_text1.delete('1.0', tk.END)
                    self.recv_text1.insert(tk.END, "No response")
                else:
                    self.recv_text2.delete('1.0', tk.END)
                    self.recv_text2.insert(tk.END, "No response")
        except Exception as e:
            messagebox.showerror('Error', str(e))

    def delete_eef_command(self, obj):
        if obj == 'Arm0':
            selected_index = self.eef_combo1.current()
            if selected_index != -1 and selected_index < len(self.command1):
                self.command1.pop(selected_index)
                self.update_combo_eef()
            else:
                messagebox.showwarning("Warning", "Please select a communication command to delete.")
        elif obj == 'Arm1':
            selected_index = self.eef_combo2.current()
            if selected_index != -1 and selected_index < len(self.command2):
                self.command2.pop(selected_index)
                self.update_combo_eef()
            else:
                messagebox.showwarning("Warning", "Please select a communication command to delete.")

    def receive_data_eef(self, obj):
        if not self.connected:
            messagebox.showerror('Error', 'Please connect robot')
            return
        try:
            terminal = FXTerminalType.TERMINAL_ARM0 if obj == 'Arm0' else FXTerminalType.TERMINAL_ARM1
            chn, data = robot.terminal_get(terminal)
            if chn > 0:
                text = data.decode(errors='replace')
                if obj == 'Arm0':
                    self.recv_text1.delete('1.0', tk.END)
                    self.recv_text1.insert(tk.END, text)
                else:
                    self.recv_text2.delete('1.0', tk.END)
                    self.recv_text2.insert(tk.END, text)
        except Exception as e:
            messagebox.showerror('Error', str(e))

    def imu_settings(self):
        settings_window = tk.Toplevel(self.root)
        settings_window.title("IMU calculations")
        settings_window.geometry("800x600")
        settings_window.configure(bg="#f0f0f0")
        settings_window.transient(self.root)
        settings_window.resizable(True, True)
        settings_window.grab_set()

        notebook = ttk.Notebook(settings_window)
        notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        floating_base_frame = ttk.Frame(notebook, padding="10")
        notebook.add(floating_base_frame, text="Floating base parameter calculation")

        self.create_floating_base_tab(floating_base_frame)

        button_frame = ttk.Frame(settings_window)
        button_frame.pack(pady=10)

        ttk.Button(button_frame, text="Save settings",
                   command=lambda: self.save_all_settings(notebook)).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Close",
                   command=settings_window.destroy).pack(side=tk.LEFT, padx=5)

    def create_floating_base_tab(self, parent):
        self.row2_selection = [0, 0, 0]
        self.row3_selection = [0, 0, 0]

        self.row2_var = tk.StringVar()
        self.row3_var = tk.StringVar()

        self.row2_var.trace('w', lambda *args: self.on_selection_change(2))
        self.row3_var.trace('w', lambda *args: self.on_selection_change(3))

        ttk.Label(parent, text="Floating base parameter calculation", font=("Arial", 14, "bold")).pack(pady=10)

        row1_frame = ttk.Frame(parent)
        row1_frame.pack(fill="x", pady=5)

        ttk.Label(row1_frame, text="The coordinate directions of the base (x-axis and y-axis)").pack(side="left",
                                                                                                     padx=5)
        ttk.Label(row1_frame,
                  text="UMI coordinate orientation (option to align base with UMI coordinate orientation)").pack(
            side="right", padx=5)

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

    def on_selection_change(self, changed_row):
        self.update_selection_lists()
        self.apply_mutual_exclusion(changed_row)
        if any(self.row2_selection) and any(self.row3_selection):
            result = self.get_abc_calculation()
            self.display_result(result)
        else:
            self.result_text.delete(1.0, tk.END)
            self.result_text.insert(tk.END,
                                    "Please complete the selections in both rows to view the calculation results.")

    def update_selection_lists(self):

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
        row2_val = self.row2_var.get()
        row3_val = self.row3_var.get()

        for btn in self.row2_buttons + self.row3_buttons:
            btn.state(["!disabled"])
        if row2_val:
            if row2_val in ["x", "-x"]:
                self.disable_axis_options(self.row3_buttons, ["x", "-x"])
            elif row2_val in ["y", "-y"]:
                self.disable_axis_options(self.row3_buttons, ["y", "-y"])
            elif row2_val in ["z", "-z"]:
                self.disable_axis_options(self.row3_buttons, ["z", "-z"])
        if row3_val:
            if row3_val in ["x", "-x"]:
                self.disable_axis_options(self.row2_buttons, ["x", "-x"])
            elif row3_val in ["y", "-y"]:
                self.disable_axis_options(self.row2_buttons, ["y", "-y"])
            elif row3_val in ["z", "-z"]:
                self.disable_axis_options(self.row2_buttons, ["z", "-z"])

    def disable_axis_options(self, buttons, options_to_disable):
        for btn in buttons:
            if btn['value'] in options_to_disable:
                btn.state(["disabled"])

    def main_function(self,vx, vy):
        def Matrix2ABC(m, abc):
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

        def FX_VectCross( a, b):
            result = [0.0] * 3
            result[0] = a[1] * b[2] - a[2] * b[1]
            result[1] = a[2] * b[0] - a[0] * b[2]
            result[2] = a[0] * b[1] - a[1] * b[0]
            return result

        def NormVect(a):
            return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])

        m_S = ""
        if NormVect(vx) < 0.01 or NormVect(vy) < 0.01:
            return m_S, [0, 0, 0]
        vz = FX_VectCross(vx, vy)
        vz_norm = NormVect(vz)
        if vz_norm < 0.99 or vz_norm > 1.01:
            return m_S, [0, 0, 0]
        m_mat = [
            [vx[0], vy[0], vz[0]],
            [vx[1], vy[1], vz[1]],
            [vx[2], vy[2], vz[2]]
        ]
        m_S += "Matrix form (column vectors are coordinate direction vectors):\n"
        m_S += f"{m_mat[0][0]:.2f}\t{m_mat[0][1]:.2f}\t{m_mat[0][2]:.2f}\n"
        m_S += f"{m_mat[1][0]:.2f}\t{m_mat[1][1]:.2f}\t{m_mat[1][2]:.2f}\n"
        m_S += f"{m_mat[2][0]:.2f}\t{m_mat[2][1]:.2f}\t{m_mat[2][2]:.2f}\n\n"
        m_abc = [0.0] * 3
        Matrix2ABC(m_mat, m_abc)
        m_S += f"ABC angles：[{m_abc[0]:.5f}, {m_abc[1]:.5f}, {m_abc[2]:.5f}]\n"
        return m_S

    def get_abc_calculation(self):
        result = f"The base coordinate direction is as follows during the rotation of the gyroscope IMU:\n"
        result += "=" * 20 + "\n"
        try:
            abc = self.main_function(self.row2_selection, self.row3_selection)
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

    def show_more_features(self):
        menu = tk.Menu(self.root, tearoff=0)
        menu.add_command(label="Tools Setting", command=self.tools_dialog)
        menu.add_separator()
        menu.add_command(label="CAN/485", command=self.eef_dialog)
        menu.add_separator()
        menu.add_command(label="IMU Calculation", command=self.imu_settings)
        menu.add_separator()
        menu.add_command(label="Sensors & Encoders", command=self.sensor_decoder_dialog)
        menu.add_separator()
        menu.add_command(label="Versions", command=self.servo_sensor_version_dialog)
        menu.add_separator()
        menu.add_command(label="Motion Planning", command=self.planning_dialog)
        # menu.add_separator()
        # menu.add_command(label="FileClient", command=self.file_client_dialog)
        menu.add_separator()
        menu.add_command(label="System Upgrade", command=self.system_update_dialog)
        # menu.add_separator()
        # menu.add_command(label="Docs", )#command=self.open_doc()
        try:
            menu.tk_popup(
                self.more_features_btn.winfo_rootx(),
                self.more_features_btn.winfo_rooty() + self.more_features_btn.winfo_height()
            )
        finally:
            menu.grab_release()

    def sensor_decoder_dialog(self):
        if not self.connected:
            messagebox.showerror('Error', "Please connect robot first!")
            return
        sensor_encoder_window = tk.Toplevel(self.root)
        sensor_encoder_window.title("Sensor and encoder functions")
        sensor_encoder_window.geometry("800x400")
        sensor_encoder_window.configure(bg="white")
        sensor_encoder_window.transient(self.root)
        sensor_encoder_window.resizable(True, True)
        sensor_encoder_window.grab_set()

        self.sensor_frame_2 = tk.Frame(sensor_encoder_window, bg="white")
        self.sensor_frame_2.pack(fill="x", padx=5, pady=(15, 10))
        self.sensor_main_tex = tk.Label(self.sensor_frame_2, text="Sensor offset reset", bg="#2196F3",
                                        fg="white", font=("Arial", 10, "bold"))
        self.sensor_main_tex.pack(fill='x', padx=(5, 20))

        self.sensor_frame_1 = tk.Frame(sensor_encoder_window, bg="white")
        self.sensor_frame_1.pack(fill="x")
        self.axis_text_ = tk.Label(self.sensor_frame_1, text="Arm0", bg="#D8F4F3")
        self.axis_text_.grid(row=0, column=0, padx=(5, 5))
        # resetoffset
        self.get_offset_btn_1 = tk.Button(self.sensor_frame_1, text="ResetOffset",
                                          command=lambda: self.clear_sensor_offset('Arm0'))
        self.get_offset_btn_1.grid(row=0, column=1, padx=(0, 20))

        self.axis_text__ = tk.Label(self.sensor_frame_1, text="Arm1", bg="#F4E4D8")
        self.axis_text__.grid(row=0, column=2, padx=(5, 5))

        self.get_offset_btn_2 = tk.Button(self.sensor_frame_1, text="ResetOffset",
                                          command=lambda: self.clear_sensor_offset('Arm1'))
        self.get_offset_btn_2.grid(row=0, column=3, padx=(0, 20))

        self.axis_text___ = tk.Label(self.sensor_frame_1, text="Body", bg="#B0C4DE")
        self.axis_text___.grid(row=0, column=4, padx=(5, 5))

        self.get_offset_btn_3 = tk.Button(self.sensor_frame_1, text="ResetOffset",
                                          command=lambda: self.clear_sensor_offset('Body'))
        self.get_offset_btn_3.grid(row=0, column=5, padx=5)

        '''encoder'''
        self.encoder_frame_1 = tk.Frame(sensor_encoder_window, bg="white")
        self.encoder_frame_1.pack(fill="x", padx=5, pady=(25, 10))
        self.encoder_frame_1 = tk.Label(self.encoder_frame_1, text="Motor encoder zeroing and error clearing",
                                        bg="#2196F3",
                                        fg="white", font=("Arial", 10, "bold"))
        self.encoder_frame_1.pack(fill='x')
        '''left arm'''
        self.motor_frame_1 = tk.Frame(sensor_encoder_window, bg="white")
        self.motor_frame_1.pack(fill="x")
        self.motor_text_1 = tk.Label(self.motor_frame_1, text="Arm0", bg="#D8F4F3", width=8)
        self.motor_text_1.grid(row=0, column=0, padx=(5, 5))

        self.motor_btn_1 = tk.Button(self.motor_frame_1, text="Motor encoder zeroing",
                                     command=lambda: self.clear_motor_as_zero('Arm0', self.motor_btn_1))
        self.motor_btn_1.grid(row=0, column=1, padx=5, pady=5)

        self.disable_soft_btn_1 = tk.Button(self.motor_frame_1, text="Disable SoftLimit",
                                            command=lambda: self.disable_soft_limit(FXObjType.OBJ_ARM0, 0xFF))
        self.disable_soft_btn_1.grid(row=0, column=2, padx=5)

        self.motor_btn_3 = tk.Button(self.motor_frame_1, text="Encoder clearing error", bg="#7ED2B4", state="disabled")
        # command=lambda: self.clear_motor_error('Arm0'))
        self.motor_btn_3.grid(row=0, column=3, padx=5)

        '''right arm'''
        self.motor_frame_2 = tk.Frame(sensor_encoder_window, bg="white")
        self.motor_frame_2.pack(fill="x")

        self.motor_text_1 = tk.Label(self.motor_frame_2, text="Arm1", bg="#F4E4D8", width=8)
        self.motor_text_1.grid(row=0, column=0, padx=(5, 5))

        self.motor_btn_11 = tk.Button(self.motor_frame_2, text="Motor encoder zeroing",
                                      command=lambda: self.clear_motor_as_zero('Arm1', self.motor_btn_11))
        self.motor_btn_11.grid(row=0, column=1, padx=5)

        self.disable_soft_btn_2 = tk.Button(self.motor_frame_2, text="Disable SoftLimit",
                                            command=lambda: self.disable_soft_limit(FXObjType.OBJ_ARM1, 0xFF))
        self.disable_soft_btn_2.grid(row=0, column=2, padx=5)

        self.motor_btn_31 = tk.Button(self.motor_frame_2, text="Encoder clearing error", bg="#7ED2B4", state="disabled")
        # command=lambda: self.clear_motor_error('Arm1'))
        self.motor_btn_31.grid(row=0, column=3, padx=5)

        '''body'''
        motor_frame_3 = tk.Frame(sensor_encoder_window, bg="white")
        motor_frame_3.pack(fill="x")
        motor_text_3 = tk.Label(motor_frame_3, text="Body", bg="#B0C4DE", width=8)
        motor_text_3.grid(row=0, column=0, padx=(5, 5))
        self.motor_btn_body = tk.Button(motor_frame_3, text="Motor encoder zeroing",
                                        command=lambda: self.clear_motor_as_zero('Body', self.motor_btn_body))
        self.motor_btn_body.grid(row=0, column=1, padx=5, pady=5)
        self.disable_soft_btn_body = tk.Button(motor_frame_3, text="Disable SoftLimit",
                                               command=lambda: self.disable_soft_limit(FXObjType.OBJ_BODY, 0xFF))
        self.disable_soft_btn_body.grid(row=0, column=3, padx=5)

        '''Head'''
        motor_frame_4 = tk.Frame(sensor_encoder_window, bg="white")
        motor_frame_4.pack(fill="x")
        motor_text_4 = tk.Label(motor_frame_4, text="Head", bg="#D8BFD8", width=8)
        motor_text_4.grid(row=0, column=0, padx=(5, 5))
        self.motor_btn_head = tk.Button(motor_frame_4, text="Motor encoder zeroing",
                                        command=lambda: self.clear_motor_as_zero('Head', self.motor_btn_head))
        self.motor_btn_head.grid(row=0, column=1, padx=5, pady=5)
        self.disable_soft_btn_head = tk.Button(motor_frame_4, text="Disable SoftLimit",
                                               command=lambda: self.disable_soft_limit(FXObjType.OBJ_HEAD, 0xFF))
        self.disable_soft_btn_head.grid(row=0, column=3, padx=5)

        '''Lift'''
        motor_frame_5 = tk.Frame(sensor_encoder_window, bg="white")
        motor_frame_5.pack(fill="x")
        motor_text_5 = tk.Label(motor_frame_5, text="Lift", bg="#FFFACD", width=8)
        motor_text_5.grid(row=0, column=0, padx=(5, 5))
        self.motor_btn_lift = tk.Button(motor_frame_5, text="Motor encoder zeroing",
                                        command=lambda: self.clear_motor_as_zero('Lift', self.motor_btn_lift))
        self.motor_btn_lift.grid(row=0, column=1, padx=5, pady=5)
        self.disable_soft_btn_lift = tk.Button(motor_frame_5, text="Disable SoftLimit",
                                               command=lambda: self.disable_soft_limit(FXObjType.OBJ_LIFT, 0x03))
        self.disable_soft_btn_lift.grid(row=0, column=3, padx=5)

    def servo_sensor_version_dialog(self):
        if not self.connected:
            messagebox.showerror('Error', "Please connect robot first!")
            return
        servo_sensor_version_window = tk.Toplevel(self.root)
        servo_sensor_version_window.title("Version")
        servo_sensor_version_window.geometry("1000x600")
        servo_sensor_version_window.configure(bg="white")
        servo_sensor_version_window.transient(self.root)
        servo_sensor_version_window.resizable(True, True)
        servo_sensor_version_window.grab_set()

        # ---------- System version section ----------
        sys_frame = tk.Frame(servo_sensor_version_window, bg="white")
        sys_frame.pack(fill="x", padx=5, pady=(15, 5))
        sys_label_title = tk.Label(sys_frame, text="System & SDK version", bg="#2196F3",
                                   fg="white", font=("Arial", 10, "bold"))
        sys_label_title.pack(fill='x', padx=(5, 20))

        row_sys = tk.Frame(sys_frame, bg="white")
        row_sys.pack(fill="x", padx=(5, 0), pady=2)
        label_sys = tk.Label(row_sys, text="System:", width=10, anchor='w', bg="white")
        label_sys.pack(side="left")
        self.system_version_label = tk.Label(row_sys, text="", bg="white", fg="black",
                                             font=("Arial", 9), anchor='w')
        self.system_version_label.pack(side="left", fill="x", expand=True, padx=5)

        label_sdk = tk.Label(row_sys, text="SDK:", width=10, anchor='w', bg="white")
        label_sdk.pack(side="left")
        self.sdk_version_label = tk.Label(row_sys, text="", bg="white", fg="black",
                                          font=("Arial", 9), anchor='w')
        self.sdk_version_label.pack(side="left", fill="x", expand=True, padx=5)

        # ---------- Servo version section ----------
        ss_frame1 = tk.Frame(servo_sensor_version_window, bg="white")
        ss_frame1.pack(fill="x", padx=5, pady=(15, 10))
        servo_text = tk.Label(ss_frame1, text="Servo version", bg="#2196F3",
                              fg="white", font=("Arial", 10, "bold"))
        servo_text.pack(fill='x', padx=(5, 20))

        devices_servo = ["Arm0", "Arm1", "Body", "Head"]
        self.servo_version_labels = {}
        for device in devices_servo:
            row_frame = tk.Frame(ss_frame1, bg="white")
            row_frame.pack(fill="x", padx=20, pady=2)
            label = tk.Label(row_frame, text=f"{device}:", width=10, anchor='w', bg="white")
            label.pack(side="left")
            val_label = tk.Label(row_frame, text="", bg="white", fg="black",
                                 font=("Arial", 9), anchor='w')
            val_label.pack(side="left", fill="x", expand=True, padx=5)
            self.servo_version_labels[device] = val_label

        # ---------- Sensor version & Serial section ----------
        ss_frame2 = tk.Frame(servo_sensor_version_window, bg="white")
        ss_frame2.pack(fill="x", padx=5, pady=(15, 10))
        sensor_text = tk.Label(ss_frame2, text="Sensor version & Serial", bg="#2196F3",
                               fg="white", font=("Arial", 10, "bold"))
        sensor_text.pack(fill='x', padx=(5, 20))

        devices_sensor = ["Arm0", "Arm1", "Body"]
        self.sensor_version_labels = {}
        self.sensor_serial_labels = {}
        for device in devices_sensor:
            row_ver = tk.Frame(ss_frame2, bg="white")
            row_ver.pack(fill="x", padx=20, pady=(2, 0))
            label_ver = tk.Label(row_ver, text=f"{device} sensor:", width=12, anchor='w', bg="white")
            label_ver.pack(side="left")
            ver_label = tk.Label(row_ver, text="", bg="white", fg="black",
                                 font=("Arial", 9), anchor='w')
            ver_label.pack(side="left", fill="x", expand=True, padx=5)
            self.sensor_version_labels[device] = ver_label

            row_ser = tk.Frame(ss_frame2, bg="white")
            row_ser.pack(fill="x", padx=20, pady=(0, 2))
            label_ser = tk.Label(row_ser, text=f"{device} serial:", width=12, anchor='w', bg="white")
            label_ser.pack(side="left")
            ser_label = tk.Label(row_ser, text="", bg="white", fg="black",
                                 font=("Arial", 9), anchor='w')
            ser_label.pack(side="left", fill="x", expand=True, padx=5)
            self.sensor_serial_labels[device] = ser_label

        # ---------- Physical state section (new) ----------
        phys_frame = tk.Frame(servo_sensor_version_window, bg="white")
        phys_frame.pack(fill="x", padx=5, pady=(15, 10))
        phys_title = tk.Label(phys_frame, text="Physical State", bg="#4CAF50",
                              fg="white", font=("Arial", 10, "bold"))
        phys_title.pack(fill='x', padx=(5, 20))

        devices_phys = ["Arm0", "Arm1", "Body", "Head", "Lift"]
        self.physical_state_labels = {}
        for device in devices_phys:
            row = tk.Frame(phys_frame, bg="white")
            row.pack(fill="x", padx=20, pady=2)
            label = tk.Label(row, text=f"{device}:", width=10, anchor='w', bg="white")
            label.pack(side="left")
            state_label = tk.Label(row, text="", bg="white", fg="black",
                                   font=("Arial", 9), anchor='w')
            state_label.pack(side="left", fill="x", expand=True, padx=5)
            self.physical_state_labels[device] = state_label

        self.system_version_label.config(text=getattr(self, 'sys_version', 'Not connected'))
        self.sdk_version_label.config(text=getattr(self, 'sdk_version', 'Not connected'))

        for device in self.physical_state_labels:
            self.physical_state_labels[device].config(
                text=getattr(self, 'physical_states', {}).get(device, 'Not connected'))

        for device in self.servo_version_labels:
            self.servo_version_labels[device].config(
                text=getattr(self, 'servo_versions', {}).get(device, 'Not connected'))
        for device in self.sensor_version_labels:
            self.sensor_version_labels[device].config(
                text=getattr(self, 'sensor_versions', {}).get(device, 'Not connected'))
        for device in self.sensor_serial_labels:
            self.sensor_serial_labels[device].config(
                text=getattr(self, 'sensor_serials', {}).get(device, 'Not connected'))

    def planning_dialog(self):
        if not self.connected:
            messagebox.showerror('Error', "Please connect robot first!")
            return
        if not messagebox.askyesno("Yes", "Motion planning must under position state"):
            return

        planning_window = tk.Toplevel(self.root)
        planning_window.title("Motion Planning")
        planning_window.geometry("1200x800")
        planning_window.configure(bg="white")
        planning_window.transient(self.root)
        planning_window.resizable(True, True)
        planning_window.grab_set()

        # Canvas + Scrollbar
        canvas = tk.Canvas(planning_window, bg="white", highlightthickness=0)
        scrollbar = tk.Scrollbar(planning_window, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        inner_frame = tk.Frame(canvas, bg="white")
        canvas.create_window((0, 0), window=inner_frame, anchor="nw", width=canvas.winfo_width())

        def configure_inner_frame(event):
            canvas.configure(scrollregion=canvas.bbox("all"))

        inner_frame.bind("<Configure>", configure_inner_frame)

        def configure_canvas(event):
            canvas.itemconfig(1, width=event.width)

        canvas.bind("<Configure>", configure_canvas)

        def on_mousewheel(event):
            canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        canvas.bind_all("<MouseWheel>", on_mousewheel)  # Windows
        canvas.bind_all("<Button-4>", lambda e: canvas.yview_scroll(-1, "units"))
        canvas.bind_all("<Button-5>", lambda e: canvas.yview_scroll(1, "units"))

        def unbind_mousewheel(event=None):
            canvas.unbind_all("<MouseWheel>")
            canvas.unbind_all("<Button-4>")
            canvas.unbind_all("<Button-5>")

        planning_window.bind("<Destroy>", unbind_mousewheel)

        joint_frame_1 = tk.Frame(inner_frame, bg="white")
        joint_frame_1.pack(fill="x", padx=5, pady=(5, 5))
        joint_title_text = tk.Label(joint_frame_1, text="Joint Space", bg="#2196F3",
                                    fg="white", font=("Arial", 10, "bold"))
        joint_title_text.pack(fill='x', padx=(5, 20))

        # JOINTS TO JOINTS
        func1_frame = ttk.LabelFrame(inner_frame, text="JOINTS TO JOINTS", padding=10,
                                     relief=tk.GROOVE, borderwidth=2, style="MyCustom.TLabelframe")
        func1_frame.pack(fill="x", padx=10, pady=(0, 5))

        arm0_row1 = tk.Frame(func1_frame, bg="white")
        arm0_row1.pack(fill="x", pady=2)
        tk.Label(arm0_row1, text="Arm0", bg="#D8F4F3", width=5).pack(side="left", padx=2)
        tk.Label(arm0_row1, text="Start joints", bg="white", width=10).pack(side="left", padx=2)
        self.joints_start_arm0_entry = tk.Entry(arm0_row1, width=50)
        self.joints_start_arm0_entry.pack(side="left", padx=2)
        self.joints_start_arm0_entry.insert(0, "0,0,0,0,0,0,0")
        tk.Button(arm0_row1, text="GetCur",
                  command=lambda: self.pln_get_cur_joints('Arm0')).pack(side="left",
                                                                        padx=2)
        tk.Label(arm0_row1, text="End joints", bg="white", width=10).pack(side="left", padx=2)
        self.joints_end_arm0_entry = tk.Entry(arm0_row1, width=50)
        self.joints_end_arm0_entry.pack(side="left", padx=2)
        self.joints_end_arm0_entry.insert(0, "17.470, -43.308, 11.804, -79.761, -10.700, -2.874, 9.134")

        arm1_row1 = tk.Frame(func1_frame, bg="white")
        arm1_row1.pack(fill="x", pady=2)
        tk.Label(arm1_row1, text="Arm1", bg="#F4E4D8", width=5).pack(side="left", padx=2)
        tk.Label(arm1_row1, text="Start joints", bg="white", width=10).pack(side="left", padx=2)
        self.joints_start_arm1_entry = tk.Entry(arm1_row1, width=50)
        self.joints_start_arm1_entry.pack(side="left", padx=2)
        self.joints_start_arm1_entry.insert(0, "0,0,0,0,0,0,0")
        tk.Button(arm1_row1, text="GetCur",
                  command=lambda: self.pln_get_cur_joints('Arm1')).pack(side="left",
                                                                        padx=2)
        tk.Label(arm1_row1, text="End joints", bg="white", width=10).pack(side="left", padx=2)
        self.joints_end_arm1_entry = tk.Entry(arm1_row1, width=50)
        self.joints_end_arm1_entry.pack(side="left", padx=2)
        self.joints_end_arm1_entry.insert(0, "-17.470, -43.308, -11.804, -79.761, 10.700, -2.874, -9.134")

        params_row1 = tk.Frame(func1_frame, bg="white")
        params_row1.pack(fill="x", pady=5)
        tk.Label(params_row1, text="Common Parameters:", bg="white", font=("Arial", 9, "bold")).pack(side="left",
                                                                                                     padx=10)
        tk.Label(params_row1, text="Freq", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.joints_freq_entry = tk.Entry(params_row1, width=6)
        self.joints_freq_entry.pack(side="left", padx=2)
        self.joints_freq_entry.insert(0, "50")
        tk.Label(params_row1, text="(1000%freq==0)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left",
                                                                                                    padx=(0, 5))

        tk.Label(params_row1, text="Vel", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.joints_vel_entry = tk.Entry(params_row1, width=6)
        self.joints_vel_entry.pack(side="left", padx=2)
        self.joints_vel_entry.insert(0, "0.1")
        tk.Label(params_row1, text="(0.01~1)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left", padx=(0, 5))

        tk.Label(params_row1, text="Acc", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.joints_acc_entry = tk.Entry(params_row1, width=6)
        self.joints_acc_entry.pack(side="left", padx=2)
        self.joints_acc_entry.insert(0, "0.1")
        tk.Label(params_row1, text="(0.01~1)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left", padx=2)

        btn_row1 = tk.Frame(func1_frame, bg="white")
        btn_row1.pack(pady=5, anchor="center")
        tk.Button(btn_row1, text="Clear params", width=10, font=("Arial", 11, "bold"), bg="#E6E6FA",
                  command=self.clear_joint_inputs).pack(side="left", padx=10)
        tk.Button(btn_row1, text="Run", width=10, font=("Arial", 11, "bold"), bg="#A2CD5A",
                  command=self.pln_run_joint_to_joint).pack(side="left", padx=10)
        tk.Button(btn_row1, text="Break", width=10, font=("Arial", 11, "bold"), bg="#FFF68F",
                  command=self.stop_motion).pack(side="left", padx=10)


        # ========== Cartesian Space =========
        cartesian_frame = tk.Frame(inner_frame, bg="white")
        cartesian_frame.pack(fill="x", padx=5, pady=(5, 5))
        cartesian_title = tk.Label(cartesian_frame, text="Cartesian Space", bg="#2196F3",
                                   fg="white", font=("Arial", 10, "bold"))
        cartesian_title.pack(fill='x', padx=(5, 20))


        # JOINTS TO JOINTS (linear motion)
        func2_frame = ttk.LabelFrame(cartesian_frame, text="Joints to joints (linear)", padding=10,
                                     relief=tk.GROOVE, borderwidth=2, style="MyCustom.TLabelframe")
        func2_frame.pack(fill="x", padx=10, pady=(10, 5))

        arm0_row2 = tk.Frame(func2_frame, bg="white")
        arm0_row2.pack(fill="x", pady=2)
        tk.Label(arm0_row2, text="Arm0", bg="#D8F4F3", width=5).pack(side="left", padx=2)
        tk.Label(arm0_row2, text="Start joints", bg="white", width=10).pack(side="left", padx=2)
        self.linear_start_arm0_entry = tk.Entry(arm0_row2, width=50)
        self.linear_start_arm0_entry.pack(side="left", padx=2)
        self.linear_start_arm0_entry.insert(0, "17.470, -43.308, 11.804, -79.761, -10.700, -2.874, 9.134")
        tk.Button(arm0_row2, text="GetCur", command=lambda: self.pln_get_cur_joints_linear('Arm0')).pack(side="left",
                                                                                                         padx=2)
        tk.Label(arm0_row2, text="End joints", bg="white", width=10).pack(side="left", padx=2)
        self.linear_end_arm0_entry = tk.Entry(arm0_row2, width=50)
        self.linear_end_arm0_entry.pack(side="left", padx=2)
        self.linear_end_arm0_entry.insert(0, "19.597, -32.480, 10.050, -58.939, -8.863, -33.821, 4.772")

        arm1_row2 = tk.Frame(func2_frame, bg="white")
        arm1_row2.pack(fill="x", pady=2)
        tk.Label(arm1_row2, text="Arm1", bg="#F4E4D8", width=5).pack(side="left", padx=2)
        tk.Label(arm1_row2, text="Start joints", bg="white", width=10).pack(side="left", padx=2)
        self.linear_start_arm1_entry = tk.Entry(arm1_row2, width=50)
        self.linear_start_arm1_entry.pack(side="left", padx=2)
        self.linear_start_arm1_entry.insert(0, "-17.470, -43.308, -11.804, -79.761, 10.700, -2.874, -9.134")
        tk.Button(arm1_row2, text="GetCur", command=lambda: self.pln_get_cur_joints_linear('Arm1')).pack(side="left",
                                                                                                         padx=2)
        tk.Label(arm1_row2, text="End joints", bg="white", width=10).pack(side="left", padx=2)
        self.linear_end_arm1_entry = tk.Entry(arm1_row2, width=50)
        self.linear_end_arm1_entry.pack(side="left", padx=2)
        self.linear_end_arm1_entry.insert(0, "-19.597,-32.480,-10.050,-58.939,8.863,-33.821,-4.772")

        # Freq, Vel, Acc）
        params_row2 = tk.Frame(func2_frame, bg="white")
        params_row2.pack(fill="x", pady=5)
        tk.Label(params_row2, text="Common Parameters:", bg="white", font=("Arial", 9, "bold")).pack(side="left",
                                                                                                     padx=10)
        tk.Label(params_row2, text="Freq", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.linear_freq_entry = tk.Entry(params_row2, width=6)
        self.linear_freq_entry.pack(side="left", padx=2)
        self.linear_freq_entry.insert(0, "50")
        tk.Label(params_row2, text="((1000%freq==0))", bg="white", fg="gray", font=("Arial", 7)).pack(side="left",
                                                                                                      padx=(0, 5))

        tk.Label(params_row2, text="Vel", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.linear_vel_entry = tk.Entry(params_row2, width=6)
        self.linear_vel_entry.pack(side="left", padx=2)
        self.linear_vel_entry.insert(0, "100")
        tk.Label(params_row2, text="(1-1000)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left", padx=(0, 5))

        tk.Label(params_row2, text="Acc", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.linear_acc_entry = tk.Entry(params_row2, width=6)
        self.linear_acc_entry.pack(side="left", padx=2)
        self.linear_acc_entry.insert(0, "100")
        tk.Label(params_row2, text="(1-1000)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left", padx=2)

        btn_row2 = tk.Frame(func2_frame, bg="white")
        btn_row2.pack(pady=5, anchor="center")
        tk.Button(btn_row2, text="Clear params", width=10, font=("Arial", 11, "bold"), bg="#E6E6FA",
                  command=self.clear_linear_inputs).pack(side="left", padx=10)
        tk.Button(btn_row2, text="Run", width=10, font=("Arial", 11, "bold"), bg="#A2CD5A",
                  command=self.pln_run_joint_to_joints_linear).pack(side="left", padx=10)
        tk.Button(btn_row2, text="Break", width=10, font=("Arial", 11, "bold"), bg="#FFF68F",
                  command=self.stop_motion).pack(side="left", padx=10)


        # ===Linear
        linear_frame = ttk.LabelFrame(cartesian_frame, text="Linear", padding=10,
                                      relief=tk.GROOVE, borderwidth=2, style="MyCustom.TLabelframe")
        linear_frame.pack(fill="x", padx=5, pady=(5, 5))
        # ARM0
        arm0_cart_row = tk.Frame(linear_frame, bg="white")
        arm0_cart_row.pack(fill="x", pady=2)
        tk.Label(arm0_cart_row, text="Arm0", bg="#D8F4F3", width=5).pack(side="left", padx=2)
        tk.Label(arm0_cart_row, text="Start XYZABC", bg="white", width=10).pack(side="left", padx=(5, 0))
        self.cart_start_arm0_entry = tk.Entry(arm0_cart_row, width=50)
        self.cart_start_arm0_entry.pack(side="left", padx=2)
        if robot.get_robot_type() == robot_type_map[1] or robot.get_robot_type() == robot_type_map[4]:
            self.cart_start_arm0_entry.insert(0, "447.829, 203.577, 336.036, -169.144, 55.011, -146.752")
        else:
            self.cart_start_arm0_entry.insert(0, "509.734, 233.609, 365.948, -169.144, 55.011, -146.752")
        tk.Button(arm0_cart_row, text="GetCur",
                  command=lambda: self.pln_get_cur_xyzabc('Arm0')).pack(side="left", padx=2)
        tk.Label(arm0_cart_row, text="End XYZABC", bg="white", width=10).pack(side="left", padx=(5, 0))
        self.cart_end_arm0_entry = tk.Entry(arm0_cart_row, width=50)
        self.cart_end_arm0_entry.pack(side="left", padx=2)
        if robot.get_robot_type() == robot_type_map[1] or robot.get_robot_type() == robot_type_map[4]:
            self.cart_end_arm0_entry.insert(0, "447.829, 203.577, 236.036, -169.144, 55.011, -146.752")
        else:
            self.cart_end_arm0_entry.insert(0, "509.734, 233.609, 265.948, -169.144, 55.011, -146.752")

        arm0_cart_row1 = tk.Frame(linear_frame, bg="white")
        arm0_cart_row1.pack(fill="x", pady=(2,10))
        tk.Label(arm0_cart_row1, text="Ref joints", bg="white", width=10).pack(side="left", padx=(50, 0))
        self.linear_ref_arm0_entry = tk.Entry(arm0_cart_row1, width=50)
        self.linear_ref_arm0_entry.pack(side="left", padx=2)
        self.linear_ref_arm0_entry.insert(0, "19.597, -32.480, 10.050, -58.939, -8.863, -33.821, 4.772")
        tk.Button(arm0_cart_row1, text="GetCur", command=lambda: self.pln_get_cur_joints_as_linear_ref('Arm0')).pack(
            side="left", padx=2)

        # ARM1
        arm1_cart_row = tk.Frame(linear_frame, bg="white")
        arm1_cart_row.pack(fill="x", pady=2)
        tk.Label(arm1_cart_row, text="Arm1", bg="#F4E4D8", width=5).pack(side="left", padx=2)
        tk.Label(arm1_cart_row, text="Start XYZABC", bg="white", width=10).pack(side="left", padx=(5, 0))
        self.cart_start_arm1_entry = tk.Entry(arm1_cart_row, width=50)
        self.cart_start_arm1_entry.pack(side="left", padx=2)
        if robot.get_robot_type() == robot_type_map[1] or robot.get_robot_type() == robot_type_map[4]:
            self.cart_start_arm1_entry.insert(0, "447.829, -203.577, 336.036, 169.144, 55.011, 146.752")
        else:
            self.cart_start_arm1_entry.insert(0, "509.734, -233.609, 365.948, 169.144, 55.011, 146.752")
        tk.Button(arm1_cart_row, text="GetCur",
                  command=lambda: self.pln_get_cur_xyzabc('Arm1')).pack(side="left", padx=2)
        tk.Label(arm1_cart_row, text="End XYZABC", bg="white", width=10).pack(side="left", padx=(5, 0))
        self.cart_end_arm1_entry = tk.Entry(arm1_cart_row, width=50)
        self.cart_end_arm1_entry.pack(side="left", padx=2)
        if robot.get_robot_type() == robot_type_map[1] or robot.get_robot_type() == robot_type_map[4]:
            self.cart_end_arm1_entry.insert(0, "447.829, -203.577, 236.036, 169.144, 55.011, 146.752")
        else:
            self.cart_end_arm1_entry.insert(0, "509.734, -233.609, 265.948, 169.144, 55.011, 146.752")

        arm1_cart_row1 = tk.Frame(linear_frame, bg="white")
        arm1_cart_row1.pack(fill="x", pady=(2,10))
        tk.Label(arm1_cart_row1, text="Ref joints", bg="white", width=10).pack(side="left", padx=(50, 0))
        self.linear_ref_arm1_entry = tk.Entry(arm1_cart_row1, width=50)
        self.linear_ref_arm1_entry.pack(side="left", padx=2)
        self.linear_ref_arm1_entry.insert(0, "-19.597, -32.480, -10.050, -58.939, 8.863, -33.821, -4.772")
        tk.Button(arm1_cart_row1, text="GetCur", command=lambda: self.pln_get_cur_joints_as_linear_ref('Arm1')).pack(
            side="left", padx=2)

        # freq vel acc
        cart_params_row = tk.Frame(linear_frame, bg="white")
        cart_params_row.pack(fill="x", pady=5)
        tk.Label(cart_params_row, text="Common Parameters:", bg="white", font=("Arial", 9, "bold")).pack(side="left",
                                                                                                         padx=10)
        tk.Label(cart_params_row, text="Freq", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.cart_freq_entry = tk.Entry(cart_params_row, width=6)
        self.cart_freq_entry.pack(side="left", padx=2)
        self.cart_freq_entry.insert(0, "50")
        tk.Label(cart_params_row, text="((1000%freq==0))", bg="white", fg="gray", font=("Arial", 7)).pack(side="left",
                                                                                                          padx=(0, 5))

        tk.Label(cart_params_row, text="Vel", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.cart_vel_entry = tk.Entry(cart_params_row, width=6)
        self.cart_vel_entry.pack(side="left", padx=2)
        self.cart_vel_entry.insert(0, "100")
        tk.Label(cart_params_row, text="(1-1000)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left",
                                                                                                  padx=(0, 5))

        tk.Label(cart_params_row, text="Acc", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.cart_acc_entry = tk.Entry(cart_params_row, width=6)
        self.cart_acc_entry.pack(side="left", padx=2)
        self.cart_acc_entry.insert(0, "100")
        tk.Label(cart_params_row, text="(1-1000)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left", padx=2)

        cart_btn_row = tk.Frame(linear_frame, bg="white")
        cart_btn_row.pack(pady=5, anchor="center")
        tk.Button(cart_btn_row, text="Clear params", width=10, font=("Arial", 11, "bold"), bg="#E6E6FA",
                  command=self.clear_linear_cart_inputs).pack(side="left", padx=10)
        tk.Button(cart_btn_row, text="Run", width=10, font=("Arial", 11, "bold"), bg="#A2CD5A",
                  command=self.pln_run_cartesian_linear).pack(side="left", padx=10)
        tk.Button(cart_btn_row, text="Break", width=10, font=("Arial", 11, "bold"), bg="#FFF68F",
                  command=self.stop_motion).pack(side="left", padx=10)

        # Linear (multi-segment)
        multi_seg_frame = ttk.LabelFrame(cartesian_frame, text="Linear (multi-segment)", padding=10,
                                         relief=tk.GROOVE, borderwidth=2, style="MyCustom.TLabelframe")
        multi_seg_frame.pack(fill="x", padx=5, pady=(10, 5))

        # ---------- Arm0 ----------
        arm0_multi_row = tk.Frame(multi_seg_frame, bg="white")
        arm0_multi_row.pack(fill="x", pady=2)
        tk.Label(arm0_multi_row, text="Arm0", bg="#D8F4F3", width=5).pack(side="left", padx=2)
        tk.Label(arm0_multi_row, text="start joints", bg="white", width=10).pack(side="left", padx=(5, 0))
        self.multi_start_joints_arm0_entry = tk.Entry(arm0_multi_row, width=50)
        self.multi_start_joints_arm0_entry.pack(side="left", padx=2)
        if robot.get_robot_type() == robot_type_map[1] or robot.get_robot_type() == robot_type_map[4]:
            self.multi_start_joints_arm0_entry.insert(0, "17.832, -35.817, 11.527, -75.747, -9.230, -14.070, 7.530")
        else:
            self.multi_start_joints_arm0_entry.insert(0, "17.970, -35.197, 11.414, -73.344, -9.154, -17.035, 7.086")
        tk.Button(arm0_multi_row, text="GetCur",
                  command=lambda: self.pln_get_cur_joints_as_ref('Arm0')).pack(side="left",
                                                                                                          padx=2)

        tk.Label(arm0_multi_row, text="Add XYZABC", bg="white", width=10).pack(side="left", padx=(10, 0))
        self.multi_add_xyzabc_arm0_entry = tk.Entry(arm0_multi_row, width=50)
        self.multi_add_xyzabc_arm0_entry.pack(side="left", padx=2)
        self.multi_add_xyzabc_arm0_entry.insert(0, "0,0,0,0,0,0")
        tk.Button(arm0_multi_row, text="Add",
                  command=lambda: self.add_multi_seg_point('Arm0')).pack(side="left", padx=2)

        arm0_multi_row1 = tk.Frame(multi_seg_frame, bg="white")
        arm0_multi_row1.pack(fill="x", pady=(2, 10))
        tk.Label(arm0_multi_row1, text="All points", bg="white", width=10).pack(side="left", padx=(50, 0))
        self.multi_points_arm0_combo = ttk.Combobox(arm0_multi_row1, width=50, state="readonly")
        self.multi_points_arm0_combo.pack(side="left", padx=2)

        if robot.get_robot_type() == robot_type_map[1] or robot.get_robot_type() == robot_type_map[4]:
            default_points0=[
            "447.833, 203.571, 236.037, -169.143, 55.012, -146.752", 
            "447.833, 203.571, 336.03700000000003, -169.143, 55.012, -146.752", 
            "447.833, 303.571, 336.03700000000003, -169.143, 55.012, -146.752", 
            "447.833, 303.571, 236.03700000000003, -169.143, 55.012, -146.752"
            ]
        else:
            default_points0 = [
                "509.731, 233.614, 265.949, -169.144, 55.011, -146.752",
                "509.731, 233.614, 65.949, -169.144, 55.011, -146.752",
                "509.731, 33.614, 65.949, -169.144, 55.011, -146.752",
                "509.731, 33.614, 265.949, -169.144, 55.011, -146.752"
            ]

        self.multi_points_arm0_list = default_points0.copy()
        self.multi_points_arm0_combo['values'] = tuple(self.multi_points_arm0_list)
        if self.multi_points_arm0_list:
            self.multi_points_arm0_combo.current(0)

            # ---------- Arm1 ----------
        arm1_multi_row = tk.Frame(multi_seg_frame, bg="white")
        arm1_multi_row.pack(fill="x", pady=2)
        tk.Label(arm1_multi_row, text="Arm1", bg="#F4E4D8", width=5).pack(side="left", padx=2)
        tk.Label(arm1_multi_row, text="start joints", bg="white", width=10).pack(side="left", padx=(5, 0))
        self.multi_start_joints_arm1_entry = tk.Entry(arm1_multi_row, width=50)
        self.multi_start_joints_arm1_entry.pack(side="left", padx=2)
        if robot.get_robot_type() == robot_type_map[1] or robot.get_robot_type() == robot_type_map[4]:
            self.multi_start_joints_arm1_entry.insert(0, "-17.832, -35.817, -11.527, -75.747, 9.230, -14.070, -7.530")
        else:
            self.multi_start_joints_arm1_entry.insert(0, "-17.970, -35.197, -11.414, -73.344, 9.154, -17.035, -7.086")
        tk.Button(arm1_multi_row, text="GetCur",
                  command=lambda: self.pln_get_cur_joints_as_ref('Arm1')).pack(side="left",
                                                                                                          padx=2)

        tk.Label(arm1_multi_row, text="Add XYZABC", bg="white", width=10).pack(side="left", padx=(10, 0))
        self.multi_add_xyzabc_arm1_entry = tk.Entry(arm1_multi_row, width=50)
        self.multi_add_xyzabc_arm1_entry.pack(side="left", padx=2)
        self.multi_add_xyzabc_arm1_entry.insert(0, "0,0,0,0,0,0")
        tk.Button(arm1_multi_row, text="Add",
                  command=lambda: self.add_multi_seg_point('Arm1')).pack(side="left", padx=2)
        arm1_multi_row1 = tk.Frame(multi_seg_frame, bg="white")
        arm1_multi_row1.pack(fill="x", pady=(2,10))
        tk.Label(arm1_multi_row1, text="All points", bg="white", width=10).pack(side="left", padx=(50, 0))
        self.multi_points_arm1_combo = ttk.Combobox(arm1_multi_row1, width=50, state="readonly")
        self.multi_points_arm1_combo.pack(side="left", padx=2)
        if robot.get_robot_type() == robot_type_map[1] or robot.get_robot_type() == robot_type_map[4]:   
            default_points1=[
            "447.833, -203.571, 236.037, 169.143, 55.012, 146.752", 
            "447.833, -203.571, 336.03700000000003, 169.143, 55.012, 146.752", 
            "447.833, -103.571, 336.03700000000003, 169.143, 55.012, 146.752", 
            "447.833, -103.571, 236.03700000000003, 169.143, 55.012, 146.752"
            ]
        else:
            default_points1 = [
                "509.731, -233.614, 265.949, 169.144, 55.011, 146.752",
                "509.731, -233.614, 65.949, 169.144, 55.011, 146.752",
                "509.731, -33.614, 65.949, 169.144, 55.011, 146.752",
                "509.731, -33.614, 265.949, 169.144, 55.011, 146.752"
            ]
        self.multi_points_arm1_list = default_points1.copy()
        self.multi_points_arm1_combo['values'] = tuple(self.multi_points_arm1_list)
        if self.multi_points_arm1_list:
            self.multi_points_arm1_combo.current(0)

        multi_params_row = tk.Frame(multi_seg_frame, bg="white")
        multi_params_row.pack(fill="x", pady=5)
        tk.Label(multi_params_row, text="Common Parameters:", bg="white", font=("Arial", 9, "bold")).pack(side="left",
                                                                                                          padx=10)
        tk.Label(multi_params_row, text="Freq:", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.multi_cart_freq_entry = tk.Entry(multi_params_row, width=6)
        self.multi_cart_freq_entry.pack(side="left", padx=2)
        self.multi_cart_freq_entry.insert(0, "50")
        tk.Label(multi_params_row, text="((1000%freq==0))", bg="white", fg="gray", font=("Arial", 7)).pack(side="left",
                                                                                                           padx=(0, 5))
        tk.Label(multi_params_row, text="Vel", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.multi_cart_vel_entry = tk.Entry(multi_params_row, width=6)
        self.multi_cart_vel_entry.pack(side="left", padx=2)
        self.multi_cart_vel_entry.insert(0, "100")
        tk.Label(multi_params_row, text="(1-1000)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left",
                                                                                                   padx=(0, 5))

        tk.Label(multi_params_row, text="Acc", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.multi_cart_acc_entry = tk.Entry(multi_params_row, width=6)
        self.multi_cart_acc_entry.pack(side="left", padx=2)
        self.multi_cart_acc_entry.insert(0, "100")
        tk.Label(multi_params_row, text="(1-1000)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left", padx=2)
        tk.Label(multi_params_row, text="Allow Range", bg="white", font=("Arial", 9)).pack(side="left",  padx=(5, 2))
        self.multi_allow_range_entry = tk.Entry(multi_params_row, width=3)
        self.multi_allow_range_entry.pack(side="left", padx=2)
        self.multi_allow_range_entry.insert(0, "5")
        tk.Label(multi_params_row, text="ZSP Type", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.multi_zsp_type_entry = tk.Entry(multi_params_row, width=3)
        self.multi_zsp_type_entry.pack(side="left", padx=2)
        self.multi_zsp_type_entry.insert(0, "1")
        tk.Label(multi_params_row, text="ZSP Params", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.multi_zsp_params_entry = tk.Entry(multi_params_row, width=20)
        self.multi_zsp_params_entry.pack(side="left", padx=2)
        self.multi_zsp_params_entry.insert(0, "0, 0, -1, 0, 0, 0")

        multi_btn_row = tk.Frame(multi_seg_frame, bg="white")
        multi_btn_row.pack(pady=5, anchor="center")
        tk.Button(multi_btn_row, text="Clear params", width=10, font=("Arial", 11, "bold"), bg="#E6E6FA",
                  command=self.clear_multi_segment_inputs).pack(side="left", padx=10)
        tk.Button(multi_btn_row, text="Run", width=10, font=("Arial", 11, "bold"), bg="#A2CD5A",
                  command=self.pln_run_multi_segment_linear).pack(side="left", padx=10)
        tk.Button(multi_btn_row, text="Break", width=10, font=("Arial", 11, "bold"), bg="#FFF68F",
                  command=self.stop_motion).pack(side="left", padx=10)

        # ====Co-arms
        syn_frame = ttk.LabelFrame(cartesian_frame, text="Arms Synchronous Linear", padding=10,
                                relief=tk.GROOVE, borderwidth=2, style="MyCustom.TLabelframe")
        syn_frame.pack(fill="x", padx=5, pady=(5, 5))
        # ARM0
        arm0_syn_row = tk.Frame(syn_frame, bg="white")
        arm0_syn_row.pack(fill="x", pady=2)
        tk.Label(arm0_syn_row, text="Arm0", bg="#D8F4F3", width=5).pack(side="left", padx=2)
        tk.Label(arm0_syn_row, text="Start joints", bg="white", width=10).pack(side="left", padx=2)
        self.syn_joints_arm0_entry = tk.Entry(arm0_syn_row, width=50)
        self.syn_joints_arm0_entry.pack(side="left", padx=2)
        self.syn_joints_arm0_entry.insert(0, "0.876, -25.548, -0.000, -87.472, -18.026, -7.201, -18.925")
        tk.Button(arm0_syn_row, text="GetCur", command=lambda: self.pln_syn_get_cur_joints('Arm0')).pack(side="left", padx=2)
        tk.Label(arm0_syn_row, text="Start XYZABC", bg="white", width=10).pack(side="left", padx=(5, 0))
        self.syn_start_arm0_entry = tk.Entry(arm0_syn_row, width=50)
        self.syn_start_arm0_entry.pack(side="left", padx=2)
        self.syn_start_arm0_entry.insert(0, "509.733, 33.610, 265.953, -169.144, 55.012, -146.752")
        tk.Button(arm0_syn_row, text="GetCur",
                  command=lambda: self.pln_syn_get_cur_xyzabc('Arm0')).pack(side="left", padx=2)

        arm0_syn_row1 = tk.Frame(syn_frame, bg="white")
        arm0_syn_row1.pack(fill="x", pady=(2,10))
        tk.Label(arm0_syn_row1, text="End XYZABC", bg="white", width=10).pack(side="left", padx=(50, 0))
        self.syn_end_arm0_entry = tk.Entry(arm0_syn_row1, width=50)
        self.syn_end_arm0_entry.pack(side="left", padx=(2,50))
        self.syn_end_arm0_entry.insert(0, "509.733, 233.610, 265.953, -169.144, 55.012, -146.752")
        tk.Label(arm0_syn_row1, text="ZSP Type", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.syn0_zsp_type_entry = tk.Entry(arm0_syn_row1, width=3)
        self.syn0_zsp_type_entry.pack(side="left", padx=2)
        self.syn0_zsp_type_entry.insert(0, "1")
        tk.Label(arm0_syn_row1, text="ZSP Params", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.syn0_zsp_params_entry = tk.Entry(arm0_syn_row1, width=20)
        self.syn0_zsp_params_entry.pack(side="left", padx=2)
        self.syn0_zsp_params_entry.insert(0, "0, 0, -1, 0, 0, 0")

        # ARM1
        arm1_syn_row = tk.Frame(syn_frame, bg="white")
        arm1_syn_row.pack(fill="x", pady=2)
        tk.Label(arm1_syn_row, text="Arm1", bg="#D8F4F3", width=5).pack(side="left", padx=2)
        tk.Label(arm1_syn_row, text="Start joints", bg="white", width=10).pack(side="left", padx=2)
        self.syn_joints_arm1_entry = tk.Entry(arm1_syn_row, width=50)
        self.syn_joints_arm1_entry.pack(side="left", padx=2)
        self.syn_joints_arm1_entry.insert(0, "-0.876, -25.548, 0.000, -87.472, 18.026, -7.201, 18.925")
        tk.Button(arm1_syn_row, text="GetCur", command=lambda: self.pln_syn_get_cur_joints('Arm1')).pack(side="left",
                                                                                                         padx=2)
        tk.Label(arm1_syn_row, text="Start XYZABC", bg="white", width=10).pack(side="left", padx=(5, 0))
        self.syn_start_arm1_entry = tk.Entry(arm1_syn_row, width=50)
        self.syn_start_arm1_entry.pack(side="left", padx=2)
        self.syn_start_arm1_entry.insert(0, "509.733, -33.610, 265.953, 169.144, 55.012, 146.752")
        tk.Button(arm1_syn_row, text="GetCur",
                  command=lambda: self.pln_syn_get_cur_xyzabc('Arm1')).pack(side="left", padx=2)

        arm1_syn_row1 = tk.Frame(syn_frame, bg="white")
        arm1_syn_row1.pack(fill="x", pady=(2,10))
        tk.Label(arm1_syn_row1, text="End XYZABC", bg="white", width=10).pack(side="left", padx=(50, 0))
        self.syn_end_arm1_entry = tk.Entry(arm1_syn_row1, width=50)
        self.syn_end_arm1_entry.pack(side="left", padx=(2,50))
        self.syn_end_arm1_entry.insert(0, "509.733, -233.610, 265.953, 169.144, 55.012, 146.752")

        tk.Label(arm1_syn_row1, text="ZSP Type", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.syn1_zsp_type_entry = tk.Entry(arm1_syn_row1, width=3)
        self.syn1_zsp_type_entry.pack(side="left", padx=2)
        self.syn1_zsp_type_entry.insert(0, "1")
        tk.Label(arm1_syn_row1, text="ZSP Params", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.syn1_zsp_params_entry = tk.Entry(arm1_syn_row1, width=20)
        self.syn1_zsp_params_entry.pack(side="left", padx=2)
        self.syn1_zsp_params_entry.insert(0, "0, 0, -1, 0, 0, 0")

        # freq vel acc
        syn_params_row = tk.Frame(syn_frame, bg="white")
        syn_params_row.pack(fill="x", pady=5)
        tk.Label(syn_params_row, text="Common Parameters:", bg="white", font=("Arial", 9, "bold")).pack(side="left",
                                                                                                         padx=10)
        tk.Label(syn_params_row, text="Freq", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.syn_freq_entry = tk.Entry(syn_params_row, width=6)
        self.syn_freq_entry.pack(side="left", padx=2)
        self.syn_freq_entry.insert(0, "50")
        tk.Label(cart_params_row, text="((1000%freq==0))", bg="white", fg="gray", font=("Arial", 7)).pack(side="left",
                                                                                                          padx=(0, 5))
        tk.Label(syn_params_row, text="Vel", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.syn_vel_entry = tk.Entry(syn_params_row, width=6)
        self.syn_vel_entry.pack(side="left", padx=2)
        self.syn_vel_entry.insert(0, "100")
        tk.Label(syn_params_row, text="(1-1000)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left",
                                                                                                  padx=(0, 5))
        tk.Label(syn_params_row, text="Acc", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.syn_acc_entry = tk.Entry(syn_params_row, width=6)
        self.syn_acc_entry.pack(side="left", padx=2)
        self.syn_acc_entry.insert(0, "100")
        tk.Label(syn_params_row, text="(1-1000)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left", padx=2)

        syn_btn_row = tk.Frame(syn_frame, bg="white")
        syn_btn_row.pack(pady=5, anchor="center")
        tk.Button(syn_btn_row, text="Clear params", width=10, font=("Arial", 11, "bold"), bg="#E6E6FA",
                  command=self.clear_syn_inputs).pack(side="left", padx=10)
        tk.Button(syn_btn_row, text="Run", width=10, font=("Arial", 11, "bold"), bg="#A2CD5A",
                  command=self.pln_run_syn).pack(side="left", padx=10)
        tk.Button(syn_btn_row, text="Break", width=10, font=("Arial", 11, "bold"), bg="#FFF68F",
                  command=self.stop_motion).pack(side="left", padx=10)

    def pln_get_cur_joints(self, obj):
        try:
            if obj == 'Arm0':
                pose = self.rt["arms"][0]["fb"]["fb_pos"]
                if pose and len(pose) == 7:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.joints_start_arm0_entry.delete(0, tk.END)
                    self.joints_start_arm0_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Arm0')
                    return
            elif obj == 'Arm1':
                pose = self.rt["arms"][1]["fb"]["fb_pos"]
                if pose and len(pose) == 7:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.joints_start_arm1_entry.delete(0, tk.END)
                    self.joints_start_arm1_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Arm1')
        except (KeyError, IndexError, TypeError) as e:
            messagebox.showerror('Error', f'Failed to get joint positions: {e}')

    def pln_get_cur_joints_as_ref(self,obj):
        try:
            if obj == 'Arm0':
                pose = self.rt["arms"][0]["fb"]["fb_pos"]
                if pose and len(pose) == 7:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.multi_start_joints_arm0_entry.delete(0, tk.END)
                    self.multi_start_joints_arm0_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Arm0')
            elif obj == 'Arm1':
                pose = self.rt["arms"][1]["fb"]["fb_pos"]
                if pose and len(pose) == 7:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.multi_start_joints_arm1_entry.delete(0, tk.END)
                    self.multi_start_joints_arm1_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Arm1')
        except (KeyError, IndexError, TypeError) as e:
            messagebox.showerror('Error', f'Failed to get joint positions: {e}')

    def pln_get_cur_joints_as_linear_ref(self,obj):
        try:
            if obj == 'Arm0':
                pose = self.rt["arms"][0]["fb"]["fb_pos"]
                if pose and len(pose) == 7:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.linear_ref_arm0_entry.delete(0, tk.END)
                    self.linear_ref_arm0_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Arm0')
            elif obj == 'Arm1':
                pose = self.rt["arms"][1]["fb"]["fb_pos"]
                if pose and len(pose) == 7:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.linear_ref_arm1_entry.delete(0, tk.END)
                    self.linear_ref_arm1_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Arm1')
        except (KeyError, IndexError, TypeError) as e:
            messagebox.showerror('Error', f'Failed to get joint positions: {e}')

    def pln_get_cur_joints_linear(self, obj):
        try:
            if obj == 'Arm0':
                pose = self.rt["arms"][0]["fb"]["fb_pos"]
                if pose and len(pose) == 7:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.linear_start_arm0_entry.delete(0, tk.END)
                    self.linear_start_arm0_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Arm0')
            elif obj == 'Arm1':
                pose = self.rt["arms"][1]["fb"]["fb_pos"]
                if pose and len(pose) == 7:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.linear_start_arm1_entry.delete(0, tk.END)
                    self.linear_start_arm1_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Arm1')
        except (KeyError, IndexError, TypeError) as e:
            messagebox.showerror('Error', f'Failed to get joint positions: {e}')

    def pln_syn_get_cur_joints(self,obj):
        try:
            if obj == 'Arm0':
                pose = self.rt["arms"][0]["fb"]["fb_pos"]
                if pose and len(pose) == 7:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.syn_joints_arm0_entry.delete(0, tk.END)
                    self.syn_joints_arm0_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Arm0')
            elif obj == 'Arm1':
                pose = self.rt["arms"][1]["fb"]["fb_pos"]
                if pose and len(pose) == 7:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.syn_joints_arm1_entry.delete(0, tk.END)
                    self.syn_joints_arm1_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Arm1')
        except (KeyError, IndexError, TypeError) as e:
            messagebox.showerror('Error', f'Failed to get joint positions: {e}')

    def clear_joint_inputs(self):
        for entry in [self.joints_start_arm0_entry, self.joints_end_arm0_entry,
                      self.joints_start_arm1_entry, self.joints_end_arm1_entry]:
            original = entry.get()
            if ',' in original:
                parts = original.split(',')
                zero_parts = []
                for part in parts:
                    zero_parts.append("0.0")
                entry.delete(0, tk.END)
                entry.insert(0, ', '.join(zero_parts))
            else:
                    entry.delete(0, tk.END)
                    entry.insert(0, "0.0")

    def pln_run_joint_to_joint(self):
        start_str0 = self.joints_start_arm0_entry.get().strip()
        end_str0 = self.joints_end_arm0_entry.get().strip()
        start_str1 = self.joints_start_arm1_entry.get().strip()
        end_str1 = self.joints_end_arm1_entry.get().strip()

        def parse_joints(s):
            parts = s.split(',')
            if len(parts) != 7:
                raise ValueError(f"need 7values, actual:{len(parts)}")
            return [float(p.strip()) for p in parts]

        try:
            start0 = parse_joints(start_str0)
            end0 = parse_joints(end_str0)
            start1 = parse_joints(start_str1)
            end1 = parse_joints(end_str1)
        except ValueError as e:
            messagebox.showerror("joints error", f"parse joints failed: {e}")
            return
        try:
            vel = float(self.joints_vel_entry.get().strip())
            acc = float(self.joints_acc_entry.get().strip())
            freq = int(self.joints_freq_entry.get().strip())
        except ValueError:
            messagebox.showerror("value error", "all parameters must be number")
            return
        if not (0.01 <= vel <= 1):
            messagebox.showerror("value error", "vel range [0.01,1]")
            return
        if not (0.01 <= acc <= 1):
            messagebox.showerror("value error", "acc range [0.01,1]")
            return
        if freq <= 0 or 1000 % freq != 0:
            messagebox.showerror("value error", "1000%freq==0 and fraq>0")
            return

        is_zero0 = all(abs(v) < 1e-6 for v in start0) and all(abs(v) < 1e-6 for v in end0)
        is_zero1 = all(abs(v) < 1e-6 for v in start1) and all(abs(v) < 1e-6 for v in end1)

        if is_zero0 and is_zero1:
            messagebox.showerror("value error", "start and end in same pose, can not run planning")
            return

        if not is_zero0 and is_zero1:
            ret = robot.plan_joints(0, start0, end0, vel, acc, freq)
            if isinstance(ret, tuple):
                raw_array, point_num = ret
                ret1=robot.config_set_traj(FXObjType.OBJ_ARM0, raw_array, point_num)
                if ret1!= 0:
                    messagebox.showerror("Failed!", f"Arm0 send planning points failed: {robot._get_operate_error_msg(ret1)}")
                    return
            else:
                messagebox.showerror("Error", f"Arm0 planning failed, error msg: {robot._get_operate_error_msg(ret)}")
                return


            mask = FXObjMask.OBJ_ARM0_FLAG
            ret_mask=robot.runtime_run_traj(mask)
            if ret_mask != mask:
                messagebox.showerror('Failed!', f"Run planning trajectory failed for arm0. Return mask: {ret_mask}")
                return

        if not is_zero1 and is_zero0:
            ret = robot.plan_joints(1, start1, end1, vel, acc, freq)
            if isinstance(ret, tuple):
                raw_array, point_num = ret
                ret1= robot.config_set_traj(FXObjType.OBJ_ARM1, raw_array, point_num)
                if ret1 != 0:
                    messagebox.showerror("Failed!", f"Arm1 send planning points failed: {robot._get_operate_error_msg(ret1)}")
                    return
            else:
                messagebox.showerror("Error", f"Arm1 planning failed, error msg: {robot._get_operate_error_msg(ret)}")
                return

            mask = FXObjMask.OBJ_ARM1_FLAG
            ret_mask=robot.runtime_run_traj(mask)
            if ret_mask != mask:
                messagebox.showerror('Failed!', f"Run planning trajectory failed for arm1. Return mask: {ret_mask}")
                return

        if not is_zero0 and not is_zero1:
            points0 = robot.plan_joints(0, start0, end0, vel, acc, freq)
            if isinstance(points0, tuple):
                ret=robot.config_set_traj(FXObjType.OBJ_ARM0, points0[0],points0[1])
                if ret!=0:
                    messagebox.showerror("Failed!", f"Arm0 send planning points failed: {robot._get_operate_error_msg(ret)}")
                    return
            else:
                messagebox.showerror("Error", f"Arm0 planning failed, error msg: {robot._get_operate_error_msg(points0)}")
                return

            points1 = robot.plan_joints(1, start1, end1, vel, acc, freq)
            if isinstance(points1, tuple):
                ret=robot.config_set_traj(FXObjType.OBJ_ARM1, points1[0],points1[1])
                if ret!=0:
                    messagebox.showerror("Failed!", f"Arm1 send Planning points failed: {robot._get_operate_error_msg(ret)}")
                    return
            else:
                messagebox.showerror("Error", f"Arm1 planning failed, error msg: {robot._get_operate_error_msg(points1)}")
                return

            mask=FXObjMask.OBJ_ARM0_FLAG | FXObjMask.OBJ_ARM1_FLAG
            ret_mask=robot.runtime_run_traj(mask)
            if ret_mask!=mask:
                messagebox.showerror('Failed!', f"Run planning trajectory failed for arm0 & arm1. Return mask: {ret_mask}")
                return

    def clear_linear_inputs(self):
        for entry in [self.linear_start_arm0_entry, self.linear_end_arm0_entry,
                      self.linear_start_arm1_entry, self.linear_end_arm1_entry]:
            original = entry.get()
            if ',' in original:
                parts = original.split(',')
                zero_parts = []
                for part in parts:
                    zero_parts.append("0.0")
                entry.delete(0, tk.END)
                entry.insert(0, ', '.join(zero_parts))
            else:
                entry.delete(0, tk.END)
                entry.insert(0, "0.0")

    def pln_run_joint_to_joints_linear(self):
        start_str0 = self.linear_start_arm0_entry.get().strip()
        end_str0 = self.linear_end_arm0_entry.get().strip()
        start_str1 = self.linear_start_arm1_entry.get().strip()
        end_str1 = self.linear_end_arm1_entry.get().strip()

        def parse_joints(s):
            parts = s.split(',')
            if len(parts) != 7:
                raise ValueError(f"need 7 values, actual:{len(parts)}")
            return [float(p.strip()) for p in parts]

        try:
            start0 = parse_joints(start_str0)
            end0 = parse_joints(end_str0)

            start1 = parse_joints(start_str1)
            end1 = parse_joints(end_str1)
        except ValueError as e:
            messagebox.showerror("joints error", f"parse joints failed: {e}")
            return
        try:
            vel = float(self.linear_vel_entry.get().strip())
            acc = float(self.linear_acc_entry.get().strip())
            freq = int(self.linear_freq_entry.get().strip())
        except ValueError:
            messagebox.showerror("value error", "all parameters must be number")
            return
        if not (1 <= vel <= 1000):
            messagebox.showerror("value error", "vel range [1,1000]")
            return
        if not (1 <= acc <= 1000):
            messagebox.showerror("value error", "acc range [1,1000]")
            return
        if freq <= 0 or 1000 % freq != 0:
            messagebox.showerror("value error", "1000%freq==0 and fraq>0")
            return

        is_zero0 = all(abs(v) < 1e-6 for v in start0) and all(abs(v) < 1e-6 for v in end0)
        is_zero1 = all(abs(v) < 1e-6 for v in start1) and all(abs(v) < 1e-6 for v in end1)

        if is_zero0 and is_zero1:
            messagebox.showerror("value error", "start and end in same pose, can not run planning")
            return

        if not is_zero0 and is_zero1:
            ret = robot.plan_linear_keep_joints(0, start0, end0, vel, acc, freq)
            if isinstance(ret, tuple):
                raw_array, point_num = ret
                ret1=robot.config_set_traj(FXObjType.OBJ_ARM0, raw_array, point_num)
                if ret1!= 0:
                    messagebox.showerror("Failed!", f"Arm0 send planning points failed: {robot._get_operate_error_msg(ret1)}")
                    return
            else:
                messagebox.showerror("Error", f"Arm0 planning failed, error msg: {robot._get_operate_error_msg(ret)}")
                return

            mask = FXObjMask.OBJ_ARM0_FLAG
            ret_mask=robot.runtime_run_traj(mask)
            if ret_mask != mask:
                messagebox.showerror('Failed!', f"Run planning trajectory failed for arm0. Return mask: {ret_mask}")
                return

        if not is_zero1 and is_zero0:
            ret = robot.plan_linear_keep_joints(1, start1, end1, vel, acc, freq)
            if isinstance(ret, tuple):
                raw_array, point_num = ret
                ret1=robot.config_set_traj(FXObjType.OBJ_ARM1, raw_array, point_num)
                if ret1!= 0:
                    messagebox.showerror("Failed!", f"Arm1 send planning points failed: {robot._get_operate_error_msg(ret1)}")
                    return
            else:
                messagebox.showerror("Error", f"Arm1 Planning failed, error msg: {robot._get_operate_error_msg(ret)}")
                return


            mask = FXObjMask.OBJ_ARM1_FLAG
            ret_mask=robot.runtime_run_traj(mask)
            if ret_mask != mask:
                messagebox.showerror('Failed!', f"Run planning trajectory failed for arm1. Return mask: {ret_mask}")
                return

        if not is_zero0 and not is_zero1:
            points0 = robot.plan_linear_keep_joints(0, start0, end0, vel, acc, freq)
            if isinstance(points0, tuple):
                ret=robot.config_set_traj(FXObjType.OBJ_ARM0, points0[0], points0[1])
                if ret!= 0:
                    messagebox.showerror("Failed!", f"Arm0 send planning points failed: {robot._get_operate_error_msg(ret)}")
                    return
            else:
                messagebox.showerror("Error", f"Arm0 planning failed, error msg: {robot._get_operate_error_msg(points0)}")
                return

            points1 = robot.plan_linear_keep_joints(1, start1, end1, vel, acc, freq)
            if isinstance(points1, tuple):
                ret=robot.config_set_traj(FXObjType.OBJ_ARM1, points1[0], points1[1])
                if ret!= 0:
                    messagebox.showerror("Failed!", f"Arm1 send planning points failed: {robot._get_operate_error_msg(ret)}")
                    return
            else:
                messagebox.showerror("Error", f"Arm1 planning failed, error msg: {robot._get_operate_error_msg(points1)}")
                return

            mask = FXObjMask.OBJ_ARM0_FLAG | FXObjMask.OBJ_ARM1_FLAG
            ret_mask = robot.runtime_run_traj(mask)
            if ret_mask != mask:
                messagebox.showerror('Failed!', f"Run planning trajectory failed for arm0 & arm1. Return mask: {ret_mask}")
                return

    def pln_get_cur_xyzabc(self, obj):
        try:
            if obj == 'Arm0':
                pose = self.rt["arms"][0]["fb"]["fb_pos"]
                arm0_joints = robot.forward_kinematics(0, pose)
                arm0_xyzabc = robot.matrix2xyzabc(arm0_joints)

                if arm0_xyzabc and len(arm0_xyzabc) == 6:
                    arm0_xyzabc_text = ", ".join(f"{v:.3f}" for v in arm0_xyzabc)
                    self.cart_start_arm0_entry.delete(0, tk.END)
                    self.cart_start_arm0_entry.insert(0, arm0_xyzabc_text)
                else:
                    messagebox.showerror('Error', 'Invalid xyzabc data for Arm0')

            elif obj == 'Arm1':
                pose = self.rt["arms"][1]["fb"]["fb_pos"]
                arm1_joints = robot.forward_kinematics(1, pose)
                arm1_xyzabc = robot.matrix2xyzabc(arm1_joints)
                if arm1_xyzabc and len(arm1_xyzabc) == 6:
                    arm1_xyzabc_text = ", ".join(f"{v:.3f}" for v in arm1_xyzabc)
                    self.cart_start_arm1_entry.delete(0, tk.END)
                    self.cart_start_arm1_entry.insert(0, arm1_xyzabc_text)
                else:
                    messagebox.showerror('Error', 'Invalid xyzabc data for Arm1')
        except (KeyError, IndexError, TypeError) as e:
            messagebox.showerror('Error', f'Failed to get xyzabc positions: {e}')

    def pln_syn_get_cur_xyzabc(self,obj):
        try:
            if obj == 'Arm0':
                pose = self.rt["arms"][0]["fb"]["fb_pos"]
                arm0_joints = robot.forward_kinematics(0, pose)
                arm0_xyzabc = robot.matrix2xyzabc(arm0_joints)

                if arm0_xyzabc and len(arm0_xyzabc) == 6:
                    arm0_xyzabc_text = ", ".join(f"{v:.3f}" for v in arm0_xyzabc)
                    self.syn_start_arm0_entry.delete(0, tk.END)
                    self.syn_start_arm0_entry.insert(0, arm0_xyzabc_text)
                else:
                    messagebox.showerror('Error', 'Invalid xyzabc data for Arm0')

            elif obj == 'Arm1':
                pose = self.rt["arms"][1]["fb"]["fb_pos"]
                arm1_joints = robot.forward_kinematics(1, pose)
                arm1_xyzabc = robot.matrix2xyzabc(arm1_joints)
                if arm1_xyzabc and len(arm1_xyzabc) == 6:
                    arm1_xyzabc_text = ", ".join(f"{v:.3f}" for v in arm1_xyzabc)
                    self.syn_start_arm1_entry.delete(0, tk.END)
                    self.syn_start_arm1_entry.insert(0, arm1_xyzabc_text)
                else:
                    messagebox.showerror('Error', 'Invalid xyzabc data for Arm1')
        except (KeyError, IndexError, TypeError) as e:
            messagebox.showerror('Error', f'Failed to get xyzabc positions: {e}')

    def clear_linear_cart_inputs(self):
        for entry in [self.cart_start_arm0_entry, self.cart_end_arm0_entry, self.linear_ref_arm0_entry,
                      self.cart_start_arm1_entry, self.cart_end_arm1_entry, self.linear_ref_arm1_entry]:
            original = entry.get()
            if ',' in original:
                parts = original.split(',')
                zero_parts = []
                for part in parts:
                    zero_parts.append("0.0")
                entry.delete(0, tk.END)
                entry.insert(0, ', '.join(zero_parts))
            else:
                entry.delete(0, tk.END)
                entry.insert(0, "0.0")

    def pln_run_cartesian_linear(self):
        start_str0 = self.cart_start_arm0_entry.get().strip()
        end_str0 = self.cart_end_arm0_entry.get().strip()
        start_str1 = self.cart_start_arm1_entry.get().strip()
        end_str1 = self.cart_end_arm1_entry.get().strip()
        ref_joints0=self.linear_ref_arm0_entry.get().strip()
        ref_joints1 = self.linear_ref_arm1_entry.get().strip()

        def parse_joints(s,num):
            parts = s.split(',')
            if len(parts) != num:
                raise ValueError(f"need {num} values, actual:{len(parts)}")
            return [float(p.strip()) for p in parts]

        try:
            start0 = parse_joints(start_str0,6)
            end0 = parse_joints(end_str0,6)

            start1 = parse_joints(start_str1,6)
            end1 = parse_joints(end_str1,6)

            ref0=parse_joints(ref_joints0,7)
            ref1=parse_joints(ref_joints1,7)

        except ValueError as e:
            messagebox.showerror("xyzabc/ref_joints error", f"parse xyzabc/ref_joints failed: {e}")
            return
        try:
            vel = float(self.cart_vel_entry.get().strip())
            acc = float(self.cart_acc_entry.get().strip())
            freq = int(self.cart_freq_entry.get().strip())
        except ValueError:
            messagebox.showerror("value error", "all parameters must be number")
            return
        if vel <= 0:
            messagebox.showerror("value error", "vel > 0")
            return
        if acc <= 0:
            messagebox.showerror("value error", "acc > 0")
            return
        if freq <= 0 or 1000 % freq != 0:
            messagebox.showerror("value error", "1000%freq==0 and fraq>0")
            return

        if all(v == 0 for v in ref0):
            messagebox.showwarning('error',"reference joints can not be all zero")
            return
        if all(v == 0 for v in ref1):
            messagebox.showwarning('error',"reference joints can not be all zero")
            return

        is_zero0 = all(abs(v) < 1e-6 for v in start0) and all(abs(v) < 1e-6 for v in end0)
        is_zero1 = all(abs(v) < 1e-6 for v in start1) and all(abs(v) < 1e-6 for v in end1)

        if is_zero0 and is_zero1:
            messagebox.showerror("value error", "start and end in same pose, can not run planning")
            return

        if not is_zero0 and is_zero1:
            points = robot.plan_linear(0, start0, end0,ref0, vel, acc, freq)
            if isinstance(points, tuple):
                ret=robot.config_set_traj(FXObjType.OBJ_ARM0, points[0],points[1])
                if ret!=0:
                    messagebox.showerror("Failed!", f"Arm0 send planning points failed: {robot._get_operate_error_msg(ret)}")
                    return
            else:
                messagebox.showerror("Error",
                                     f"Arm0 planning failed, error msg: {robot._get_operate_error_msg(points)}")
                return


            mask = FXObjMask.OBJ_ARM0_FLAG
            ret_mask=robot.runtime_run_traj(mask)
            if ret_mask != mask:
                messagebox.showerror('Failed!', f"Run planning trajectory failed for arm0. Return mask: {ret_mask}")
                return

        if not is_zero1 and is_zero0:
            points = robot.plan_linear(1, start1, end1,ref1, vel, acc, freq)
            if isinstance(points,tuple):
                ret=robot.config_set_traj(FXObjType.OBJ_ARM1, points[0],points[1])
                if ret!=0:
                    messagebox.showerror("Failed!", f"Arm1 send planning points failed: {robot._get_operate_error_msg(ret)}")
                    return
            else:
                messagebox.showerror("Error",
                                     f"Arm1 planning failed, error msg: {robot._get_operate_error_msg(points)}")
                return


            mask = FXObjMask.OBJ_ARM1_FLAG
            ret_mask=robot.runtime_run_traj(mask)
            if ret_mask != mask:
                messagebox.showerror('Failed!', f"Run planning trajectory failed for arm1. Return mask: {ret_mask}")
                return

        if not is_zero0 and not is_zero1:
            points0 = robot.plan_linear(0, start0, end0, ref0,vel, acc, freq)
            if isinstance(points0,tuple):
                ret=robot.config_set_traj(FXObjType.OBJ_ARM0, points0[0],points0[1])
                if ret!=0:
                    messagebox.showerror("Failed!", f"Arm0 send Planning points failed: {robot._get_operate_error_msg(ret)}")
                    return
            else:
                messagebox.showerror("Error",
                                     f"Arm0 planning failed, error msg: {robot._get_operate_error_msg(points0)}")
                return

            points1 = robot.plan_linear(1, start1, end1,ref1,vel, acc, freq)
            if isinstance(points1,tuple):
                ret=robot.config_set_traj(FXObjType.OBJ_ARM1, points1[0],points1[1])
                if ret!=0:
                    messagebox.showerror("Failed!", f"Arm0 send Planning points failed: {robot._get_operate_error_msg(ret)}")
                    return
            else:
                messagebox.showerror("Error",
                                     f"Arm1 planning failed, error msg: {robot._get_operate_error_msg(points1)}")
                return


            mask = FXObjMask.OBJ_ARM0_FLAG | FXObjMask.OBJ_ARM1_FLAG
            ret_mask = robot.runtime_run_traj(mask)
            if ret_mask != mask:
                messagebox.showerror('Failed!', f"Run planning trajectory failed for arm0 & arm1. Return mask: {ret_mask}")
                return

    def is_duplicate_xyzabc(self, point_list, target_list):
        new_tuple = tuple(point_list)
        for existing_str in target_list:
            try:
                existing = [float(x) for x in existing_str.split(',') if x.strip()]
                if tuple(existing) == new_tuple:
                    return True
            except ValueError:
                continue
        return False

    def add_multi_seg_point(self, arm):
        if arm == 'Arm0':
            entry = self.multi_add_xyzabc_arm0_entry
            combo = self.multi_points_arm0_combo
            points_list = self.multi_points_arm0_list
        else:
            entry = self.multi_add_xyzabc_arm1_entry
            combo = self.multi_points_arm1_combo
            points_list = self.multi_points_arm1_list
        point_str = entry.get().strip()
        if not point_str:
            return
        try:
            point_nums = [float(x) for x in point_str.split(',') if x.strip()]
        except ValueError:
            return
        if self.is_duplicate_xyzabc(point_nums, points_list):
            messagebox.showwarning('error',f"Point {point_str} already exists, not added.")
            return

        point_nums = [float(x) for x in point_str.split(',') if x.strip()]
        if all(v == 0 for v in point_nums):
            messagebox.showwarning('error',"zero points is not allowed")
            return

        points_list.append(point_str)
        combo['values'] = tuple(points_list)
        combo.set(point_str)

    def clear_multi_segment_inputs(self):
        for entry in [self.multi_start_joints_arm0_entry, self.multi_start_joints_arm1_entry]:
            original = entry.get()
            if ',' in original:
                parts = original.split(',')
                zero_parts = []
                for part in parts:
                    zero_parts.append("0.0")
                entry.delete(0, tk.END)
                entry.insert(0, ', '.join(zero_parts))
            else:
                entry.delete(0, tk.END)
                entry.insert(0, "0.0")

        for entry in [self.multi_add_xyzabc_arm0_entry, self.multi_add_xyzabc_arm1_entry]:
            original = entry.get()
            if ',' in original:
                parts = original.split(',')
                zero_parts = []
                for part in parts:
                    zero_parts.append("0.0")
                entry.delete(0, tk.END)
                entry.insert(0, ', '.join(zero_parts))
            else:
                entry.delete(0, tk.END)
                entry.insert(0, "0.0")

        self.multi_points_arm0_list = []
        self.multi_points_arm0_combo['values'] = []
        if hasattr(self, 'multi_points_arm0_combo'):
            self.multi_points_arm0_combo.set('')

        self.multi_points_arm1_list = []
        self.multi_points_arm1_combo['values'] = []
        if hasattr(self, 'multi_points_arm1_combo'):
            self.multi_points_arm1_combo.set('')

    def pln_run_multi_segment_linear(self):
        try:
            start_joints_arm0 = [float(x) for x in self.multi_start_joints_arm0_entry.get().strip().split(',') if
                                 x.strip()]
            start_joints_arm1 = [float(x) for x in self.multi_start_joints_arm1_entry.get().strip().split(',') if
                                 x.strip()]

            points_arm0 = [[float(x) for x in ps.split(',') if x.strip()] for ps in self.multi_points_arm0_list if
                           ps.strip()]
            points_arm1 = [[float(x) for x in ps.split(',') if x.strip()] for ps in self.multi_points_arm1_list if
                           ps.strip()]
            vel = float(self.multi_cart_vel_entry.get().strip())
            acc = float(self.multi_cart_acc_entry.get().strip())
            freq = int(self.multi_cart_freq_entry.get().strip())
            allow_range = float(self.multi_allow_range_entry.get().strip())
            zsp_type = int(self.multi_zsp_type_entry.get().strip())
            zsp_params = [float(x) for x in self.multi_zsp_params_entry.get().strip().split(',') if x.strip()]
        except ValueError:
            messagebox.showerror("value error", "all parameters must be number")
            return
        if vel <= 0:
            messagebox.showerror("value error", "vel > 0")
            return
        if acc <= 0:
            messagebox.showerror("value error", "acc > 0")
            return
        if freq <= 0 or 1000 % freq != 0:
            messagebox.showerror("value error", "1000%freq==0 and fraq>0")
            return

        if len(points_arm1) == 0 and len(points_arm0) == 0:
            messagebox.showerror("Error", "all parameters are zero")

        if len(points_arm1)==0 and len(points_arm0) >= 2:
            if all(v == 0 for v in start_joints_arm0):
                messagebox.showwarning('value error', "reference joints can not be all zero")
                return
            ret = robot.plan_linear_multi_points_set_start(0, start_joints_arm0, points_arm0[0], points_arm0[1], allow_range,
                                                            zsp_type, zsp_params, vel, acc, freq)
            if ret != 0:
                messagebox.showerror("Error", f"Arm0 planning failed, error msg: {robot._get_operate_error_msg(ret)}")
                return

            for next_one in points_arm0[2:]:
                ret1 = robot.plan_linear_multi_points_set_next(0, next_one, allow_range,zsp_type, zsp_params, vel, acc)
                if ret1 != 0:
                    messagebox.showerror("Error", f"Arm0 planning failed, error msg: {robot._get_operate_error_msg(ret1)}")
                    return

            ret2 = robot.plan_linear_multi_points_get_points()
            if isinstance(ret2, tuple):
                raw_array, point_num = ret2
                ret=robot.config_set_traj(FXObjType.OBJ_ARM0, raw_array,point_num)
                if ret!=0:
                    messagebox.showerror("Failed!", f"Arm0 send planning points failed: {robot._get_operate_error_msg(ret)}")
                    return
            else:
                messagebox.showerror("Error", f"Arm0 planning failed, error msg: {robot._get_operate_error_msg(ret2)}")
                return


            mask = FXObjMask.OBJ_ARM0_FLAG
            ret_mask = robot.runtime_run_traj(mask)
            if ret_mask != mask:
                messagebox.showerror('Failed!', f"run planning trajectory failed for arm0. Return mask: {ret_mask}")
                return

        if len(points_arm1) >= 2 and len(points_arm0)==0:
            if all(v == 0 for v in start_joints_arm1):
                messagebox.showwarning('value error', "reference joints can not be all zero")
                return

            ret = robot.plan_linear_multi_points_set_start(1, start_joints_arm1, points_arm1[0], points_arm1[1], allow_range,
                                                            zsp_type, zsp_params, vel, acc, freq)
            if ret != 0:
                messagebox.showerror("Error", f"Arm1 planning failed, error msg: {robot._get_operate_error_msg(ret)}")
                return

            for next_one in points_arm0[2:]:
                ret = robot.plan_linear_multi_points_set_next(1, next_one, allow_range,zsp_type, zsp_params, vel, acc)
                if ret != 0:
                    messagebox.showerror("Error", f"Arm1 planning failed, error msg: {robot._get_operate_error_msg(ret)}")
                    return

            ret2 = robot.plan_linear_multi_points_get_points()
            if isinstance(ret2, tuple):
                raw_array, point_num = ret2
                ret=robot.config_set_traj(FXObjType.OBJ_ARM1, raw_array,point_num)
                if ret!=0:
                    messagebox.showerror("Failed!", f"Arm1 send planning points failed: {robot._get_operate_error_msg(ret)}")
                    return
            else:
                messagebox.showerror("Error", f"Arm1 planning failed, error msg: {robot._get_operate_error_msg(ret2)}")
                return


            mask = FXObjMask.OBJ_ARM1_FLAG
            ret_mask = robot.runtime_run_traj(mask)
            if ret_mask != mask:
                messagebox.showerror('Failed!', f"run planning trajectory failed for arm1. Return mask: {ret_mask}")
                return

        if len(points_arm1) >= 2 and len(points_arm0) >= 2:
            if all(v == 0 for v in start_joints_arm0) and all(v == 0 for v in start_joints_arm1):
                messagebox.showwarning('value error', "reference joints can not be all zero")
                return
            ret0 = robot.plan_linear_multi_points_set_start(0, start_joints_arm0, points_arm0[0], points_arm0[1],
                                                           allow_range,
                                                           zsp_type, zsp_params, vel, acc, freq)
            if ret0 != 0:
                messagebox.showerror("Error", f"Arm0 planning failed, error msg: {robot._get_operate_error_msg(ret0)}")
                return

            for next_one in points_arm0[2:]:
                ret1 = robot.plan_linear_multi_points_set_next(0, next_one, allow_range, zsp_type, zsp_params, vel, acc)
                if ret1 != 0:
                    messagebox.showerror("Error", f"Arm0 planning failed, error msg: {robot._get_operate_error_msg(ret1)}")
                    return

            point0 = robot.plan_linear_multi_points_get_points()
            if isinstance(point0, tuple):
                ret2=robot.config_set_traj(FXObjType.OBJ_ARM0, point0[0],point0[1])
                if ret2!=0:
                    messagebox.showerror("Failed!", f"Arm0 send planning points failed: {robot._get_operate_error_msg(ret2)}")
                    return
            else:
                messagebox.showerror("Error",
                                     f"Arm0 planning failed, error msg: {robot._get_operate_error_msg(point0)}")
                return

            if all(v == 0 for v in start_joints_arm1):
                messagebox.showwarning('value error', "reference joints can not be all zero")
                return
            ret01 = robot.plan_linear_multi_points_set_start(1, start_joints_arm1, points_arm1[0], points_arm1[1],
                                                           allow_range,
                                                           zsp_type, zsp_params, vel, acc, freq)
            if ret01 != 0:
                messagebox.showerror("Error", f"Arm1 planning failed, error msg: {robot._get_operate_error_msg(ret01)}")
                return

            for next_one1 in points_arm1[2:]:
                ret11 = robot.plan_linear_multi_points_set_next(1, next_one1, allow_range, zsp_type, zsp_params, vel, acc)
                if ret11 != 0:
                    messagebox.showerror("Error", f"Arm1 planning failed, error msg: {robot._get_operate_error_msg(ret11)}")
                    return

            point1 = robot.plan_linear_multi_points_get_points()
            if isinstance(point1, tuple):
                ret21=robot.config_set_traj(FXObjType.OBJ_ARM1, point1[0],point1[1])
                if ret21!=0:
                    messagebox.showerror("Failed!", f"Arm1 send planning points failed: {robot._get_operate_error_msg(ret21)}")
                    return
            else:
                messagebox.showerror("Error", f"Arm1 planning failed, error msg: {robot._get_operate_error_msg(point1)}")
                return


            mask = FXObjMask.OBJ_ARM0_FLAG | FXObjMask.OBJ_ARM1_FLAG
            ret_mask = robot.runtime_run_traj(mask)
            if ret_mask != mask:
                messagebox.showerror('Failed!', f"Run planning trajectory failed for arm0 & arm1. Return mask: {ret_mask}")
                return

    def clear_syn_inputs(self):
        for entry in [self.syn_joints_arm0_entry, self.syn_joints_arm1_entry]:
            original = entry.get()
            if ',' in original:
                parts = original.split(',')
                zero_parts = []
                for part in parts:
                    zero_parts.append("0.0")
                entry.delete(0, tk.END)
                entry.insert(0, ', '.join(zero_parts))
            else:
                entry.delete(0, tk.END)
                entry.insert(0, "0.0")

        for entry in [self.syn_start_arm0_entry, self.syn_start_arm1_entry]:
            original = entry.get()
            if ',' in original:
                parts = original.split(',')
                zero_parts = []
                for part in parts:
                    zero_parts.append("0.0")
                entry.delete(0, tk.END)
                entry.insert(0, ', '.join(zero_parts))
            else:
                    entry.delete(0, tk.END)
                    entry.insert(0, "0.0")

        for entry in [self.syn_end_arm0_entry, self.syn_end_arm1_entry]:
            original = entry.get()
            if ',' in original:
                parts = original.split(',')
                zero_parts = []
                for part in parts:
                    zero_parts.append("0.0")
                entry.delete(0, tk.END)
                entry.insert(0, ', '.join(zero_parts))
            else:
                    entry.delete(0, tk.END)
                    entry.insert(0, "0.0")

    def pln_run_syn(self):
        start_str0 = self.syn_start_arm0_entry.get().strip()
        start_str1 = self.syn_start_arm1_entry.get().strip()
        end_str0 = self.syn_end_arm0_entry.get().strip()
        end_str1 = self.syn_end_arm1_entry.get().strip()
        start_joints_str0 = self.syn_joints_arm0_entry.get().strip()
        start_joints_str1 = self.syn_joints_arm1_entry.get().strip()
        zsp_params_str0 = self.syn0_zsp_params_entry.get().strip()
        zsp_params_str1 = self.syn1_zsp_params_entry.get().strip()

        def parse_joints(s, num):
            parts = s.split(',')
            if len(parts) != num:
                raise ValueError(f"need {num} values, actual:{len(parts)}")
            return [float(p.strip()) for p in parts]
        try:
            start0 = parse_joints(start_str0, 6)
            end0 = parse_joints(end_str0, 6)
            start1 = parse_joints(start_str1, 6)
            end1 = parse_joints(end_str1, 6)
            start_joints0 = parse_joints(start_joints_str0, 7)
            start_joints1 = parse_joints(start_joints_str1, 7)
            zsp_params0=parse_joints(zsp_params_str0,6)
            zsp_params1 = parse_joints(zsp_params_str1, 6)
        except ValueError as e:
            messagebox.showerror("value error", f"parse paramerters failed: {e}")
            return

        try:
            vel = float(self.syn_vel_entry.get().strip())
            acc = float(self.syn_acc_entry.get().strip())
            freq = int(self.syn_freq_entry.get().strip())
            zsp_type0 = int(self.syn0_zsp_type_entry.get().strip())
            zsp_type1 = int(self.syn1_zsp_type_entry.get().strip())
        except ValueError:
            messagebox.showerror("value error", "all parameters must be number")
            return
        if vel <= 0:
            messagebox.showerror("value error", "vel > 0")
            return
        if acc <= 0:
            messagebox.showerror("value error", "acc > 0")
            return
        if freq <= 0 or 1000 % freq != 0:
            messagebox.showerror("value error", "1000%freq==0 and fraq>0")
            return
        if all(v == 0 for v in start_joints0) and all(v == 0 for v in start_joints1):
            messagebox.showwarning('value error', "Sart joints can not be all zero")
            return
        if all(v == 0 for v in start0) and all(v == 0 for v in start1):
            messagebox.showwarning('value error', "Sart XYZABC can not be all zero")
            return
        if all(v == 0 for v in end0) and all(v == 0 for v in end1):
            messagebox.showwarning('value error', "End XYZABC can not be all zero")
            return

        arms_structure_params = ArmsSynchronousPlanningParams()
        arms_structure_params.World_Co_Flag = 0
        arms_structure_params.Sync_Type = 0
        arms_structure_params.Freq = freq
        arms_structure_params.Vel = vel
        arms_structure_params.Acc = acc
        arms_structure_params.Arm0_ZSP_Type = zsp_type0
        arms_structure_params.Arm1_ZSP_Type =zsp_type1

        for i in range(7):
            arms_structure_params.Arm0_Ref_Joints[i] = start_joints0[i]
            arms_structure_params.Arm1_Ref_Joints[i] = start_joints1[i]

            if i < 6:
                arms_structure_params.Arm0_Start_XYZABC[i] = start0[i]
                arms_structure_params.Arm1_Start_XYZABC[i] = start1[i]

                arms_structure_params.Arm0_End_XYZABC[i] = end0[i]
                arms_structure_params.Arm1_End_XYZABC[i] = end1[i]

                arms_structure_params.Arm0_ZSP_Para[i] = zsp_params0[i]
                arms_structure_params.Arm1_ZSP_Para[i] = zsp_params1[i]

        ret = robot.plan_linear_synchronous(arms_structure_params)
        if not isinstance(ret, tuple):
            messagebox.showerror("Error", f"Arm0 planning failed, error msg: {robot._get_operate_error_msg(ret)}")
            return

        raw_array0, raw_array1, point_num = ret
        ret=robot.config_set_traj(FXObjType.OBJ_ARM0, raw_array0, point_num)
        if ret!=0:
            messagebox.showerror("Failed!", f"Arm0 send planning points failed: {robot._get_operate_error_msg(ret)}")
            return
        ret=robot.config_set_traj(FXObjType.OBJ_ARM1, raw_array1, point_num)
        if ret!=0:
            messagebox.showerror("Failed!", f"Arm1 send planning points failed: {robot._get_operate_error_msg(ret)}")
            return

        mask = FXObjMask.OBJ_ARM0_FLAG | FXObjMask.OBJ_ARM1_FLAG
        ret_mask = robot.runtime_run_traj(mask)
        if ret_mask != mask:
            messagebox.showerror('Failed!', f"Run planning trajectory failed for arm0 & arm1. Return mask: {ret_mask}")
            return

    def stop_motion(self):
        ret_mask = robot.runtime_stop_traj(FXObjMask.OBJ_ARM0_FLAG | FXObjMask.OBJ_ARM1_FLAG)
        if ret_mask != (FXObjMask.OBJ_ARM0_FLAG | FXObjMask.OBJ_ARM1_FLAG):
            messagebox.showerror('Failed!', f"brake planning trajectory failed for arm0 & arm1. Return mask: {ret_mask}")
            return

    def disable_soft_limit(self, obj_type: int, axis_mask: int):
        if not self.connected:
            messagebox.showerror('Error', 'Please connect robot')
            return
        try:
            ret=robot.config_disable_soft_limit(obj_type, axis_mask)
            if ret!= 0:
                messagebox.showerror('Failed', f'{obj_type} disable soft limit failed: {robot._get_operate_error_msg(ret)} ')
        except Exception as e:
            messagebox.showerror('Error', f'Disable soft limit error: {e}')

    def clear_sensor_offset(self, obj):
        try:
            obj_type = self._obj_name_to_type(obj)
            ret=robot.config_clear_sensor_offset(obj_type)
            if ret!= 0:
                messagebox.showerror('Failed', f'{obj_type} clear sensor offset failed: {robot._get_operate_error_msg(ret)} ')
        except Exception as e:
            messagebox.showerror('Error', f'Clear sensor offset error: {e}')

    def clear_motor_as_zero(self, obj, btn):
        """Motor encoder zeroing (reset encoder offset) for specified arm."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        btn.config(state="disabled")
        try:
            obj_type = self._obj_name_to_type(obj)
            axis_mask = 0x7F  # All 7 axes
            ret=robot.config_reset_enc_offset(obj_type, axis_mask)
            if ret!= 0:
                messagebox.showerror('Failed!', f"{obj} reset encoder offset failed: {robot._get_operate_error_msg(ret)} ")
                return
        except Exception as e:
            messagebox.showerror('Error', str(e))

    def clear_motor_error(self, obj):
        """Clear encoder error for specified obj."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        try:
            if obj == 'A':
                obj_type = FXObjType.OBJ_ARM0
            elif obj == 'B':
                obj_type = FXObjType.OBJ_ARM1
            else:
                raise ValueError("obj must be 'A' or 'B'")
            axis_mask = 0x7F  # All 7 axes
            ret=robot.config_clear_enc_error(obj_type, axis_mask)
            if ret!= 0:
                messagebox.showerror('Failed!', f"{obj}Clear encoder error failed: {robot._get_operate_error_msg(ret)}")
                return
        except Exception as e:
            messagebox.showerror('Error', str(e))

    def file_client_dialog(self):
        dialog = tk.Toplevel(self.root)
        dialog.title("File Transfer")
        dialog.geometry("300x150")
        dialog.configure(bg="white")
        dialog.transient(self.root)
        dialog.grab_set()

        send_btn = tk.Button(dialog, text="Send File to Robot", width=20,
                             command=self._send_file_to_robot, bg="#4CAF50", fg="white")
        send_btn.pack(pady=20)

        recv_btn = tk.Button(dialog, text="Receive File from Robot", width=20,
                             command=self._receive_file_from_robot, bg="#2196F3", fg="white")
        recv_btn.pack(pady=10)

    def _send_file_to_robot(self):
        local_path = filedialog.askopenfilename(title="Select file to send")
        if not local_path:
            return
        remote_path = simpledialog.askstring("Remote Path", "Enter remote path (e.g., /home/robot/file.bin):")
        if not remote_path:
            return
        if robot.send_file(local_path, remote_path) == 0:
            messagebox.showinfo("Success", f"File sent to {remote_path}")
        else:
            messagebox.showerror("Failed", "Send file failed")
            return

    def _receive_file_from_robot(self):
        remote_path = simpledialog.askstring("Remote Path",
                                             "Enter remote path to receive (e.g., /home/robot/file.bin):")
        if not remote_path:
            return
        local_path = filedialog.asksaveasfilename(title="Save file as")
        if not local_path:
            return
        if robot.recv_file(local_path, remote_path) == 0:
            messagebox.showinfo("Success", f"File received and saved to {local_path}")
        else:
            messagebox.showerror("Failed", "Receive file failed")
            return

    def Estop(self):
        if not self.connected:
            messagebox.showerror('Error', 'Please connect robot')
            return
        try:
            robot.emergency_stop(FXObjMask.OBJ_ALL_FLAG)
        except Exception as e:
            messagebox.showerror('Error', f'Emergency stop failed: {e}')

    def show_impedance_dialog(self, obj):
        # if not self.connected:
        #     messagebox.showerror('Error', "Please connect robot first!")
        #     return

        impedance_dialog = tk.Toplevel(self.root)
        impedance_dialog.title(f"Impedance parameter settings for {obj}")
        impedance_dialog.geometry("1200x500")
        impedance_dialog.configure(bg="white")
        impedance_dialog.transient(self.root)
        impedance_dialog.resizable(True, True)
        impedance_dialog.grab_set()

        if obj == 'Arm0':
            main_frame = tk.Frame(impedance_dialog, padx=20, pady=20, bg='white')
            main_frame.pack(fill="both", expand=True)
            title_label = tk.Label(
                main_frame,
                text=f"Set the impedance parameters of {obj}",
                font=('Arial', 10, 'bold'),
                fg='#2c3e50',
                bg='white'
            )
            title_label.pack(pady=(0, 10))

            params_frame = tk.Frame(main_frame, bg='white')
            params_frame.pack(fill="x", pady=(5, 10))
            joint_kd_a_button = tk.Button(params_frame, text="JointImp parameters", width=20,
                                          command=lambda: self.joint_kd_set('Arm0'))
            joint_kd_a_button.grid(row=0, column=0, padx=5, pady=10)
            k_a_label = tk.Label(params_frame, text='K:', width=5, bg="white")
            k_a_label.grid(row=0, column=1)
            k_a_entry = tk.Entry(params_frame, textvariable=self.k_a_entry, width=50)
            k_a_entry.grid(row=0, column=2, sticky="ew")
            d_a_label = tk.Label(params_frame, text='D:', width=5, bg="white")
            d_a_label.grid(row=0, column=3)
            d_a_entry = tk.Entry(params_frame, textvariable=self.d_a_entry, width=30)
            d_a_entry.grid(row=0, column=4)

            cart_kd_a_button = tk.Button(params_frame, text="CartImp parameters", width=20,
                                         command=lambda: self.cart_kd_set('Arm0'))
            cart_kd_a_button.grid(row=1, column=0, padx=5, pady=(20, 10))
            k_a_label_ = tk.Label(params_frame, text='K:', width=5, bg="white")
            k_a_label_.grid(row=1, column=1)
            cart_k_a_entry = tk.Entry(params_frame, textvariable=self.cart_k_a_entry, width=50)
            cart_k_a_entry.grid(row=1, column=2, sticky="ew")
            d_a_label_ = tk.Label(params_frame, text='D:', width=5, bg="white")
            d_a_label_.grid(row=1, column=3)
            cart_d_a_entry = tk.Entry(params_frame, textvariable=self.cart_d_a_entry, width=30)
            cart_d_a_entry.grid(row=1, column=4)

            # Force/Torque parameters row
            force_torque_frame = tk.Frame(main_frame, bg='white')
            force_torque_frame.pack(fill="x", pady=(5, 10))
            set_ft_btn = tk.Button(force_torque_frame, text="Set Force/Torque", width=20,
                                   command=lambda: self.force_torque_set('Arm0'))
            set_ft_btn.grid(row=0, column=0, padx=5, pady=10)
            tk.Label(force_torque_frame, text="Force(dir_x,dir_y,dir_z,force(-50~50N),distance(<50mm)):",
                     font=('Arial', 9), bg='white', width=50).grid(row=0, column=1)
            force_entry = tk.Entry(force_torque_frame, textvariable=self.force_a_entry, width=30)
            force_entry.grid(row=0, column=2)
            tk.Label(force_torque_frame, text="Torque(dir_x,dir_y,dir_z,torque(N*m),distance(deg))", font=('Arial', 9),
                     bg='white', width=50).grid(row=1, column=1)
            torque_entry = tk.Entry(force_torque_frame, textvariable=self.torque_a_entry, width=30)
            torque_entry.grid(row=1, column=2)

            params_save_frame = tk.Frame(main_frame, bg='white')
            params_save_frame.pack(fill="x", pady=(20, 10))
            load_ini_param_a_button = tk.Button(params_save_frame, text="Load default parameters",
                                                command=self.load_default_param)
            load_ini_param_a_button.pack(side='left', padx=(200, 0))

            save_param_a_button = tk.Button(params_save_frame, text="Save parameters",
                                            command=lambda: self.save_param('Arm0'))
            save_param_a_button.pack(side='left', padx=(50, 0))
            load_param_a_button = tk.Button(params_save_frame, text="Import parameters",
                                            command=lambda: self.load_param('Arm0'))
            load_param_a_button.pack(side='left', padx=(50, 10))


        elif obj == 'Arm1':
            main_frame1 = tk.Frame(impedance_dialog, padx=20, pady=20, bg='white')
            main_frame1.pack(fill="both", expand=True)
            title_label1 = tk.Label(
                main_frame1,
                text=f"Set the impedance parameters of {obj}",
                font=('Arial', 10, 'bold'),
                fg='#2c3e50',
                bg='white'
            )
            title_label1.pack(pady=(0, 10))

            params_frame1 = tk.Frame(main_frame1, bg='white')
            params_frame1.pack(fill="x", pady=(5, 10))

            joint_kd_a_button1 = tk.Button(params_frame1, text="Set joint impedance parameters", width=20,
                                           command=lambda: self.joint_kd_set('Arm1'))
            joint_kd_a_button1.grid(row=0, column=0, padx=5, pady=10)
            k_a_label1 = tk.Label(params_frame1, text='K:', width=5, bg="white")
            k_a_label1.grid(row=0, column=1)
            k_b_entry = tk.Entry(params_frame1, textvariable=self.k_b_entry, width=50)
            k_b_entry.grid(row=0, column=2, sticky="ew")
            d_b_label = tk.Label(params_frame1, text='D:', width=5, bg="white")
            d_b_label.grid(row=0, column=3)
            d_b_entry = tk.Entry(params_frame1, textvariable=self.d_b_entry, width=30)
            d_b_entry.grid(row=0, column=4)

            cart_kd_b_button = tk.Button(params_frame1, text="CartImp parameters", width=20,
                                         command=lambda: self.cart_kd_set('Arm1'))
            cart_kd_b_button.grid(row=1, column=0, padx=5, pady=(20, 10))
            k_b_label_ = tk.Label(params_frame1, text='K:', width=5, bg="white")
            k_b_label_.grid(row=1, column=1)
            cart_k_b_entry = tk.Entry(params_frame1, textvariable=self.cart_k_b_entry, width=50)
            cart_k_b_entry.grid(row=1, column=2, sticky="ew")
            d_b_label_ = tk.Label(params_frame1, text='D:', width=5, bg="white")
            d_b_label_.grid(row=1, column=3)
            cart_d_b_entry = tk.Entry(params_frame1, textvariable=self.cart_d_b_entry, width=30)
            cart_d_b_entry.grid(row=1, column=4)

            # Force/Torque parameters row
            force_torque_frame = tk.Frame(main_frame1, bg='white')
            force_torque_frame.pack(fill="x", pady=(10, 5))
            set_ft_btn = tk.Button(force_torque_frame, text="Set Force/Torque", width=20,
                                   command=lambda: self.force_torque_set('Arm1'))
            set_ft_btn.grid(row=0, column=0, padx=5, pady=10)
            tk.Label(force_torque_frame, text="Force(dir_x,dir_y,dir_z,force(-50~50N),distance(<50mm)):",
                     font=('Arial', 9), bg='white', width=50).grid(row=0, column=1)
            force_entry = tk.Entry(force_torque_frame, textvariable=self.force_b_entry, width=25)
            force_entry.grid(row=0, column=2)
            tk.Label(force_torque_frame, text="Torque(dir_x,dir_y,dir_z,torque(N*m),distance(deg))", font=('Arial', 9),
                     bg='white', width=50).grid(row=1, column=1)
            torque_entry = tk.Entry(force_torque_frame, textvariable=self.torque_b_entry, width=25)
            torque_entry.grid(row=1, column=2)

            params_save_frame = tk.Frame(main_frame1, bg='white')
            params_save_frame.pack(fill="x", pady=(20, 10))
            load_ini_param_a_button = tk.Button(params_save_frame, text="Load default parameters",
                                                command=self.load_default_param)
            load_ini_param_a_button.pack(side='left', padx=(200, 0))
            save_param_a_button = tk.Button(params_save_frame, text="Save parameters",
                                            command=lambda: self.save_param('Arm1'))
            save_param_a_button.pack(side='left', padx=(50, 0))
            load_param_a_button = tk.Button(params_save_frame, text="Import parameters",
                                            command=lambda: self.load_param('Arm1'))
            load_param_a_button.pack(side='left', padx=(50, 10))

        elif obj == 'Body':
            main_frame1 = tk.Frame(impedance_dialog, padx=20, pady=20, bg='white')
            main_frame1.pack(fill="both", expand=True)
            title_label1 = tk.Label(
                main_frame1,
                text=f"Set the PD parameters of {obj}",
                font=('Arial', 10, 'bold'),
                fg='#2c3e50',
                bg='white'
            )
            title_label1.pack(pady=(0, 10))

            params_frame1 = tk.Frame(main_frame1, bg='white')
            params_frame1.pack(fill="x", pady=(5, 10))

            joint_kd_a_button1 = tk.Button(params_frame1, text="PD parameters", width=20,
                                           command=lambda: self.joint_kd_set('Body'))
            joint_kd_a_button1.grid(row=0, column=0, padx=5, pady=10)
            k_a_label1 = tk.Label(params_frame1, text='PDP:', width=5, bg="white")
            k_a_label1.grid(row=0, column=1)
            k_b_entry = tk.Entry(params_frame1, textvariable=self.pdp_entry, width=50)
            k_b_entry.grid(row=0, column=2, sticky="ew")
            d_b_label = tk.Label(params_frame1, text='PDD:', width=5, bg="white")
            d_b_label.grid(row=0, column=3)
            d_b_entry = tk.Entry(params_frame1, textvariable=self.pdd_entry, width=30)
            d_b_entry.grid(row=0, column=4)

            params_save_frame = tk.Frame(main_frame1, bg='white')
            params_save_frame.pack(fill="x", pady=(20, 10))
            load_ini_param_a_button = tk.Button(params_save_frame, text="Load default parameters",
                                                command=self.load_default_body_pd)
            load_ini_param_a_button.pack(side='left', padx=(200, 0))
            save_param_a_button = tk.Button(params_save_frame, text="Save parameters",
                                            command=lambda: self.save_param('Body'))
            save_param_a_button.pack(side='left', padx=(50, 0))
            load_param_a_button = tk.Button(params_save_frame, text="Import parameters",
                                            command=lambda: self.load_param('Body'))
            load_param_a_button.pack(side='left', padx=(50, 10))

    def joint_kd_set(self, obj):
        if not self.connected:
            messagebox.showerror('Error', 'Please connect robot')
            return

        if obj == 'Arm0':
            k_var = self.k_a_entry
            d_var = self.d_a_entry
            obj_type = FXObjType.OBJ_ARM0
            num_joints = 7
        elif obj == 'Arm1':
            k_var = self.k_b_entry
            d_var = self.d_b_entry
            obj_type = FXObjType.OBJ_ARM1
            num_joints = 7
        elif obj == 'Body':
            k_var = self.pdp_entry
            d_var = self.pdd_entry
            obj_type = FXObjType.OBJ_BODY
            num_joints = 6
        else:
            messagebox.showerror('Error', f'Unknown obj: {obj}')
            return

        k_str = k_var.get().strip()
        if not k_str:
            messagebox.showerror("Error", "K/PDP parameter cannot be empty!")
            return
        is_valid, result = self.validate_point(k_str, num_joints)
        if not is_valid:
            messagebox.showerror("Error", f"Invalid K/PDP format: {result}")
            return
        k_list = [float(x) for x in result.split(',')]

        d_str = d_var.get().strip()
        if not d_str:
            messagebox.showerror("Error", "D/PDD parameter cannot be empty!")
            return
        is_valid, result = self.validate_point(d_str, num_joints)
        if not is_valid:
            messagebox.showerror("Error", f"Invalid D/PDD format: {result}")
            return
        d_list = [float(x) for x in result.split(',')]


        if obj in ('Arm0', 'Arm1'):
            ret = robot.runtime_set_joint_k(obj_type, k_list)
            if ret != 0:
                messagebox.showerror('Failed!', f"Set {obj} K parameters failed. Error msg: {robot._get_operate_error_msg(ret)}")
                return
            ret = robot.runtime_set_joint_d(obj_type, d_list)
            if ret != 0:
                messagebox.showerror('Failed!', f"Set {obj} D parameters failed. Error msg: {robot._get_operate_error_msg(ret)}")
                return
        else:
            ret = robot.runtime_set_body_pdp(k_list)
            if ret != 0:
                messagebox.showerror('Failed!', f"Set {obj} P parameters failed. Error msg: {robot._get_operate_error_msg(ret)}")
                return
            ret = robot.runtime_set_body_pdd(d_list)
            if ret != 0:
                messagebox.showerror('Failed!', f"Set {obj} PD parameters failed. Error msg: {robot._get_operate_error_msg(ret)}")
                return

    def cart_kd_set(self, obj):
        if not self.connected:
            messagebox.showerror('Error', 'Please connect robot')
            return

        if obj == 'Arm0':
            k_var = self.cart_k_a_entry
            d_var = self.cart_d_a_entry
            obj_type = FXObjType.OBJ_ARM0
            num_joints = 7
        elif obj == 'Arm1':
            k_var = self.cart_k_b_entry
            d_var = self.cart_d_b_entry
            obj_type = FXObjType.OBJ_ARM1
            num_joints = 7
        elif obj == 'Body':
            k_var = self.pdp_entry
            d_var = self.pdd_entry
            obj_type = None
            num_joints = 6
        else:
            messagebox.showerror('Error', f'Unknown obj: {obj}')
            return

        k_str = k_var.get().strip()
        if not k_str:
            messagebox.showerror("Error", "K/PDP parameter cannot be empty!")
            return
        is_valid, result = self.validate_point(k_str, num_joints)
        if not is_valid:
            messagebox.showerror("Error", f"Invalid K/PDP format: {result}")
            return
        k_list = [float(x) for x in result.split(',')]

        d_str = d_var.get().strip()
        if not d_str:
            messagebox.showerror("Error", "D/PDD parameter cannot be empty!")
            return
        is_valid, result = self.validate_point(d_str, num_joints)
        if not is_valid:
            messagebox.showerror("Error", f"Invalid D/PDD format: {result}")
            return
        d_list = [float(x) for x in result.split(',')]


        if obj in ('Arm0', 'Arm1'):
            ret = robot.runtime_set_cart_k(obj_type, k_list)
            if ret != 0:
                messagebox.showerror('Failed!', f"Set {obj} Cartesian K parameters failed. Error msg: {robot._get_operate_error_msg(ret)}")
                return
            ret = robot.runtime_set_cart_d(obj_type, d_list)
            if ret != 0:
                messagebox.showerror('Failed!', f"Set {obj} Cartesian D parameters failed. Error msg: {robot._get_operate_error_msg(ret)}")
                return
        else:
            ret = robot.runtime_set_body_pdp(k_list)
            if ret != 0:
                messagebox.showerror('Failed!', f"Set {obj} P parameters failed. Error msg: {robot._get_operate_error_msg(ret)}")
                return
            ret = robot.runtime_set_body_pdd(d_list)
            if ret != 0:
                messagebox.showerror('Failed!', f"Set {obj} PD parameters failed. Error msg: {robot._get_operate_error_msg(ret)}")
                return

    def load_default_param(self):
        self.cart_k_b_entry.set("3000,3000,3000,100,100,100,50")
        self.cart_k_a_entry.set("3000,3000,3000,100,100,100,50")
        self.cart_d_a_entry.set("0.1,0.1,0.1,0.1,0.1,0.1,0.11")
        self.cart_d_b_entry.set("0.1,0.1,0.1,0.1,0.1,0.1,0.11")

        self.k_a_entry.set("3,3,3,2,1,1,1")
        self.k_b_entry.set("3,3,3,2,1,1,1")
        self.d_a_entry.set("0.2,0.2,0.2,0.2,0.2,0.2,0.2")
        self.d_b_entry.set("0.2,0.2,0.2,0.2,0.2,0.2,0.2")

        self.force_a_entry.set("0,1,0,25,25")
        self.torque_a_entry.set("0,1,0,5,10")
        self.force_b_entry.set("0,1,0,25,25")
        self.torque_b_entry.set("0,1,0,5,10")

    def load_default_body_pd(self):
        self.pdp_entry.set("12,15,15,12,2,2")
        self.pdd_entry.set("3,3,3,2,1,1")

    def load_default_tools(self):
        self.arm0_tool_dyn_entry.set("0,0,0,0,0,0,0,0,0,0")
        self.arm1_tool_dyn_entry.set("0,0,0,0,0,0,0,0,0,0")
        self.arm0_tool_kine_entry.set("0,0,0,0,0,0")
        self.arm1_tool_kine_entry.set("0,0,0,0,0,0")

    def save_param(self, obj):
        self.params = []
        if obj == 'Arm0':
            params_to_save = [
                self.k_a_entry.get(),
                self.d_a_entry.get(),
                self.cart_k_a_entry.get(),
                self.cart_d_a_entry.get(),
                self.force_a_entry.get(),
                self.torque_a_entry.get(),
            ]
        elif obj == 'Arm1':
            params_to_save = [
                self.k_b_entry.get(),
                self.d_b_entry.get(),
                self.cart_k_b_entry.get(),
                self.cart_d_b_entry.get(),
                self.force_b_entry.get(),
                self.torque_b_entry.get(),
            ]
        elif obj == 'Body':
            params_to_save = [
                self.pdp_entry.get(),
                self.pdd_entry.get()
            ]
        else:
            messagebox.showerror("Error", f"Unknown obj: {obj}")
            return

        file_path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            title=f"Save {obj} parameters"
        )
        if not file_path:
            return

        try:
            with open(file_path, 'w') as f:
                for param in params_to_save:
                    f.write(param.strip() + '\n')
            messagebox.showinfo("Success", f"{obj} parameters saved successfully")
        except Exception as e:
            messagebox.showerror("Error", f"Error saving file: {str(e)}")

    def load_param(self, obj):
        file_path = filedialog.askopenfilename(
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            title=f"Select parameter file for {obj}"
        )
        if not file_path:
            return

        try:
            with open(file_path, 'r') as f:
                lines = [line.strip() for line in f.readlines() if line.strip()]
        except Exception as e:
            messagebox.showerror("Error", f"Error reading file: {str(e)}")
            return

        if obj == 'Arm0':
            if len(lines) >= 4:
                self.k_a_entry.set(lines[0])
                self.d_a_entry.set(lines[1])
                self.cart_k_a_entry.set(lines[2])
                self.cart_d_a_entry.set(lines[3])
                self.force_a_entry.set(lines[4]),
                self.torque_a_entry.set(lines[5])
            else:
                messagebox.showerror("Error", "File does not contain enough parameters (need 4)")
                return
        elif obj == 'Arm1':
            if len(lines) >= 4:
                self.k_b_entry.set(lines[0])
                self.d_b_entry.set(lines[1])
                self.cart_k_b_entry.set(lines[2])
                self.cart_d_b_entry.set(lines[3])
                self.force_b_entry.set(lines[4]),
                self.torque_b_entry.set(lines[5])
            else:
                messagebox.showerror("Error", "File does not contain enough parameters (need 4)")
                return
        elif obj == 'Body':
            if len(lines) >= 2:
                self.pdp_entry.set(lines[0])
                self.pdd_entry.set(lines[1])
            else:
                messagebox.showerror("Error", "File does not contain enough parameters (need 2)")
                return
        else:
            messagebox.showerror("Error", f"Unknown obj: {obj}")

    def force_torque_set(self, obj):
        """Set force and torque control parameters for Arm0/Arm1."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        if obj not in ('Arm0', 'Arm1'):
            messagebox.showerror('Error', f'Force/Torque not supported for {obj}')
            return
        try:
            if obj == 'Arm0':
                force_str = self.force_a_entry.get().strip()
                torque_str = self.torque_a_entry.get().strip()
                obj_type = FXObjType.OBJ_ARM0
            else:
                force_str = self.force_b_entry.get().strip()
                torque_str = self.torque_b_entry.get().strip()
                obj_type = FXObjType.OBJ_ARM1

            # Parse force list (5 floats)
            force_list = [float(x) for x in force_str.split(',')] if force_str else [0] * 5
            if len(force_list) != 5:
                messagebox.showerror('Error', 'Force Ctrl must have 5 comma-separated values')
                return
            torque_list = [float(x) for x in torque_str.split(',')] if torque_str else [0] * 5
            if len(torque_list) != 5:
                messagebox.showerror('Error', 'Torque Ctrl must have 5 comma-separated values')
                return

            ret = robot.runtime_set_force_ctrl(obj_type, force_list)
            if ret != 0:
                messagebox.showerror('Failed!', f"Set force ctrl failed for {obj}. Error msg: {robot._get_operate_error_msg(ret)}")
                return
            ret = robot.runtime_set_torque_ctrl(obj_type, torque_list)
            if ret != 0:
                messagebox.showerror('Failed!', f"Set torque ctrl failed for {obj}. Error msg: {robot._get_operate_error_msg(ret)}")
                return
        except Exception as e:
            messagebox.showerror('Error', f"Force/Torque set failed: {e}")

    def _obj_name_to_type(self, name):
        mapping = {
            'Arm0': FXObjType.OBJ_ARM0,
            'Arm1': FXObjType.OBJ_ARM1,
            'Body': FXObjType.OBJ_BODY,
            'Head': FXObjType.OBJ_HEAD,
            'Lift': FXObjType.OBJ_LIFT
        }
        return mapping.get(name, FXObjType.OBJ_ARM0)

    def _obj_name_to_mask(self, name):
        mapping = {
            'Arm0': FXObjMask.OBJ_ARM0_FLAG,
            'Arm1': FXObjMask.OBJ_ARM1_FLAG,
            'Body': FXObjMask.OBJ_BODY_FLAG,
            'Head': FXObjMask.OBJ_HEAD_FLAG,
            'Lift': FXObjMask.OBJ_LIFT_FLAG,
        }
        return mapping.get(name, 0)

    def pd_state(self,obj):
        """Switch to PD mode (only for Body Hands)."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        if obj not in ('Body'):
            messagebox.showerror('Error', f'{obj} does not support PD mode')
            return
        try:
            obj_type = self._obj_name_to_type(obj)
            if obj == 'Body':
                vel = int(self.body_speed_entry.get())
                acc = int(self.body_accel_entry.get())
                k_str = self.pdp_entry.get().strip()
                d_str = self.pdd_entry.get().strip()
            # Parse K and D lists
            if obj == 'Body':
                k_list = [float(x) for x in k_str.split(',')] if k_str else [0] * 6
                d_list = [float(x) for x in d_str.split(',')] if d_str else [0] * 6
                if len(k_list) != 6 or len(d_list) != 6:
                    messagebox.showerror('Error', 'body pdp/pdd must have 6 values')
                    return
            else:
                k_list = [float(x) for x in k_str.split(',')] if k_str else [0] * 7
                d_list = [float(x) for x in d_str.split(',')] if d_str else [0] * 7
                if len(k_list) != 7 or len(d_list) != 7:
                    messagebox.showerror('Error', 'K/D must have 7 values')
                    return
            ret = robot.switch_to_pd_mode(obj_type, 1000, vel, acc, k_list, d_list)
            if ret != 0:
                messagebox.showerror('Failed!',
                                     f'{obj} switch to PD failed: {robot._get_operate_error_msg(ret)}')
                return
        except Exception as e:
            messagebox.showerror('Error', f'PD switch failed: {e}')

    def jointImp_state(self, obj):
        """Switch to Joint Impedance mode (only for Arm0/Arm1)."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        if obj not in ('Arm0', 'Arm1', 'Body'):
            messagebox.showerror('Error', f'{obj} does not support Joint Impedance mode')
            return
        try:
            obj_type = self._obj_name_to_type(obj)
            if obj == 'Arm0':
                vel = int(self.left_speed_entry.get())
                acc = int(self.left_accel_entry.get())
                k_str = self.k_a_entry.get().strip()
                d_str = self.d_a_entry.get().strip()
            if obj == 'Arm1':
                vel = int(self.right_speed_entry.get())
                acc = int(self.right_accel_entry.get())
                k_str = self.k_b_entry.get().strip()
                d_str = self.d_b_entry.get().strip()
            if obj == 'Body':
                vel = int(self.body_speed_entry.get())
                acc = int(self.body_accel_entry.get())
                k_str = self.pdp_entry.get().strip()
                d_str = self.pdd_entry.get().strip()
            # Parse K and D lists
            if obj == 'Body':
                k_list = [float(x) for x in k_str.split(',')] if k_str else [0] * 6
                d_list = [float(x) for x in d_str.split(',')] if d_str else [0] * 6
                if len(k_list) != 6 or len(d_list) != 6:
                    messagebox.showerror('Error', 'body pdp/pdd must have 6 values')
                    return
            else:
                k_list = [float(x) for x in k_str.split(',')] if k_str else [0] * 7
                d_list = [float(x) for x in d_str.split(',')] if d_str else [0] * 7
                if len(k_list) != 7 or len(d_list) != 7:
                    messagebox.showerror('Error', 'K/D must have 7 values')
                    return
            ret = robot.switch_to_imp_joint_mode(obj_type, 2000, vel, acc, k_list, d_list)
            if ret != 0:
                messagebox.showerror('Failed!', f'{obj} switch to joint impedance failed: {robot._get_operate_error_msg(ret)}')
                return
        except Exception as e:
            messagebox.showerror('Error', f'Joint Impedance switch failed: {e}')

    def cartImp_state(self, obj):
        """Switch to Cartesian Impedance mode (only for Arm0/Arm1)."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        if obj not in ('Arm0', 'Arm1'):
            messagebox.showerror('Error', f'{obj} does not support Cartesian Impedance mode')
            return
        try:
            obj_type = self._obj_name_to_type(obj)
            if obj == 'Arm0':
                vel = int(self.left_speed_entry.get())
                acc = int(self.left_accel_entry.get())
                k_str = self.cart_k_a_entry.get().strip()
                d_str = self.cart_d_a_entry.get().strip()
            else:  # Arm1
                vel = int(self.right_speed_entry.get())
                acc = int(self.right_accel_entry.get())
                k_str = self.cart_k_b_entry.get().strip()
                d_str = self.cart_d_b_entry.get().strip()
            k_list = [float(x) for x in k_str.split(',')] if k_str else [0] * 7
            d_list = [float(x) for x in d_str.split(',')] if d_str else [0] * 7
            if len(k_list) != 7 or len(d_list) != 7:
                messagebox.showerror('Error', 'Cartesian K/D must have 7 values')
                return
            ret = robot.switch_to_imp_cart_mode(obj_type, 2000, vel, acc, k_list, d_list)
            if ret != 0:
                messagebox.showerror('Failed!', f'{obj} switch to cartesian impedance failed: {robot._get_operate_error_msg(ret)}')
                return
        except Exception as e:
            messagebox.showerror('Error', f'Cartesian Impedance switch failed: {e}')

    def forceImp_state(self, obj):
        """Switch to Force Impedance mode (only for Arm0/Arm1)."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        if obj not in ('Arm0', 'Arm1'):
            messagebox.showerror('Error', f'{obj} does not support Force Impedance mode')
            return
        try:
            obj_type = self._obj_name_to_type(obj)
            if obj == 'Arm0':
                force_str = self.force_a_entry.get().strip()
                torque_str = self.torque_a_entry.get().strip()
            else:
                force_str = self.force_b_entry.get().strip()
                torque_str = self.torque_b_entry.get().strip()
            # Parse to list of floats
            force_ctrl = [float(x) for x in force_str.split(',')] if force_str else [0.0] * 5
            torque_ctrl = [float(x) for x in torque_str.split(',')] if torque_str else [0.0] * 5
            if len(force_ctrl) != 5 or len(torque_ctrl) != 5:
                messagebox.showerror('Error', 'Force Ctrl and Torque Ctrl must each have 5 comma-separated values')
                return
            ret = robot.switch_to_imp_force_mode(obj_type, 2000, force_ctrl, torque_ctrl)
            if ret != 0:
                messagebox.showerror('Failed!', f'{obj} switch to force impedance failed: {robot._get_operate_error_msg(ret)}')
                return
        except ValueError as e:
            messagebox.showerror('Error', f'Invalid number format in Force/Torque: {e}')
        except Exception as e:
            messagebox.showerror('Error', f'Force Impedance switch failed: {e}')

    def drag_state(self, obj):

        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        if obj not in ('Arm0', 'Arm1'):
            messagebox.showerror('Error', f'{obj} does not support drag mode')
            return
        try:
            obj_type = self._obj_name_to_type(obj)

            num_joints = 7
            if obj == 'Arm0':
                k_var = self.k_a_entry
                d_var = self.d_a_entry
                cart_k_ver = self.cart_k_a_entry
                cart_d_ver = self.cart_d_a_entry
            if obj == 'Arm1':
                k_var = self.k_b_entry
                d_var = self.d_b_entry
                cart_k_ver = self.cart_k_b_entry
                cart_d_ver = self.cart_d_b_entry
            k_str = k_var.get().strip()
            if not k_str:
                messagebox.showerror("Error", "K parameter cannot be empty!")
                return
            is_valid, result = self.validate_point(k_str, num_joints)
            if not is_valid:
                messagebox.showerror("Error", f"Invalid K format: {result}")
                return
            k_list = [float(x) for x in result.split(',')]

            d_str = d_var.get().strip()
            if not d_str:
                messagebox.showerror("Error", "D parameter cannot be empty!")
                return
            is_valid, result = self.validate_point(d_str, num_joints)
            if not is_valid:
                messagebox.showerror("Error", f"Invalid D format: {result}")
                return
            d_list = [float(x) for x in result.split(',')]

            cart_k_str = cart_k_ver.get().strip()
            if not cart_k_str:
                messagebox.showerror("Error", "K parameter cannot be empty!")
                return
            is_valid, cart_result = self.validate_point(cart_k_str, num_joints)
            if not is_valid:
                messagebox.showerror("Error", f"Invalid K format: {result}")
                return
            cart_k_list = [float(x) for x in cart_result.split(',')]

            cart_d_str = cart_d_ver.get().strip()
            if not d_str:
                messagebox.showerror("Error", "D parameter cannot be empty!")
                return
            is_valid, cart_result = self.validate_point(cart_d_str, num_joints)
            if not is_valid:
                messagebox.showerror("Error", f"Invalid D format: {cart_result}")
                return
            cart_d_list = [float(x) for x in cart_result.split(',')]

            if obj == 'Arm0':
                mode = self.drag_combo.get()
            else:
                mode = self.drag_combo_r.get()
            if mode == "joint":
                ret = robot.switch_to_drag_joint(obj_type, 1000, k_list, d_list)
            elif mode == "cartX":
                ret = robot.switch_to_drag_cart_x(obj_type, 1000, cart_k_list, cart_d_list)
            elif mode == "cartY":
                ret = robot.switch_to_drag_cart_y(obj_type, 1000, cart_k_list, cart_d_list)
            elif mode == "cartZ":
                ret = robot.switch_to_drag_cart_z(obj_type, 1000, cart_k_list, cart_d_list)
            elif mode == "cartR":
                ret = robot.switch_to_drag_cart_r(obj_type, 1000, cart_k_list, cart_d_list)
            else:
                messagebox.showerror('Error', f'Unknown drag mode: {mode}')
                return
            if ret != 0:
                messagebox.showerror('Failed!', f'{obj} switch to {mode} drag failed: {robot._get_operate_error_msg(ret)}')
        except Exception as e:
            messagebox.showerror('Error', f'Drag mode switch failed: {e}')

    def error_get(self, obj):
        """Get servo error codes for the specified object and display in hex format."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        try:
            obj_type = self._obj_name_to_type(obj)
            ret,msg = robot.get_servo_error_codes(obj_type)
            if ret !=0:
                messagebox.showerror("Failed!", f"{obj} get error codes failed: {robot._get_operate_error_msg(ret)}")
                return
            else:
                messagebox.showinfo(f'{obj} Servo Error Details:', msg)
        except Exception as e:
            messagebox.showerror('Error', f"Failed to get error codes: {e}")

    def release_brake(self, obj):
        """Release brake for the specified object (unlock)."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        try:
            result = messagebox.askokcancel("Confirm",
                                            f"Confirm to perform the UnBrake operation on {obj}?")
            if result:
                obj_type = self._obj_name_to_type(obj)
                if obj in ('Arm0', 'Arm1'):
                    axis_mask = 0x7F  # 7 axes
                elif obj == 'Body':
                    axis_mask = 0x3F  # 6 axes
                elif obj == 'Head':
                    axis_mask = 0x07  # 3 axes
                elif obj == 'Lift':
                    axis_mask = 0x03  # 2 axes
                else:
                    axis_mask = 0xFF
                ret=robot.config_brake_unlock(obj_type, axis_mask)
                if ret!= 0:
                    messagebox.showerror('Failed!', f"{obj} release brake failed: {robot._get_operate_error_msg(ret)}")
                    return
        except Exception as e:
            messagebox.showerror('Error', f"Release brake failed: {e}")

    def brake(self, obj):
        """Apply brake (lock) for the specified object."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        try:
            obj_type = self._obj_name_to_type(obj)
            # Lock brakes for all axes
            if obj in ('Arm0', 'Arm1'):
                axis_mask = 0x7F
            elif obj == 'Body':
                axis_mask = 0x3F
            elif obj == 'Head':
                axis_mask = 0x07
            elif obj == 'Lift':
                axis_mask = 0x03
            else:
                axis_mask = 0xFF
            ret=robot.config_brake_lock(obj_type, axis_mask)
            if ret!= 0:
                messagebox.showerror('Failed!', f"{obj} brake lock failed: {robot._get_operate_error_msg(ret)}")
                return
        except Exception as e:
            messagebox.showerror('Error', f"Brake lock failed: {e}")

    def system_update_dialog(self):
        if not self.connected:
            messagebox.showerror('Error', "Please connect robot first!")
            return
        hidden_window = tk.Toplevel(self.root)
        hidden_window.title("System Upgrade")
        hidden_window.geometry("800x400")
        hidden_window.configure(bg="white")
        hidden_window.transient(self.root)
        hidden_window.resizable(True, True)
        hidden_window.grab_set()

        title_frame = tk.Frame(hidden_window, bg="white")
        title_frame.pack(fill="x", padx=5, pady=(15, 10))
        title_label = tk.Label(title_frame, text="Robot configuration file", bg="#2196F3",
                               fg="white", font=("Arial", 10, "bold"))
        title_label.pack(fill='x')

        state_a_frame = tk.Frame(hidden_window, bg="white")
        state_a_frame.pack(fill="x", pady=5)
        self.download_ini_path = tk.StringVar()
        reset_a_button = tk.Button(state_a_frame, text="Download current config", width=30,
                                   command=self.get_ini)
        reset_a_button.pack(side='left', expand=True)

        param_c_btn = tk.Button(state_a_frame, text='Configuration comparison', width=30,
                                command=self.compare_parameters_dialog, fg='#033341', bg='#DFC88C')
        param_c_btn.pack(side='left', expand=True)

        '''====system upgrade===='''
        self.system_pkg_path = tk.StringVar()
        self.config_pkg_path = tk.StringVar()

        update_frame0 = tk.Frame(hidden_window, bg="white")
        update_frame0.pack(fill="x", padx=5, pady=(25, 10))

        tk.Label(update_frame0, text="Update Operations", bg="#2196F3",
                 fg="white", font=("Arial", 10, "bold")).pack(fill='x', expand=True)

        update_frame1 = tk.Frame(hidden_window, bg="white")
        update_frame1.pack(fill="x", padx=5, pady=(10, 5))

        btn1 = tk.Button(update_frame1, text="Select system upgrade package", width=40,
                         command=self.choose_system_pkg)
        btn1.pack(side='left', padx=(0, 10))

        lbl1 = tk.Label(update_frame1, textvariable=self.system_pkg_path, bg="white",
                        fg="gray", anchor="w", width=50)
        lbl1.pack(side='left', fill='x', expand=True)

        update_frame2 = tk.Frame(hidden_window, bg="white")
        update_frame2.pack(fill="x", padx=5, pady=(5, 10))

        btn2 = tk.Button(update_frame2, text="Select config file", width=40,
                         command=self.choose_config_pkg)
        btn2.pack(side='left', padx=(0, 10))

        lbl2 = tk.Label(update_frame2, textvariable=self.config_pkg_path, bg="white",
                        fg="gray", anchor="w", width=50)
        lbl2.pack(side='left', fill='x', expand=True)

        update_frame3 = tk.Frame(hidden_window, bg="white")
        update_frame3.pack(fill="x", padx=5, pady=(25, 10))

        tk.Button(update_frame3, text="Update", width=30,
                  command=self.update_sys, bg="#F6FC39",
                  fg="#151513",
                  font=("Arial", 10, "bold")).pack(side='left', expand=True)

        tips_text_frame = tk.Frame(hidden_window, bg="white")
        tips_text_frame.pack(fill="x", pady=5)
        label = tk.Label(tips_text_frame,
                         text='Configuration files and system packages can be updated individually or simultaneously. \nAfter the update, the system will restart, and the software must reconnect.',
                         bg='white')
        label.pack(padx=5, pady=10)

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

    def compare_parameters_dialog(self):
        compare_window = tk.Toplevel(self.root)
        compare_window.title("Configuration comparison update")
        compare_window.geometry("1000x900")
        compare_window.configure(bg="#f0f0f0")
        compare_window.transient(self.root)
        compare_window.resizable(True, True)
        compare_window.grab_set()

        self.template_path = ""
        self.target_path = ""
        self.template_structure = {}

        top_frame = tk.Frame(compare_window, bg="#f0f0f0")
        top_frame.pack(pady=5, fill=tk.X)

        btn_template = tk.Button(top_frame, text="Select Template File",
                                 command=lambda: self.load_template(compare_window))
        btn_template.pack(side=tk.LEFT, padx=5)

        self.template_label = tk.Label(top_frame, text="No template file selected", fg="gray", bg="#f0f0f0")
        self.template_label.pack(side=tk.LEFT, padx=5)

        btn_target = tk.Button(top_frame, text="Select Target File", command=lambda: self.load_target(compare_window))
        btn_target.pack(side=tk.LEFT, padx=5)

        self.target_label = tk.Label(top_frame, text="No target file selected", fg="gray", bg="#f0f0f0")
        self.target_label.pack(side=tk.LEFT, padx=5)

        middle_frame = tk.Frame(compare_window, bg="#f0f0f0")
        middle_frame.pack(pady=5, fill=tk.BOTH, expand=True)

        left_frame = tk.LabelFrame(middle_frame, text="Template File Content", bg="#f0f0f0")
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        self.template_text = scrolledtext.ScrolledText(left_frame, wrap=tk.NONE, font=("Consolas", 10))
        self.template_text.pack(fill=tk.BOTH, expand=True)
        self.template_text.config(state=tk.DISABLED)

        right_frame = tk.LabelFrame(middle_frame, text="Target File Content (editable)", bg="#f0f0f0")
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)
        self.target_text = scrolledtext.ScrolledText(right_frame, wrap=tk.NONE, font=("Consolas", 10))
        self.target_text.pack(fill=tk.BOTH, expand=True)

        bottom_frame = tk.Frame(compare_window, bg="#f0f0f0")
        bottom_frame.pack(pady=5, fill=tk.BOTH, expand=True)

        report_frame = tk.LabelFrame(bottom_frame, text="Missing Report", bg="#f0f0f0")
        report_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.report_text = scrolledtext.ScrolledText(report_frame, wrap=tk.WORD, height=10, font=("Segoe UI", 9))
        self.report_text.pack(fill=tk.BOTH, expand=True)
        self.report_text.config(state=tk.DISABLED)

        action_frame = tk.Frame(compare_window, bg="#f0f0f0")
        action_frame.pack(pady=5, fill=tk.X)

        btn_refresh = tk.Button(action_frame, text="Refresh Comparison",
                                command=lambda: self.refresh_comparison(compare_window))
        btn_refresh.pack(side=tk.LEFT, padx=5)

        btn_save = tk.Button(action_frame, text="Save Target As New File",
                             command=lambda: self.save_target(compare_window), bg="lightgreen")
        btn_save.pack(side=tk.LEFT, padx=5)

        btn_close = tk.Button(action_frame, text="Close", command=compare_window.destroy)
        btn_close.pack(side=tk.RIGHT, padx=5)

    def load_template(self, parent):
        file_path = filedialog.askopenfilename(
            title="Select Template File",
            filetypes=[("INI files", "*.ini"), ("All files", "*.*")]
        )
        if not file_path:
            return
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                content = f.read()
        except UnicodeDecodeError:
            try:
                with open(file_path, "r", encoding="gbk") as f:
                    content = f.read()
            except Exception as e:
                messagebox.showerror("Error", f"Cannot read template file: {e}")
                return
        except Exception as e:
            messagebox.showerror("Error", f"Cannot read template file: {e}")
            return

        self.template_text.config(state=tk.NORMAL)
        self.template_text.delete(1.0, tk.END)
        self.template_text.insert(tk.END, content)
        self.template_text.config(state=tk.DISABLED)

        self.template_structure = self.parse_text_to_structure(content)
        self.template_path = file_path
        self.template_label.config(text=os.path.basename(file_path), fg="black")

        if self.target_text.get(1.0, tk.END).strip():
            self.refresh_comparison(parent)
        else:
            self.clear_report("Loaded template. Load target file and click Refresh Comparison.")

    def load_target(self, parent):
        file_path = filedialog.askopenfilename(
            title="Select Target File",
            filetypes=[("INI files", "*.ini"), ("All files", "*.*")]
        )
        if not file_path:
            return
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                content = f.read()
        except UnicodeDecodeError:
            try:
                with open(file_path, "r", encoding="gbk") as f:
                    content = f.read()
            except Exception as e:
                messagebox.showerror("Error", f"Cannot read target file: {e}")
                return
        except Exception as e:
            messagebox.showerror("Error", f"Cannot read target file: {e}")
            return

        self.target_text.delete(1.0, tk.END)
        self.target_text.insert(tk.END, content)
        self.target_path = file_path
        self.target_label.config(text=os.path.basename(file_path), fg="black")

        if self.template_structure:
            self.refresh_comparison(parent)

    def parse_text_to_structure(self, text):
        structure = {}
        current_section = None
        lines = text.splitlines()
        for line in lines:
            line = line.strip()
            if not line:
                continue
            if line.startswith('[') and line.endswith(']'):
                current_section = line[1:-1].strip()
                if current_section not in structure:
                    structure[current_section] = set()
            elif '=' in line:
                if current_section is None:
                    continue
                key = line.split('=', 1)[0].strip()
                structure[current_section].add(key)
        return structure

    def refresh_comparison(self, parent):
        if not self.template_structure:
            self.clear_report("Please select a template file first.")
            return

        target_content = self.target_text.get(1.0, tk.END)
        target_structure = self.parse_text_to_structure(target_content)

        missing_sections, missing_keys_per_section = self.compare_structures(self.template_structure, target_structure)
        self.display_report(missing_sections, missing_keys_per_section)

    def compare_structures(self, template, target):
        missing_sections = []
        missing_keys_per_section = {}

        for section, tmpl_keys in template.items():
            if section not in target:
                missing_sections.append(section)
            else:
                target_keys = target[section]
                missing_keys = tmpl_keys - target_keys
                if missing_keys:
                    missing_keys_per_section[section] = sorted(missing_keys)

        return missing_sections, missing_keys_per_section

    def display_report(self, missing_sections, missing_keys_per_section):
        self.report_text.config(state=tk.NORMAL)
        self.report_text.delete(1.0, tk.END)

        if not missing_sections and not missing_keys_per_section:
            self.report_text.insert(tk.END, "Success! No missing sections or keys in target file.\n")
        else:
            if missing_sections:
                self.report_text.insert(tk.END, "Missing Sections:\n", "header")
                for sec in missing_sections:
                    self.report_text.insert(tk.END, f"  - [{sec}]\n", "section")
                self.report_text.insert(tk.END, "\n")

            if missing_keys_per_section:
                self.report_text.insert(tk.END, "Missing Keys in Sections:\n", "header")
                for sec, keys in missing_keys_per_section.items():
                    self.report_text.insert(tk.END, f"  Section [{sec}] missing keys:\n", "section")
                    for key in keys:
                        self.report_text.insert(tk.END, f"      - {key}\n", "key")
                    self.report_text.insert(tk.END, "\n")

            self.report_text.insert(tk.END, "Tip: Add the missing sections/keys in the right text box, then save.")

        self.report_text.tag_config("header", foreground="blue", font=("Segoe UI", 9, "bold"))
        self.report_text.tag_config("section", foreground="darkgreen", font=("Consolas", 9))
        self.report_text.tag_config("key", foreground="maroon", font=("Consolas", 9))
        self.report_text.config(state=tk.DISABLED)

    def clear_report(self, msg):
        self.report_text.config(state=tk.NORMAL)
        self.report_text.delete(1.0, tk.END)
        self.report_text.insert(tk.END, msg)
        self.report_text.config(state=tk.DISABLED)

    def save_target(self, parent):
        content = self.target_text.get(1.0, tk.END).rstrip()
        if not content.strip():
            messagebox.showwarning("Warning", "Target text box is empty. Nothing to save.")
            return

        initial_dir = os.path.dirname(self.target_path) if self.target_path else os.getcwd()
        initial_file = "modified_" + os.path.basename(self.target_path) if self.target_path else "new_config.ini"
        save_path = filedialog.asksaveasfilename(
            title="Save Target File As",
            initialdir=initial_dir,
            initialfile=initial_file,
            defaultextension=".txt",
            filetypes=[("INI files", "*.ini"), ("All files", "*.*")]
        )
        if not save_path:
            return

        try:
            with open(save_path, "w", encoding="utf-8") as f:
                f.write(content)
            messagebox.showinfo("Success", f"File saved to:\n{save_path}")
            self.target_path = save_path
            self.target_label.config(text=os.path.basename(save_path), fg="black")
        except Exception as e:
            messagebox.showerror("Save Failed", f"Cannot save file: {e}")

    def get_ini(self):
        if self.connected:
            file_path = filedialog.asksaveasfilename(
                defaultextension=".ini",
                filetypes=[("ini files", "*.ini"), ("All files", "*.*")],
                title="Save robot configuration parameter file"
            )
            if file_path:
                if robot.recv_file(file_path, "/home/FUSION/Config/cfg/robot.ini") == 0:
                    messagebox.showinfo('Success', 'Configuration file has been saved')
                    self.download_ini_path.set(file_path)
                else:
                    messagebox.showerror("Failed", "Receive file failed")

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
                if robot.send_file(file_path, "/home/FUSION/Config/cfg/robot.ini") == 0:
                    messagebox.showinfo('Success', 'Configuration file has been saved')
                else:
                    messagebox.showinfo('Failed', 'Configuration file download failed')
        else:
            messagebox.showerror('Error', 'Please connect robot')

    def choose_system_pkg(self):
        self.system_file_path = filedialog.askopenfilename(
            filetypes=[("All files", "*.UPDATE")],
            title="Select system update file"
        )
        if self.system_file_path:
            self.system_pkg_path.set(self.system_file_path)

    def choose_config_pkg(self):
        self.ini_file_path = filedialog.askopenfilename(
            filetypes=[("All files", "*.ini")],
            title="Select ini update file"
        )
        if self.ini_file_path:
            self.config_pkg_path.set(self.ini_file_path)

    def update_sys(self):
        if self.connected:
            result = messagebox.askokcancel("Confirm",
                                            "After uploading the file, restarting will automatically update the system version. \nConfirm the upload?")
            if result:
                print(f"----{self.system_file_path, self.ini_file_path}")
                if not self.system_file_path and not self.ini_file_path:
                    messagebox.showerror('Error', 'Please select update package or ini file')
                    return
                if robot.system_update(self.system_file_path, self.ini_file_path) == 0:
                    messagebox.showinfo('Success',
                                        'The system files have been uploaded. Please restart the controller to update automatically.')
                else:
                    messagebox.showerror('Error', 'System file upload failed, please upload again.')
                    return
        else:
            messagebox.showerror('Error', 'Please connect robot')


if __name__ == "__main__":
    DBL_EPSILON = sys.float_info.epsilon
    arm_main_state_with = 130
    data_queue = queue.Queue()
    crr_pth = os.getcwd()
    robot = GentoRobot()
    robot.enable_lock()
    sdk_version = robot.get_sdk_version()

    root = tk.Tk()
    style = ttk.Style()
    style.configure(
        "MyCustom.TLabelframe",
        font=("Arial", 12, "italic"),
        foreground="darkblue",
        background="white"
    )
    app = App(root)
    root.mainloop()
