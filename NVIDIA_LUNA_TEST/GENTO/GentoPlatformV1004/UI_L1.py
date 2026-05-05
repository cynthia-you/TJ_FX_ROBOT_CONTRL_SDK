import tkinter as tk
from tkinter import messagebox, ttk, scrolledtext, filedialog, simpledialog
import threading
import time
import queue
import os
import math
import sys
from tkinter.constants import DISABLED

from PythonSdk.GentoRobot import GentoRobot, RobotDataManager, error_dict, FXObjType, FXObjMask, FXTerminalType
import ast


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


def FX_VectCross(a, b):
    result = [0.0] * 3
    result[0] = a[1] * b[2] - a[2] * b[1]
    result[1] = a[2] * b[0] - a[0] * b[2]
    result[2] = a[0] * b[1] - a[1] * b[0]
    return result


def NormVect(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def main_function(vx, vy):
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


class App:
    def __init__(self, root):
        self.root = root
        self.root.title("GentoPlatformV1004")
        self.root.geometry("1350x800")
        self.root.configure(bg="#f0f0f0")

        self.data_manager = None
        self.rt = None
        self.sg = None

        self.version = ''
        self.drag_mode = False

        self.params = []
        self.init_kd_variables()
        self.points1 = []
        self.points2 = []
        self.body_points = []
        self.head_points = []
        self.lift_points = []

        self.command1 = []
        self.command2 = []

        self.display_mode = 0



        self.mode_names = ["Position","CmdPosition", "Velocity", "SensorTorque", "TorqueExt","MotorTorque","ExtPosition" ]
        self.data_keys = [('fb_pos'), ('cmd_pos'), ('fb_vel'), ('fb_sensor'), ('fb_ext_torque'),('joint_torque'),('ext_pos')]


        self.arm_rt_key=[('fb_pos'), ('cmd_pos'), ('fb_vel'), ('fb_sensor'), ('fb_ext_torque')]
        self.arm_sg_key=[('joint_torque'),('ext_pos')]

        self.body_rt_key=[('fb_pos'), ('cmd_pos'), ('fb_vel'), ('fb_sensor')]
        self.body_sg_key=[('joint_torque'),('ext_pos')]

        self.head_rt_key=[('fb_pos'), ('cmd_pos')]
        self.head_sg_key=[('ext_pos')]

        self.lift_rt_key=[('fb_pos'), ('cmd_pos')]
        self.lift_sg_key=[('joint_torque')]

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
                                       anchor='w', wraplength=120, justify='left')
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
        data_frame = ttk.LabelFrame(right_frame, text="Realtime Data", padding=10,
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
                                        anchor='w', wraplength=120, justify='left')
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
        data_frame = ttk.LabelFrame(right_frame, text="Realtime Data", padding=10,
                                    relief=tk.GROOVE, borderwidth=2,
                                    style="MyCustom.TLabelframe")
        data_frame.pack(fill="x", pady=(0, 5))

        # Joint positions row
        joint_frame = tk.Frame(data_frame, bg="white")
        joint_frame.pack(fill="x", pady=(0, 5))
        tk.Label(joint_frame, text="J1~J7:", font=('Arial', 10, 'bold'), width=6,
                 bg='white').pack(side="left", padx=(0, 2))
        self.r_joint_text = tk.Text(joint_frame, width=55, height=1,
                                    font=('Arial', 9), bg='white',
                                    relief=tk.SUNKEN, bd=1, wrap=tk.NONE)
        self.r_joint_text.tag_configure("center", justify='center')
        self.r_joint_text.pack(side="left", fill="x")
        self.r_joint_text.insert("1.0", "0.000,0.000,0.000,0.000,0.000,0.000,0.000")
        self.r_joint_text.tag_add("center", "1.0", "end")
        self.r_joint_text.config(state="disabled")

        # # Cartesian positions row
        # cart_frame = tk.Frame(data_frame, bg="white")
        # cart_frame.pack(fill="x", pady=(0, 5))
        # tk.Label(cart_frame, text="XYZABC (flange):", font=('Arial', 10, 'bold'),
        #          bg='white').pack(side="left", padx=(0, 2))
        # self.right_cartesian_text = tk.Text(cart_frame, width=55, height=1,
        #                                     font=('Arial', 9), bg='white',
        #                                     relief=tk.SUNKEN, bd=1, wrap=tk.NONE)
        # self.right_cartesian_text.tag_configure("center", justify='center')
        # self.right_cartesian_text.pack(side="left", fill="x", expand=True)
        # self.right_cartesian_text.insert("1.0", "0.000,0.000,0.000,0.000,0.000,0.000")
        # self.right_cartesian_text.tag_add("center", "1.0", "end")
        # self.right_cartesian_text.config(state="disabled")

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
                                          anchor='w', wraplength=120, justify='left')
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
        self.body_jointimp_btn = tk.Button(state_row1, text="Torque", width=10,
                                           command=lambda: self.jointImp_state('Body'), bg="#d9d0ca", fg="black",
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
        self.body_brake_btn = tk.Button(control_frame, text="Brake", width=10, state=tk.DISABLED,
                                        font=("Arial", 10, "bold"))
        self.body_brake_btn.pack(side="left", padx=(0, 5))
        self.body_unbrake_btn = tk.Button(control_frame, text="UnBrake", width=10, state=tk.DISABLED,
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
                                          anchor='w', wraplength=120, justify='left')
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
        self.head_brake_btn = tk.Button(control_frame, text="Brake", width=10, command=None, font=("Arial", 10, "bold"))
        self.head_brake_btn.pack(side="left", padx=(0, 5))
        self.head_unbrake_btn = tk.Button(control_frame, text="UnBrake", width=10, command=None,
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
        self.head_cmd_entry.insert(0, "0,0,0,0,0,0")
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
                                          anchor='w', wraplength=120, justify='left')
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
        self.arm_ip_entry.insert(0, "6,6,7,190")
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


        # Mode switch button
        self.mode_btn = tk.Button(
            self.control_frame,
            text="Position",
            width=15,
            command=self.toggle_display_mode,
            bg="#3BA4FD",
            fg="white",
            font=("Arial", 10, "bold"))
        self.mode_btn.pack(side="right", padx=5)

        # Status indicator
        status_frame = tk.Frame(self.control_frame, bg="#e0e0e0")
        status_frame.pack(side="right", padx=5)
        self.status_light = tk.Label(status_frame, text="●", font=("Arial", 16), fg="red")
        self.status_light.pack(side="left", padx=5)
        self.status_label = tk.Label(status_frame, text="disconnected", bg="#e0e0e0", font=("Arial", 9))
        self.status_label.pack(side="left")

    def create_status_bar(self):
        """Create the bottom status bar"""
        self.status_bar = tk.Frame(self.root, height=20)
        self.status_bar.pack(side="bottom", fill="x")
        self.version_label = tk.Label(
            self.status_bar, text=f"", fg="black", font=("Arial", 9))
        self.version_label.pack(side="left", padx=15)
        self.time_label = tk.Label(
            self.status_bar, text="", fg="black", font=("Arial", 9))
        self.time_label.pack(side="right", padx=15)
        self.update_time()

    # ==================== Core Control Methods (Adapted to new MarvinRobot API) ====================
    def toggle_connection(self):
        global_robot_ip = self.arm_ip_entry.get()
        if not self.connected:
            try:
                ip_parts = [int(x) for x in global_robot_ip.split(',')]
                if len(ip_parts) != 4:
                    raise ValueError
                version = robot.link(ip_parts[0], ip_parts[1], ip_parts[2], ip_parts[3], log_switch=1)
                if version <= 0:
                    messagebox.showerror('Failed!', f"Robot connection failed, error code: {version}")
                    return
                self.version = format(version, '08X')

                self.connected = True
                self.connect_btn.config(text="Disconnect", bg="#F44336")
                self.status_label.config(text="Connected")
                self.status_light.config(fg="green")
                self.mode_btn.config(state="normal")

                self.data_manager = RobotDataManager(robot)
                self.update_data()

                self.refresh_kd_from_sg()

            except Exception as e:
                messagebox.showerror('Error', f"Connection failed: {e}")
        else:
            self.connected = False
            if hasattr(self, 'data_manager') and self.data_manager:
                self.data_manager.stop()
                self.data_manager = None
            self.connect_btn.config(text="Connect Robot", bg="#4CAF50")
            self.status_label.config(text="Disconnected")
            self.status_light.config(fg="red")
            self.mode_btn.config(state="disabled")

    def refresh_kd_from_sg(self):
        """
        Refresh all K/D parameters from the current SG data.
        If the retrieved values are all zero (or invalid), keep the UI defaults.
        """
        if not self.connected or not self.data_manager:
            return

        sg_dict = self.data_manager.latest_sg
        if not sg_dict or "error" in sg_dict:
            # fallback to direct robot.sg if data_manager not ready
            try:
                sg = robot.sg
                # build a temporary dict similar to get_sg_dict for consistency
                sg_dict = robot.get_sg_dict()
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
        joint_pos_l=''
        joint_pos_r=''
        body_pos=''
        head_pos=''
        lift_pos=''
        key = self.data_keys[self.display_mode]

        state_map = {
            0: "IDLE",
            1: "Position",
            2: "ImpJoint",
            3: "ImpCart",
            4: "ImpForce",
            5: "DragJoint",
            6: "DragCartX",
            7: "DragCartY",
            8: "DragCartZ",
            9: "DragCartR",
            10: "Release",
            11: "PD",
            100: "Error",
            101: "Transferring",
            200: "Unknown"
        }
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

    def toggle_display_mode(self):
        self.display_mode = (self.display_mode + 1) % 7
        self.mode_btn.config(text=self.mode_names[self.display_mode])
        self.update_ui()

    def update_time(self):
        current_time = time.strftime("%Y-%m-%d %H:%M:%S")
        self.time_label.config(text=current_time)
        self.version_label.config(text=f"controller version:{self.version}")
        self.root.after(1000, self.update_time)

    def on_mousewheel(self, event):
        self.main_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    def on_menu_click(self, item):
        print(f"Menu clicked: {item}")

    def on_close(self):
        if messagebox.askokcancel("Exit", "Are you sure you want to exit the application?"):
            self.root.destroy()

    def vel_acc_set(self, obj):
        try:
            if not robot.comm_clear(50):
                messagebox.showerror('Failed!', "clear buffer failed")
                return
            if obj == 'Arm0':
                vel = int(self.left_speed_entry.get())
                acc = int(self.left_accel_entry.get())
                robot.runtime_set_vel_ratio(FXObjType.OBJ_ARM0, vel)
                robot.runtime_set_acc_ratio(FXObjType.OBJ_ARM0, acc)
            elif obj == 'Arm1':
                vel = int(self.right_speed_entry.get())
                acc = int(self.right_accel_entry.get())
                robot.runtime_set_vel_ratio(FXObjType.OBJ_ARM1, vel)
                robot.runtime_set_acc_ratio(FXObjType.OBJ_ARM1, acc)
            elif obj == 'Body':
                vel = int(self.body_speed_entry.get())
                acc = int(self.body_accel_entry.get())
                robot.runtime_set_vel_ratio(FXObjType.OBJ_BODY, vel)
                robot.runtime_set_acc_ratio(FXObjType.OBJ_BODY, acc)
            elif obj == 'Head':
                vel = int(self.head_speed_entry.get())
                acc = int(self.head_accel_entry.get())
                robot.runtime_set_vel_ratio(FXObjType.OBJ_HEAD, vel)
                robot.runtime_set_acc_ratio(FXObjType.OBJ_HEAD, acc)
            elif obj == 'Lift':
                vel = int(self.lift_speed_entry.get())
                acc = int(self.lift_accel_entry.get())
                robot.runtime_set_vel_ratio(FXObjType.OBJ_LIFT, vel)
                robot.runtime_set_acc_ratio(FXObjType.OBJ_LIFT, acc)
            else:
                raise ValueError(f"Unknown obj: {obj}")
            robot.comm_send()
        except Exception as e:
            messagebox.showerror('Error', f"Operation failed: {e}")

    def reset_error(self, obj):
        try:
            mask = self._obj_name_to_mask(obj)
            robot.reset_error(mask)

        except Exception as e:
            messagebox.showerror('Error', f"Reset failed: {e}")

    def idle_state(self, obj):
        try:
            obj_type = self._obj_name_to_type(obj)
            robot.switch_to_idle(obj_type, 1000)
        except Exception as e:
            messagebox.showerror('Error', f"Set idle failed: {e}")

    def cr_state(self, obj):
        if obj not in ('Arm0', 'Arm1'):
            messagebox.showerror('Error', f'Invalid obj: {obj}')
            return
        try:
            arm_idx = 0 if obj == 'Arm0' else 1
            robot.switch_to_collab_release(arm_idx, 1000)
        except Exception as e:
            messagebox.showerror('Error', f"CR failed: {e}")

    def position_state(self, obj):
        try:
            obj_type = self._obj_name_to_type(obj)
            # Get current speed/acceleration (default 20 if not set)
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
            robot.switch_to_position_mode(obj_type, 1000, vel, acc)
        except Exception as e:
            messagebox.showerror('Error', f"Set position state failed: {e}")

    def get_current_pos(self, obj):
        try:
            pose = None
            if obj == 'Arm0':
                pose = self.rt["arms"][0]["fb"]["joint_pos"]
                print(f"pose:{pose}")
                if pose and len(pose) == 7:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.entry.delete(0, tk.END)
                    self.entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Arm0')
            elif obj == 'Arm1':
                pose = self.rt["arms"][1]["fb"]["joint_pos"]
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
            elif obj == 'Head':
                pose = self.rt["head"]["fb_pos"]
                if pose and len(pose) == 3:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.head_cmd_entry.delete(0, tk.END)
                    self.head_cmd_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Head')
            elif obj == 'Lift':
                pose = self.rt["lift"]["fb_pos"]
                if pose and len(pose) == 2:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.lift_cmd_entry.delete(0, tk.END)
                    self.lift_cmd_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Lift')
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

        if self.is_duplicate(result, points_list):
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

        self._set_combo_selection(self.combo1, self.points1)
        self._set_combo_selection(self.combo2, self.points2)
        self._set_combo_selection(self.body_combo, self.body_points)
        self._set_combo_selection(self.head_combo, self.head_points)
        self._set_combo_selection(self.lift_combo, self.lift_points)

    def _set_combo_selection(self, combo, points_list):
        if points_list:
            combo.current(0)
        else:
            combo.set('')
        combo.update_idletasks()

    def is_duplicate(self, point_list, target_list):
        point_tuple = tuple(point_list)
        for existing_point_str in target_list:
            try:
                existing_point = ast.literal_eval(existing_point_str)
                if tuple(existing_point) == point_tuple:
                    return True
            except:
                continue
        return False

    def run_pos(self, obj):
        try:
            if obj == 'Arm0':
                selected = self.combo1.get()
                print(f'arm0:{selected}')
                if selected:
                    is_valid, value_str = self.validate_point(selected, 7)
                    if is_valid:
                        values = value_str.split(',')
                        point_list = [float(value.strip()) for value in values]
                        if not robot.comm_clear(50):
                            messagebox.showerror('Failed!', "clear buffer failed")
                        success = robot.runtime_set_joint_pos_cmd(FXObjType.OBJ_ARM0, point_list)
                        if not success:
                            messagebox.showerror('Failed!', f"set {obj} run pose failed")
                        robot.comm_send()
                        time.sleep(0.1)
                    else:
                        messagebox.showerror("Error", f"Invalid format: {selected}")
                else:
                    messagebox.showwarning("Warning", "No point selected for Arm0")
            elif obj == 'Arm1':
                selected = self.combo2.get()
                print(f'arm1:{selected}')
                if selected:
                    is_valid, value_str = self.validate_point(selected, 7)
                    if is_valid:
                        values = value_str.split(',')
                        point_list = [float(value.strip()) for value in values]
                        if not robot.comm_clear(50):
                            messagebox.showerror('Failed!', "clear buffer failed")
                        success = robot.runtime_set_joint_pos_cmd(FXObjType.OBJ_ARM1, point_list)
                        if not success:
                            messagebox.showerror('Failed!', f"set {obj} run pose failed")
                        robot.comm_send()
                        time.sleep(0.1)
                    else:
                        messagebox.showerror("Error", f"Invalid format: {selected}")
                else:
                    messagebox.showwarning("Warning", "No point selected for Arm1")
            elif obj == 'Body':
                selected = self.body_combo.get()
                print(f'body:{selected}')
                if selected:
                    is_valid, value_str = self.validate_point(selected, 6)
                    if is_valid:
                        values = value_str.split(',')
                        point_list = [float(value.strip()) for value in values]
                        if not robot.comm_clear(50):
                            messagebox.showerror('Failed!', "clear buffer failed")
                        success = robot.runtime_set_joint_pos_cmd(FXObjType.OBJ_BODY, point_list)
                        if not success:
                            messagebox.showerror('Failed!', f"set {obj} run pose failed")
                        robot.comm_send()
                        time.sleep(0.1)
                    else:
                        messagebox.showerror("Error", f"Invalid format: {selected}")
                else:
                    messagebox.showwarning("Warning", "No point selected for Body")
            elif obj == 'Head':
                selected = self.head_combo.get()
                print(f'head:{selected}')
                if selected:
                    is_valid, value_str = self.validate_point(selected, 3)
                    if is_valid:
                        values = value_str.split(',')
                        point_list = [float(value.strip()) for value in values]
                        if not robot.comm_clear(50):
                            messagebox.showerror('Failed!', "clear buffer failed")
                        success = robot.runtime_set_joint_pos_cmd(FXObjType.OBJ_HEAD, point_list)
                        if not success:
                            messagebox.showerror('Failed!', f"set {obj} run pose failed")
                        robot.comm_send()
                        time.sleep(0.1)
                    else:
                        messagebox.showerror("Error", f"Invalid format: {selected}")
                else:
                    messagebox.showwarning("Warning", "No point selected for Head")
            elif obj == 'Lift':
                selected = self.lift_combo.get()
                print(f'lift:{selected}')
                if selected:
                    is_valid, value_str = self.validate_point(selected, 2)
                    if is_valid:
                        values = value_str.split(',')
                        point_list = [float(value.strip()) for value in values]
                        if not robot.comm_clear(50):
                            messagebox.showerror('Failed!', "clear buffer failed")
                        success = robot.runtime_set_joint_pos_cmd(FXObjType.OBJ_LIFT, point_list)
                        if not success:
                            messagebox.showerror('Failed!', f"set {obj} run pose failed")
                        robot.comm_send()
                        time.sleep(0.1)
                    else:
                        messagebox.showerror("Error", f"Invalid format: {selected}")
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

            if not robot.comm_clear(50):
                messagebox.showerror('Failed!', "Clear buffer failed")
                return
            if not robot.runtime_set_tool_kd(FXObjType.OBJ_ARM0,k0_list,d0_list):
                messagebox.showerror('Failed!', f"set tools failed for arm0")
            if not robot.runtime_set_tool_kd(FXObjType.OBJ_ARM1,k1_list,d1_list):
                messagebox.showerror('Failed!', f"set tools failed for arm1")
            robot.comm_send()
        except Exception as e:
            messagebox.showerror('Error', str(e))

    def eef_dialog(self):
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

            robot.terminal_clear(terminal)
            time.sleep(0.02)
            robot.terminal_set(terminal, com, sample_data)

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

    def get_abc_calculation(self):
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
        menu.add_command(label="Motion Planning", command=self.planning_dialog)
        menu.add_separator()
        menu.add_command(label="FileClient", command=self.file_client_dialog)
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
        self.get_offset_btn_1.grid(row=0, column=1, padx=(0,20))

        self.axis_text__ = tk.Label(self.sensor_frame_1, text="Arm1", bg="#F4E4D8")
        self.axis_text__.grid(row=0, column=2, padx=(5, 5))

        self.get_offset_btn_2 = tk.Button(self.sensor_frame_1, text="ResetOffset",
                                          command=lambda: self.clear_sensor_offset('Arm1'))
        self.get_offset_btn_2.grid(row=0, column=3, padx=(0,20))

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
        self.motor_text_1 = tk.Label(self.motor_frame_1, text="Arm0", bg="#D8F4F3",width=8)
        self.motor_text_1.grid(row=0, column=0, padx=(5, 5))

        self.motor_btn_1 = tk.Button(self.motor_frame_1, text="Motor encoder zeroing",
                                     command=lambda: self.clear_motor_as_zero('Arm0',self.motor_btn_1))
        self.motor_btn_1.grid(row=0, column=1, padx=5, pady=5)

        self.disable_soft_btn_1 = tk.Button(self.motor_frame_1, text="Disable SoftLimit",
                                            command=lambda: self.disable_soft_limit(FXObjType.OBJ_ARM0, 0xFF))
        self.disable_soft_btn_1.grid(row=0, column=2, padx=5)

        self.motor_btn_3 = tk.Button(self.motor_frame_1, text="Encoder clearing error", bg="#7ED2B4",state="disabled")
                                     # command=lambda: self.clear_motor_error('Arm0'))
        self.motor_btn_3.grid(row=0, column=3, padx=5)

        '''right arm'''
        self.motor_frame_2 = tk.Frame(sensor_encoder_window, bg="white")
        self.motor_frame_2.pack(fill="x")

        self.motor_text_1 = tk.Label(self.motor_frame_2, text="Arm1", bg="#F4E4D8",width=8)
        self.motor_text_1.grid(row=0, column=0, padx=(5, 5))

        self.motor_btn_11 = tk.Button(self.motor_frame_2, text="Motor encoder zeroing",
                                      command=lambda: self.clear_motor_as_zero('Arm1',self.motor_btn_11))
        self.motor_btn_11.grid(row=0, column=1, padx=5)

        self.disable_soft_btn_2 = tk.Button(self.motor_frame_2, text="Disable SoftLimit",
                                            command=lambda: self.disable_soft_limit(FXObjType.OBJ_ARM1, 0xFF))
        self.disable_soft_btn_2.grid(row=0, column=2, padx=5)

        self.motor_btn_31 = tk.Button(self.motor_frame_2, text="Encoder clearing error", bg="#7ED2B4",state="disabled")
                                      # command=lambda: self.clear_motor_error('Arm1'))
        self.motor_btn_31.grid(row=0, column=3, padx=5)

        '''body'''
        motor_frame_3 = tk.Frame(sensor_encoder_window, bg="white")
        motor_frame_3.pack(fill="x")
        motor_text_3 = tk.Label(motor_frame_3, text="Body", bg="#B0C4DE",width=8)
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
        motor_text_4 = tk.Label(motor_frame_4, text="Head", bg="#D8BFD8",width=8)
        motor_text_4.grid(row=0, column=0, padx=(5, 5))
        self.motor_btn_head= tk.Button(motor_frame_4, text="Motor encoder zeroing",
                                       command=lambda: self.clear_motor_as_zero('Head', self.motor_btn_head))
        self.motor_btn_head.grid(row=0, column=1, padx=5, pady=5)
        self.disable_soft_btn_head = tk.Button(motor_frame_4, text="Disable SoftLimit",
                                               command=lambda: self.disable_soft_limit(FXObjType.OBJ_HEAD, 0xFF))
        self.disable_soft_btn_head.grid(row=0, column=3, padx=5)


        '''Lift'''
        motor_frame_5 = tk.Frame(sensor_encoder_window, bg="white")
        motor_frame_5.pack(fill="x")
        motor_text_5 = tk.Label(motor_frame_5, text="Lift", bg="#FFFACD",width=8)
        motor_text_5.grid(row=0, column=0, padx=(5, 5))
        self.motor_btn_lift = tk.Button(motor_frame_5, text="Motor encoder zeroing",
                                     command=lambda: self.clear_motor_as_zero('Lift', self.motor_btn_lift))
        self.motor_btn_lift.grid(row=0, column=1, padx=5, pady=5)
        self.disable_soft_btn_lift = tk.Button(motor_frame_5, text="Disable SoftLimit",
                                               command=lambda: self.disable_soft_limit(FXObjType.OBJ_LIFT, 0x03))
        self.disable_soft_btn_lift.grid(row=0, column=3, padx=5)


    def planning_dialog(self):
        planning_window = tk.Toplevel(self.root)
        planning_window.title("Motion Planning")
        planning_window.geometry("1200x1000")
        planning_window.configure(bg="white")
        planning_window.transient(self.root)
        planning_window.resizable(True, True)
        planning_window.grab_set()

        joint_frame_1 = tk.Frame(planning_window, bg="white")
        joint_frame_1.pack(fill="x", padx=5, pady=(5, 5))
        joint_title_text = tk.Label(joint_frame_1, text="Joint Space", bg="#2196F3",
                                    fg="white", font=("Arial", 10, "bold"))
        joint_title_text.pack(fill='x', padx=(5, 20))

        # JOINTS TO JOINTS
        func1_frame = ttk.LabelFrame(planning_window, text="JOINTS TO JOINTS", padding=10,
                                     relief=tk.GROOVE, borderwidth=2, style="MyCustom.TLabelframe")
        func1_frame.pack(fill="x", padx=10, pady=(0, 5))

        arm0_row1 = tk.Frame(func1_frame, bg="white")
        arm0_row1.pack(fill="x", pady=2)
        tk.Label(arm0_row1, text="Arm0", bg="#D8F4F3", width=5).pack(side="left", padx=2)
        tk.Label(arm0_row1, text="Start joints", bg="white", width=10).pack(side="left", padx=2)
        self.joints_start_arm0_entry = tk.Entry(arm0_row1, width=45)
        self.joints_start_arm0_entry.pack(side="left", padx=2)
        self.joints_start_arm0_entry.insert(0, "0,0,0,0,0,0,0")
        tk.Button(arm0_row1, text="GetCur",
                  command=lambda: self.get_current_joints('Arm0', self.joints_start_arm0_entry)).pack(side="left",
                                                                                                      padx=2)
        tk.Label(arm0_row1, text="End joints", bg="white", width=10).pack(side="left", padx=2)
        self.joints_end_arm0_entry = tk.Entry(arm0_row1, width=45)
        self.joints_end_arm0_entry.pack(side="left", padx=2)
        self.joints_end_arm0_entry.insert(0, "0,0,0,0,0,0,0")

        arm1_row1 = tk.Frame(func1_frame, bg="white")
        arm1_row1.pack(fill="x", pady=2)
        tk.Label(arm1_row1, text="Arm1", bg="#F4E4D8", width=5).pack(side="left", padx=2)
        tk.Label(arm1_row1, text="Start joints", bg="white", width=10).pack(side="left", padx=2)
        self.joints_start_arm1_entry = tk.Entry(arm1_row1, width=45)
        self.joints_start_arm1_entry.pack(side="left", padx=2)
        self.joints_start_arm1_entry.insert(0, "0,0,0,0,0,0,0")
        tk.Button(arm1_row1, text="GetCur",
                  command=lambda: self.get_current_joints('Arm1', self.joints_start_arm1_entry)).pack(side="left",
                                                                                                      padx=2)
        tk.Label(arm1_row1, text="End joints", bg="white", width=10).pack(side="left", padx=2)
        self.joints_end_arm1_entry = tk.Entry(arm1_row1, width=45)
        self.joints_end_arm1_entry.pack(side="left", padx=2)
        self.joints_end_arm1_entry.insert(0, "0,0,0,0,0,0,0")

        params_row1 = tk.Frame(func1_frame, bg="white")
        params_row1.pack(fill="x", pady=5)
        tk.Label(params_row1, text="Common Parameters:", bg="white", font=("Arial", 9, "bold")).pack(side="left",
                                                                                                     padx=10)
        tk.Label(params_row1, text="Freq:", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.joints_freq_entry = tk.Entry(params_row1, width=6)
        self.joints_freq_entry.pack(side="left", padx=2)
        self.joints_freq_entry.insert(0, "50")
        tk.Label(params_row1, text="(freq%1000==0)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left",
                                                                                                    padx=(0, 5))

        tk.Label(params_row1, text="Vel:", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.joints_vel_entry = tk.Entry(params_row1, width=6)
        self.joints_vel_entry.pack(side="left", padx=2)
        self.joints_vel_entry.insert(0, "0.1")
        tk.Label(params_row1, text="(0.01~1)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left", padx=(0, 5))

        tk.Label(params_row1, text="Acc:", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.joints_acc_entry = tk.Entry(params_row1, width=6)
        self.joints_acc_entry.pack(side="left", padx=2)
        self.joints_acc_entry.insert(0, "0.1")
        tk.Label(params_row1, text="(0.01~1)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left", padx=2)

        btn_row1 = tk.Frame(func1_frame, bg="white")
        btn_row1.pack(pady=5, anchor="center")
        tk.Button(btn_row1, text="Run", width=10, font=("Arial", 11, "bold"), bg="#A2CD5A",
                  command=self.run_joint_to_joint).pack(side="left", padx=10)
        tk.Button(btn_row1, text="Break", width=10, font=("Arial", 11, "bold"), bg="#FFF68F",
                  command=self.stop_motion).pack(side="left", padx=10)

        # JOINTS TO JOINTS (linear motion)
        func2_frame = ttk.LabelFrame(planning_window, text="JOINTS TO JOINTS (linear motion)", padding=10,
                                     relief=tk.GROOVE, borderwidth=2, style="MyCustom.TLabelframe")
        func2_frame.pack(fill="x", padx=10, pady=(0, 5))

        arm0_row2 = tk.Frame(func2_frame, bg="white")
        arm0_row2.pack(fill="x", pady=2)
        tk.Label(arm0_row2, text="Arm0", bg="#D8F4F3", width=5).pack(side="left", padx=2)
        tk.Label(arm0_row2, text="Start joints", bg="white", width=10).pack(side="left", padx=2)
        self.linear_start_arm0_entry = tk.Entry(arm0_row2, width=45)
        self.linear_start_arm0_entry.pack(side="left", padx=2)
        self.linear_start_arm0_entry.insert(0, "0,0,0,0,0,0,0")
        tk.Button(arm0_row2, text="GetCur",
                  command=lambda: self.get_current_joints('Arm0', self.linear_start_arm0_entry)).pack(side="left",
                                                                                                      padx=2)
        tk.Label(arm0_row2, text="End joints", bg="white", width=10).pack(side="left", padx=2)
        self.linear_end_arm0_entry = tk.Entry(arm0_row2, width=45)
        self.linear_end_arm0_entry.pack(side="left", padx=2)
        self.linear_end_arm0_entry.insert(0, "0,0,0,0,0,0,0")

        arm1_row2 = tk.Frame(func2_frame, bg="white")
        arm1_row2.pack(fill="x", pady=2)
        tk.Label(arm1_row2, text="Arm1", bg="#F4E4D8", width=5).pack(side="left", padx=2)
        tk.Label(arm1_row2, text="Start joints", bg="white", width=10).pack(side="left", padx=2)
        self.linear_start_arm1_entry = tk.Entry(arm1_row2, width=45)
        self.linear_start_arm1_entry.pack(side="left", padx=2)
        self.linear_start_arm1_entry.insert(0, "0,0,0,0,0,0,0")
        tk.Button(arm1_row2, text="GetCur",
                  command=lambda: self.get_current_joints('Arm1', self.linear_start_arm1_entry)).pack(side="left",
                                                                                                      padx=2)
        tk.Label(arm1_row2, text="End joints", bg="white", width=10).pack(side="left", padx=2)
        self.linear_end_arm1_entry = tk.Entry(arm1_row2, width=45)
        self.linear_end_arm1_entry.pack(side="left", padx=2)
        self.linear_end_arm1_entry.insert(0, "0,0,0,0,0,0,0")

        # Freq, Vel, Acc）
        params_row2 = tk.Frame(func2_frame, bg="white")
        params_row2.pack(fill="x", pady=5)
        tk.Label(params_row2, text="Common Parameters:", bg="white", font=("Arial", 9, "bold")).pack(side="left",
                                                                                                     padx=10)
        tk.Label(params_row2, text="Freq:", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.linear_freq_entry = tk.Entry(params_row2, width=6)
        self.linear_freq_entry.pack(side="left", padx=2)
        self.linear_freq_entry.insert(0, "50")
        tk.Label(params_row2, text="(freq%1000==0)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left",
                                                                                                    padx=(0, 5))

        tk.Label(params_row2, text="Vel:", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.linear_vel_entry = tk.Entry(params_row2, width=6)
        self.linear_vel_entry.pack(side="left", padx=2)
        self.linear_vel_entry.insert(0, "0.1")
        tk.Label(params_row2, text="(0.01~1)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left", padx=(0, 5))

        tk.Label(params_row2, text="Acc:", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.linear_acc_entry = tk.Entry(params_row2, width=6)
        self.linear_acc_entry.pack(side="left", padx=2)
        self.linear_acc_entry.insert(0, "0.1")
        tk.Label(params_row2, text="(0.01~1)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left", padx=2)

        btn_row2 = tk.Frame(func2_frame, bg="white")
        btn_row2.pack(pady=5, anchor="center")
        tk.Button(btn_row2, text="Run", width=10, font=("Arial", 11, "bold"), bg="#A2CD5A",
                  command=self.run_linear_joint_to_joint).pack(side="left", padx=10)
        tk.Button(btn_row2, text="Break", width=10, font=("Arial", 11, "bold"), bg="#FFF68F",
                  command=self.stop_motion).pack(side="left", padx=10)

        # ========== Cartesian Space ==========
        cartesian_frame = tk.Frame(planning_window, bg="white")
        cartesian_frame.pack(fill="x", padx=5, pady=(5, 5))

        #
        cartesian_title = tk.Label(cartesian_frame, text="Cartesian Space", bg="#2196F3",
                                   fg="white", font=("Arial", 10, "bold"))
        cartesian_title.pack(fill='x', padx=(5, 20))

        # Linear
        linear_frame = ttk.LabelFrame(cartesian_frame, text="Linear", padding=10,
                                      relief=tk.GROOVE, borderwidth=2, style="MyCustom.TLabelframe")
        linear_frame.pack(fill="x", padx=5, pady=(5, 5))
        # ARM0
        arm0_cart_row = tk.Frame(linear_frame, bg="white")
        arm0_cart_row.pack(fill="x", pady=2)
        tk.Label(arm0_cart_row, text="Arm0", bg="#D8F4F3", width=5).pack(side="left", padx=2)
        tk.Label(arm0_cart_row, text="Start XYZABC", bg="white", width=12).pack(side="left", padx=2)
        self.cart_start_arm0_entry = tk.Entry(arm0_cart_row, width=45)
        self.cart_start_arm0_entry.pack(side="left", padx=2)
        self.cart_start_arm0_entry.insert(0, "0,0,0,0,0,0")
        tk.Button(arm0_cart_row, text="GetCur",
                  command=lambda: self.get_current_pose('Arm0', self.cart_start_arm0_entry)).pack(side="left", padx=2)
        tk.Label(arm0_cart_row, text="End XYZABC", bg="white", width=12).pack(side="left", padx=2)
        self.cart_end_arm0_entry = tk.Entry(arm0_cart_row, width=45)
        self.cart_end_arm0_entry.pack(side="left", padx=2)
        self.cart_end_arm0_entry.insert(0, "0,0,0,0,0,0")

        # ARM1
        arm1_cart_row = tk.Frame(linear_frame, bg="white")
        arm1_cart_row.pack(fill="x", pady=2)
        tk.Label(arm1_cart_row, text="Arm1", bg="#F4E4D8", width=5).pack(side="left", padx=2)
        tk.Label(arm1_cart_row, text="Start XYZABC", bg="white", width=12).pack(side="left", padx=2)
        self.cart_start_arm1_entry = tk.Entry(arm1_cart_row, width=45)
        self.cart_start_arm1_entry.pack(side="left", padx=2)
        self.cart_start_arm1_entry.insert(0, "0,0,0,0,0,0")
        tk.Button(arm1_cart_row, text="GetCur",
                  command=lambda: self.get_current_pose('Arm1', self.cart_start_arm1_entry)).pack(side="left", padx=2)
        tk.Label(arm1_cart_row, text="End XYZABC", bg="white", width=12).pack(side="left", padx=2)
        self.cart_end_arm1_entry = tk.Entry(arm1_cart_row, width=45)
        self.cart_end_arm1_entry.pack(side="left", padx=2)
        self.cart_end_arm1_entry.insert(0, "0,0,0,0,0,0")
        # freq vel acc
        cart_params_row = tk.Frame(linear_frame, bg="white")
        cart_params_row.pack(fill="x", pady=5)
        tk.Label(cart_params_row, text="Common Parameters:", bg="white", font=("Arial", 9, "bold")).pack(side="left",
                                                                                                         padx=10)

        tk.Label(cart_params_row, text="Freq:", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.cart_freq_entry = tk.Entry(cart_params_row, width=6)
        self.cart_freq_entry.pack(side="left", padx=2)
        self.cart_freq_entry.insert(0, "50")
        tk.Label(cart_params_row, text="(freq%1000==0)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left",
                                                                                                        padx=(0, 5))

        tk.Label(cart_params_row, text="Vel:", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.cart_vel_entry = tk.Entry(cart_params_row, width=6)
        self.cart_vel_entry.pack(side="left", padx=2)
        self.cart_vel_entry.insert(0, "100")
        tk.Label(cart_params_row, text="(1-1000)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left",
                                                                                                  padx=(0, 5))

        tk.Label(cart_params_row, text="Acc:", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.cart_acc_entry = tk.Entry(cart_params_row, width=6)
        self.cart_acc_entry.pack(side="left", padx=2)
        self.cart_acc_entry.insert(0, "200")
        tk.Label(cart_params_row, text="(1-1000)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left", padx=2)

        cart_btn_row = tk.Frame(linear_frame, bg="white")
        cart_btn_row.pack(pady=5, anchor="center")
        tk.Button(cart_btn_row, text="Run", width=10, font=("Arial", 11, "bold"), bg="#A2CD5A",
                  command=self.run_cartesian_linear).pack(side="left", padx=10)
        tk.Button(cart_btn_row, text="Break", width=10, font=("Arial", 11, "bold"), bg="#FFF68F",
                  command=self.stop_motion).pack(side="left", padx=10)

    def run_joint_to_joint(self):
        """执行 JOINT TO JOINT 运动"""
        # 从 self.joints_start_arm0_entry, self.joints_end_arm0_entry 等获取关节值
        # 解析字符串为浮点数列表，然后调用运动规划接口
        pass

    def run_linear_joint_to_joint(self):
        """执行线性运动 JOINT TO JOINT (linear)"""
        # 从 self.linear_start_arm0_entry 等获取值，执行笛卡尔空间线性规划
        pass

    def stop_motion(self):
        """紧急停止运动"""
        pass

    def get_current_pose(self, arm_name, entry_widget):
        """获取当前末端位姿 (X,Y,Z,A,B,C)，填入 entry"""
        # 示例：从机器人读取当前位姿
        # pose = [0.0]*6
        # entry_widget.delete(0, tk.END)
        # entry_widget.insert(0, ",".join(str(v) for v in pose))
        pass

    def run_cartesian_linear(self):
        """执行笛卡尔空间线性运动"""
        # 从 self.cart_start_arm0_entry, self.cart_end_arm0_entry 等获取位姿值
        # 解析为浮点数列表，并获取对应的 freq, vel, acc
        pass

    def disable_soft_limit(self, obj_type: int, axis_mask: int):
        if not self.connected:
            messagebox.showerror('Error', 'Please connect robot')
            return
        try:
            success = robot.config_disable_soft_limit(obj_type, axis_mask)
            if success:
                messagebox.showinfo("Info", f"Soft limit disabled for {obj_type}")
            else:
                messagebox.showerror('Error', f'Failed to disable soft limit for {obj_type}')
        except Exception as e:
            messagebox.showerror('Error', f'Disable soft limit error: {e}')

    def clear_sensor_offset(self, obj):
        obj_type = self._obj_name_to_type(obj)
        return robot.config_clear_sensor_offset(obj_type)

    def clear_motor_as_zero(self, obj, btn):
        """Motor encoder zeroing (reset encoder offset) for specified arm."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        btn.config(state="disabled")
        try:
            if obj == 'Arm0':
                obj_type = FXObjType.OBJ_ARM0
            elif obj == 'Arm1':
                obj_type = FXObjType.OBJ_ARM1
            elif obj == 'Body':
                obj_type = FXObjType.OBJ_BODY
            elif obj == 'Head':
                obj_type = FXObjType.OBJ_HEAD
            elif obj == 'Lift':
                obj_type = FXObjType.OBJ_LIFT
            axis_mask = 0x7F  # All 7 axes
            if not robot.comm_clear(50):
                messagebox.showerror('Failed!', "Clear buffer failed")
                return
            if not robot.config_reset_enc_offset(obj_type, axis_mask):
                messagebox.showerror('Failed!', f"Reset encoder offset failed for obj {obj}")
            robot.comm_send()
        except Exception as e:
            messagebox.showerror('Error', str(e))

    def clear_motor_error(self, arm):
        """Clear encoder error for specified arm."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        try:
            if arm == 'A':
                obj_type = FXObjType.OBJ_ARM0
            elif arm == 'B':
                obj_type = FXObjType.OBJ_ARM1
            else:
                raise ValueError("arm must be 'A' or 'B'")
            axis_mask = 0x7F  # All 7 axes
            if not robot.comm_clear(50):
                messagebox.showerror('Failed!', "Clear buffer failed")
                return
            if not robot.config_clear_enc_error(obj_type, axis_mask):
                messagebox.showerror('Failed!', f"Clear encoder error failed for arm {arm}")
            robot.comm_send()
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
        if not robot.comm_clear(50):
            messagebox.showerror('Failed!', "Clear buffer failed")
            return
        if robot.send_file(local_path, remote_path):
            messagebox.showinfo("Success", f"File sent to {remote_path}")
        else:
            messagebox.showerror("Failed", "Send file failed")
        robot.comm_send()

    def _receive_file_from_robot(self):
        remote_path = simpledialog.askstring("Remote Path",
                                             "Enter remote path to receive (e.g., /home/robot/file.bin):")
        if not remote_path:
            return
        local_path = filedialog.asksaveasfilename(title="Save file as")
        if not local_path:
            return
        if not robot.comm_clear(50):
            messagebox.showerror('Failed!', "Clear buffer failed")
            return
        if robot.recv_file(local_path, remote_path):
            messagebox.showinfo("Success", f"File received and saved to {local_path}")
        else:
            messagebox.showerror("Failed", "Receive file failed")
        robot.comm_send()

    def Estop(self):
        if not self.connected:
            messagebox.showerror('Error', 'Please connect robot')
        try:
            if not robot.comm_clear(50):
                messagebox.showerror('Failed!', "Clear buffer failed")
                return
            robot.emergency_stop(FXObjMask.OBJ_ALL_FLAG)
            robot.comm_send()


        except Exception as e:
            messagebox.showerror('Error', f'Emergency stop failed: {e}')

    def show_impedance_dialog(self, obj):
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
                fg='#2c3e50'
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

            params_save_frame = tk.Frame(main_frame, bg='white')
            params_save_frame.pack(fill="x", pady=(0, 10))
            save_param_a_button = tk.Button(params_save_frame, text="Save parameters",
                                            command=lambda: self.save_param('Arm0'))
            save_param_a_button.pack(side='left', padx=(200, 0))
            load_param_a_button = tk.Button(params_save_frame, text="Import parameters",
                                            command=lambda: self.load_param('Arm0'))
            load_param_a_button.pack(side='left', padx=(100, 10))

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
            force_torque_frame.pack(fill="x", pady=(10, 5))
            tk.Label(force_torque_frame, text="Force(dir_x,dir_y,dir_z,force(-50~50N),distance(<50mm)):",
                     font=('Arial', 9), bg='white').pack(side="left",
                                                         padx=(0, 5))
            force_entry = tk.Entry(force_torque_frame, textvariable=self.force_a_entry, width=25)
            force_entry.pack(side="left", padx=(0, 20))
            tk.Label(force_torque_frame, text="Torque(dir_x,dir_y,dir_z,torque(N*m),distance(deg))", font=('Arial', 9),
                     bg='white').pack(side="left",
                                      padx=(0, 5))
            torque_entry = tk.Entry(force_torque_frame, textvariable=self.torque_a_entry, width=25)
            torque_entry.pack(side="left", padx=(0, 20))

            force_torque_btn = tk.Frame(main_frame, bg='white')
            force_torque_btn.pack(fill="x", padx=(200, 0), pady=(0, 10))
            set_ft_btn = tk.Button(force_torque_btn, text="Set Force/Torque", width=15,
                                   command=lambda: self.force_torque_set('Arm0'),
                                   bg="#FFB6C1", font=("Arial", 9, "bold"))
            set_ft_btn.pack(side="left")

        elif obj == 'Arm1':
            main_frame1 = tk.Frame(impedance_dialog, padx=20, pady=20, bg='white')
            main_frame1.pack(fill="both", expand=True)
            title_label1 = tk.Label(
                main_frame1,
                text=f"Set the impedance parameters of {obj}",
                font=('Arial', 10, 'bold'),
                fg='#2c3e50'
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

            params_save_frame1 = tk.Frame(main_frame1, bg='white')
            params_save_frame1.pack(fill="x", pady=(0, 10))
            save_param_b_button = tk.Button(params_save_frame1, text="Save parameters",
                                            command=lambda: self.save_param('Arm1'))
            save_param_b_button.pack(side='left', padx=(200, 0))
            load_param_b_button = tk.Button(params_save_frame1, text="Import parameters",
                                            command=lambda: self.load_param('Arm1'))
            load_param_b_button.pack(side='left', padx=(100, 0))

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
            tk.Label(force_torque_frame, text="Force(dir_x,dir_y,dir_z,force(-50~50N),distance(<50mm)):",
                     font=('Arial', 9), bg='white').pack(side="left",
                                                         padx=(0, 5))
            force_entry = tk.Entry(force_torque_frame, textvariable=self.force_b_entry, width=25)
            force_entry.pack(side="left", padx=(0, 20))
            tk.Label(force_torque_frame, text="Torque(dir_x,dir_y,dir_z,torque(N*m),distance(deg))", font=('Arial', 9),
                     bg='white').pack(side="left",
                                      padx=(0, 5))
            torque_entry = tk.Entry(force_torque_frame, textvariable=self.torque_b_entry, width=25)
            torque_entry.pack(side="left", padx=(0, 20))

            force_torque_btn = tk.Frame(main_frame1, bg='white')
            force_torque_btn.pack(fill="x", padx=(200, 0), pady=(0, 10))
            set_ft_btn = tk.Button(force_torque_btn, text="Set Force/Torque", width=15,
                                   command=lambda: self.force_torque_set('Arm1'),
                                   bg="#FFB6C1", font=("Arial", 9, "bold"))
            set_ft_btn.pack(side="left")

        elif obj == 'Body':
            main_frame1 = tk.Frame(impedance_dialog, padx=20, pady=20, bg='white')
            main_frame1.pack(fill="both", expand=True)
            title_label1 = tk.Label(
                main_frame1,
                text=f"Set the PD parameters of {obj}",
                font=('Arial', 10, 'bold'),
                fg='#2c3e50'
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

            params_save_frame1 = tk.Frame(main_frame1, bg='white')
            params_save_frame1.pack(fill="x", pady=(0, 10))
            save_param_b_button = tk.Button(params_save_frame1, text="Save parameters",
                                            command=lambda: self.save_param('Body'))
            save_param_b_button.pack(side='left', padx=(200, 0))
            load_param_b_button = tk.Button(params_save_frame1, text="Import parameters",
                                            command=lambda: self.load_param('Body'))
            load_param_b_button.pack(side='left', padx=(100, 0))

    def init_kd_variables(self):
        self.cart_k_b_entry = tk.StringVar(value="3000,3000,3000,100,100,100,50")
        self.cart_k_a_entry = tk.StringVar(value="3000,3000,3000,100,100,100,50")
        self.cart_d_a_entry = tk.StringVar(value="0.1,0.1,0.1,0.1,0.1,0.1,0.11")
        self.cart_d_b_entry = tk.StringVar(value="0.1,0.1,0.1,0.1,0.1,0.1,0.11")

        self.k_a_entry = tk.StringVar(value="3,3,3,2,1,1,1")
        self.k_b_entry = tk.StringVar(value="3,3,3,2,1,1,1")
        self.d_a_entry = tk.StringVar(value="0.2,0.2,0.2,0.2,0.2,0.2,0.2")
        self.d_b_entry = tk.StringVar(value="0.2,0.2,0.2,0.2,0.2,0.2,0.2")

        self.pdp_entry = tk.StringVar(value="12, 12, 12, 10, 9, 9")
        self.pdd_entry = tk.StringVar(value="0.5,0.5,0.5,0.5,0.5,0.5")

        self.force_a_entry = tk.StringVar(value="0,1,0,25,25")
        self.torque_a_entry = tk.StringVar(value="0,1,0,5,10")
        self.force_b_entry = tk.StringVar(value="0,1,0,25,25")
        self.torque_b_entry = tk.StringVar(value="0,1,0,5,10")

        self.arm0_tool_dyn_entry = tk.StringVar(value="0,0,0,0,0,0,0,0,0,0")
        self.arm1_tool_dyn_entry = tk.StringVar(value="0,0,0,0,0,0,0,0,0,0")

        self.arm0_tool_kine_entry = tk.StringVar(value="0,0,0,0,0,0")
        self.arm1_tool_kine_entry = tk.StringVar(value="0,0,0,0,0,0")

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

        if not robot.comm_clear(50):
            messagebox.showerror('Failed!', "Clear buffer failed")
            return

        if obj in ('Arm0', 'Arm1'):
            success = robot.runtime_set_joint_k(obj_type, k_list) and robot.runtime_set_joint_d(obj_type, d_list)
        else:  # Body
            success = robot.runtime_set_body_pdp(k_list) and robot.runtime_set_body_pdd(d_list)

        if not success:
            messagebox.showerror('Failed!', f"Set {obj} parameters failed")
        robot.comm_send()

    def cart_kd_set(self, obj):
        if not self.connected:
            messagebox.showerror('Error', 'Please connect robot')
            return

        # Determine object type and entry widgets
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
            obj_type = None  # Body uses separate API, no obj_type needed
            num_joints = 6
        else:
            messagebox.showerror('Error', f'Unknown obj: {obj}')
            return

        # Validate K (Cartesian stiffness or Body P)
        k_str = k_var.get().strip()
        if not k_str:
            messagebox.showerror("Error", "K/PDP parameter cannot be empty!")
            return
        is_valid, result = self.validate_point(k_str, num_joints)
        if not is_valid:
            messagebox.showerror("Error", f"Invalid K/PDP format: {result}")
            return
        k_list = [float(x) for x in result.split(',')]

        # Validate D (Cartesian damping or Body D)
        d_str = d_var.get().strip()
        if not d_str:
            messagebox.showerror("Error", "D/PDD parameter cannot be empty!")
            return
        is_valid, result = self.validate_point(d_str, num_joints)
        if not is_valid:
            messagebox.showerror("Error", f"Invalid D/PDD format: {result}")
            return
        d_list = [float(x) for x in result.split(',')]

        # Clear comm buffer
        if not robot.comm_clear(50):
            messagebox.showerror('Failed!', "Clear buffer failed")
            return

        # Set parameters based on object type
        if obj in ('Arm0', 'Arm1'):
            success = robot.runtime_set_cart_k(obj_type, k_list) and robot.runtime_set_cart_d(obj_type, d_list)
        else:  # Body
            success = robot.runtime_set_body_pdp(k_list) and robot.runtime_set_body_pdd(d_list)

        if not success:
            messagebox.showerror('Failed!', f"Set {obj} Cartesian/PD parameters failed")
        robot.comm_send()

    def save_param(self, obj):
        self.params = []
        if obj == 'Arm0':
            params_to_save = [
                self.k_a_entry.get(),
                self.d_a_entry.get(),
                self.cart_k_a_entry.get(),
                self.cart_d_a_entry.get()
            ]
        elif obj == 'Arm1':
            params_to_save = [
                self.k_b_entry.get(),
                self.d_b_entry.get(),
                self.cart_k_b_entry.get(),
                self.cart_d_b_entry.get()
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
            else:
                messagebox.showerror("Error", "File does not contain enough parameters (need 4)")
        elif obj == 'Arm1':
            if len(lines) >= 4:
                self.k_b_entry.set(lines[0])
                self.d_b_entry.set(lines[1])
                self.cart_k_b_entry.set(lines[2])
                self.cart_d_b_entry.set(lines[3])
            else:
                messagebox.showerror("Error", "File does not contain enough parameters (need 4)")
        elif obj == 'Body':
            if len(lines) >= 2:
                self.pdp_entry.set(lines[0])
                self.pdd_entry.set(lines[1])
            else:
                messagebox.showerror("Error", "File does not contain enough parameters (need 2)")
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

            if not robot.comm_clear(50):
                messagebox.showerror('Failed!', "Clear buffer failed")
                return
            success = robot.runtime_set_force_ctrl(obj_type, force_list) and \
                      robot.runtime_set_torque_ctrl(obj_type, torque_list)
            if not success:
                messagebox.showerror('Failed!', f"Set force/torque failed for {obj}")
            robot.comm_send()
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

    def jointImp_state(self, obj):
        """Switch to Joint Impedance mode (only for Arm0/Arm1)."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        if obj not in ('Arm0', 'Arm1', 'Body'):
            messagebox.showerror('Error', f'{obj} does not support Joint Impedance mode')
            return
        try:
            if not robot.comm_clear(50):
                messagebox.showerror('Failed!', "clear buffer failed")
                return
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
            ret = robot.switch_to_imp_joint_mode(obj_type, 1000, vel, acc, k_list, d_list)
            if ret != 0:
                messagebox.showerror('Failed!', f'Switch to Joint Impedance failed for {obj}')
            robot.comm_send()
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
            if not robot.comm_clear(50):
                messagebox.showerror('Failed!', "clear buffer failed")
                return
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
            ret = robot.switch_to_imp_cart_mode(obj_type, 1000, vel, acc, k_list, d_list)
            if ret != 0:
                messagebox.showerror('Failed!', f'Switch to Cartesian Impedance failed for {obj}')
            robot.comm_send()
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
            if not robot.comm_clear(50):
                messagebox.showerror('Failed!', "clear buffer failed")
                return
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
            ret = robot.switch_to_imp_force_mode(obj_type, 1000, force_ctrl, torque_ctrl)
            if ret != 0:
                messagebox.showerror('Failed!', f'Switch to Force Impedance failed for {obj}')
            robot.comm_send()
        except ValueError as e:
            messagebox.showerror('Error', f'Invalid number format in Force/Torque: {e}')
        except Exception as e:
            messagebox.showerror('Error', f'Force Impedance switch failed: {e}')

    def drag_state(self, obj):
        """Switch to drag mode based on combo selection."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        if obj not in ('Arm0', 'Arm1'):
            messagebox.showerror('Error', f'{obj} does not support drag mode')
            return
        try:
            obj_type = self._obj_name_to_type(obj)
            # Get selected drag type from the corresponding combo box
            if obj == 'Arm0':
                mode = self.drag_combo.get()
            else:
                mode = self.drag_combo_r.get()
            if mode == "joint":
                ret = robot.switch_to_drag_joint(obj_type, 1000)
            elif mode == "cartX":
                ret = robot.switch_to_drag_cart_x(obj_type, 1000)
            elif mode == "cartY":
                ret = robot.switch_to_drag_cart_y(obj_type, 1000)
            elif mode == "cartZ":
                ret = robot.switch_to_drag_cart_z(obj_type, 1000)
            elif mode == "cartR":
                ret = robot.switch_to_drag_cart_r(obj_type, 1000)
            else:
                messagebox.showerror('Error', f'Unknown drag mode: {mode}')
                return
            if ret != 0:
                messagebox.showerror('Failed!', f'Switch to {mode} drag failed for {obj}')
        except Exception as e:
            messagebox.showerror('Error', f'Drag mode switch failed: {e}')

    def error_get(self, obj):
        """Get servo error codes for the specified object and display in hex format."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        try:
            obj_type = self._obj_name_to_type(obj)
            msg = robot.get_servo_error_codes(obj_type)
            if msg is None:
                messagebox.showwarning("Warning", f"Failed to get error codes for {obj}")
            else:
                messagebox.showinfo(f'{obj} Servo Error Details', msg)
        except Exception as e:
            messagebox.showerror('Error', f"Failed to get error codes: {e}")

    def release_brake(self, obj):
        """Release brake for the specified object (unlock)."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        try:
            if not robot.comm_clear(50):
                messagebox.showerror('Failed!', "clear buffer failed")
                return
            obj_type = self._obj_name_to_type(obj)
            # Unlock brakes for all axes (axis_mask = 0xFF for up to 8 axes)
            # Adjust axis count based on object type
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
            success = robot.config_brake_unlock(obj_type, axis_mask)
            if not success:
                messagebox.showerror('Failed!', f"Release brake failed for {obj}")
            robot.comm_send()
        except Exception as e:
            messagebox.showerror('Error', f"Release brake failed: {e}")

    def brake(self, obj):
        """Apply brake (lock) for the specified object."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        try:
            if not robot.comm_clear(50):
                messagebox.showerror('Failed!', "clear buffer failed")
                return
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
            success = robot.config_brake_lock(obj_type, axis_mask)
            if not success:
                messagebox.showerror('Failed!', f"Brake lock failed for {obj}")
            robot.comm_send()
        except Exception as e:
            messagebox.showerror('Error', f"Brake lock failed: {e}")


if __name__ == "__main__":
    DBL_EPSILON = sys.float_info.epsilon
    arm_main_state_with = 130
    data_queue = queue.Queue()
    crr_pth = os.getcwd()
    robot = GentoRobot()

    # '''ini kine of ARM0 & ARM1'''
    # arm0_kine=robot.init_single_arm_kinematics(0,"ccs_m6_40.MvKDCfg")
    # arm1_kine =robot.init_single_arm_kinematics(1,"ccs_m6_40.MvKDCfg")
    #
    # robot.kinematics_log_switch(arm0_kine,0)
    # robot.kinematics_log_switch(arm1_kine, 0)

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
