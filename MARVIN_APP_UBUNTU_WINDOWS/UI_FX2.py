import tkinter as tk
from tkinter import messagebox, ttk, scrolledtext, filedialog, simpledialog
import threading
import time
import ast
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
ini_result1 = kk1.load_config(arm_type=0, config_path=glob.glob('config/*.MvKDCfg')[0])
initial_kine_tag1 = kk1.initial_kine(robot_type=ini_result1['TYPE'][0],
                                     dh=ini_result1['DH'][0],
                                     pnva=ini_result1['PNVA'][0],
                                     j67=ini_result1['BD'][0])

ini_result2 = kk2.load_config(arm_type=1, config_path=glob.glob('config/*.MvKDCfg')[0])
initial_kine_tag2 = kk2.initial_kine(robot_type=ini_result2['TYPE'][1],
                                     dh=ini_result2['DH'][1],
                                     pnva=ini_result2['PNVA'][1],
                                     j67=ini_result2['BD'][1])

button_w = 10

# Define constants
DBL_EPSILON = sys.float_info.epsilon

# Create queue
data_queue = queue.Queue()
def read_data(robot_id,com):
    '''Receive HEX data from CAN/485'''
    while True:
        try:
            tag, receive_hex_data = robot.get_485_data(robot_id, com)
            if tag >= 1:
                print(f"Received HEX data: {receive_hex_data}")
                data_queue.put(receive_hex_data)
            else:
                time.sleep(0.001)
        except Exception as e:
            # print(f"Read data error: {e}")
            time.sleep(0.001)


def get_received_data():
    '''Get received data and count'''
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
    """Convert a 3x3 rotation matrix to ABC Euler angles"""
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
    """Vector cross product"""
    result = [0.0] * 3
    result[0] = a[1] * b[2] - a[2] * b[1]
    result[1] = a[2] * b[0] - a[0] * b[2]
    result[2] = a[0] * b[1] - a[1] * b[0]
    return result


def NormVect(a):
    """Vector magnitude (norm)"""
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def main_function(vx, vy):
    """Main function - vx and vy are column vectors"""
    m_S = ""

    if NormVect(vx) < 0.01 or NormVect(vy) < 0.01:
        return m_S, [0, 0, 0]

    vz = FX_VectCross(vx, vy)

    vz_norm = NormVect(vz)
    if vz_norm < 0.99 or vz_norm > 1.01:
        return m_S, [0, 0, 0]

    # Build 3x3 matrix - vx, vy, vz as column vectors
    m_mat = [
        [vx[0], vy[0], vz[0]],  # Col 1: vx, Col 2: vy, Col 3: vz
        [vx[1], vy[1], vz[1]],
        [vx[2], vy[2], vz[2]]
    ]

    # Display matrix form
    m_S += "Matrix form (column vectors are coordinate direction vectors):\n"
    m_S += f"{m_mat[0][0]:.2f}\t{m_mat[0][1]:.2f}\t{m_mat[0][2]:.2f}\n"
    m_S += f"{m_mat[1][0]:.2f}\t{m_mat[1][1]:.2f}\t{m_mat[1][2]:.2f}\n"
    m_S += f"{m_mat[2][0]:.2f}\t{m_mat[2][1]:.2f}\t{m_mat[2][2]:.2f}\n\n"

    # Calculate ABC angles
    m_abc = [0.0] * 3
    Matrix2ABC(m_mat, m_abc)

    m_S += f"ABC angles: [{m_abc[0]:.5f}, {m_abc[1]:.5f}, {m_abc[2]:.5f}]\n"

    return m_S


# Format data
def format_vector(vector):
    return ", ".join([f"{v:.2f}" for v in vector])


def preview_text_file():
    """Preview a text file in a new window"""
    file_path = os.path.join(crr_pth, "config/python_doc_contrl.md")

    # Create new window
    preview_window = tk.Toplevel(root)
    preview_window.title(f"Document Preview: {file_path.split('/')[-1]}")
    preview_window.geometry("600x400")

    # Create scrollable text area
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

    # Add close button
    close_btn = tk.Button(
        preview_window,
        text="Close Preview",
        command=preview_window.destroy
    )
    close_btn.pack(pady=10)

    # Read and display file content
    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            content = file.read()
            text_area.insert(tk.INSERT, content)
            text_area.config(state=tk.DISABLED)  # Set read-only
    except Exception as e:
        messagebox.showerror("Error", f"Cannot read file:\n{str(e)}")


def preview_text_file_1():
    """Show data collection index description"""
    messagebox.showinfo("Data Collection Index Guide", f"Data collection ID numbers:\n"
                                       "Left Arm\n"
                                       "0-6: Left arm joint position\n"
                                       "10-16: Left arm joint velocity\n"
                                       "20-26: Left arm external encoder position\n"
                                       "30-36: Left arm joint command position\n"
                                       "40-46: Left arm joint current (per mille)\n"
                                       "50-56: Left arm joint sensor torque (Nm)\n"
                                       "60-66: Left arm friction estimate\n"
                                       "70-76: Left arm friction velocity estimate\n"
                                       "80-85: Left arm joint external force estimate\n"
                                       "90-95: Left arm end-point external force estimate\n\n"
                                       "Note: index 76 is the left arm dynamics identification marker column\n"
                                       "\nRight Arm\n"
                                       "100-106: Right arm joint position\n"
                                       "110-116: Right arm joint velocity\n"
                                       "120-126: Right arm external encoder position\n"
                                       "130-136: Right arm joint command position\n"
                                       "140-146: Right arm joint current (per mille)\n"
                                       "150-156: Right arm joint sensor torque (Nm)\n"
                                       "160-166: Right arm friction estimate\n"
                                       "170-176: Right arm friction velocity estimate\n"
                                       "180-185: Right arm joint external force estimate\n"
                                       "190-195: Right arm end-point external force estimate\n\n"
                                       "Note: index 176 is the right arm dynamics identification marker column\n"
                        )


class DataSubscriber:
    """Data subscriber that periodically updates data"""

    def __init__(self, callback):
        self.callback = callback
        self.running = True
        self.thread = threading.Thread(target=self.generate_data, daemon=True)
        self.thread.start()

    def generate_data(self):
        """Subscribe to robot data"""
        while self.running:
            result = robot.subscribe(dcss)
            # Callback to update UI
            self.callback(result)
            time.sleep(0.5)  # Update every 0.5 seconds

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

        # Initialize data
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
                     'fb_joint_pos': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Feedback joint position
                     'fb_joint_vel': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Feedback joint velocity
                     'fb_joint_posE': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Feedback joint position (ext. encoder)
                     'fb_joint_cmd': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Joint position command
                     'fb_joint_cToq': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Feedback joint current
                     'fb_joint_sToq': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Sensor torque
                     'fb_joint_them': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Feedback joint temperature
                     'est_joint_firc': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                     'est_joint_firc_dot': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                     'est_joint_force': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Axis external force
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

        # Initialize two point lists
        self.points1 = []
        self.points2 = []

        self.command1=[]
        self.command2=[]

        # Initialize parameter list
        self.params = []

        self.period_file_path_1 = tk.StringVar()
        self.period_file_path_2 = tk.StringVar()

        self.file_path_50 = tk.StringVar()

        self.file_path_tool = tk.StringVar()

        self.processed_data = []

        self.m_sq_offset_1 = 0.0
        self.m_sq_offset_2 = 0.0

        # Current display mode: 0=position, 1=sensor, 2=current
        self.display_mode = 0
        self.mode_names = ["Position Data", "Velocity Data", "Sensor Data", "Current Data", "Temperature Data",
                           "Ext. Encoder Position", "Command Position", "Joint Ext. Force"]
        self.data_keys = [('fb_joint_pos'), ('fb_joint_vel'), ('fb_joint_sToq'), ('fb_joint_cToq'), ('fb_joint_them'),
                          ('fb_joint_posE'), ('fb_joint_cmd'), ('est_joint_force')]

        # Store references to main UI components
        self.main_interface_widgets = []
        # Create top control panel
        self.create_control_panel()

        # Create main content area
        self.create_main_content()

        # Create bottom status bar
        self.create_status_bar()

        # Initially not connected
        self.connected = False
        self.data_subscriber = None

        # Bind window close event
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        # Password setting
        self.correct_password = "1"

    def create_control_panel(self):
        """Create top control panel"""
        self.control_frame = tk.Frame(self.root, bg="#e0e0e0", padx=10, pady=10)
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
        self.connect_btn.pack(side="left", padx=10)

        self.arm_ip_entry = tk.Entry(self.control_frame, )
        self.arm_ip_entry.insert(0, "192.168.1.190")
        self.arm_ip_entry.pack(side="left", padx=10)

        # More features menu button
        self.more_features_btn = tk.Button(
            self.control_frame,
            text="More Features",
            width=15,
            command=self.show_more_features,
            bg="#9C27B0",
            fg="white",
            font=("Arial", 10, "bold")
        )
        self.more_features_btn.pack(side="right", padx=5)

        # # View documentation
        # self.readme_button = tk.Button(self.control_frame, text="View Docs", width=15, command=preview_text_file,
        #                                font=("Arial", 10, "bold"))
        # self.readme_button.pack(side="right", padx=10)

        '''###########################mor more more############################'''
        # Hidden features button
        self.hidden_features_btn = tk.Button(self.control_frame, text="System Upgrade", width=15, bg="#F5FC34",
                                             command=self.authenticate_and_show_hidden, font=("Arial", 10, "bold"))
        self.hidden_features_btn.pack(side="right", padx=5, pady=10)

        # Display mode toggle button
        self.mode_btn = tk.Button(
            self.control_frame,
            text="Position Data",
            width=15,
            command=self.toggle_display_mode,
            state="disabled",
            bg="#2196F3",
            fg="#fffef9",
            font=("Arial", 10, "bold"))
        self.mode_btn.pack(side="right", padx=10)

        # Emergency stop
        self.stop_btn = tk.Button(
            self.control_frame,
            text="E-STOP",
            width=15,
            command=self.stop_command,
            bg="#ef4136",
            fg="#fffef9",
            font=("Arial", 10, "bold"))
        self.stop_btn.pack(side="right", padx=10)

        # Connection status indicator
        status_frame = tk.Frame(self.control_frame, bg="#e0e0e0")
        status_frame.pack(side="right", padx=10)

        tk.Label(status_frame, text="Connection:", bg="#e0e0e0", font=("Arial", 9)).pack(side="left")
        self.status_light = tk.Label(status_frame, text="●", font=("Arial", 16), fg="red")
        self.status_light.pack(side="left", padx=5)
        self.status_label = tk.Label(status_frame, text="Disconnected", bg="#e0e0e0", font=("Arial", 9))
        self.status_label.pack(side="left")

    '''###############################################################################################################'''

    def show_more_features(self):
        """Show more features menu"""
        # Create menu
        menu = tk.Menu(self.root, tearoff=0)

        # Add menu items
        menu.add_command(label="Additional Settings", command=self.additional_settings)
        # menu.add_command(label="Data Management", command=self.open_data_management)
        # menu.add_command(label="Log Viewer", command=self.open_log_viewer)
        # menu.add_separator()
        # menu.add_command(label="Calibration Tool", command=self.open_calibration_tool)
        # menu.add_command(label="Diagnostic Tool", command=self.open_diagnostic_tool)
        menu.add_separator()
        menu.add_command(label="View Documentation", command=self.open_doc)
        # menu.add_command(label="About", command=self.open_about)

        # Show menu
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
        """Open system settings window"""
        settings_window = tk.Toplevel(self.root)
        settings_window.title("Additional Features")
        settings_window.geometry("800x600")
        settings_window.configure(bg="#f0f0f0")
        settings_window.transient(self.root)
        settings_window.grab_set()

        # Create tabs
        notebook = ttk.Notebook(settings_window)
        notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Floating base settings tab
        floating_base_frame = ttk.Frame(notebook, padding="10")
        notebook.add(floating_base_frame, text="Floating Base Parameter Calculation")

        # Network settings tab
        network_frame = ttk.Frame(notebook, padding="10")
        notebook.add(network_frame, text="Network Settings")

        # # Interface settings tab
        # interface_frame = ttk.Frame(notebook, padding="10")
        # notebook.add(interface_frame, text="Interface Settings")

        # Populate floating base settings tab
        self.create_floating_base_tab(floating_base_frame)

        # Populate network settings tab
        self.create_network_settings_tab(network_frame)

        # # Populate interface settings tab
        # self.create_interface_settings_tab(interface_frame)

        # Buttons
        button_frame = ttk.Frame(settings_window)
        button_frame.pack(pady=10)

        ttk.Button(button_frame, text="Save Settings",
                   command=lambda: self.save_all_settings(notebook)).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Close",
                   command=settings_window.destroy).pack(side=tk.LEFT, padx=5)

    def create_network_settings_tab(self, parent):
        """Create network settings tab content"""
        ttk.Label(parent, text="Network Settings", font=("Arial", 14, "bold")).pack(pady=10)

        network_frame = ttk.LabelFrame(parent, text="Network Configuration", padding="15")
        network_frame.pack(fill=tk.X, pady=10)

        ttk.Label(network_frame, text="Default IP Address:").grid(row=0, column=0, sticky="w", pady=5)
        self.default_ip_entry = ttk.Entry(network_frame, width=20)
        self.default_ip_entry.insert(0, "192.168.1.190")
        self.default_ip_entry.grid(row=0, column=1, pady=5, padx=10)

        ttk.Label(network_frame, text="Port:").grid(row=1, column=0, sticky="w", pady=5)
        self.port_entry = ttk.Entry(network_frame, width=20)
        self.port_entry.insert(0, "502")
        self.port_entry.grid(row=1, column=1, pady=5, padx=10)

        ttk.Label(network_frame, text="Timeout (seconds):").grid(row=2, column=0, sticky="w", pady=5)
        self.timeout_entry = ttk.Entry(network_frame, width=20)
        self.timeout_entry.insert(0, "10")
        self.timeout_entry.grid(row=2, column=1, pady=5, padx=10)

    def create_floating_base_tab(self, parent):
        """Create floating base parameter settings tab"""
        # Store selection results
        self.row2_selection = [0, 0, 0]  # Corresponds to X,Y,Z
        self.row3_selection = [0, 0, 0]  # Corresponds to X,Y,Z

        # Store radio button variables
        self.row2_var = tk.StringVar()
        self.row3_var = tk.StringVar()

        # Add variable tracing to update on selection change
        self.row2_var.trace('w', lambda *args: self.on_selection_change(2))
        self.row3_var.trace('w', lambda *args: self.on_selection_change(3))

        ttk.Label(parent, text="Floating Base Parameter Calculation", font=("Arial", 14, "bold")).pack(pady=10)

        # First row
        row1_frame = ttk.Frame(parent)
        row1_frame.pack(fill="x", pady=5)

        ttk.Label(row1_frame, text="Base coordinate direction (X-axis and Y-axis)").pack(side="left", padx=5)
        ttk.Label(row1_frame, text="IMU coordinate direction (base-IMU alignment option)").pack(side="right", padx=5)

        # Second and third row container
        axis_frame = ttk.Frame(parent)
        axis_frame.pack(fill="x", pady=10)

        # Left labels
        ttk.Label(axis_frame, text="X-axis").grid(row=0, column=0, padx=10, pady=10, sticky="w")
        ttk.Label(axis_frame, text="Y-axis").grid(row=1, column=0, padx=10, pady=10, sticky="w")

        # Options for second and third rows
        options = ["x", "-x", "y", "-y", "z", "-z"]

        # Second row radio buttons
        self.row2_buttons = []
        for i, option in enumerate(options):
            btn = ttk.Radiobutton(axis_frame, text=option, value=option,
                                  variable=self.row2_var,
                                  command=lambda: self.on_selection_change(2))
            btn.grid(row=0, column=i + 1, padx=5, pady=5)
            self.row2_buttons.append(btn)

        # Third row radio buttons
        self.row3_buttons = []
        for i, option in enumerate(options):
            btn = ttk.Radiobutton(axis_frame, text=option, value=option,
                                  variable=self.row3_var,
                                  command=lambda: self.on_selection_change(3))
            btn.grid(row=1, column=i + 1, padx=5, pady=5)
            self.row3_buttons.append(btn)

        # Result display area
        self.result_frame = ttk.LabelFrame(parent, text="Calculation Result")
        self.result_frame.pack(fill="both", expand=True, pady=10)

        self.result_text = tk.Text(self.result_frame, height=8, wrap=tk.WORD)
        scrollbar = ttk.Scrollbar(self.result_frame, orient=tk.VERTICAL, command=self.result_text.yview)
        self.result_text.config(yscrollcommand=scrollbar.set)

        self.result_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y, pady=5)

    def create_interface_settings_tab(self, parent):
        """Create interface settings tab content"""
        ttk.Label(parent, text="Interface Settings", font=("Arial", 14, "bold")).pack(pady=10)

        interface_frame = ttk.LabelFrame(parent, text="Interface Configuration", padding="15")
        interface_frame.pack(fill=tk.X, pady=10)

        self.theme_var = tk.StringVar(value="Light")
        ttk.Label(interface_frame, text="Theme:").grid(row=0, column=0, sticky="w", pady=5)
        ttk.Combobox(interface_frame, textvariable=self.theme_var,
                     values=["Light", "Dark", "Auto"], state="readonly", width=15).grid(row=0, column=1, pady=5, padx=10)

        self.language_var = tk.StringVar(value="Chinese")
        ttk.Label(interface_frame, text="Language:").grid(row=1, column=0, sticky="w", pady=5)
        ttk.Combobox(interface_frame, textvariable=self.language_var,
                     values=["Chinese", "English", "Japanese"], state="readonly", width=15).grid(row=1, column=1, pady=5, padx=10)

        self.auto_connect_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(interface_frame, text="Auto-connect on startup",
                        variable=self.auto_connect_var).grid(row=2, column=0, columnspan=2, sticky="w", pady=5)

        self.auto_save_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(interface_frame, text="Auto-save settings",
                        variable=self.auto_save_var).grid(row=3, column=0, columnspan=2, sticky="w", pady=5)

    def on_selection_change(self, changed_row):
        """Called when selection changes, handles interlock logic and updates result"""
        # Update selection lists
        self.update_selection_lists()
        # Apply mutual exclusion logic
        self.apply_mutual_exclusion(changed_row)
        # If both rows have selections, calculate and display result
        if any(self.row2_selection) and any(self.row3_selection):
            result = self.get_abc_calculation()
            self.display_result(result)
        else:
            self.result_text.delete(1.0, tk.END)
            self.result_text.insert(tk.END, "Please complete both row selections to view calculation result")

    def update_selection_lists(self):
        """Update selection lists based on current selection"""
        # Reset selection lists
        self.row2_selection = [0, 0, 0]
        self.row3_selection = [0, 0, 0]

        # Update second row selection
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


        # Update third row selection
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
        """Apply interlock logic, disable conflicting options"""
        row2_val = self.row2_var.get()
        row3_val = self.row3_var.get()

        # Reset all button states
        for btn in self.row2_buttons + self.row3_buttons:
            btn.state(["!disabled"])

        # If second row has selection, disable corresponding axis in third row
        if row2_val:
            if row2_val in ["x", "-x"]:
                self.disable_axis_options(self.row3_buttons, ["x", "-x"])
            elif row2_val in ["y", "-y"]:
                self.disable_axis_options(self.row3_buttons, ["y", "-y"])
            elif row2_val in ["z", "-z"]:
                self.disable_axis_options(self.row3_buttons, ["z", "-z"])

        # If third row has selection, disable corresponding axis in second row
        if row3_val:
            if row3_val in ["x", "-x"]:
                self.disable_axis_options(self.row2_buttons, ["x", "-x"])
            elif row3_val in ["y", "-y"]:
                self.disable_axis_options(self.row2_buttons, ["y", "-y"])
            elif row3_val in ["z", "-z"]:
                self.disable_axis_options(self.row2_buttons, ["z", "-z"])

    def disable_axis_options(self, buttons, options_to_disable):
        """Disable specified options"""
        for btn in buttons:
            if btn['value'] in options_to_disable:
                btn.state(["disabled"])

    def get_abc_calculation(self):
        """Calculation function, returns multi-line result"""
        result = f"Base coordinate rotation in gyroscope IMU:\n"
        result += "=" * 20 + "\n"

        try:
            # Call the calculation function
            abc = main_function(self.row2_selection, self.row3_selection)
            result += abc
            result += "\n"
        except Exception as e:
            result += f"Calculation error: {str(e)}\n"

        result += "=" * 20 + "\n\n"
        result += ("Please update the ABC angles to robot.ini [R.A0.BASIC] section:\n"
                   "              GYROSETA, GYROSETB, GYROSETC\n"
                   "Note: calculate left and right arms separately. [R.A0.BASIC] is left arm, [R.A1.BASIC] is right arm.")
        return result

    def main_function(self, selection1, selection2):
        """Simulated calculation function, replace with actual umi2abc function"""
        # This should be the main_function imported from umi2abc
        # Returning simulated result for now
        return f"A: 45.0°, B: 30.0°, C: 15.0°"

    def display_result(self, result):
        """Display result in text box"""
        self.result_text.delete(1.0, tk.END)
        self.result_text.insert(tk.END, result)

    def save_all_settings(self, notebook):
        """Save all settings"""
        try:
            # Get network settings
            ip = self.default_ip_entry.get()
            port = self.port_entry.get()
            timeout = self.timeout_entry.get()

            # Get interface settings
            theme = self.theme_var.get()
            language = self.language_var.get()
            auto_connect = self.auto_connect_var.get()
            auto_save = self.auto_save_var.get()

            messagebox.showinfo("Save Successful",
                                f"Settings saved:\n"
                                f"IP: {ip}\n"
                                f"Port: {port}\n"
                                f"Theme: {theme}\n"
                                f"Language: {language}")
        except Exception as e:
            messagebox.showerror("Save Error", f"Error saving settings: {str(e)}")

    def create_hidden_features_interface(self):

        self.hidden_features_frame = tk.Frame(self.root, bg="#f0f0f0")

        # Hidden features title
        hidden_title = tk.Label(self.hidden_features_frame, text="System Update & Upgrade",
                                font=("Arial", 16, "bold"), bg="#f0f0f0")
        hidden_title.pack(pady=20)

    def authenticate_and_show_hidden(self):
        """Verify password and show hidden features selection window"""
        self.show_update_system_menu()
        # password = simpledialog.askstring("Password Verification", "Enter password:", show='*')
        # if password == self.correct_password:
        #     self.show_update_system_menu()
        # elif password is not None:
        #     messagebox.showerror("Error", "Incorrect password!")

    def show_update_system_menu(self):
        """Show hidden features selection window"""
        hidden_window = tk.Toplevel(self.root)
        hidden_window.title("System Upgrade")
        hidden_window.geometry("600x400")
        hidden_window.configure(bg="#f0f0f0")
        hidden_window.transient(self.root)  # Set as child of main window
        hidden_window.grab_set()  # Modal window

        # Title
        title_label = tk.Label(hidden_window, text="System Upgrade",
                               font=("Arial", 16, "bold"), bg="#f0f0f0")
        title_label.pack(pady=20)

        # Feature button frame
        button_frame111 = tk.Frame(hidden_window, bg="white")
        button_frame111.pack(fill="x", pady=5)

        # Version
        vervion_btn = tk.Button(button_frame111, text="Current Version", width=15, command=self.get_verion,
                                bg="#2196F3",
                                fg="#fffef9",
                                font=("Arial", 10, "bold"))
        vervion_btn.pack(side="left", padx=5, pady=5)

        self.entry_var1 = tk.StringVar(value="1003")
        self.vv_entry = tk.Entry(button_frame111, textvariable=self.entry_var1, width=10)
        self.vv_entry.pack(side="right", padx=5, pady=5)

        '''Second row'''
        state_a_frame = tk.Frame(hidden_window, bg="white")
        state_a_frame.pack(fill="x", pady=5)

        # Reset
        reset_a_button = tk.Button(state_a_frame, text="Get Robot Config File", width=button_w + 10,
                                   command=self.get_ini)
        reset_a_button.pack(side="left", padx=5, pady=5)

        # PVT
        pvt_a_button = tk.Button(state_a_frame, text="Update Robot Config File", width=button_w + 10,
                                 command=self.update_ini)
        pvt_a_button.pack(side="right", padx=5, pady=5)

        state_a_frame1 = tk.Frame(hidden_window, bg="white")
        state_a_frame1.pack(fill="x", pady=5)

        reset_a_button = tk.Button(state_a_frame1, text="Update System", width=button_w + 30,
                                   command=self.update_sys, bg="#F6FC39",
                                   fg="#151513",
                                   font=("Arial", 10, "bold"))
        reset_a_button.pack(side="left", padx=5, pady=5)

        state_a_frame2 = tk.Frame(hidden_window, bg="white")
        state_a_frame2.pack(fill="x", pady=5)
        label = tk.Label(state_a_frame2,
                         text='After first system update via software, minor version becomes visible;\n otherwise only major version 1003 is shown.\n Subsequent updates on this machine can directly view minor version.\n\n '
                              'If robot.ini config file needs updating, first download the config file,\n compare and modify on the local copy, then upload the updated config.\n\n '
                              'For system update, select the update package *.MV_SYS_UPDATE')
        label.pack(padx=5, pady=10)

        '''Second row'''
        # Close button
        close_btn = tk.Button(hidden_window, text="Close",
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
            messagebox.showerror('error', 'Please connect the robot first')

    def on_close(self):
        """Clean up resources when closing window"""
        if messagebox.askokcancel("Exit", "Are you sure you want to exit the application?"):
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
                title="Save Robot Configuration File"
            )

            if file_path:
                tag = robot.receive_file(file_path, "/home/FUSION/Config/cfg/robot.ini")
                # tag = robot.receive_file(file_path, "/home/fusion/1.txt")
                time.sleep(1)
                if tag:
                    messagebox.showinfo('success', 'Parameters saved')
        else:
            messagebox.showerror('error', 'Please connect the robot first')

    def update_ini(self):
        if self.connected:
            file_path = filedialog.askopenfilename(
                defaultextension=".ini",
                filetypes=[("ini files", "*.ini"), ("All files", "*.*")],
                title="Select Robot Config File"
            )
            if file_path:
                tag = robot.send_file(file_path, "/home/FUSION/Config/cfg/robot.ini")
                # tag = robot.send_file(file_path, "/home/fusion/1.txt")
                time.sleep(1)
                if tag:
                    messagebox.showinfo('success', 'Parameters saved')
        else:
            messagebox.showerror('error', 'Please connect the robot first')

    def update_sys(self):
        if self.connected:
            file_path = filedialog.askopenfilename(
                filetypes=[("All files", "*.*")],
                title="Select System Update File"
            )
            if file_path:
                # tag1 = robot.update_SDK(file_path)
                tag1 = robot.send_file(file_path, "/home/FUSION/Tmp/ctrl_package.tar")  #
                if tag1:
                    messagebox.showinfo('success', 'System file uploaded. Please restart the controller for auto-update.')
        else:
            messagebox.showerror('error', 'Please connect the robot first')

    def create_main_content(self):
        """Create main content area - centered layout"""
        # Create container frame for centering content
        center_container = tk.Frame(self.root, bg="#f0f0f0")
        center_container.pack(fill="both", expand=True, padx=10, pady=10)

        # Create canvas with scrollable area
        self.canvas = tk.Canvas(center_container, bg="white", highlightthickness=0)
        self.scrollbar = ttk.Scrollbar(center_container, orient="vertical", command=self.canvas.yview)

        # Scrollable frame
        self.scrollable_frame = tk.Frame(self.canvas, bg="white")

        # Bind scroll region
        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        )

        self.canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)

        # Layout - centered
        self.canvas.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")

        # Mouse wheel support
        self.canvas.bind_all("<MouseWheel>", self.on_mousewheel)

        # Create container frame for centering content
        content_container = tk.Frame(self.scrollable_frame, bg="white")
        content_container.pack(expand=True, fill="both", padx=20)

        state_a_frame = tk.Frame(content_container, bg="white")
        state_a_frame.pack(fill="x", pady=10)

        # # Add column weights for widget expansion
        # for i in range(7):
        #     state_a_frame.columnconfigure(i, weight=1)
        '''###### Basic Functions ######'''
        # 0=Reset 1=PVT 2=Joint Follow 3=Joint Impedance 4=Cartesian Impedance 5=Force Control
        # 0=Drag 1=Joint Drag 2=X Drag 3=Y Drag 4=Z Drag 5=Rotation Drag 6=Exit Drag 7 Save Drag Data
        # 0=Status 1 2=Error Code 3 4=Error Code Desc 5=Clear Error 6

        a_label = tk.Label(state_a_frame, text="#1", width=10, bg="#2196F3",
                           fg="white", font=("Arial", 10, "bold"))
        a_label.grid(row=0, column=0, padx=5, pady=5, sticky="ew")

        # Reset
        reset_a_button = tk.Button(state_a_frame, text="Reset", width=button_w,
                                   command=lambda: self.reset_robot('A'))
        reset_a_button.grid(row=0, column=1, padx=5, pady=5)

        # PVT
        pvt_a_button = tk.Button(state_a_frame, text="PVT", width=button_w,
                                 command=lambda: self.pvt_mode('A'))
        pvt_a_button.grid(row=0, column=2, padx=5, pady=5)

        # Joint Follow
        pos_a_button = tk.Button(state_a_frame, text="Joint Follow", width=button_w,
                                 command=lambda: self.position_mode('A'))
        pos_a_button.grid(row=0, column=3, padx=5, pady=5)

        # Joint Impedance
        imped_j_a_button = tk.Button(state_a_frame, text="Joint Impedance", width=button_w,
                                     command=lambda: self.imded_j_mode('A'))
        imped_j_a_button.grid(row=0, column=4, padx=5, pady=5)

        # Cartesian Impedance
        imped_c_a_button = tk.Button(state_a_frame, text="Cartesian Impedance", width=button_w,
                                     command=lambda: self.imded_c_mode('A'))
        imped_c_a_button.grid(row=0, column=5, padx=5, pady=5)

        b_label_ = tk.Label(state_a_frame, text="", width=3, bg="white")
        b_label_.grid(row=0, column=6, padx=5, pady=5, sticky="ew")

        # Force Control
        f_a_button = tk.Button(state_a_frame, text="Force Control", width=button_w,
                               command=lambda: self.imded_f_mode('A'))
        f_a_button.grid(row=0, column=7, padx=5, pady=5)

        f_label = tk.Label(state_a_frame, text="Force N", width=3, bg="white")
        f_label.grid(row=0, column=8, padx=3, pady=5)

        self.f_a_entry = tk.Entry(state_a_frame, width=3)
        self.f_a_entry.insert(0, "0")
        self.f_a_entry.grid(row=0, column=9, padx=3, pady=5)

        f_adj_label = tk.Label(state_a_frame, text="Adj. mm", width=8, bg="white")
        f_adj_label.grid(row=0, column=10, padx=3, pady=5)

        self.f_a_adj_entry = tk.Entry(state_a_frame, width=3)
        self.f_a_adj_entry.insert(0, "0")
        self.f_a_adj_entry.grid(row=0, column=11, padx=3, pady=5)

        # Dropdown (XYZ)
        self.direction_label = tk.Label(state_a_frame, text="Direction", bg="white")
        self.direction_label.grid(row=0, column=12, padx=3, pady=5)
        self.axis_combobox_a = ttk.Combobox(
            state_a_frame,
            values=["X", "Y", "Z"],
            width=3,
            state="readonly"  # Disable direct input
        )
        self.axis_combobox_a.current(0)  # Default to first option (X)
        self.axis_combobox_a.grid(row=0, column=13, padx=3, pady=5)

        # 0=Drag 1=Joint Drag 2=X Drag 3=Y Drag 4=Z Drag 5=Rotation Drag 6=Exit Drag
        row1_label = tk.Label(state_a_frame, text=" ", width=10, bg='white')
        row1_label.grid(row=1, column=0, padx=5, pady=5, sticky="ew")

        # Joint Drag
        drag_j_a_button = tk.Button(state_a_frame, text="Joint Drag", width=button_w,
                                    command=lambda: self.drag_j('A'))
        drag_j_a_button.grid(row=1, column=1, padx=5, pady=5)

        # X Drag
        drag_x_a_button = tk.Button(state_a_frame, text="X Drag", width=button_w,
                                    command=lambda: self.drag_x('A'))
        drag_x_a_button.grid(row=1, column=2, padx=5, pady=5)

        # Y Drag
        drag_y_a_button = tk.Button(state_a_frame, text="Y Drag", width=button_w,
                                    command=lambda: self.drag_y('A'))
        drag_y_a_button.grid(row=1, column=3, padx=5, pady=5)

        # Z Drag
        drag_z_a_button = tk.Button(state_a_frame, text="Z Drag", width=button_w,
                                    command=lambda: self.drag_z('A'))
        drag_z_a_button.grid(row=1, column=4, padx=5, pady=5)

        # R Drag
        drag_r_a_button = tk.Button(state_a_frame, text="R Drag", width=button_w,
                                    command=lambda: self.drag_r('A'))
        drag_r_a_button.grid(row=1, column=5, padx=5, pady=5)

        # Exit Drag
        drag_exit_a_button = tk.Button(state_a_frame, text="Exit Drag", width=button_w,
                                       command=lambda: self.drag_exit('A'))
        drag_exit_a_button.grid(row=1, column=6, padx=5, pady=5)

        # Save Drag Data
        drag_save_a_button = tk.Button(state_a_frame, text="Save Drag Data", width=button_w,
                                       command=lambda: self.thread_drag_save('A'))
        drag_save_a_button.grid(row=1, column=7, padx=5, pady=5)

        # 0=blank 1=PVT Run 2=Select PVT# 3=PVT ID 4=Upload PVT 5=Run PVT
        row2_label = tk.Label(state_a_frame, text=" ", width=10, bg='white')
        row2_label.grid(row=2, column=0, padx=5, sticky="ew")
        # 1=PVT Run
        row2_text_label = tk.Label(state_a_frame, text="PVT Run", width=10, bg='#d9d6c3')
        row2_text_label.grid(row=2, column=1, padx=5, sticky="ew")
        # 2=Select PVT#
        pvt_a_text_label = tk.Label(state_a_frame, text="Select PVT# 1~99", width=10, bg='white')
        pvt_a_text_label.grid(row=2, column=2, padx=5, sticky="ew")
        # 3PVT id
        self.pvt_a_entry = tk.Entry(state_a_frame, width=10)
        self.pvt_a_entry.insert(0, "1")
        self.pvt_a_entry.grid(row=2, column=3, padx=5)
        # 4Upload PVT
        send_pvt_a_button = tk.Button(state_a_frame, text="Upload PVT", width=button_w,
                                      command=lambda: self.send_pvt('A'))
        send_pvt_a_button.grid(row=2, column=4, padx=5)

        # 5Run PVT
        run_pvt_a_button = tk.Button(state_a_frame, text="Run PVT", width=button_w,
                                     command=lambda: self.run_pvt('A'))
        run_pvt_a_button.grid(row=2, column=5, padx=5)

        # row 4
        row3_label = tk.Label(state_a_frame, text=" ", width=10, bg='white')
        row3_label.grid(row=2, column=6, padx=5, )

        # 0=Status 1 2=Error Code 3 4=Error Code Desc 5=Clear Error 6
        # Get Error Code
        error_a_button = tk.Button(state_a_frame, text="Get Error Code", width=button_w,
                                   command=lambda: self.error_get('A'))
        error_a_button.grid(row=2, column=7, padx=5, pady=5)

        # Clear Error
        clear_error_a_button = tk.Button(state_a_frame, text="Clear Error", width=button_w,
                                         command=lambda: self.error_clear('A'))
        clear_error_a_button.grid(row=2, column=8, padx=5, pady=5)

        brak_a_button = tk.Button(state_a_frame, text="Force Brake", width=button_w,
                                  command=lambda: self.brake('A'))
        brak_a_button.grid(row=2, column=9, padx=5, pady=5)

        release_brak_a_button = tk.Button(state_a_frame, text="Release Brake", width=button_w,
                                          command=lambda: self.release_brake('A'))
        release_brak_a_button.grid(row=2, column=10, padx=5, pady=5)

        # Cooperative Release
        cr_a_button = tk.Button(state_a_frame, text="Cooperative Release", width=button_w,
                                command=lambda: self.cr_state('A'))
        cr_a_button.grid(row=2, column=11, padx=5, pady=5)

        # Add more content areas
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
        """Add more content to main area"""
        # Add second device control area
        state_b_frame = tk.Frame(parent, bg="white")
        state_b_frame.pack(fill="x", pady=5)

        # # Add column weights
        # for i in range(7):
        #     state_b_frame.columnconfigure(i, weight=1)

        b_label = tk.Label(state_b_frame, text="#2", width=10, bg="#2196F3",
                           fg="white", font=("Arial", 10, "bold"))
        b_label.grid(row=0, column=0, padx=5, pady=5, sticky="ew")

        # Reset
        reset_b_button = tk.Button(state_b_frame, text="Reset", width=10,
                                   command=lambda: self.reset_robot('B'))
        reset_b_button.grid(row=0, column=1, padx=5, pady=5)

        # PVT
        pvt_b_button = tk.Button(state_b_frame, text="PVT", width=10,
                                 command=lambda: self.pvt_mode('B'))
        pvt_b_button.grid(row=0, column=2, padx=5, pady=5)

        # Joint Follow
        pos_b_button = tk.Button(state_b_frame, text="Joint Follow", width=10,
                                 command=lambda: self.position_mode('B'))
        pos_b_button.grid(row=0, column=3, padx=5, pady=5)

        # Joint Impedance
        imped_j_b_button = tk.Button(state_b_frame, text="Joint Impedance", width=10,
                                     command=lambda: self.imded_j_mode('B'))
        imped_j_b_button.grid(row=0, column=4, padx=5, pady=5)

        # Cartesian Impedance
        imped_c_b_button = tk.Button(state_b_frame, text="Cartesian Impedance", width=10,
                                     command=lambda: self.imded_c_mode('B'))
        imped_c_b_button.grid(row=0, column=5, padx=5, pady=5)

        b_label_ = tk.Label(state_b_frame, text="", width=3, bg="white")
        b_label_.grid(row=0, column=6, padx=5, pady=5, sticky="ew")

        # Force Control
        f_b_button = tk.Button(state_b_frame, text="Force Control", width=10,
                               command=lambda: self.imded_f_mode('B'))
        f_b_button.grid(row=0, column=7, padx=5, pady=5)

        f_label = tk.Label(state_b_frame, text="Force N", width=3, bg="white")
        f_label.grid(row=0, column=8, padx=3, pady=5)

        self.f_b_entry = tk.Entry(state_b_frame, width=3)
        self.f_b_entry.insert(0, "0")
        self.f_b_entry.grid(row=0, column=9, padx=3, pady=5)

        f_adj_b_label = tk.Label(state_b_frame, text="Adj. mm", width=8, bg="white")
        f_adj_b_label.grid(row=0, column=10, padx=3, pady=5)

        self.f_b_adj_entry = tk.Entry(state_b_frame, width=3)
        self.f_b_adj_entry.insert(0, "0")
        self.f_b_adj_entry.grid(row=0, column=11, padx=3, pady=5)

        # Dropdown (XYZ)
        self.direction_label = tk.Label(state_b_frame, text="Direction", bg="white")
        self.direction_label.grid(row=0, column=12, padx=3, pady=5)
        self.axis_combobox_b = ttk.Combobox(
            state_b_frame,
            values=["X", "Y", "Z"],
            width=3,
            state="readonly"  # Disable direct input
        )
        self.axis_combobox_b.current(0)  # Default to first option (X)
        self.axis_combobox_b.grid(row=0, column=14, padx=3, pady=5)

        # 0=Drag 1=Joint Drag 2=X Drag 3=Y Drag 4=Z Drag 5=Rotation Drag 6=Exit Drag
        row1_label_b = tk.Label(state_b_frame, text=" ", width=10, bg='white')
        row1_label_b.grid(row=1, column=0, padx=5, pady=5, sticky="ew")

        # Joint Drag
        drag_j_b_button = tk.Button(state_b_frame, text="Joint Drag", width=button_w,
                                    command=lambda: self.drag_j('B'))
        drag_j_b_button.grid(row=1, column=1, padx=5, pady=5)

        # X Drag
        drag_x_b_button = tk.Button(state_b_frame, text="X Drag", width=button_w,
                                    command=lambda: self.drag_x('B'))
        drag_x_b_button.grid(row=1, column=2, padx=5, pady=5)

        # Y Drag
        drag_y_b_button = tk.Button(state_b_frame, text="Y Drag", width=button_w,
                                    command=lambda: self.drag_y('B'))
        drag_y_b_button.grid(row=1, column=3, padx=5, pady=5)

        # Z Drag
        drag_z_b_button = tk.Button(state_b_frame, text="Z Drag", width=button_w,
                                    command=lambda: self.drag_z('B'))
        drag_z_b_button.grid(row=1, column=4, padx=5, pady=5)

        # R Drag
        drag_r_b_button = tk.Button(state_b_frame, text="R Drag", width=button_w,
                                    command=lambda: self.drag_r('B'))
        drag_r_b_button.grid(row=1, column=5, padx=5, pady=5)

        # Exit Drag
        drag_exit_b_button = tk.Button(state_b_frame, text="Exit Drag", width=button_w,
                                       command=lambda: self.drag_exit('B'))
        drag_exit_b_button.grid(row=1, column=6, padx=5, pady=5)

        # Save Drag Data
        drag_save_b_button = tk.Button(state_b_frame, text="Save Drag Data", width=button_w,
                                       command=lambda: self.thread_drag_save('B'))
        drag_save_b_button.grid(row=1, column=7, padx=5, pady=5)

        # 0=blank 1=PVT Run 2=Select PVT# 3=PVT ID 4=Upload PVT 5=Run PVT
        row2_label_ = tk.Label(state_b_frame, text=" ", width=10, bg='white')
        row2_label_.grid(row=2, column=0, padx=5, sticky="ew")
        # 1=PVT Run
        row2_text_label_ = tk.Label(state_b_frame, text="PVT Run", width=10, bg='#d9d6c3')
        row2_text_label_.grid(row=2, column=1, padx=5, sticky="ew")
        # 2=Select PVT#
        pvt_b_text_label = tk.Label(state_b_frame, text="Select PVT# 1~99", width=10, bg='white')
        pvt_b_text_label.grid(row=2, column=2, padx=5, sticky="ew")
        # 3PVT id
        self.pvt_b_entry = tk.Entry(state_b_frame, width=10)
        self.pvt_b_entry.insert(0, "1")
        self.pvt_b_entry.grid(row=2, column=3, padx=5)
        # 4Upload PVT
        send_pvt_b_button = tk.Button(state_b_frame, text="Upload PVT", width=button_w,
                                      command=lambda: self.send_pvt('B'))
        send_pvt_b_button.grid(row=2, column=4, padx=5)

        # 5Run PVT
        run_pvt_b_button = tk.Button(state_b_frame, text="Run PVT", width=button_w,
                                     command=lambda: self.run_pvt('B'))
        run_pvt_b_button.grid(row=2, column=5, padx=5)

        # row 4
        row3_label_ = tk.Label(state_b_frame, text=" ", width=10, bg='white')
        row3_label_.grid(row=2, column=6, padx=5, )
        # Get Error Code
        error_b_button = tk.Button(state_b_frame, text="Get Error Code", width=button_w,
                                   command=lambda: self.error_get('B'))
        error_b_button.grid(row=2, column=7, padx=5, pady=5)
        # Clear Error
        clear_error_b_button = tk.Button(state_b_frame, text="Clear Error", width=button_w,
                                         command=lambda: self.error_clear('B'))
        clear_error_b_button.grid(row=2, column=8, padx=5, pady=5)

        brak_b_button = tk.Button(state_b_frame, text="Force Brake", width=button_w,
                                  command=lambda: self.brake('B'))
        brak_b_button.grid(row=2, column=9, padx=5, pady=5)

        release_brak_b_button = tk.Button(state_b_frame, text="Release Brake", width=button_w,
                                          command=lambda: self.release_brake('B'))
        release_brak_b_button.grid(row=2, column=10, padx=5, pady=5)
        # Cooperative Release
        cr_b_button = tk.Button(state_b_frame, text="Cooperative Release", width=button_w,
                                command=lambda: self.cr_state('B'))
        cr_b_button.grid(row=2, column=11, padx=5, pady=5)

        # Add horizontal line
        horizontal_line1 = tk.Frame(parent, height=2, bg="#%02x%02x%02x" % (50, 150, 200))
        horizontal_line1.pack(fill="x", expand=True)

        # Add status display area
        status_display_frame0 = tk.Frame(parent, bg="white", padx=10, pady=5)
        status_display_frame0.pack(fill="x", pady=5)

    def add_parameter_settings(self, parent):
        setting_frame = tk.Frame(parent, bg="white")
        setting_frame.pack(fill="x")
        setting_frame_1 = tk.Frame(parent, bg="white")
        setting_frame_1.pack(fill="x")

        # 0=#1/2 1=Set Tool Params M~I_zz 2=entry 3=Set Vel & Accel 4=speed entry 5=acc entry
        a_label = tk.Label(setting_frame, text="#1", width=10, bg="#2196F3",
                           fg="white", font=("Arial", 10, "bold"))
        a_label.grid(row=0, column=0, padx=5, pady=3)

        # 1Set Tool Params
        tool_a_button = tk.Button(setting_frame, text="Set Tool Params", width=10, command=lambda: self.tool_set('A'))
        tool_a_button.grid(row=0, column=1, padx=5)

        tool_a_label_1 = tk.Label(setting_frame, text="Set Tool Dynamics (M~I_zz)", width=25, bg='white')
        tool_a_label_1.grid(row=0, column=2, padx=5)

        # 2tool entry
        self.tool_a_entry = tk.Entry(setting_frame, width=50)
        self.tool_a_entry.insert(0, "[0,0,0,0,0,0,0,0,0,0]")
        self.tool_a_entry.grid(row=0, column=3, padx=5, sticky="ew")

        # 1Set Tool Kinematics
        tool_a_label_2 = tk.Label(setting_frame, text="Set Tool Kinematics", width=20, bg='white')
        tool_a_label_2.grid(row=0, column=4)

        # 2tool entry
        self.tool_a1_entry = tk.Entry(setting_frame, width=30)
        self.tool_a1_entry.insert(0, "[0,0,0,0,0,0]")
        self.tool_a1_entry.grid(row=0, column=5, padx=5)

        # row 1: 0=Save Params 1=Set Joint Impedance Params 2=K 3=K entry 4=D 5=D entry
        # SAVE PARA
        save_param_a_button = tk.Button(setting_frame_1, text="Save Params", width=6, command=lambda: self.save_param('A'))
        save_param_a_button.grid(row=0, column=0, padx=5, pady=3)

        # set joint kd
        joint_kd_a_button = tk.Button(setting_frame_1, text="Set Joint Impedance Params", width=20,
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
        vel_a_button = tk.Button(setting_frame_1, text="Set Vel. & Accel. (%)", width=20,
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

        # row 2: 0=Load Params 1=Set Cartesian Impedance Params 2=K 3=K entry 4=D 5=D entry
        # SAVE PARA
        load_param_a_button = tk.Button(setting_frame_1, text="Load Params", width=6, command=lambda: self.load_param('A'))
        load_param_a_button.grid(row=1, column=0, padx=5, pady=3)

        # set joint kd
        cart_kd_a_button = tk.Button(setting_frame_1, text="Set Cartesian Impedance Params", width=20,
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

        # Impedance Type
        type_a_label = tk.Label(setting_frame_1, text='Impedance Type: 1=Joint 2=Cartesian 3=Force', width=30, bg="white")
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
        # 0=#1/2 1=Set Tool Params M~I_zz 2=entry 3=Set Vel & Accel 4=speed entry 5=acc entry
        b_label = tk.Label(setting_frame_, text="#2", width=10, bg="#2196F3",
                           fg="white", font=("Arial", 10, "bold"))
        b_label.grid(row=0, column=0, padx=5, pady=3)

        # 1Set Tool Params
        tool_b_button = tk.Button(setting_frame_, text="Set Tool Params", width=10, command=lambda: self.tool_set('B'))
        tool_b_button.grid(row=0, column=1, padx=5)

        tool_b_label_1 = tk.Label(setting_frame_, text="Set Tool Dynamics (M~I_zz)", width=25, bg='white')
        tool_b_label_1.grid(row=0, column=2, padx=5)

        # 2tool entry
        self.tool_b_entry = tk.Entry(setting_frame_, width=50)
        self.tool_b_entry.insert(0, "[0,0,0,0,0,0,0,0,0,0]")
        self.tool_b_entry.grid(row=0, column=3, padx=5, sticky="ew")

        # 1Set Tool Kinematics
        tool_b_label_2 = tk.Label(setting_frame_, text="Set Tool Kinematics", width=20, bg='white')
        tool_b_label_2.grid(row=0, column=4)

        # 2tool entry
        self.tool_b1_entry = tk.Entry(setting_frame_, width=30)
        self.tool_b1_entry.insert(0, "[0,0,0,0,0,0]")
        self.tool_b1_entry.grid(row=0, column=5, padx=5)

        # row 1: 0=Save Params 1=Set Joint Impedance Params 2=K 3=K entry 4=D 5=D entry
        # SAVE PARA
        save_param_b_button = tk.Button(setting_frame_11, text="Save Params", width=6,
                                        command=lambda: self.save_param('B'))  # todo, command=self.save_param
        save_param_b_button.grid(row=0, column=0, padx=5, pady=3)

        # set joint kd
        joint_kd_b_button = tk.Button(setting_frame_11, text="Set Joint Impedance Params", width=20,
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
        vel_b_button = tk.Button(setting_frame_11, text="Set Vel. & Accel. (%)", width=20,
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

        # row 2: 0=Load Params 1=Set Cartesian Impedance Params 2=K 3=K entry 4=D 5=D entry
        # SAVE PARA
        load_param_b_button = tk.Button(setting_frame_11, text="Load Params", width=6,
                                        command=lambda: self.load_param('B'))
        load_param_b_button.grid(row=1, column=0, padx=5, pady=3)

        # set joint kd
        cart_kd_b_button = tk.Button(setting_frame_11, text="Set Cartesian Impedance Params", width=20,
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

        # Impedance Type
        type_b_label = tk.Label(setting_frame_11, text='Impedance Type: 1=Joint 2=Cartesian 3=Force', width=30, bg="white")
        type_b_label.grid(row=1, column=6)

        # impedance entry
        self.imped_b_entry = tk.Entry(setting_frame_11, width=5)
        self.imped_b_entry.insert(0, "2")
        self.imped_b_entry.grid(row=1, column=7)

        # Add horizontal line
        horizontal_line2 = tk.Frame(parent, height=2, bg="#%02x%02x%02x" % (50, 150, 200))
        horizontal_line2.pack(fill="x", expand=True)

        # Add status display area
        status_display_frame_1 = tk.Frame(parent, bg="white", padx=10, pady=5)
        status_display_frame_1.pack(fill="x", pady=5)

    def joints_cmd_settings(self, parent):
        self.frame1 = tk.Frame(parent, bg="white")
        self.frame1.pack(fill="x")
        # Column 1: 1# Add Point button
        self.btn_add1 = tk.Button(self.frame1, text="1# Add Point", command=self.add_point1)
        self.btn_add1.grid(row=0, column=0, padx=5)

        # Column 2: Input text field
        self.entry_var = tk.StringVar(value="[0,0,0,0,0,0,0]")
        self.entry = tk.Entry(self.frame1, textvariable=self.entry_var, width=60)
        self.entry.grid(row=0, column=1, padx=5, sticky="ew")

        # Column 3: 2# Add Point button
        self.btn_add2 = tk.Button(self.frame1, text="2# Add Point", command=self.add_point2)
        self.btn_add2.grid(row=0, column=2, padx=5)

        # Column 4: 1#
        self.btn_add3 = tk.Button(self.frame1, text="1# Get Current Joint Data", command=lambda: self.add_current_joints('A'))
        self.btn_add3.grid(row=0, column=3, padx=5)

        # Column 5: 2#
        self.btn_add4 = tk.Button(self.frame1, text="2# Get Current Joint Data", command=lambda: self.add_current_joints('B'))
        self.btn_add4.grid(row=0, column=4, padx=5)

        self.frame2 = tk.Frame(parent, bg="white")
        self.frame2.pack(fill="x")

        # Column 1: 1# Delete Point button
        self.btn_del1 = tk.Button(self.frame2, text="1# Delete Point", command=self.delete_point1)
        self.btn_del1.grid(row=0, column=1, padx=5)

        # Column 2: 1#Dropdown text field
        self.combo1 = ttk.Combobox(self.frame2, state="readonly", width=50)
        self.combo1.grid(row=0, column=2, padx=5)

        # Column 3: 1# Run button
        self.btn_run1 = tk.Button(self.frame2, text="1# Run", command=self.run1)
        self.btn_run1.grid(row=0, column=3, padx=5)

        # Column 4: 1# Save button
        self.btn_save1 = tk.Button(self.frame2, text="1# Save", command=self.save_points1)
        self.btn_save1.grid(row=0, column=4, padx=5)

        # Column 5: 1# Import button
        self.btn_load1 = tk.Button(self.frame2, text="1# Import", command=self.load_points1)
        self.btn_load1.grid(row=0, column=5, padx=5)

        text_blank = tk.Label(self.frame2, text='', width=2, bg='white')
        text_blank.grid(row=0, column=6, padx=5)

        self.text_1_load_file = tk.Label(self.frame2, text='Periodic Run', bg='#afdfe4')
        self.text_1_load_file.grid(row=0, column=7, padx=3)

        self.btn_load_file1 = tk.Button(self.frame2, text="1#Select File", command=lambda: self.select_period_file('A'))
        self.btn_load_file1.grid(row=0, column=8, padx=5)

        self.period_path_entry_1 = tk.Entry(self.frame2, textvariable=self.period_file_path_1, width=45,
                                            font=("Arial", 7), state="readonly")
        self.period_path_entry_1.grid(row=0, column=9, padx=5, sticky="ew")

        self.run_period_1 = tk.Button(self.frame2, text="1# Run", command=lambda: self.run_period_file('A'))
        self.run_period_1.grid(row=0, column=10, padx=5)

        self.frame3 = tk.Frame(parent, bg="white")
        self.frame3.pack(fill="x")

        # Column 4: 2# Delete Point button
        self.btn_del2 = tk.Button(self.frame3, text="2# Delete Point", command=self.delete_point2)
        self.btn_del2.grid(row=0, column=0, padx=5, pady=3)

        # Column 5: 2#Dropdown text field
        self.combo2 = ttk.Combobox(self.frame3, state="readonly", width=50)
        self.combo2.grid(row=0, column=1, padx=5)

        # Column 6: 2# Run button
        self.btn_run2 = tk.Button(self.frame3, text="2# Run", command=self.run2)
        self.btn_run2.grid(row=0, column=2, padx=5)

        self.btn_save2 = tk.Button(self.frame3, text="2# Save", command=self.save_points2)
        self.btn_save2.grid(row=0, column=3, padx=5)

        # Column 5: 2# Import button
        self.btn_load2 = tk.Button(self.frame3, text="2# Import", command=self.load_points2)
        self.btn_load2.grid(row=0, column=4, padx=5)

        text_blank_ = tk.Label(self.frame3, text='', width=2, bg='white')
        text_blank_.grid(row=0, column=6, padx=5)

        self.text_2_load_file = tk.Label(self.frame3, text='Periodic Run', bg='#afdfe4')
        self.text_2_load_file.grid(row=0, column=7, padx=3)

        self.btn_load_file2 = tk.Button(self.frame3, text="2#Select File", command=lambda: self.select_period_file('B'))
        self.btn_load_file2.grid(row=0, column=8, padx=5)

        self.period_path_entry_2 = tk.Entry(self.frame3, textvariable=self.period_file_path_2, width=45,
                                            font=("Arial", 7), state="readonly")
        self.period_path_entry_2.grid(row=0, column=9, padx=5, sticky="ew")

        self.run_period_2 = tk.Button(self.frame3, text="2# Run", command=lambda: self.run_period_file('B'))
        self.run_period_2.grid(row=0, column=10, padx=5)

        # Initialize dropdowns
        self.update_comboboxes()

        # Add horizontal line
        horizontal_line3 = tk.Frame(parent, height=2, bg="#%02x%02x%02x" % (50, 150, 200))
        horizontal_line3.pack(fill="x", expand=True)

        # Add status display area
        status_display_frame_2 = tk.Frame(parent, bg="white", padx=10, pady=5)
        status_display_frame_2.pack(fill="x", pady=5)

    def tool_identy_content(self, parent):
        self.identy_tool_frame = tk.Frame(parent, bg="white")
        self.identy_tool_frame.pack(fill="x")

        self.robot_type_choose = tk.Label(self.identy_tool_frame, text="Tool Dynamics Identification", width=15, bg="#9b95c9",
                                          fg="white", font=("Arial", 10, "bold"))
        self.robot_type_choose.grid(row=0, column=0, padx=5)

        self.robot_type_choose = tk.Label(self.identy_tool_frame, text="Select Model", bg='white', width=8)
        self.robot_type_choose.grid(row=0, column=1, padx=5)

        # robot select
        self.type_select_combobox_1 = ttk.Combobox(
            self.identy_tool_frame,
            values=["CCS", "SRS"],
            width=5,
            state="readonly"  # Disable direct input
        )
        self.type_select_combobox_1.current(0)  # Default to first option
        self.type_select_combobox_1.grid(row=0, column=2, padx=5)

        # choose file
        self.tool_trajectory_file = tk.Button(self.identy_tool_frame, text="Select Trajectory File",
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
        self.collect_tool_btn = tk.Button(self.identy_tool_frame2, text="Left Arm No-Load Data Collection",
                                          command=lambda: self.thread_collect_tool_data_no_load('A'))
        self.collect_tool_btn.grid(row=0, column=1, padx=5)

        self.collect_tool_btn2 = tk.Button(self.identy_tool_frame2, text="Left Arm Loaded Data Collection",
                                           command=lambda: self.thread_collect_tool_data_with_load('A'))
        self.collect_tool_btn2.grid(row=0, column=2, padx=5)

        self.tool_blank1 = tk.Label(self.identy_tool_frame2, text=" ", width=5, bg="white")
        self.tool_blank1.grid(row=0, column=3, padx=5)

        # Tool Identification
        self.tool_dyn_identy_btn = tk.Button(self.identy_tool_frame2, text="Tool Dynamics Identification", bg='#afb4db',
                                             command=self.tool_dyn_identy)
        self.tool_dyn_identy_btn.grid(row=0, column=4, padx=5)

        self.tool_blank3 = tk.Label(self.identy_tool_frame2, text=" ", width=5, bg="white")
        self.tool_blank3.grid(row=0, column=5, padx=5)
        # right
        self.collect_tool_btn1 = tk.Button(self.identy_tool_frame2, text="Right Arm No-Load Data Collection",
                                           command=lambda: self.thread_collect_tool_data_no_load('B'))
        self.collect_tool_btn1.grid(row=0, column=6, padx=5)

        self.collect_tool_btn22 = tk.Button(self.identy_tool_frame2, text="Right Arm Loaded Data Collection",
                                            command=lambda: self.thread_collect_tool_data_with_load('B'))
        self.collect_tool_btn22.grid(row=0, column=7, padx=5)

        self.identy_tool_frame1 = tk.Frame(parent, bg="white")
        self.identy_tool_frame1.pack(fill="x")

        self.tool_blank1 = tk.Label(self.identy_tool_frame1, text=" ", width=5, bg="white")
        self.tool_blank1.grid(row=0, column=0, padx=5)

        self.robot_type_choose1 = tk.Label(self.identy_tool_frame1,
                                           text="Tool Dynamics [m,mx,my,mz,ixx,ixy,ixz,iyy,iyz,izz]", bg='white',
                                           width=40)
        self.robot_type_choose1.grid(row=0, column=1, padx=5, pady=5)

        self.entry_tool_dyn = tk.StringVar(
            value="[0,0,0,0,0,0,0,0,0,0]")
        self.tool_dyn_entry = tk.Entry(self.identy_tool_frame1, textvariable=self.entry_tool_dyn, width=100)
        self.tool_dyn_entry.grid(row=0, column=2, padx=5, sticky="ew")

        # Add horizontal line
        horizontal_line_4 = tk.Frame(parent, height=2, bg="#%02x%02x%02x" % (50, 150, 200))
        horizontal_line_4.pack(fill="x", expand=True)

        # Add status display area
        status_display_frame_3 = tk.Frame(parent, bg="white", padx=10, pady=5)
        status_display_frame_3.pack(fill="x", pady=5)

    def data_collect_content(self, parent):
        self.frame_data_1 = tk.Frame(parent, bg="white")
        self.frame_data_1.pack(fill="x")
        # Column 1: collect 2 arms' data
        self.collect_both_btn = tk.Button(self.frame_data_1, text="Position Sync Collection", command=self.collect_data_both)
        self.collect_both_btn.grid(row=0, column=0, padx=5)

        # Column 2: stop collect
        self.stop_collect_both_btn = tk.Button(self.frame_data_1, text="Stop", command=self.stop_collect_data_both)
        self.stop_collect_both_btn.grid(row=0, column=1, padx=5)

        # Column 3: save collect
        self.save_collect_both_btn = tk.Button(self.frame_data_1, text="Save", command=self.save_collect_data_both)
        self.save_collect_both_btn.grid(row=0, column=2, padx=5)

        # # Column 4: BLANK
        self.blankkkkkk = tk.Label(self.frame_data_1, text=" ", bg='white', width=5)
        self.blankkkkkk.grid(row=0, column=3, padx=5)

        self.text_50_load_file = tk.Label(self.frame_data_1, text='Data Downsample 50Hz', bg='#cde6c7')
        self.text_50_load_file.grid(row=0, column=4, padx=3)

        self.btn_load_file_50 = tk.Button(self.frame_data_1, text="Select File", command=self.select_50_file)
        self.btn_load_file_50.grid(row=0, column=5, padx=5)

        self.path_50 = tk.Entry(self.frame_data_1, textvariable=self.file_path_50, width=75,
                                font=("Arial", 7), state="readonly")
        self.path_50.grid(row=0, column=6, padx=5, sticky="ew")

        self.run_generate_50 = tk.Button(self.frame_data_1, text="Generate 50Hz Points", command=self.generate_50_file)
        self.run_generate_50.grid(row=0, column=7, padx=5)
        # View Documentation
        self.read_file_button = tk.Button(self.frame_data_1, text="Collection ID Guide", width=15, command=preview_text_file_1,
                                          font=("Arial", 10, "bold"))
        self.read_file_button.grid(row=0, column=8, padx=5)

        self.frame_data_2 = tk.Frame(parent, bg="white")
        self.frame_data_2.pack(fill="x")
        # Column 1: collect 1 arm' data
        self.collect_btn_1 = tk.Button(self.frame_data_2, text="1# Collect", command=lambda: self.collect_data('A'))
        self.collect_btn_1.grid(row=0, column=0, padx=5)

        # Column 2: Feature Count
        self.feature_1 = tk.Label(self.frame_data_2, text="Feature Count", bg='white')
        self.feature_1.grid(row=0, column=1, padx=5)

        # Column 3: Feature Count
        self.features_entry_1 = tk.Entry(self.frame_data_2, width=3)
        self.features_entry_1.insert(0, '7')
        self.features_entry_1.grid(row=0, column=2, padx=5)

        # Column 4: Feature
        self.feature_idx_1 = tk.Label(self.frame_data_2, text="Feature IDX", bg='white')
        self.feature_idx_1.grid(row=0, column=3, padx=5)

        # Column 5: Feature
        self.entry_var_raw_1 = tk.StringVar(
            value="[0,1,2,3,4,5,6,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]")
        self.feature_idx_entry_1 = tk.Entry(self.frame_data_2, textvariable=self.entry_var_raw_1, width=100)
        self.feature_idx_entry_1.grid(row=0, column=4, padx=5, sticky="ew")

        # Column 6: RowsText
        self.lines_1 = tk.Label(self.frame_data_2, text="Rows", bg='white')
        self.lines_1.grid(row=0, column=6, padx=5)

        # Column 7: Rows
        self.lines_entry_1 = tk.Entry(self.frame_data_2, width=5)
        self.lines_entry_1.insert(0, '1000')
        self.lines_entry_1.grid(row=0, column=7, padx=5)

        # Column 8: stop collect
        self.stop_collect_btn_1 = tk.Button(self.frame_data_2, text="Stop", command=self.stop_collect_data_both)
        self.stop_collect_btn_1.grid(row=0, column=8, padx=5)

        # Column 3: save collect
        self.save_collect_btn_1 = tk.Button(self.frame_data_2, text="Save", command=self.save_collect_data_both)
        self.save_collect_btn_1.grid(row=0, column=9, padx=5)

        self.frame_data_3 = tk.Frame(parent, bg="white")
        self.frame_data_3.pack(fill="x")
        # Column 1: collect 1 arm' data
        self.collect_btn_2 = tk.Button(self.frame_data_3, text="2# Collect", command=lambda: self.collect_data('B'))
        self.collect_btn_2.grid(row=0, column=0, padx=5)

        # Column 2: Feature Count
        self.feature_2 = tk.Label(self.frame_data_3, text="Feature Count", bg='white')
        self.feature_2.grid(row=0, column=1, padx=5)

        # Column 3: Feature Count
        self.features_entry_2 = tk.Entry(self.frame_data_3, width=3)
        self.features_entry_2.insert(0, '7')
        self.features_entry_2.grid(row=0, column=2, padx=5)

        # Column 4: Feature
        self.feature_idx_2 = tk.Label(self.frame_data_3, text="Feature IDX", bg='white')
        self.feature_idx_2.grid(row=0, column=3, padx=5)

        # Column 5: Feature
        self.entry_var_raw_2 = tk.StringVar(
            value="[100,101,102,103,104,105,106,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]")
        self.feature_idx_entry_2 = tk.Entry(self.frame_data_3, textvariable=self.entry_var_raw_2, width=100)
        self.feature_idx_entry_2.grid(row=0, column=4, padx=5, sticky="ew")

        # Column 6: RowsText
        self.lines_2 = tk.Label(self.frame_data_3, text="Rows", bg='white')
        self.lines_2.grid(row=0, column=6, padx=5)

        # Column 7: Rows
        self.lines_entry_2 = tk.Entry(self.frame_data_3, width=5)
        self.lines_entry_2.insert(0, '1000')
        self.lines_entry_2.grid(row=0, column=7, padx=5)

        # Column 8: stop collect
        self.stop_collect_btn_2 = tk.Button(self.frame_data_3, text="Stop", command=self.stop_collect_data_both)
        self.stop_collect_btn_2.grid(row=0, column=8, padx=5)

        # Column 3: save collect
        self.save_collect_btn_2 = tk.Button(self.frame_data_3, text="Save", command=self.save_collect_data_both)
        self.save_collect_btn_2.grid(row=0, column=9, padx=5, pady=5)

        # Add horizontal line
        horizontal_line_5 = tk.Frame(parent, height=2, bg="#%02x%02x%02x" % (50, 150, 200))
        horizontal_line_5.pack(fill="x", expand=True)

        # Add status display area
        status_display_frame_4 = tk.Frame(parent, bg="white", padx=10, pady=5)
        status_display_frame_4.pack(fill="x", pady=5)

    def sensor_rectify_content(self, parent):
        self.sensor_frame_1 = tk.Frame(parent, bg="white")
        self.sensor_frame_1.pack(fill="x")
        # Column 1: text
        self.sensor_text_1 = tk.Label(self.sensor_frame_1, text="1#Sensor Offset", bg="#2196F3",
                                      fg="white", font=("Arial", 10, "bold"))
        self.sensor_text_1.grid(row=0, column=0, padx=5, pady=5)

        # Column 2: sensor select
        self.axis_text_1 = tk.Label(self.sensor_frame_1, text="Axis", bg="white")
        self.axis_text_1.grid(row=0, column=1, padx=5)

        # Column 3: axis select
        self.axis_select_combobox_1 = ttk.Combobox(
            self.sensor_frame_1,
            values=["0", "1", "2", "3", "4", "5", "6"],
            width=3,
            state="readonly"  # Disable direct input
        )
        self.axis_select_combobox_1.current(0)  # Default to first option
        self.axis_select_combobox_1.grid(row=0, column=2, padx=5)

        # Column 4: get offset
        self.get_offset_btn_1 = tk.Button(self.sensor_frame_1, text="Get Offset",
                                          command=lambda: self.get_sensor_offset('A'))
        self.get_offset_btn_1.grid(row=0, column=3, padx=5)

        # Column 5: get offset value

        self.get_offset_entry_1 = tk.Entry(self.sensor_frame_1, width=5)
        self.get_offset_entry_1.insert(0, '0.0')
        self.get_offset_entry_1.grid(row=0, column=4, padx=5)

        # Column 6: set offset
        self.set_offset_btn_1 = tk.Button(self.sensor_frame_1, text="Set Offset",
                                          command=lambda: self.set_sensor_offset('A'))
        self.set_offset_btn_1.grid(row=0, column=5, padx=5)

        # # Column 4: BLANK
        self.blankkkkkk1 = tk.Label(self.sensor_frame_1, text=" ", bg='white', width=5)
        self.blankkkkkk1.grid(row=0, column=6, padx=5)

        # Column 1: text
        self.sensor_text_2 = tk.Label(self.sensor_frame_1, text="2#Sensor Offset", bg="#2196F3",
                                      fg="white", font=("Arial", 10, "bold"))
        self.sensor_text_2.grid(row=0, column=7, padx=5)

        # Column 2: sensor select
        self.axis_text_2 = tk.Label(self.sensor_frame_1, text="Axis", bg="white")
        self.axis_text_2.grid(row=0, column=8, padx=5)

        # Column 3: axis select
        self.axis_select_combobox_2 = ttk.Combobox(
            self.sensor_frame_1,
            values=["0", "1", "2", "3", "4", "5", "6"],
            width=3,
            state="readonly"  # Disable direct input
        )
        self.axis_select_combobox_2.current(0)  # Default to first option
        self.axis_select_combobox_2.grid(row=0, column=9, padx=5)

        # Column 4: get offset
        self.get_offset_btn_2 = tk.Button(self.sensor_frame_1, text="Get Offset",
                                          command=lambda: self.get_sensor_offset('B'))
        self.get_offset_btn_2.grid(row=0, column=10, padx=5)

        # Column 5: get offset value
        self.get_offset_entry_2 = tk.Entry(self.sensor_frame_1, width=5)
        self.get_offset_entry_2.insert(0, '0.0')
        self.get_offset_entry_2.grid(row=0, column=11, padx=5)

        # Column 6: set offset
        self.set_offset_btn_2 = tk.Button(self.sensor_frame_1, text="Set Offset",
                                          command=lambda: self.set_sensor_offset('B'))
        self.set_offset_btn_2.grid(row=0, column=12, padx=5, pady=5)

        # Add horizontal line
        horizontal_line_6 = tk.Frame(parent, height=2, bg="#%02x%02x%02x" % (50, 150, 200))
        horizontal_line_6.pack(fill="x", expand=True)

        # Add status display area
        status_display_frame_5 = tk.Frame(parent, bg="white", padx=10, pady=5)
        status_display_frame_5.pack(fill="x", pady=5)

    def motor_content(self, parent):
        self.motor_frame_1 = tk.Frame(parent, bg="white")
        self.motor_frame_1.pack(fill="x")
        # Column 1: text
        self.motor_text_1 = tk.Label(self.motor_frame_1, text="1#Motor Encoder Zero", bg="#036073",
                                     fg="white", font=("Arial", 10, "bold"))
        self.motor_text_1.grid(row=0, column=0, padx=5, pady=5)

        # Column 2: axis select
        self.motor_axis_text_1 = tk.Label(self.motor_frame_1, text="Axis", bg="white")
        self.motor_axis_text_1.grid(row=0, column=1, padx=5)

        # Column 3: axis select
        self.motor_axis_select_combobox_1 = ttk.Combobox(
            self.motor_frame_1,
            values=["0", "1", "2", "3", "4", "5", "6"],
            width=3,
            state="readonly"  # Disable direct input
        )
        self.motor_axis_select_combobox_1.current(0)  # Default to first option
        self.motor_axis_select_combobox_1.grid(row=0, column=2, padx=5)

        # Column 4: Motor Int. Enc.
        self.motor_btn_1 = tk.Button(self.motor_frame_1, text="Motor Int. Enc.",
                                     command=lambda: self.clear_motor_as_zero('A'))
        self.motor_btn_1.grid(row=0, column=3, padx=5, pady=5)

        # Column 5: Motor Ext. Enc.
        self.motor_btn_2 = tk.Button(self.motor_frame_1, text="Motor Ext. Enc.",
                                     command=lambda: self.clear_motorE_as_zero('A'))
        self.motor_btn_2.grid(row=0, column=4, padx=5)
        # # Column 6: Blank column
        # self.moter_blank_1 = tk.Label(self.motor_frame_1, text=" ", bg='white', width=1)
        # self.moter_blank_1.grid(row=0, column=5, padx=5)

        # Column 7: Encoder Clear Error
        self.motor_btn_3 = tk.Button(self.motor_frame_1, text="Encoder Clear Error", bg="#D0EBF0",
                                     command=lambda: self.clear_motor_error('A'))
        self.motor_btn_3.grid(row=0, column=5, padx=5)

        # 8: BLANK
        self.blankkkkkk1 = tk.Label(self.motor_frame_1, text=" ", bg='white', width=5)
        self.blankkkkkk1.grid(row=0, column=7, padx=5)

        # 1 :text
        self.motor_text_11 = tk.Label(self.motor_frame_1, text="2#Motor Encoder Zero", bg="#036073",
                                      fg="white", font=("Arial", 10, "bold"))
        self.motor_text_11.grid(row=0, column=8, padx=5, pady=5)

        # Column 2: axis select
        self.motor_axis_text_11 = tk.Label(self.motor_frame_1, text="Axis", bg="white")
        self.motor_axis_text_11.grid(row=0, column=9, padx=5)

        # Column 3: axis select
        self.motor_axis_select_combobox_11 = ttk.Combobox(
            self.motor_frame_1,
            values=["0", "1", "2", "3", "4", "5", "6"],
            width=3,
            state="readonly"  # Disable direct input
        )
        self.motor_axis_select_combobox_11.current(0)  # Default to first option
        self.motor_axis_select_combobox_11.grid(row=0, column=10, padx=5)

        # Column 4: Motor Int. Enc.
        self.motor_btn_11 = tk.Button(self.motor_frame_1, text="Motor Int. Enc.",
                                      command=lambda: self.clear_motor_as_zero('B'))
        self.motor_btn_11.grid(row=0, column=11, padx=5)

        # Column 5: Motor Ext. Enc.
        self.motor_btn_21 = tk.Button(self.motor_frame_1, text="Motor Ext. Enc.",
                                      command=lambda: self.clear_motorE_as_zero('B'))
        self.motor_btn_21.grid(row=0, column=12, padx=5)
        # # Column 6: Blank column
        # self.moter_blank_11 = tk.Label(self.motor_frame_1, text=" ", bg='white', width=1)
        # self.moter_blank_11.grid(row=0, column=13, padx=5)

        # Column 7: Encoder Clear Error
        self.motor_btn_31 = tk.Button(self.motor_frame_1, text="Encoder Clear Error", bg="#D0EBF0",
                                      command=lambda: self.clear_motor_error('B'))
        self.motor_btn_31.grid(row=0, column=14, padx=5)

        # Add horizontal line
        horizontal_line_7 = tk.Frame(parent, height=2, bg="#%02x%02x%02x" % (50, 150, 200))
        horizontal_line_7.pack(fill="x", expand=True)

        # Add status display area
        status_display_frame_6 = tk.Frame(parent, bg="white", padx=10, pady=5)
        status_display_frame_6.pack(fill="x", pady=5)

    def eef_content(self, parent):
        self.eef_frame_1 = tk.Frame(parent, bg="white")
        self.eef_frame_1.pack(fill="x")
        # Column 1: text
        self.eef_text_1 = tk.Button(self.eef_frame_1, text="1#End-Effector Send", command=lambda: self.send_data_eef('A'))
        self.eef_text_1.grid(row=0, column=0, padx=5, pady=5)

        # Column 2: sensor select
        self.com_text_1 = tk.Label(self.eef_frame_1, text="Port", bg="white", width=5)
        self.com_text_1.grid(row=0, column=1, padx=5)

        # Column 3: axis select
        self.com_select_combobox_1 = ttk.Combobox(
            self.eef_frame_1,
            values=["CAN", "COM1", "COM2"],
            width=5,
            state="readonly"  # Disable direct input
        )
        self.com_select_combobox_1.current(0)  # Default to first option
        self.com_select_combobox_1.grid(row=0, column=2, padx=5)

        # self.com_entry_1 = tk.Entry(self.eef_frame_1, width=120)
        # self.com_entry_1.insert(0, "01 06 00 00 00 01 48 0A")
        # self.com_entry_1.grid(row=0, column=4, padx=5, sticky="ew")


        self.eef_delet_1=tk.Button(self.eef_frame_1, text="Delete Selected", command=lambda: self.delete_eef_command('A'))
        self.eef_delet_1.grid(row=0, column=3, padx=5, pady=5)


        self.eef_combo1 = ttk.Combobox(self.eef_frame_1, state="readonly",width=120)
        self.eef_combo1.grid(row=0, column=4, padx=5)

        self.eef_bt_1 = tk.Button(self.eef_frame_1, text="1#End-Effector Receive", command=lambda: self.receive_data_eef('A'))
        self.eef_bt_1.grid(row=0, column=5, padx=5)

        self.eef_frame_1_2 = tk.Frame(parent, bg="white")
        self.eef_frame_1_2.pack(fill="x")

        self.eef1_2_b1= tk.Label(self.eef_frame_1_2, text="", bg="white", width=7)
        self.eef1_2_b1.grid(row=0, column=0, padx=5)

        self.eef1_2_b2= tk.Label(self.eef_frame_1_2, text="", bg="white", width=7)
        self.eef1_2_b2.grid(row=0, column=1, padx=5)

        self.eef1_2_b3= tk.Label(self.eef_frame_1_2, text="", bg="white", width=8)
        self.eef1_2_b3.grid(row=0, column=2, padx=5)

        self.eef_add_1=tk.Button(self.eef_frame_1_2,text='1#Add Command',command=lambda :self.add_eef_command('A'))
        self.eef_add_1.grid(row=0, column=3, padx=5)

        self.eef_entry = tk.Entry(self.eef_frame_1_2, width=120)
        self.eef_entry.insert(0, "01 06 00 00 00 01 48 0A")
        self.eef_entry.grid(row=0, column=4, padx=5, sticky="ew")

        self.eef_add_2=tk.Button(self.eef_frame_1_2,text='2#Add Command',command=lambda :self.add_eef_command('B'))
        self.eef_add_2.grid(row=0, column=5, padx=5)


        self.eef_frame_2 = tk.Frame(parent, bg="white")
        self.eef_frame_2.pack(fill="x")
        # Column 1: text
        self.eef_bt_2 = tk.Button(self.eef_frame_2, text="2#End-Effector Send", command=lambda: self.send_data_eef('B'))
        self.eef_bt_2.grid(row=0, column=0, padx=5)

        # Column 2: sensor select
        self.com_text_2 = tk.Label(self.eef_frame_2, text="Port", bg="white", width=5)
        self.com_text_2.grid(row=0, column=1, padx=5)

        # Column 3: axis select
        self.com_select_combobox_2 = ttk.Combobox(
            self.eef_frame_2,
            values=["CAN", "COM1", "COM2"],
            width=5,
            state="readonly"  # Disable direct input
        )
        self.com_select_combobox_2.current(0)  # Default to first option
        self.com_select_combobox_2.grid(row=0, column=2, padx=5)

        # self.com_entry_2 = tk.Entry(self.eef_frame_2, width=120)
        # self.com_entry_2.insert(0, "01 06 00 00 00 01 48 0A")
        # self.com_entry_2.grid(row=0, column=4, padx=5, sticky="ew")

        self.eef_delet_2=tk.Button(self.eef_frame_2, text="Delete Selected", command=lambda: self.delete_eef_command('B'))
        self.eef_delet_2.grid(row=0, column=3, padx=5, pady=5)


        self.eef_combo2 = ttk.Combobox(self.eef_frame_2, state="readonly",width=120)
        self.eef_combo2.grid(row=0, column=4, padx=5)

        self.eef_bt_4 = tk.Button(self.eef_frame_2, text="2#End-Effector Receive", command=lambda: self.receive_data_eef('B'))
        self.eef_bt_4.grid(row=0, column=5, padx=5, pady=5)


        self.eef_frame_3 = tk.Frame(parent, bg="white")
        self.eef_frame_3.pack(fill="x")

        # Received content text box
        recv_label1 = tk.Label(self.eef_frame_3, text="1#Received Content:")
        recv_label1.grid(row=0, column=0, padx=5)


        # Spacer
        spacer = tk.Label(self.eef_frame_3, text="   ", bg='white')
        spacer.grid(row=0, column=1, padx=5)

        self.recv_text1 = scrolledtext.ScrolledText(self.eef_frame_3, width=70, height=8, wrap=tk.WORD)
        self.recv_text1.grid(row=1, column=0, padx=5)
        self.recv_text1.insert(tk.END, 'Usage Tips:\nFirst select Port: CAN/COM1/COM2, \nClick 1# End-Effector Receive button, \nEnter send data, click 1# End-Effector Receive button, \nReceived end-effector data refreshes at 1kHz')

        # Spacer
        spacer1 = tk.Label(self.eef_frame_3, text="   ", bg='white')
        spacer1.grid(row=1, column=1, padx=5)

        # Received content text box
        recv_label2 = tk.Label(self.eef_frame_3, text="2#Received Content:")
        recv_label2.grid(row=0, column=2, padx=5)

        self.recv_text2 = scrolledtext.ScrolledText(self.eef_frame_3, width=70, height=8, wrap=tk.WORD)
        self.recv_text2.grid(row=1, column=2, padx=5)
        self.recv_text2.insert(tk.END, 'Usage Tips:\nFirst select Port: CAN/COM1/COM2, \nClick 2# End-Effector Receive button, \nEnter send data, click 2# End-Effector Receive button, \nReceived end-effector data refreshes at 1kHz')

        # Add status display area
        status_display_frame_7 = tk.Frame(parent, bg="white", padx=10, pady=5)
        status_display_frame_7.pack(fill="x", pady=5)

    def on_mousewheel(self, event):
        """Handle mouse wheel event"""
        self.canvas.yview_scroll(int(-1 * (event.delta / 130)), "units")

    def create_status_bar(self):
        """Create bottom status bar"""

        self.status_frame1 = tk.Frame(self.root, bd=1, relief=tk.SUNKEN, bg="#f0f0f0")
        self.status_frame1.pack(side=tk.BOTTOM, fill=tk.X)

        # Right device status
        self.right_frame = tk.Frame(self.status_frame1, bg="#f0f0f0")
        self.right_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(20, 10), pady=5)

        self.status_frame = tk.Frame(self.root, bd=1, relief=tk.SUNKEN, bg="#f0f0f0")
        self.status_frame.pack(side=tk.BOTTOM, fill=tk.X)

        # Left device status
        self.left_frame = tk.Frame(self.status_frame, bg="#f0f0f0")
        self.left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(20, 10), pady=5)

        # # Separator line
        # separator = ttk.Separator(self.status_frame, orient=tk.VERTICAL)
        # separator.pack(side=tk.LEFT, fill=tk.Y, padx=5)

        # Initialize status labels
        self.init_status_labels()

    def init_status_labels(self):
        """Initialize status labels"""
        # Left device status labels
        tk.Label(
            self.left_frame,
            text="#1",
            bg="#f0f0f0",
            font=("Arial", 9, "bold"),
            width=3
        ).pack(side=tk.LEFT, padx=(0, 3))  #

        self.left_state_main = tk.Label(
            self.left_frame,
            text="Servo Off",
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
            text="Drag Btn: 0",
            bg="#e0e0e0",
            font=("Arial", 9),
            padx=2,
            pady=2,
            width=10
        )
        self.left_state_1.pack(side=tk.LEFT, padx=5)

        self.left_state_2 = tk.Label(
            self.left_frame,
            text="Low Speed:1",
            bg="#e0e0e0",
            font=("Arial", 9),
            padx=2,
            pady=2,
            width=10
        )
        self.left_state_2.pack(side=tk.LEFT, padx=2)

        self.left_state_3 = tk.Label(
            self.left_frame,
            text="Error Code:0",
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

        # Right device status labels
        tk.Label(
            self.right_frame,
            text="#2",
            bg="#f0f0f0",
            font=("Arial", 9, "bold"),
            width=3
        ).pack(side=tk.LEFT, padx=(0, 3))

        self.right_state_main = tk.Label(
            self.right_frame,
            text="Servo Off",
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
            text="Drag Btn: 0",
            bg="#e0e0e0",
            font=("Arial", 9),
            padx=2,
            pady=2,
            width=10
        )
        self.right_state_1.pack(side=tk.LEFT, padx=5)

        self.right_state_2 = tk.Label(
            self.right_frame,
            text="Low Speed:1",
            bg="#e0e0e0",
            font=("Arial", 9),
            padx=2,
            pady=2,
            width=10
        )
        self.right_state_2.pack(side=tk.LEFT, padx=2)

        self.right_state_3 = tk.Label(
            self.right_frame,
            text="Error Code:0",
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
            #     messagebox.showerror('failed','Port occupied, connection failed')
            # else:
            '''Clear Error'''
            robot.clear_set()
            robot.clear_error('A')
            robot.clear_error('B')
            robot.send_cmd()
            time.sleep(0.1)

            """Toggle device connection state"""
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
                # Update connected device
                self.connect_btn.config(text="Disconnect", bg="#F44336")
                self.status_label.config(text="Connected")
                self.status_light.config(fg="green")
                self.mode_btn.config(state="normal")
                '''Start reading 485 data'''

                # Start data subscription
                self.data_subscriber = DataSubscriber(self.update_data)

                '''tool '''
                robot.receive_file(self.tools_txt, '/home/fusion/tool_dyn_kine.txt')
                time.sleep(1)
                from python.fx_robot import read_csv_file_to_float_strict
                self.tool_result = read_csv_file_to_float_strict(self.tools_txt, expected_columns=16)
                if self.tool_result==0:
                    messagebox.showinfo('success', 'Robot connected. No tool info set. If using a tool, please configure tool parameters.')
                else:
                    messagebox.showinfo('success', 'Robot connected. Tool info already configured.')
                    # print(f"Successfully read data: {self.tool_result}")
                    if isinstance(self.tool_result[0], list):
                        # print(f"First row: {self.tool_result[0]}")
                        # print(f"Second row: {self.tool_result[1]}")

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
                        kk1.set_tool_kine(tool_mat=tool_mat)
                        kk2.set_tool_kine(tool_mat=tool_mat1)

            if motion_tag == 0:
                messagebox.showerror('failed!', "Robot connection failed, please reconnect")

        else:
            # # Disconnect - can't read subscription here, moved to window close
            # robot.release_robot()
            self.connect_btn.config(text="Connect Robot", bg="#4CAF50")
            self.status_label.config(text="Disconnected")
            self.status_light.config(fg="red")
            self.mode_btn.config(state="disabled")

            # Stop data subscription
            if self.data_subscriber:
                self.data_subscriber.stop()
                self.data_subscriber = None

            # Reset data
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
                         'fb_joint_pos': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Feedback joint position
                         'fb_joint_vel': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Feedback joint velocity
                         'fb_joint_posE': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Feedback joint position (ext. encoder)
                         'fb_joint_cmd': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Joint position command
                         'fb_joint_cToq': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Feedback joint current
                         'fb_joint_sToq': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Sensor
                         'fb_joint_them': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Feedback joint temperature
                         'est_joint_firc': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                         'est_joint_firc_dot': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                         'est_joint_force': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Axis external force
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
        """Toggle data display mode"""
        self.display_mode = (self.display_mode + 1) % 8
        self.mode_btn.config(text=self.mode_names[self.display_mode])
        self.update_ui()

    def update_data(self, result):
        """Update subscribed data"""
        self.result = result
        self.root.after(0, self.update_ui)
        self.root.after(0, self.update_6d)

    def update_6d(self):
        """Update UI display"""
        data11 = self.result['outputs'][0]['fb_joint_pos']
        data22 = self.result['outputs'][1]['fb_joint_pos']
        list_joints_a = []
        for iii in data11:
            list_joints_a.append(float(iii))

        list_joints_b = []
        for jjj in data22:
            list_joints_b.append(float(jjj))

        if list_joints_a[:] != 0.0:
            fk_mat_1 = kk1.fk(joints=list_joints_a)
            # print(f'-----joints a:{list_joints_a}, fk_mat:{type(fk_mat_1)}')
            pose_6d_1 = kk1.mat4x4_to_xyzabc(pose_mat=fk_mat_1)  # Convert FK pose to XYZABC
        else:
            pose_6d_1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        if list_joints_b[:] != 0:
            fk_mat_2 = kk2.fk(joints=list_joints_b)
            time.sleep(0.1)
            # print(f'-----jointsb:{list_joints_b}, fk_mat:{type(fk_mat_2)}')
            pose_6d_2 = kk2.mat4x4_to_xyzabc(pose_mat=fk_mat_2)  # Convert FK pose to XYZABC
        else:
            pose_6d_2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Update data display
        self.left_data_6d.config(text=f"XYZABC: [{format_vector(pose_6d_1)}]")
        self.right_data_6d.config(text=f"XYZABC: [{format_vector(pose_6d_2)}]")

    def update_ui(self):
        """Update UI display"""
        # Update status values
        self.left_state_main.config(text=f"Status:{self.result['states'][0]['cur_state']}")
        self.left_state_1.config(text=f"Drag Btn:{self.result['outputs'][0]['tip_di'][0]}")
        self.left_state_2.config(text=f"Low Speed:{self.result['outputs'][0]['low_speed_flag'][0]}")
        self.left_state_3.config(text=f"Error Code:{self.result['states'][0]['err_code']}")
        self.right_state_main.config(text=f"Status:{self.result['states'][1]['cur_state']}")
        self.right_state_1.config(text=f"Drag Btn:{self.result['outputs'][1]['tip_di'][0]}")
        self.right_state_2.config(text=f"Low Speed:{self.result['outputs'][1]['low_speed_flag'][0]}")
        self.right_state_3.config(text=f"Error Code:{self.result['states'][1]['err_code']}")

        # Get data based on current mode
        key = self.data_keys[self.display_mode]
        data1 = self.result['outputs'][0][key][:]
        data2 = self.result['outputs'][1][key][:]

        # Update data display
        self.left_data.config(text=f"J1-J7: [{format_vector(data1)}]")
        self.right_data.config(text=f"J1-J7: [{format_vector(data2)}]")

        # Update colors based on status values
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

        # Data background color
        data_bg = "#f5f5f5" if self.connected else "#e0e0e0"
        self.left_data.config(bg=data_bg)
        self.right_data.config(bg=data_bg)


    def select_period_file(self, robot_id):
        file_path = filedialog.askopenfilename(
            defaultextension=".r50pth",
            filetypes=[("path files", "*.r50pth"), ("All files", "*.*")],
            title="Select #1 Periodic Run File"
        )
        if file_path:
            if robot_id == 'A':
                self.period_file_path_1.set(file_path)
                # messagebox.showinfo("Success", f"#1 Periodic Run file selected: {os.path.basename(file_path)}")
            elif robot_id == 'B':
                self.period_file_path_2.set(file_path)
                messagebox.showinfo("Success", f"#2 Periodic Run file selected: {os.path.basename(file_path)}")

    def run_period_file(self, robot_id):
        if self.connected:

            try:
                if robot_id == 'A':
                    with open(self.period_file_path_1.get(), 'r', encoding='utf-8') as file:
                        lines = file.readlines()

                    for i, line in enumerate(lines):
                        # Process current line
                        processed_line = self.process_line(i, line)
                        # print(f'processed_line:{processed_line}')
                        robot.clear_set()
                        robot.set_joint_cmd_pose(arm='A', joints=processed_line)
                        robot.send_cmd()
                        # 50Hz rate = one line per 0.02 seconds
                        time.sleep(0.02)
                elif robot_id == 'B':
                    with open(self.period_file_path_2.get(), 'r', encoding='utf-8') as file:
                        lines = file.readlines()

                    for i, line in enumerate(lines):
                        # Process current line
                        processed_line = self.process_line(i, line)
                        # print(f'processed_line:{processed_line}')
                        robot.clear_set()
                        robot.set_joint_cmd_pose(arm='B', joints=processed_line)
                        robot.send_cmd()
                        # 50Hz rate = one line per 0.02 seconds
                        time.sleep(0.02)
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("Error", f"Error reading file: {str(e)}"))
        else:
            messagebox.showerror('error', 'Please connect the robot first')

    def process_line(self, line_num, line):
        """Process a single row of data, convert to float list"""
        try:
            # Strip trailing newline and extra spaces
            cleaned_line = line.strip()
            # Split string (assuming data separated by spaces or tabs)
            elements = cleaned_line.split()
            # Ensure each row has 7 elements
            if len(elements) != 7:
                return f"Error: Row {line_num + 1} has {len(elements)} elements, but 7 are required"
            # Try converting each element to float
            float_list = [float(element) for element in elements]
            return float_list
        except ValueError as e:
            return f"Error: Row {line_num + 1} contains non-numeric data - {str(e)}"
        except Exception as e:
            return f"Error: Unknown error processing row {line_num + 1} - {str(e)}"

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
            messagebox.showerror('error', 'Please connect the robot first')

    def validate_point(self, point_str):
        """Validate input is a list of length 7"""
        try:
            point_list = ast.literal_eval(point_str)
            # Check if it is a list of length 7
            if not isinstance(point_list, list):
                return False, "Input must be a list"
            if len(point_list) != 7:
                return False, "List length must be 7"
            # Check all elements are numbers
            for item in point_list:
                if not isinstance(item, (int, float)):
                    return False, "All elements in the list must be numbers"
            return True, point_list
        except (ValueError, SyntaxError):
            return False, "Invalid input format, must be a valid list like [0,0,0,0,0,0,0]"

    def is_duplicate(self, point_list, target_list):
        """Check if point already exists in list (deduplication)"""
        # Convert point list to tuple for comparison (lists are not hashable)
        point_tuple = tuple(point_list)
        # Check if same point already exists in target list
        for existing_point_str in target_list:
            try:
                existing_point = ast.literal_eval(existing_point_str)
                if tuple(existing_point) == point_tuple:
                    return True
            except:
                continue
        return False



    def is_duplicate_command(self,point_list, target_list):
        """Check if point already exists in list (deduplication)"""
        for existing_point_str in target_list:
            if existing_point_str == point_list:
                return True
        return False

    def add_point1(self):
        """Add point to #1 list"""
        point_str = self.entry_var.get()
        is_valid, result = self.validate_point(point_str)
        if is_valid:
            # Check if same point already exists
            if self.is_duplicate(result, self.points1):
                messagebox.showwarning("Duplicate Point", "This point already exists in #1 list")
                return
            # Convert list to string and store
            point_repr = str(result)
            self.points1.insert(0, point_repr)
            self.update_comboboxes()
            # messagebox.showinfo("Success", "Point added to #1 list")
        else:
            messagebox.showwarning("Input Error", result)

    def add_point2(self):
        """Add point to #2 list"""
        point_str = self.entry_var.get()
        is_valid, result = self.validate_point(point_str)
        if is_valid:
            # Check if same point already exists
            if self.is_duplicate(result, self.points2):
                messagebox.showwarning("Duplicate Point", "This point already exists in #2 list")
                return
            # Convert list to string and store
            point_repr = str(result)
            self.points2.insert(0, point_repr)
            self.update_comboboxes()
            # messagebox.showinfo("Success", "Point added to #2 list")
        else:
            messagebox.showwarning("Input Error", result)

    def delete_point1(self):
        """Delete selected point from #1 list"""
        selected_index = self.combo1.current()
        if selected_index != -1 and selected_index < len(self.points1):
            self.points1.pop(selected_index)
            self.update_comboboxes()
            # messagebox.showinfo("Success", "Point deleted from #1 list")
        else:
            messagebox.showwarning("Warning", "Please select a point to delete")

    def delete_point2(self):
        """Delete selected point from #2 list"""
        selected_index = self.combo2.current()
        if selected_index != -1 and selected_index < len(self.points2):
            self.points2.pop(selected_index)
            self.update_comboboxes()
            # messagebox.showinfo("Success", "Point deleted from #2 list")
        else:
            messagebox.showwarning("Warning", "Please select a point to delete")

    def save_points1(self):
        """Save #1 list to TXT file"""
        if not self.points1:
            messagebox.showwarning("Warning", "#1 list is empty, nothing to save")
            return
        file_path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            title="Save #1 Point List"
        )
        if file_path:
            try:
                with open(file_path, 'w') as f:
                    for point in self.points1:
                        f.write(point + '\n')
                # messagebox.showinfo("Success", f"#1 point list saved to: {os.path.basename(file_path)}")
            except Exception as e:
                messagebox.showerror("Error", f"Error saving file: {str(e)}")
    def save_points2(self):
        """Save #2 list to TXT file"""
        if not self.points2:
            messagebox.showwarning("Warning", "#2 list is empty, nothing to save")
            return
        file_path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            title="Save #2 Point List"
        )
        if file_path:
            try:
                with open(file_path, 'w') as f:
                    for point in self.points2:
                        f.write(point + '\n')
                # messagebox.showinfo("Success", f"#2 point list saved to: {os.path.basename(file_path)}")
            except Exception as e:
                messagebox.showerror("Error", f"Error saving file: {str(e)}")

    def load_points1(self):
        """Import from TXT file to #1 list"""
        file_path = filedialog.askopenfilename(
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            title="Select file to import to #1"
        )
        if file_path:
            try:
                with open(file_path, 'r') as f:
                    lines = f.readlines()
                # Validate and import points
                valid_points = []
                invalid_lines = []
                for i, line in enumerate(lines, 1):
                    line = line.strip()
                    if line:  # Skip empty lines
                        is_valid, result = self.validate_point(line)
                        if is_valid:
                            # Check for duplicates
                            if not self.is_duplicate(result, self.points1 + valid_points):
                                valid_points.append(str(result))
                        else:
                            invalid_lines.append(f"Row {i}: {line}")
                # Add valid points
                if valid_points:
                    # self.points1.extend(valid_points)
                    self.points1 = valid_points
                    self.update_comboboxes()
                    # messagebox.showinfo("Success", f"Imported from file {len(valid_points)}  points to #1 list")
                # Show invalid lines
                if invalid_lines:
                    messagebox.showwarning("Warning",
                                           f"The following lines have invalid format and were skipped:\n" +
                                           "\n".join(invalid_lines[:10]) +
                                           ("\n..." if len(invalid_lines) > 10 else ""))
            except Exception as e:
                messagebox.showerror("Error", f"Error reading file: {str(e)}")

    def load_points2(self):
        """Import from TXT file to #2 list"""
        file_path = filedialog.askopenfilename(
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            title="Select file to import to #2"
        )
        if file_path:
            try:
                with open(file_path, 'r') as f:
                    lines = f.readlines()
                # Validate and import points
                valid_points = []
                invalid_lines = []
                for i, line in enumerate(lines, 1):
                    line = line.strip()
                    if line:  # Skip empty lines
                        is_valid, result = self.validate_point(line)
                        if is_valid:
                            # Check for duplicates
                            if not self.is_duplicate(result, self.points2 + valid_points):
                                valid_points.append(str(result))
                        else:
                            invalid_lines.append(f"Row {i}: {line}")
                # Add valid points
                if valid_points:
                    self.points2 = valid_points
                    self.update_comboboxes()
                    # messagebox.showinfo("Success", f"Imported from file {len(valid_points)}  points to #2 list")
                # Show invalid lines
                if invalid_lines:
                    messagebox.showwarning("Warning",
                                           f"The following lines have invalid format and were skipped:\n" +
                                           "\n".join(invalid_lines[:10]) +
                                           ("\n..." if len(invalid_lines) > 10 else ""))
            except Exception as e:
                messagebox.showerror("Error", f"Error reading file: {str(e)}")



    def add_eef_command(self,robot_id):
        """Add point to #1 list"""
        command_str = self.eef_entry.get()
        if robot_id=='A':
            # Check if same point already exists
            if self.is_duplicate_command(command_str, self.command1):
                messagebox.showwarning("Duplicate Command", "This command already exists in #1 list")
                return
            else:
                self.command1.insert(0, command_str)
        elif robot_id=='B':
            # Check if same point already exists
            if self.is_duplicate_command(command_str, self.command2):
                messagebox.showwarning("Duplicate Command", "This command already exists in #1 list")
                return
            else:
                self.command2.insert(0, command_str)
        self.update_combo_eef()

    def delete_eef_command(self,robot_id):
        """Delete selected point from #2 list"""
        if robot_id=='A':
            selected_index = self.eef_combo1.current()
            if selected_index != -1 and selected_index < len(self.command1):
                self.command1.pop(selected_index)
                self.update_combo_eef()
            else:
                messagebox.showwarning("Warning", "Please select a communication command to delete")
        elif robot_id=='B':
            selected_index = self.eef_combo1.current()
            if selected_index != -1 and selected_index < len(self.command2):
                self.command2.pop(selected_index)
                self.update_combo_eef()
            else:
                messagebox.showwarning("Warning", "Please select a communication command to delete")


    def update_comboboxes(self):
        """Update both dropdown contents"""
        self.combo1['values'] = self.points1
        self.combo2['values'] = self.points2

        # If options exist, select the first one
        if self.points1:
            self.combo1.current(0)
        else:
            self.combo1.set('')
        if self.points2:
            self.combo2.current(0)
        else:
            self.combo2.set('')

    def update_combo_eef(self):
        # Update end-effector commands list
        self.eef_combo1['values'] = self.command1
        self.eef_combo2['values'] = self.command2
        # If options exist, select the first one
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
            """#1 Run button function"""
            selected = self.combo1.get()
            if selected:
                # Validate selected point is a valid 7-element list
                is_valid, point_list = self.validate_point(selected)
                if is_valid:
                    # messagebox.showinfo("1# Run", f"Running selected point: {point_list}")
                    robot.clear_set()
                    robot.set_joint_cmd_pose(arm='A', joints=point_list)
                    robot.send_cmd()
                else:
                    messagebox.showerror("Error", f"Selected point format is invalid: {selected}")
            else:
                messagebox.showwarning("Warning", "No points available to run")
        else:
            messagebox.showerror('error', 'Please connect the robot first')

    def run2(self):
        if self.connected:
            """#2 Run button function"""
            selected = self.combo2.get()
            if selected:
                # Validate selected point is a valid 7-element list
                is_valid, point_list = self.validate_point(selected)
                if is_valid:
                    # messagebox.showinfo("2# Run", f"Running selected point: {point_list}")
                    robot.clear_set()
                    robot.set_joint_cmd_pose(arm='B', joints=point_list)
                    robot.send_cmd()
                else:
                    messagebox.showerror("Error", f"Selected point format is invalid: {selected}")
            else:
                messagebox.showwarning("Warning", "No points available to run")
        else:
            messagebox.showerror('error', 'Please connect the robot first')

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
            title="Save Motion Parameters"
        )
        if file_path:
            try:
                with open(file_path, 'w') as f:
                    for point in self.params:
                        f.write(point + '\n')
                # messagebox.showinfo("Success", f"Motion parameters saved to: {os.path.basename(file_path)}")
            except Exception as e:
                messagebox.showerror("Error", f"Error saving file: {str(e)}")

    def load_param(self, robot_id):
        if robot_id == 'A':
            file_path = filedialog.askopenfilename(
                filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
                title="Select parameter file to import to #1"
            )
            if file_path:
                try:
                    with open(file_path, 'r') as f:
                        lines = f.readlines()
                    valid_points = []
                    for i, line in enumerate(lines, 1):
                        line = line.strip()
                        if line:  # Skip empty lines
                            valid_points.append(line)
                    print(f'valid_points:{valid_points}')

                    # Add valid points
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
                        # messagebox.showinfo("Success", f"Imported from file {len(valid_points)}  parameters to #1")

                except Exception as e:
                    messagebox.showerror("Error", f"Error reading file: {str(e)}")

        elif robot_id == 'B':
            file_path = filedialog.askopenfilename(
                filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
                title="Select parameter file to import to #2"
            )

            if file_path:
                try:
                    with open(file_path, 'r') as f:
                        lines = f.readlines()
                    valid_points = []
                    for i, line in enumerate(lines, 1):
                        line = line.strip()
                        if line:  # Skip empty lines
                            valid_points.append(line)
                    print(f'valid_points:{valid_points}')

                    # Add valid points
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
                        # messagebox.showinfo("Success", f"Imported from file {len(valid_points)}  parameters to #2")

                except Exception as e:
                    messagebox.showerror("Error", f"Error reading file: {str(e)}")

    def stop_command(self):
        if self.connected:
            robot.soft_stop('AB')
        else:
            messagebox.showerror('error', 'Please connect the robot first')

    def reset_robot(self, robot_id):
        if self.connected:
            robot.clear_set()
            robot.set_state(arm=robot_id, state=0)  # state=0: servo off
            robot.send_cmd()
        else:
            messagebox.showerror('error', 'Please connect the robot first')

    def pvt_mode(self, robot_id):
        if self.connected:
            robot.clear_set()
            robot.set_state(arm=robot_id, state=2)
            robot.send_cmd()
        else:
            messagebox.showerror('error', 'Please connect the robot first')

    def position_mode(self, robot_id):
        if self.connected:
            robot.clear_set()
            robot.set_state(arm=robot_id, state=1)
            robot.send_cmd()
        else:
            messagebox.showerror('error', 'Please connect the robot first')

    def cr_state(self, robot_id):
        if self.connected:
            robot.clear_set()
            robot.set_state(arm=robot_id, state=4)
            robot.send_cmd()
        else:
            messagebox.showerror('error', 'Please connect the robot first')

    def imded_j_mode(self, robot_id):
        if self.connected:
            robot.clear_set()
            robot.set_state(arm=robot_id, state=3)
            robot.set_impedance_type(arm=robot_id, type=1)
            robot.send_cmd()
        else:
            messagebox.showerror('error', 'Please connect the robot first')

    def imded_c_mode(self, robot_id):
        if self.connected:
            robot.clear_set()
            robot.set_state(arm=robot_id, state=3)
            robot.set_impedance_type(arm=robot_id, type=2)
            robot.send_cmd()
        else:
            messagebox.showerror('error', 'Please connect the robot first')

    def imded_f_mode(self, robot_id):
        if self.connected:
            robot.clear_set()
            robot.set_state(arm=robot_id, state=3)
            robot.set_impedance_type(arm=robot_id, type=3)
            robot.send_cmd()
            time.sleep(0.001)

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
            messagebox.showerror('error', 'Please connect the robot first')

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
            messagebox.showerror('error', 'Please set to Joint Impedance mode before selecting Joint Drag')

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
            messagebox.showerror('error', 'Please set to Cartesian Impedance mode before selecting Cartesian X Drag')

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
            messagebox.showerror('error', 'Please set to Cartesian Impedance mode before selecting Cartesian Y Drag')

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
            messagebox.showerror('error', 'Please set to Cartesian Impedance mode before selecting Cartesian Z Drag')

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
            messagebox.showerror('error', 'Please set to Cartesian Impedance mode before selecting Cartesian R Drag')

    def drag_exit(self, robot_id):
        robot.clear_set()
        robot.set_drag_space(arm=robot_id, dgType=0)
        robot.send_cmd()

    def thread_drag_save(self, robot_id):
        """Execute drag_save in a new thread"""
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
            # Check if stop is needed (optional)
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
            # Check if stop is needed (optional)
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

        # Use after() to execute file dialog and messagebox in GUI thread
        self.root.after(0, self._save_data_dialog, robot_id)

    def _save_data_dialog(self, robot_id):
        """Execute file save operation in GUI main thread"""
        file_path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt")],
            title="Save Drag Trajectory Data"
        )

        if file_path:
            try:
                robot.save_collected_data_to_path(file_path)
                time.sleep(2)
                messagebox.showinfo("Success", f"Drag trajectory data saved to: {os.path.basename(file_path)}，\nPlease exit drag mode.")
            except Exception as e:
                messagebox.showerror("Error", f"Error saving file: {str(e)}")

    def error_get(self, robot_id):
        if self.connected:
            errors = robot.get_servo_error_code(robot_id)
            print(f'servo error:{errors}')
            if errors:
                messagebox.showinfo(f'{robot_id} arm error', errors)
        else:
            messagebox.showerror('error', 'Please connect the robot first')

    def error_clear(self, robot_id):
        if self.connected:
            robot.clear_error(robot_id)
        else:
            messagebox.showerror('error', 'Please connect the robot first')

    def brake(self, robot_id):
        if self.connected:
            messagebox.showinfo('Notice', 'Please confirm servo parameter is set to 166 hybrid control mode')
            if robot_id == 'A':
                robot.set_param('int', 'BRAK0', 1)
            elif robot_id == 'B':
                robot.set_param('int', 'BRAK1', 1)
        else:
            messagebox.showerror('error', 'Please connect the robot first')

    def release_brake(self, robot_id):
        if self.connected:
            messagebox.showinfo('Notice', 'Please confirm servo parameter is set to 166 hybrid control mode')
            if robot_id == 'A':
                robot.set_param('int', 'BRAK0', 2)
            elif robot_id == 'B':
                robot.set_param('int', 'BRAK1', 2)
        else:
            messagebox.showerror('error', 'Please connect the robot first')

    def send_pvt(self, robot_id):
        if self.connected:
            file_path = filedialog.askopenfilename(
                title="Select Data File",
                filetypes=[("Text files", "*.txt"), ("fmv files", "*.fmv"), ("All files", "*.*")]
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
            messagebox.showerror('error', 'Please connect the robot first')

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
            messagebox.showerror('error', 'Please connect the robot first')

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
                messagebox.showerror("Error", "Tool kinematics parameters cannot be empty!")
            if len(kine_p) != 6:
                messagebox.showerror("Error", f"Tool kinematics must have 6 values, currently has {len(kine_p)}!")
            try:
                kine_p = [float(item) for item in kine_p]
            except ValueError:
                messagebox.showerror("Error", "Tool kinematics parameters must be valid numbers!")

            if not dyn_p:
                messagebox.showerror("Error", "Tool dynamics parameters cannot be empty!")
            if len(dyn_p) != 10:
                messagebox.showerror("Error", f"Tool dynamics must have 10 values, currently has {len(dyn_p)}!")
            try:
                dyn_p = [float(item) for item in dyn_p]
            except ValueError:
                messagebox.showerror("Error", "Tool dynamics parameters must be valid numbers!")
            robot.clear_set()
            robot.set_tool(arm=robot_id, kineParams=kine_p, dynamicParams=dyn_p)
            robot.send_cmd()

            tool_mat = kk1.xyzabc_to_mat4x4(xyzabc=kine_p)
            if robot_id == "A":
                kk1.set_tool_kine( tool_mat=tool_mat)
            elif robot_id == "B":
                kk2.set_tool_kine( tool_mat=tool_mat)

            '''save in txt and send it to controller'''
            if not self.tool_result:
                lines = ['0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n',
                         '0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n']
                # Write back to file
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
            messagebox.showerror('error', 'Please connect the robot first')

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
            messagebox.showerror('error', 'Please connect the robot first')

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
                messagebox.showerror("Error", "Joint K parameter cannot be empty!")
            if len(k) != 7:
                messagebox.showerror("Error", f"Joint K must have 7 values, currently has {len(k)}!")
            try:
                k = [float(item) for item in k]
            except ValueError:
                messagebox.showerror("Error", "Joint K parameter must be valid numbers!")

            if not d:
                messagebox.showerror("Error", "Joint D parameter cannot be empty!")
            if len(d) != 7:
                messagebox.showerror("Error", f"Joint D must have 7 values, currently has {len(d)}!")
            try:
                d = [float(item) for item in d]
            except ValueError:
                messagebox.showerror("Error", "Joint D parameter must be valid numbers!")
            robot.clear_set()
            robot.set_joint_kd_params(arm=robot_id, K=k, D=d)
            robot.send_cmd()
        else:
            messagebox.showerror('error', 'Please connect the robot first')

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
                messagebox.showerror("Error", "Cartesian K parameter cannot be empty!")
            if len(k) != 7:
                messagebox.showerror("Error", f"Cartesian K must have 7 values, currently has {len(k)}!")
            try:
                k = [float(item) for item in k]
            except ValueError:
                messagebox.showerror("Error", "Cartesian K parameter must be valid numbers!")

            if not d:
                messagebox.showerror("Error", "Cartesian D parameter cannot be empty!")
            if len(d) != 7:
                messagebox.showerror("Error", f"Cartesian D must have 7 values, currently has {len(d)}!")
            try:
                d = [float(item) for item in d]
            except ValueError:
                messagebox.showerror("Error", "Cartesian D parameter must be valid numbers!")
            robot.clear_set()
            robot.set_cart_kd_params(arm=robot_id, K=k, D=d, type=type)
            robot.send_cmd()
        else:
            messagebox.showerror('error', 'Please connect the robot first')

    def thread_collect_tool_data_no_load(self, robot_id):
        """Execute collect_tool_data_no_load in a new thread"""
        thread = threading.Thread(target=self.collect_tool_data_no_load, args=(robot_id))
        thread.daemon = True
        thread.start()

    def collect_tool_data_no_load(self, robot_id):
        if self.connected:
            folder_path = filedialog.askdirectory(
                title="Select folder to save identification data",
                mustexist=True
            )

            if folder_path:
                pvt_file = self.file_path_tool.get()
                robot.send_pvt_file(robot_id, pvt_file, 97)
                time.sleep(0.5)

                '''Set up data saving before robot motion'''
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

                '''Set PVT number to run'''
                robot.clear_set()
                robot.set_pvt_id(robot_id, 97)
                robot.send_cmd()

                time.sleep(60)  # Simulated trajectory run time

                '''Stop data collection'''
                robot.stop_collect_data()
                time.sleep(0.5)

                '''Save collected data'''
                save_pvt_path = os.path.join(folder_path, 'pvt.txt')
                robot.save_collected_data_to_path(save_pvt_path)

                time.sleep(1)

                '''Data preprocessing'''
                processed_data = []
                with open(save_pvt_path, 'r') as file:
                    lines = file.readlines()
                    # Remove header row
                lines = lines[1:]
                for i, line in enumerate(lines):
                    # Remove trailing newline and split by'$'
                    parts = line.strip().split('$')
                    # Extract numeric part of each field (remove non-numeric prefix)
                    numbers = []
                    for part in parts:
                        if part:  # Ignore empty strings
                            # Find numeric part after last space
                            num_str = part.split()[-1]
                            numbers.append(num_str)

                    # Remove first two columns (index 0 and 1), keep remaining
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
                messagebox.showinfo('success', f'Successfully saved {robot_id} arm no-load identification data')
        else:
            messagebox.showerror('error', 'Please connect the robot first')

    def thread_collect_tool_data_with_load(self, robot_id):
        """Execute collect_tool_data_with_load in a new thread"""
        thread = threading.Thread(target=self.collect_tool_data_with_load, args=(robot_id))
        thread.daemon = True
        thread.start()

    def collect_tool_data_with_load(self, robot_id):
        if self.connected:
            folder_path = filedialog.askdirectory(
                title="Select folder to save identification data",
                mustexist=True
            )

            if folder_path:
                pvt_file = self.file_path_tool.get()
                robot.send_pvt_file(robot_id, pvt_file, 97)
                time.sleep(0.5)

                '''Set up data saving before robot motion'''
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

                '''Set PVT number to run'''
                robot.clear_set()
                robot.set_pvt_id(robot_id, 97)
                robot.send_cmd()

                time.sleep(60)  # Simulated trajectory run time

                '''Stop data collection'''
                robot.stop_collect_data()
                time.sleep(0.5)

                '''Save collected data'''
                save_pvt_path = os.path.join(folder_path, 'pvt.txt')
                robot.save_collected_data_to_path(save_pvt_path)

                time.sleep(1)

                '''Data preprocessing'''
                processed_data = []
                with open(save_pvt_path, 'r') as file:
                    lines = file.readlines()
                    # Remove header row
                lines = lines[1:]
                for i, line in enumerate(lines):
                    # Remove trailing newline and split by'$'
                    parts = line.strip().split('$')
                    # Extract numeric part of each field (remove non-numeric prefix)
                    numbers = []
                    for part in parts:
                        if part:  # Ignore empty strings
                            # Find numeric part after last space
                            num_str = part.split()[-1]
                            numbers.append(num_str)

                    # Remove first two columns (index 0 and 1), keep remaining
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
                messagebox.showinfo('success', f'Successfully saved {robot_id} arm loaded identification data')
        else:
            messagebox.showerror('error', 'Please connect the robot first')

    def tool_dyn_identy(self):
        # Format data
        def format_vector6(vector):
            return ", ".join([f"{v:.6f}" for v in vector])

        print(f"ccs srs:{self.type_select_combobox_1.get()}")
        print(f"tool data:{self.save_tool_data_path}")
        if self.type_select_combobox_1.get() == 'CCS':
            tool_identy_tag, identy_results = kk1.identify_tool_dyn(robot_type=1, ipath=self.save_tool_data_path)
            print(f'tool_identy_tag:{tool_identy_tag}, identy_results:{identy_results}')
            if tool_identy_tag == False:
                messagebox.showerror('wrong', f'Tool Dynamics IdentificationErrorNotice:{identy_results}')
            if tool_identy_tag:
                self.entry_tool_dyn.set(format_vector6(identy_results))
                messagebox.showinfo('success', 'Tool dynamics identification complete')

        else:
            tool_identy_tag, identy_results = kk1.identify_tool_dyn(robot_type=2, ipath=self.save_tool_data_path)
            print(f'tool_identy_tag:{tool_identy_tag}, identy_results:{identy_results}')
            if tool_identy_tag == False:
                messagebox.showerror('wrong', f'Tool Dynamics IdentificationErrorNotice:{identy_results}')
            else:
                self.entry_tool_dyn.set(format_vector6(identy_results))
                messagebox.showinfo('success', 'Tool dynamics identification complete')

    def data_clear_preprocess(self, input, output):
        save_list = []
        with open(input, 'r') as file:
            lines = file.readlines()
        # Remove header row
        lines = lines[1:]
        for i, line in enumerate(lines):
            # Remove trailing newline and split by'$'
            parts = line.strip().split('$')
            # Extract numeric part of each field (remove non-numeric prefix)
            numbers = []
            for part in parts:
                if part:  # Ignore empty strings
                    # Find numeric part after last space
                    num_str = part.split()[-1]
                    numbers.append(num_str)

            # Remove first two columns (index 0 and 1), keep remaining
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
            messagebox.showerror('error', 'Please connect the robot first')

    def stop_collect_data_both(self):
        if self.connected:
            robot.clear_set()
            robot.stop_collect_data()
            robot.send_cmd()
        else:
            messagebox.showerror('error', 'Please connect the robot first')

    def save_collect_data_both(self):
        if self.connected:
            file_path = filedialog.asksaveasfilename(
                defaultextension=".txt",
                filetypes=[("Text files", "*.txt")],
                title="Save Dual-Arm Motion Data"
            )
            if file_path:
                try:
                    robot.save_collected_data_to_path(file_path)
                    # messagebox.showinfo("Success", f"Dual-arm motion data saved to: {os.path.basename(file_path)}")
                except Exception as e:
                    messagebox.showerror("Error", f"Error saving file: {str(e)}")
        else:
            messagebox.showerror('error', 'Please connect the robot first')

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
                messagebox.showerror("Error", f"Collection feature count cannot exceed 35!")
            if len(idx) != 35:
                messagebox.showerror("Error", f"Collection features must be 35, currently has {idx}!")
            if 1000000 < rows:
                rows = 1000000
                messagebox.showerror("Error", f"Maximum data collection is 1 million rows, set to 1000000")
            if rows < 1000:
                rows = 1000
                messagebox.showerror("Error", f"Minimum data collection is 1000 rows, set to 1000")
            robot.clear_set()
            robot.collect_data(targetNum=cols, targetID=idx, recordNum=rows)
            robot.send_cmd()
        else:
            messagebox.showerror('error', 'Please connect the robot first')

    def tool_trajectory(self):
        file_path = filedialog.askopenfilename(
            defaultextension=".fmv",
            filetypes=[("fmv files", "*.fmv"), ("All files", "*.*")],
            title="Select excitation trajectory file for tool identification"
        )
        if file_path:
            self.save_tool_data_path = file_path.split('IdenTraj')[0]
            self.file_path_tool.set(file_path)

    def select_50_file(self):
        file_path = filedialog.askopenfilename(
            defaultextension=".txt",
            filetypes=[("txt files", "*.txt"), ("All files", "*.*")],
            title="Select downsampled data file"
        )
        if file_path:
            self.file_path_50.set(file_path)
            # messagebox.showinfo("Success", f"Downsampled data file selected: {os.path.basename(file_path)}")

            if len(self.processed_data) != 0:
                self.processed_data = []

            with open(file_path, 'r') as file:
                lines = file.readlines()
            # Remove header row
            lines = lines[1:]
            for i, line in enumerate(lines):
                # Sample every 20 rows (1KHz -> 50Hz)
                if i % 20 != 0:
                    continue
                # Remove trailing newline and split by'$'
                parts = line.strip().split('$')
                # Extract numeric part of each field (remove non-numeric prefix)
                numbers = []
                for part in parts:
                    if part:  # Ignore empty strings
                        # Find numeric part after last space
                        num_str = part.split()[-1]
                        numbers.append(num_str)

                # Remove first two columns (index 0 and 1), keep remaining
                if len(numbers) >= 2:
                    numbers = numbers[2:]
                self.processed_data.append(numbers)

    def generate_50_file(self):
        """Save #2 list to TXT file"""
        if len(self.processed_data) == 0:
            messagebox.showerror("Error", "Resampled data is empty, nothing to save")

        file_path = filedialog.asksaveasfilename(
            defaultextension=".r50pth",
            filetypes=[("50pth files", "*.r50pth"), ("All files", "*.*")],
            title="Save Downsampled Data"
        )

        if file_path:
            try:
                # Write processed data to new file
                with open(file_path, 'w') as out_file:
                    for row in self.processed_data:
                        out_file.write(' '.join(row) + '\n')
                # messagebox.showinfo("Success", f"Downsampled data saved to: {os.path.basename(file_path)}")
            except Exception as e:
                messagebox.showerror("Error", f"Error saving file: {str(e)}")

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
                #     messagebox.showerror("error","Error getting parameters")

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
                #     messagebox.showerror("error","Error getting parameters")
        else:
            messagebox.showerror('error', 'Please connect the robot first')

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
                        messagebox.showerror("error","Failed to save parameters")

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
                        messagebox.showerror("error", "Failed to save parameters")

        else:
            messagebox.showerror('error', 'Please connect the robot first')

    def clear_motor_as_zero(self, robot_id):
        if self.connected:
            if robot_id == 'A':
                if self.result['states'][0]["cur_state"] != 0:
                    messagebox.showerror('error', 'Left arm must be in Reset state for motor encoder zeroing')
                else:
                    axis = int(self.motor_axis_select_combobox_1.get())
                    robot.set_param(type='int', paraName="RESETMOTENC0", value=axis)
            elif robot_id == 'B':
                if self.result['states'][1]["cur_state"] != 0:
                    messagebox.showerror('error', 'Right arm must be in Reset state for motor encoder zeroing')
                else:
                    axis = int(self.motor_axis_select_combobox_11.get())
                    robot.set_param(type='int', paraName="RESETMOTENC1", value=axis)
        else:
            messagebox.showerror('error', 'Please connect the robot first')

    def clear_motorE_as_zero(self, robot_id):
        if self.connected:
            if robot_id == 'A':
                if self.result['states'][0]["cur_state"] != 0:
                    messagebox.showerror('error', 'Left arm must be in Reset state for external encoder zeroing')
                else:
                    axis = int(self.motor_axis_select_combobox_1.get())
                    robot.set_param(type='int', paraName="RESETEXTENC0", value=axis)
            elif robot_id == 'B':
                if self.result['states'][1]["cur_state"] != 0:
                    messagebox.showerror('error', 'Right arm must be in Reset state for external encoder zeroing')
                else:
                    axis = int(self.motor_axis_select_combobox_11.get())
                    robot.set_param(type='int', paraName="RESETEXTENC1", value=axis)
        else:
            messagebox.showerror('error', 'Please connect the robot first')

    def clear_motor_error(self, robot_id):
        if self.connected:
            if robot_id == 'A':
                if self.result['states'][0]["cur_state"] != 0:
                    messagebox.showerror('error', 'Left arm must be in Reset state for encoder error clearing')
                else:
                    axis = int(self.motor_axis_select_combobox_1.get())
                    robot.set_param(type='int', paraName="CLEARMOTENC0", value=axis)
            elif robot_id == 'B':
                if self.result['states'][1]["cur_state"] != 0:
                    messagebox.showerror('error', 'Right arm must be in Reset state for encoder error clearing')
                else:
                    axis = int(self.motor_axis_select_combobox_11.get())
                    robot.set_param(type='int', paraName="CLEARMOTENC1", value=axis)
        else:
            messagebox.showerror('error', 'Please connect the robot first')

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

                # 1: CAN port; 2: com1; 3:com2
                if com_ == 'CAN':
                    com = 1
                elif com_ == 'COM1':
                    com = 2
                elif com_ == 'COM2':
                    com = 3
                # print(f'com:{com}')
                success, sdk_return = robot.set_485_data(robot_id, sample_data, len(sample_data), com)
                received_count, received_data = get_received_data()
                if received_count>0:
                    if len(received_data)>0:
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
            messagebox.showerror('error', 'Please connect the robot first')

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

                # 1: CAN port; 2: com1; 3:com2
                if com_ == 'CAN':
                    com = 1
                elif com_ == 'COM1':
                    com = 2
                elif com_ == 'COM2':
                    com = 3
                self.eef_thread=threading.Thread(target=read_data, args=(robot_id, com), daemon=True)
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
            messagebox.showerror('error', 'Please connect the robot first')


# Launch application
if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
