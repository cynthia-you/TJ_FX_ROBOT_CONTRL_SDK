## Latest APP Version 1202
- Added robot arm error code lookup at the bottom
- Added floating base parameter calculation feature
- Stabilized dynamics and kinematics parameter settings — saved to the controller, so data from the last save is displayed on next APP launch
- End-effector CAN/485 now supports adding multiple protocol commands
- Support for clearing internal/external motor encoder offsets and fixing motor encoder errors


## Important Notice Before Using MARVIN_APP

Please place a **unique** robot model configuration file in the `config/` folder located in the same directory as the software:

- **CCS 6 kg model** has two versions:
  - Version **3.1** — calculation config file: `ccs_m6_31.MvKDCfg`
  - Version **4.0** — calculation config file: `ccs_m6_40.MvKDCfg`
  
  > The parameters differ between versions — please confirm your version before selecting parameters.

- **CCS 3 kg model** — calculation config file: `ccs_m3.MvKDCfg`
- **SRS model** — config file: `srs.MvKDCfg`

> ⚠️ Having multiple `*.MvKDCfg` files present will cause a parsing error.


## Installing the APP

1. We have tested and provide executables for **Windows** and **Ubuntu 24.04 x86**. If your environment differs, please download the source code, compile the libraries, and either run directly or build an executable APP.

2. **MARVIN_APP base requirements:** Python 3, PyInstaller

3. **Before running, please confirm:**

   3.1. Base environment: Python 3 (no specific version requirement), PyInstaller

   3.2. Ensure the shared libraries `libMarvinSDK.so` and `libKine.so` under `./contrlSDK` and `./kinematicsSDK` have been **recompiled** for your host machine environment (recompilation is strongly recommended).
   SDK source code: [https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/tree/master](https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/tree/master)

   3.3. Replace the `libMarvinSDK.so` and `libKine.so` generated in step 3.2 into `./MARVIN_APP_UBUNTU_WINDOWS/python/`

4. **Running the APP:**

   4.1. Run from source: `UI_FX.py` (or `UI_FX*.py`)

   4.2. Build an executable for distribution to machines without a Python environment:
   ```
   python setup.py
   ```


## MARVIN APP User Manual

### Table of Contents

```
1. Basic Operations                                          2
   1.1  Software and Interface Overview                      2
   1.2  Emergency Stop                                       2
   1.3  Connecting to the Robot                              3
   1.4  Switching Views                                      4
   1.5  Modifying Sensor Bias                                5
   1.6  Getting Errors and Clearing Errors                   6
        1.6.1  Servo Errors                                  6
        1.6.2  Robot Arm Errors                              7
   1.7  Tool Settings                                        7
   1.8  Checking Robot Joint Positive Direction              9
   1.9  Checking Current and Sensor Direction               10
        1.9.1  Detecting Left Arm Current and Sensor Direction  11
        1.9.2  Detecting Right Arm Current and Sensor Direction 16

2. Position Mode Motion Control                             21

3. Impedance Control Mode                                   22
   3.1  Joint Impedance                                     22
   3.2  Cartesian Impedance                                 23
   3.3  Cartesian Impedance – Force Control                 24
   3.4  Saving and Importing Impedance Parameters           26
   3.5  Joint Impedance – Drag Teaching                     27
        3.5.1  Saving Joint Drag Data                       27
   3.6  Cartesian Impedance – Drag Teaching                 28
        3.6.1  Saving Cartesian Drag Data                   29

4. Other Features                                          30
   4.1  Cyclic Execution                                    30
   4.2  End-Effector 485 Serial Communication              32
   4.3  PVT Run Mode                                        33
   4.4  Tool Dynamics Parameter Identification              34
   4.5  Collision Recovery / Arm Lock Adjustment           36
        4.5.1  Collaborative Release Mode                   36
        4.5.2  Brake Release Mode                           37
   4.6  Motor Internal/External Encoder Zero Reset and Error Clear  38
   4.7  Floating Base Parameter Settings                    38
        4.7.1  Two Default IMU Mounting Orientations        39
        4.7.2  Mounting and Corresponding Configuration Methods  45
```
