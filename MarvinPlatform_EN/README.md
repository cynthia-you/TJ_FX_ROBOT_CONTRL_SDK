## APP has been refactored into MarvinPlatform_EN

## Before using the software, we would like to kindly remind you:

Please place a unique machine configuration file in the config/ folder at the same level as the software:
ccs 6kg models have two versions: 3.1 (configuration file for calculation: ccs_m6_31.MvKDCfg), 4.0 (configuration file for calculation: ccs_m6_40.MvKDCfg). The parameters of the two versions are different, please confirm the version before selecting the parameters.
The calculation configuration file for ccs 3kg models is ccs_m3.MvKDCfg;
The srs model uses srs.MvKDCfg. Multiple *.MvKDCfg files will cause parsing errors.

## Installing the APP:
1. We have tested and provided software executable under WINDOWS and UBUNTU24.04_X86. If your environment is different, please download the source code, compile the libraries, and run directly or generate an executable APP to run.
2. MarvinPlatform basic environment: python3, pyinstaller, pillow
3. Before running, please ensure:
3.1. MarvinPlatform basic environment: python3 (no specific version required), pyinstaller, pillow
3.2. Ensure that the dynamic libraries SO: libMarvinSDK.so and libKine.so in ./contrlSDK and ./kinematicsSDK are recompiled in the current upper computer environment (recompiling is recommended). SDK source code address: [https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/master/](https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/tree/master)
3.3. Replace the libMarvinSDK.so and libKine.so generated in 3.2 into ./MarvinPlatform_EN/python/
4. Running the APP:
4.1. Run ui.py from the source code
4.2. After generating an executable file, run it for easier distribution to other computers without a Python environment: python setup.py

## MARVIN APP User Manual
