@echo off
echo 开始编译所有程序...

:: 编译运动学程序
g++ my_dd.cpp -L. -lKine -o kine.exe
if %errorlevel% == 0 (
    echo ✓ kine.exe 编译成功
) else (
    echo ✗ kine.exe 编译失败
)

:: 编译控制的程序
::检查连接
g++ link_check_demo.cpp -L. -lMarvinSDK -o check_link.exe
if %errorlevel% == 0 (
    echo ✓ check_link.exe 编译成功
) else (
    echo ✗ check_link.exe 编译失败
)


::扭矩：关节阻抗
g++ torque_joint_impedance_demo.cpp -L. -lMarvinSDK -o torque_joint.exe
if %errorlevel% == 0 (
    echo ✓ torque_joint.exe 编译成功
) else (
    echo ✗ torque_joint.exe 编译失败
)


::扭矩：笛卡尔阻抗
g++ torque_cart_impedance_demo.cpp -L. -lMarvinSDK -o torque_cart.exe
if %errorlevel% == 0 (
    echo ✓ torque_cart.exe 编译成功
) else (
    echo ✗ torque_cart.exe 编译失败
)

::扭矩：力控
g++ torque_force_impedance_demo.cpp -L. -lMarvinSDK -o torque_force.exe
if %errorlevel% == 0 (
    echo ✓ torque_force.exe 编译成功
) else (
    echo ✗ torque_force.exe 编译失败
)

::扭矩：关节拖动
g++ drag_demo.cpp -L. -lMarvinSDK -o drag.exe
if %errorlevel% == 0 (
    echo ✓ drag.exe 编译成功
) else (
    echo ✗ drag.exe 编译失败
)


::位置跟随
g++ position_demo.cpp -L. -lMarvinSDK -o position_demo.exe
if %errorlevel% == 0 (
    echo ✓ position_demo.exe 编译成功
) else (
    echo ✗ position_demo.exe 编译失败
)

::500HZ轨迹执行
g++ pvt_demo.cpp -L. -lMarvinSDK -o pvt_demo.exe
if %errorlevel% == 0 (
    echo ✓ pvt_demo.exe 编译成功
) else (
    echo ✗ pvt_demo.exe 编译失败
)


::末端485
g++ eef_485_demo.cpp -L. -lMarvinSDK -o eef_485.exe
if %errorlevel% == 0 (
    echo ✓ eef_485.exe 编译成功
) else (
    echo ✗ eef_485.exe 编译失败
)


::末端CAN
g++ eef_can_demo.cpp -L. -lMarvinSDK -o eef_can.exe
if %errorlevel% == 0 (
    echo ✓ eef_can.exe 编译成功
) else (
    echo ✗ eef_can.exe 编译失败
)


::获取设置参数
g++ get_set_param_demo.cpp -L. -lMarvinSDK -o get_set_param.exe
if %errorlevel% == 0 (
    echo ✓ get_set_param.exe 编译成功
) else (
    echo ✗ get_set_param.exe 编译失败
)


::松抱闸案例
g++ apply-brake_release-brake_demo.cpp -L. -lMarvinSDK -o apply-brake_release-brake.exe
if %errorlevel% == 0 (
    echo ✓ apply-brake_release-brake.exe 编译成功
) else (
    echo ✗ apply-brake_release-brake.exe 编译失败
)

::协作释放
g++ collaborative_release_demo.cpp -L. -lMarvinSDK -o collaborative_release_demo.exe
if %errorlevel% == 0 (
    echo ✓ collaborative_release_demo.exe 编译成功
) else (
    echo ✗ collaborative_release_demo.exe 编译失败
)

::关节拖动保存数据
g++ drag_JointImdedance_save_data_arm_A.cpp -L. -lMarvinSDK -o drag_JointImdedance_save_data_arm_A.exe
if %errorlevel% == 0 (
    echo ✓ drag_JointImdedance_save_data_arm_A.exe 编译成功
) else (
    echo ✗ drag_JointImdedance_save_data_arm_A.exe 编译失败
)



::笛卡尔拖动保存数据
g++ drag_CartImdedance_save_data_arm_A.cpp -L. -lMarvinSDK -o drag_CartImdedance_save_data_arm_A.exe
if %errorlevel% == 0 (
    echo ✓ drag_CartImdedance_save_data_arm_A.exe 编译成功
) else (
    echo ✗ drag_CartImdedance_save_data_arm_A.exe 编译失败
)



echo 编译完成！
pause