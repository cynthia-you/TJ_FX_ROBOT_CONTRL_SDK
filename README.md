## 最新APP版本1128
    底部增加机械臂错误码查询
    增加浮动基座参数计算功能
    稳定动力学参数和运动学参数设置保存到控制器功能,再次启动APP可显示上次保存数据
    末端CAN/485 可加多条协议指令
    
    

## 你好使用MARVIN_APP软件前，需要温馨提示您：

    软件同级config/文件夹内请放入独一无二的机型配置文件:ccs 6公斤的机型的计算配置文件为ccs_m6.MvKDCfg,ccs 3公斤的机型的计算配置文件为ccs_m3.MvKDCfg； srs机型为srs.MvKDCfg. 多个*.MvKDCfg会解析出错



## 安装APP:
    1. 我们测试并提供在WINDOWS 和UBUNTU24.04_X86下可执行的软件,如果与您的环境不一致,请下载源码后编译库,直接运行或者生成可执行APP运行
    2. MARVIN_APP基础环境:python3, pyinstaller
    3. 运行前请确认:
            3.1. MARVIN_APP基础环境:python3(无特定版本要求), pyinstaller
            3.2. 确保在 ./contrlSDK  和 ./kinematicsSDK 下的动态库SO: libMarvinSDK.so 和 libKine.so 是当前上位机环境下重新编译生成的(最好是重新编译),SDK源码地址：[https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/master/](https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/tree/master)
            3.3. 将3.2生成的 libMarvinSDK.so 和 libKine.so 替换到./MARVIN_APP_UBUNTU_WINDOWS/python/ 下
    4. 运行APP:
            4.1. 源码运行 UI_FX.py（或者是UI_FX*.py）
            4.2. 生成可执行文件后运行,以便于分发到其他无PY环境的电脑上: python  setup.py



## MARVIN APP使用说明文档
### 目录
        一 基础操作	2
        1.1 软件与界面说明	2
        1.2 急停	2
        1.3 连接机器人	3
        1.4 切换查看	4
        1.5 修改传感器偏置	5
        1.6 获取错误与清错	6
        1.6.1 伺服的错误	6
        1.6.2 机械臂错误	7
        1.7 工具设置	7
        1.8 检查机器人关节正方向	9
        1.9 检查电流和传感器方向	10
        1.9.1 检测左臂电流和传感器方向	11
        1.9.2 检测右臂电流和传感器方向	16
        二 位置模式运动控制	21
        三 阻抗控制模式	22
        3.1 关节阻抗	22
        3.2 笛卡尔阻抗	23
        3.3 笛卡尔阻抗-力控	24
        3.4 阻抗参数保存与导入	26
        3.5 关节阻抗——拖动	27
        3.5.1 关节拖动数据保存	27
        3.6 笛卡尔阻抗——拖动	28
        3.6.1 笛卡尔拖动数据保存	29
        四 其他功能	30
        4.1 周期运行	30
        4.2 末端485串口通信	32
        4.3 PVT运行模式	33
        4.4 工具动力学参数辨识	34
        4.5 撞机机械臂抱死调整	36
        4.5.1 协作释放模式	36
        4.5.2 松抱闸模式	37
        4.6 电机内外编码器清零和编码器清错	38
        4.7 浮动基座参数设置	38
        4.7.1 IMU两种默认安装方式	39
        4.7.2 安装和对应的配置方法	45











    

