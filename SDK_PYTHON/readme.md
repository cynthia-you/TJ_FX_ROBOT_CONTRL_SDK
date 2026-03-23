# ATTENTION
    1.  请先熟练使用MARVIN_APP或MarvinPlatform软件。操作APP可以让您更加了解marvin机器人的操作使用逻辑，便于后期用代码开发。
    2.  DEMO_C++/ 和 DEMO_PYTHON/ 下为接口使用DEMO。每个demo顶部有该DEMO的案例说明和使用逻辑，请您一定先阅读，根据现场情况修改后运行。
        这些demo的使用逻辑和使用参数为研发测试使用开发的，仅供参考，并非实际生产代码。
            比如:
                a.速度百分比和加速度百分比为了安全我们都设置为百分之十：10，在您经过丰富的测试后可调到全速100。
                b.参数设置之间sleep 1秒或者500毫秒， 实际上参数设置之间小睡1毫秒即可。
                c.设置目标关节后，测试里小睡几秒等机械臂运行到位，而在生产时可以通过循环订阅机械臂当前位置判断是否走到指定点位或者通过订阅低速标志来判断。
                d.刚度系数和阻尼系数的设置也是参考值，不同的控制器版本可能值会有提升，详询技术人员。



## 一、SDK文档
    请阅读SDK文档，了解接口的功能和输入参数类型数值含义。

    SDK的主文档为master分支下的主文档：readme.md

    机器人控制SDK文档：
        c++_doc_contrl.md
        python_doc_contrl.md

    机器人计算SDK文档：
        c++_doc_kine.md
        python_doc_kine.md

## 二、SDK库文件夹下文件说明
    SDK_PYTHON文件下文件为：
    TJ_FX_ROBOT_CONTRL_SDK-master
    |————DEMO_PYTHON  #SDK在python下的使用案例
    |————SDK_PYTHON
            |————fx_kine.py #计算接口
            |————fx_robot.py #控制接口
            |————libKine.dll #windows下计算库动态库
            |————libKine.so #linux下计算库动态库
            |————libMarvinSDK.dll #windows下控制库动态库
            |————libMarvinSDK.so #linux下控制库动态库

    注意：请检查SDK_PYTHON下动态库是否为最新编译

## 三、 SDK库编译
    SDK_PYTHON为天机双臂机器人和人形机器人基于python开发的SDK，其分为:
        控制SDK：SDK_PYTHON/fx_robot.py 
        和运动学计算的SDK:SDK_PYTHON/fx_kine.py

### 3.1使用自动化编译脚本：
        mster下marvinSDK_windows.bat运行可自动编译C++和python调用的dll文件
        mster下marvinSDK_ubuntu.sh运行可自动编译C++和python调用的so文件

### 3.2.1 编译so动态库:
    linux设备编译:
        控制SDK(contrlSDK)，以下方法均可编译: 
			1. g++ *.cpp  -Wall -w -O2 -fPIC -shared -o libMarvinSDK.so -lpthread -lrt -DCMPL_LIN
			2./contrlSDK/makefile 生成libMarvinSDK.so
        运动学SDK(kinematicsSDK)，以下方法均可编译: 
			1. g++ *.cpp  -Wall -w -O2 -fPIC -shared -o libKine.so -lpthread -lrt 
			2./kinematicsSDK/makefile 生成libKine.so
	编译的libKine.so 和 libMarvinSDK.so 供编译机器下的下C++和python使用

### 3.2.2 编译c++调用的dll动态库:
    1)windows下使用MinGW编译dll动态库:
			控制SDK(contrlSDK): g++ *.cpp -Wall -w -O2 -shared -o libMarvinSDK.dll -lws2_32 -lwinmm -DCMPL_WIN
            运动学SDK(kinematicsSDK): g++ *.cpp *.c -Wall -w -O2 -fPIC -shared -o libKine.dll
    编译的libKine.dll 和 libMarvinSDK.dll 供WINDOWS下C++使用

			
### 3.2.3 编译python调用的dll动态库
    1)linux下编译dll动态库:
        控制SDK(contrlSDK):  x86_64-w64-mingw32-g++ *.cpp -Wall -O2 -shared -o libMarvinSDK.dll -DBUILDING_DLL -DCMPL_WIN -static -static-libgcc -static-libstdc++ -lws2_32 -lpthread -lwinmm
        运动学SDK(kinematicsSDK): g++ *.cpp *.c -Wall -w -O2 -fPIC -shared -o libKine.dll 

	2）windows下使用MinGW编译dll动态库：
			控制SDK（contrlSDK）：g++ *.cpp -Wall -w -O2 -shared -o libMarvinSDK.dll -DBUILDING_DLL -D_WIN32 -DCMPL_WIN -fPIC -static -static-libgcc -static-libstdc++ -lws2_32 -lwinmm
			运动学SDK(kinematicsSDK)：g++ *.cpp -Wall -w -O2 -shared -o libKine.dll -DBUILDING_DLL -D_WIN32 -fPIC -static -static-libgcc -static-libstdc++ -lws2_32 -lwinmm
    编译的libKine.dll 和 libMarvinSDK.dll 供WINDOWS下python使用
    