# 天机-孚晞 MARVIN机器人计算SDK
## 机器人型号： MARVIN人形双臂, 单臂
## 版本： 1004
## 支持平台： LINUX 及 WINDOWS
## LINUX支持： ubuntu18.04 - ubuntu24.04
## 更新日期：2025-09



## 一、接口介绍
### 接口快速全览见[kinematicsSDK/FxRobot.h]

### 注意
    一定要确认RobotSerial是左臂0 还是右臂1
    在DEMO中仅示例了单臂（左臂）的计算
    如果人形，则左右臂都要计算，两个手臂需要独立初始化导入运动学参数，初始化运动学参数。
    使用前，请一定确认机型，导入正确的配置文件，文件导错，计算会错误啊啊啊,甚至看起来运行正常，但是值错误！！！

### 逆解结构体数据介绍
    该结构体用于逆解解算中输入输出数据的解算。
    typedef struct
    {
        ///////////////输入//////////////////////
    	Matrix4					m_Input_IK_TargetTCP; //末端位置姿态4x4列表，可通过正解接口获取，或者指定末端的位置和旋转
    	Vect7					m_Input_IK_RefJoint; //参考输入角度，约束构想接近参考解读，防止解出来的构型跳变。该构型的肩、肘、腕组成初始臂角平面，以肩到腕方向为Z向量，参考角第四关节不能为零。
    	FX_INT32L				m_Input_IK_ZSPType; //零空间约束类型（0：使求解结果与参考关节角的欧式距离最小适用于一般冗余优化；1：与参考臂角平面最近，需要额外提供平面参数zsp_para）
    	FX_DOUBLE				m_Input_IK_ZSPPara[6]; //若选择零空间约束类型zsp_type为1，则需额外输入参考角平面参数，目前仅支持平移方向的参数约束，即[x,y,z,a,b,c]=[0,0,0,0,0,0],可选择x,y,z其中一个方向调整
    	FX_DOUBLE				m_Input_ZSP_Angle; //末端位姿不变的情况下，零空间臂角相对于参考平面的旋转角度（单位：度）,可正向调节也可逆向调节. 在ref_joints为初始臂角平面情况下，使用右手法则，绕Z向量正向旋转为臂角增加方向，绕Z向量负向旋转为臂角减少方向
    	FX_DOUBLE               m_DGR1; //(仅在IK_NSP接口中设置起效)判断第二关节发生奇异的角度范围，数值范围为0.05-10(单位：度)，不设置情况下默认0.05度
    	FX_DOUBLE               m_DGR2; //(仅在IK_NSP接口中设置起效)判断第六关节发生奇异的角度范围，数值范围为0.05-10(单位：度)，不设置情况下默认0.05度
    	FX_DOUBLE               m_DGR3; //预留接口
    	///////////////输出//////////////////////
    	Vect7	                m_Output_RetJoint; //逆运动学解出的关节角度（单位：度）
    	Matrix8                 m_OutPut_AllJoint; //逆运动学的全部解（每一行代表一组解, 分别存放1 - 7关节的角度值）（单位：度）
    	FX_INT32L               m_OutPut_Result_Num; //逆运动学全部解的组数（七自由度CCS构型最多四组解，SRS最多八组解）
    	FX_BOOL                 m_Output_IsOutRange; //当前位姿是否超出位置可达空间（False：未超出；True：超出）,如果超出可达空间,则需调整参考角度(参考角度和目标点位相差过大).
    	FX_BOOL                 m_Output_IsDeg[7]; //各关节是否发生奇异（False：未奇异；True：奇异）
    	FX_BOOL                 m_Output_JntExdTags[7]; //各关节是否超出位置正负限制（False：未超出；True：超出）
    	FX_DOUBLE               m_Output_JntExdABS; //所有关节中超出限位的最大角度的绝对值，比如解出一组关节角度，7关节超限，的值为-95，已知软限位为-90度，m_Output_JntExdABS=5.
    	FX_BOOL                 m_Output_IsJntExd; //是否有关节超出位置正负限制（False：未超出；True：超出）
    	Vect7	                m_Output_RunLmtP; //各个关节运行的正限位
    	Vect7	                m_Output_RunLmtN; //各个关节运行的负限位   
    }FX_InvKineSolvePara;

    

###    1. 导入运动学相关参数
FX_BOOL  LOADMvCfg(FX_CHAR* path, FX_INT32L TYPE[2], FX_DOUBLE GRV[2][3], FX_DOUBLE DH[2][8][4], FX_DOUBLE PNVA[2][7][4], FX_DOUBLE BD[2][4][3],FX_DOUBLE Mass[2][7], FX_DOUBLE MCP[2][7][3], FX_DOUBLE I[2][7][6])

    • Eg.:LOADMvCfg((char *)"xxx.MvKDCfg", TYPE, GRV, DH, PNVA, BD, Mass, MCP, I)
    • xxx.MvKDCfg文件为本地机械臂配置文件srs.MvKDCfg/ccs.MvKDCfg(请确认机型和DH参数是否对应), 可相对路径.
    • srs.MvKDCfg/ccs.MvKDCfg 文件中包含与运动学计算相关的双臂参数，进行计算之前需要导入机械臂配置相关文件
    • TYPE=1007，Pilot-SRS机型（双臂为MARVIN）；TYPE=1017，Pilot-CCS机型双臂为MARVIN）！
    • GRV参数为双臂重力方向，如[0.000,9.810,0.000];
    • DH参数为双臂MDH参数，包含各关节MDH参数及法兰MDH参数；
    • PNVA参数为双臂各关节正负限制位置以及所允许的正负最大加速度及加加速度；
    • BD参数为Pilot-CCS机型特定参数，为六七关节自干涉允许范围的拟合二阶多项式曲线，其他机型中该参数均为0；
    • Mass参数为双臂各关节质量；MCP参数为双臂各关节质心；I参数为双臂各关节惯量
    • MDH参数单位为度和毫米（mm），速度加速度单位为度/秒，关节质量、关节质心、关节惯量单位均为国际标准单位

###    2. 初始化运动学相关参数
FX_BOOL  FX_Robot_Init_Type(FX_INT32L RobotSerial, FX_INT32L RobotType)

FX_BOOL  FX_Robot_Init_Kine(FX_INT32L RobotSerial, FX_DOUBLE DH[8][4])

FX_BOOL  FX_Robot_Init_Lmt(FX_INT32L RobotSerial, FX_DOUBLE PNVA[7][4], FX_DOUBLE J67[4][3])

    • 运动学相关计算前，需要按照该顺序调用初始化函数，将配置中导入的参数进行初始化
    • FX_INT32L RobotSerial：0，左臂；1，右臂

###    3. 工具设置
FX_BOOL  FX_Robot_Tool_Set(FX_INT32L RobotSerial, Matrix4 tool)

    • 若末端带有负载，对各关节参数初始化后，需要对工具进行设置
    输入：
        1. FX_INT32L RobotSerial：0，左臂；1，右臂
        2. Matrix4 tool：工具相对于末端法兰的齐次变换矩阵
    输出：
        成功：True/1; 失败：False/0
    • 设置工具后，正解结果为工具TCP相对于基座的平移和旋转。

FX_BOOL  FX_Robot_Tool_Rmv(FX_INT32L RobotSerial)

    • 移除工具
    输入：
        1. FX_INT32L RobotSerial：0，左臂；1，右臂
    输出：
        成功：True/1; 失败：False/0



###    4. 计算正运动学
FX_BOOL  FX_Robot_Kine_FK(FX_INT32L RobotSerial, FX_DOUBLE joints[7], Matrix4 pgos)

    • 输入七关节角度及RobotSerial（参数含义参考初始化参数部分），输出为4*4的法兰末端位姿矩阵
    输入：
        1. FX_INT32L RobotSerial：0，左臂；1，右臂
        2. 需要得到末端齐次变换矩阵的输入关节角度，单位：度
        3. 初始化输出的末端其次变换矩阵
    输出：
        成功：True/1; 失败：False/0

###   5.计算正运动学和零空间(臂角平面)参数
FX_BOOL  FX_Robot_Kine_FK_NSP(FX_INT32L RobotSerial, FX_DOUBLE joints[7], Matrix4 pgos, Matrix3 nspg);

  • 输入七关节角度及RobotSerial（参数含义参考初始化参数部分），输出为4*4的法兰末端位姿矩阵,并得到基于该角度下的零空间参数
    输入：
        1. FX_INT32L RobotSerial：0，左臂；1，右臂
        2. FX_DOUBLE joints[7]:需要得到末端齐次变换矩阵的输入关节角度，单位：度
        3. Matrix4 pgos:初始化输出的末端其次变换矩阵
        4. Matrix3 nspg:初始化零空间参数矩阵
    输出：
        成功：True/1; 失败：False/0
        
 • 特别提示:零空间参数矩阵 nspg(3,3), 其中第一列可以作为逆解结构体里面m_Input_IK_ZSPPara的x y z的输入值。

    

###    6. 计算逆运动学
FX_BOOL  FX_Robot_Kine_IK(FX_INT32L RobotSerial, FX_InvKineSolvePara *solve_para)

    • 输入RobotSerial（参数含义参考初始化参数部分）及solve_para结构体，输出包含在solve_para中
    输入：
        1. FX_INT32L RobotSerial：0，左臂；1，右臂
        2. solve_para结构体中，包含以下内容：
            • 输入项
                • Matrix4 m_Input_IK_TargetTCP ：4*4的目标点末端的位姿矩阵
                • Vect7   m_Input_IK_RefJoint  ：逆运动学的各关节参考角（单位：度）
            • 输出项
                • Vect7   m_Output_RetJoint      ：逆运动学解出的关节角度（选解策略为与参考关节角最近）（单位：度）
                • Matrix8 m_OutPut_AllJoint      ：逆运动学的全部解（每一行代表一组解,分别存放1-7关节的角度值）（单位：度）
                • FX_INT32L m_OutPut_Result_Num  ：逆运动学全部解的组数（七自由度CCS构型最多四组解，SRS最多八组解）
                • FX_BOOL m_Output_IsOutRange    ：用于判断当前位姿是否超出位置可达空间（0：未超出；1：超出）,如果超出可达空间,则需调整参考角度(参考角度和目标点位相差过大).
                • FX_BOOL m_Output_IsDeg[7]      ：用于判断各关节是否发生奇异（0：未奇异；1：奇异）
                • FX_DOUBLE m_Output_JntExdABS   : 各关节超限绝对值总和(FX_Robot_PLN_MOVL_KeepJ使用)
                • FX_BOOL m_Output_IsJntExd      : 用于判断是否有关节超出位置正负限制（0：未超出；1：超出）
                • FX_BOOL m_Output_JntExdTags[7] ：用于判断各关节是否超出位置正负限制（0：未超出；1：超出）
    输出：
        成功：True/1; 失败：False/0
        失败情况: 
                1. 输入矩阵超出机器人可达关节空间
                2. 第四关节为0, 奇异

    • 特别提示:
            结构体以下输出项的TAG仅绑定对m_Output_RetJoint输出的关节描述
                • FX_BOOL m_Output_IsOutRange    ：用于判断当前位姿是否超出位置可达空间（0：未超出；1：超出）,如果超出可达空间,则需调整参考角度(参考角度和目标点位相差过大).
                • FX_BOOL m_Output_IsDeg[7]      ：用于判断各关节是否发生奇异（0：未奇异；1：奇异）
                • FX_DOUBLE m_Output_JntExdABS   : 各关节超限绝对值总和(FX_Robot_PLN_MOVL_KeepJ使用)
                • FX_BOOL m_Output_IsJntExd      : 用于判断是否有关节超出位置正负限制（0：未超出；1：超出）
                • FX_BOOL m_Output_JntExdTags[7] ：用于判断各关节是否超出位置正负限制（0：未超出；1：超出）

            如果选用用多组解m_OutPut_AllJoint. 请自行对选的解做判断,符合以下三个条件才能控制机械臂正常驱动:
                1. 第二关节的角度不在正负0.05度范围内(在此范围将奇异)
                2. 对输出的各个关节做软限位判定:
                    调用接口LOADMvCfg((char*)"ccs_m6.MvKDCfg", TYPE, GRV, DH, PNVA, BD, Mass, MCP, I)后,
                    PNVA矩阵里的前两列对应各个关节的正负限位
                    选取的解的每个关节都满足在限位置内
                3. 如果条件1和2都满足,还要做六七关节干涉判定:
                    判定方法:
                        调用接口LOADMvCfg((char*)"ccs_m6.MvKDCfg", TYPE, GRV, DH, PNVA, BD, Mass, MCP, I)后,
                        BD矩阵里依次为++, -+,  --, +- 四个象限的干涉参数
                        以CCS为例:
                            如果选的解的六七关节都为正, 则选用在++象限里的参数:[0.018004, -2.3205, 108.0],三个参数分别视为a0,a1,a2, 
                            第6关节的值为j6,此时使用公式j7=a0^2)*j6+ a1*j6+a2  将得到第7个关节的最大限制位置
                            如果选取的解里面的第7关节小于j7, 则不发生干涉, 本组解可被驱动到达.



###    7. 计算末端位姿不变、改变零空间（臂角方向）的逆运动学
FX_BOOL  FX_Robot_Kine_IK_NSP(FX_INT32L RobotSerial, FX_InvKineSolvePara *solve_para)

    • 输入RobotSerial（参数含义参考初始化参数部分）及solve_para结构体，输出包含在solve_para中
    输入：
        1. FX_INT32L RobotSerial：0，左臂；1，右臂
        2. solve_para结构体：
            Matrix4    m_Input_IK_TargetTCP：目标末端位姿矩阵：4×4齐次变换矩阵
            Vect7      m_Input_IK_RefJoint：用于零空间优化的初始参考角度，当存在多解时，系统会选择最接近该参考角度的解（单位：度）。该构型的肩、肘、腕组成初始臂角平面，以肩到腕方向为Z向量。
            FX_INT32L	 m_Input_IK_ZSPType：零空间约束类型（0：使求解结果与参考关节角的欧式距离最小适用于一般冗余优化；1：与参考臂角平面最近，需要额外提供平面参数m_Input_IK_ZSPPara[6]）
            *FX_DOUBLE	 m_Input_IK_ZSPPara[6]：若选择零空间约束类型为1，则需额外输入参考角平面参数，目前仅支持平移方向的参数约束，即[x,y,z,a,b,c]=[0,0,0,0,0,0],可选择x,y,z其中一个方向调整
            FX_DOUBLE	 m_Input_ZSP_Angle：末端位姿不变的情况下，零空间臂角相对于参考平面的旋转角度（单位：度）。在m_Input_IK_RefJoint为初始臂角平面情况下，使用右手法则，绕Z向量正向旋转为臂角增加方向，绕Z向量负向旋转为臂角减少方向
            *FX_DOUBLE  m_DGR1,m_DGR2：选择123关节和567关节发生奇异允许的角度范围，如无额外要求无需输入，默认值为0.05（单位：度）

    输出：
        成功：True/1; 失败：False/0


###    8. 计算雅可比矩阵
FX_BOOL  FX_Robot_Kine_Jacb(FX_INT32L RobotSerial, FX_DOUBLE joints[7], FX_Jacobi* jcb)

    • 输入关节角度及RobotSerial（参数含义参考初始化参数部分），输出为6*7的雅可比矩阵
    输入：
        1. FX_INT32L RobotSerial：0，左臂；1，右臂
        2. joints ，转雅可比矩阵的输入关节角，单位：度。
        3. jcb 初始化为0的7的雅可比矩阵
    输出：
        成功：True/1; 失败：False/0

###    9. 直线规划（MOVL）
FX_BOOL  FX_Robot_PLN_MOVL(FX_INT32L RobotSerial, Vect6 Start_XYZABC, Vect6 End_XYZABC, Vect7 Ref_Joints, FX_DOUBLE Vel, FX_DOUBLE ACC, FX_INT8* OutPutPath)

    • 输入RobotSerial（参数含义参考初始化参数部分）、起始点位姿、结束点位姿、当前位置参考关节角度、直线规划速度及直线规划加速度，输出为包含该段规划的关节点位文件
    输入：
        1. FX_INT32L RobotSerial：0，左臂；1，右臂
        2. Start_XYZABC起始点末端的位姿信息，六维信息，目标末端点相对于基座的平移和欧拉旋转，可用正解FX_Robot_Kine_FK接口得到目标末端位姿矩阵，再用FX_Matrix42XYZABCDEG求得XYZABC。（单位：平移为毫米， 旋转为度）
        3. End_XYZABC终止点末端的位姿信息，六维信息，目标末端点相对于基座的平移和欧拉旋转，可用正解FX_Robot_Kine_FK接口得到目标末端位姿矩阵，再用FX_Matrix42XYZABCDEG求得XYZABC。（单位：平移为毫米， 旋转为度）
        4. Ref_Joints约束了规划的起始关节点信息。单位：度。 
        5. Vel 约束了输出的规划文件的速度。单位毫米/秒， 最小为0.1mm/s， 最大为1000 mm/s
        6. ACC 约束了输出的规划文件的加速度。单位毫米/平方秒， 最小为0.1mm/s^2， 最大为10000 mm/s^2
        7. OutPutPath 规划文件的保存路径
    输出：
        成功：True/1; 失败：False/0


    • 输出点位频率为500Hz
    • 函数规划成功会保存规划的PVT文件，无文件保存则规划失败；或者读函数返回。
    • 特别提示:直线规划前,需要将起始关节位置调正解接口,将数据更新到起始关节。
    • FX_Robot_PLN_MOVL的特点在于根据提供的起始目标笛卡尔位姿和终止目标笛卡尔位姿规划一段直线路径点，该接口不约束到达终点时的机器人构型。
    

###    10.直线规划，约束机器人气势和结束的各个关节角度（MOVLJ）
FX_BOOL  FX_Robot_PLN_MOVL_KeepJ(FX_INT32L RobotSerial, Vect7 startjoints, Vect7 stopjoints, FX_DOUBLE vel, FX_DOUBLE ACC， FX_CHAR* OutPutPath);

    • 输入RobotSerial（参数含义参考初始化参数部分）、起始点位姿、结束点位姿、当前位置参考关节角度、直线规划速度及直线规划加速度，输出为包含该段规划的关节点位文件
    输入：
        1. FX_INT32L RobotSerial：0，左臂；1，右臂
        2. startjoints:起始点各个关节位置（单位：角度）
        3. stopjointss:终点各个关节位置（单位：角度）
        4. vel 约束了输出的规划文件的速度。单位毫米/秒， 最小为0.1mm/s， 最大为1000 mm/s
        5. ACC 约束了输出的规划文件的加速度。单位毫米/平方秒， 最小为0.1mm/s^2， 最大为10000 mm/s^2
        6. OutPutPath：规划文件的保存路径
    输出：
        成功：True/1; 失败：False/0
        
    • 输出点位频率为500Hz
    • 函数规划成功会保存规划的PVT文件，无文件保存则规划失败；或者读函数返回。
    • 特别提示:直线规划前,需要将起始关节位置调正解接口,将数据更新到起始关节。
    • 该接口是不同于FX_Robot_PLN_MOVL的规划接口，FX_Robot_PLN_MOVL_KeepJ根据起始关节和结束关节规划一条直线路径。
    
    

###    11. 工具动力学参数辨识
FX_INT32  FX_Robot_Iden_LoadDyn(FX_INT32 Type,FX_CHAR* path,FX_DOUBLE* mass, Vect3 mr, Vect6 I);

    • 输入当前机型Type(获取机型Type参考导入运动学参数部分)及文件存放路径，输出为工具相对于法兰端的质量、质心及惯量
    • Type:  1:CCS机型，2:SRS机型
    • path:工具辨识轨迹数据, 指定到文件目录LoadData即可（LoadData文件夹内包含参数辨识的所需文件），只需要输入该文件夹存放的绝对/相对路径，如："/xxxx/xxxx/LoadData"；
    • 函数返回有具体的辨识结果说明：
        typedef enum {
        LOAD_IDEN_NoErr = 0, // 成功
        LOAD_IDEN_CalErr = 1, //  计算错误，需重新采集数据计算
        LOAD_IDEN_OpenSmpDateFieErr = 2, // 打开采集数据文件错误，须检查采样文件
        LOAD_IDEN_OpenCfgFileErr = 3, //  配置文件被修改
        LOAD_IDEN_DataSmpErr = 4 // 采集时间不够，缺少有效数据
        }LoadIdenErrCode;
            
    • 其中 NoLoadData.csv 文件为无负载下采集的数据，在无负载情况下采集；LoadData.csv 文件需要在更换末端携带负载后重新采集（注意左右臂不可同时辨识，需要两个手臂逐一采集空载和带载辨识）

###    12. 位置姿态4×4矩阵转XYZABC
FX_BOOL FX_Matrix42XYZABCDEG(FX_DOUBLE m[4][4],FX_DOUBLE xyzabc[6])

    • 输入为4*4的法兰末端位姿矩阵
    • 输出位姿信息XYZ及欧拉角ABC（单位：mm/度）
    输出：
        成功：True/1; 失败：False/0
###     13. XYZABC转位置姿态4×4矩阵
FX_VOID FX_XYZABC2Matrix4DEG(FX_DOUBLE xyzabc[6], FX_DOUBLE m[4][4])

    • 输入为位姿信息XYZ及欧拉角ABC（单位：mm/度）
    • 输出4*4的法兰末端位姿矩阵

# 二、案例脚本
## C++开发的使用编译见：
    ./DEMO_C++/readme.md






