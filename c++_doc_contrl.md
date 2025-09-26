# 天机-孚晞 机器人控制工具包 MarvinSDK
## 机器人型号： MARVIN人形双臂, 单臂
## 版本： 1003
## 支持平台： LINUX 及 WINDOWS
## LINUX支持： ubuntu18.04 - ubuntu24.04
## 更新日期：2025-09


## 一、 MarvinSDK为上位机控制机器人（双臂系统）的二次开发工具包，提供功能大类有：
(1) 1KHz 通信
    下发指令和订阅机器人数据是1KHz 通信， 采用UDP通信。


(2) 控制状态切换

    ① 下使能
    ② 位置跟随模式
    ③ 位置PVT模式
    ④ 扭矩模式
        1) 关节阻抗控制/关节阻抗控制位置跟随
        2) 坐标阻抗控制/坐标阻抗控制位置跟随
        3) 力控制/力控制位置跟随

(3) 控制状态参数（1KHz）

    ① 参数
        1) 目标跟随速度加速度设定
        2) 关节阻抗参数设定
        3) 坐标阻抗参数设定
        4) 力控制参数设定
        5) 工具运动学/动力学参数设定
    ② 指令
        1) 位置跟随目标指令 
        2) 力控目标指令 

(4) 数据反馈和采集（1KHz）

    ① 实时反馈
        1) 位置
        2) 速度
        3) 外编位置
        4) 电流
        5) 传感器扭矩
        6) 摩檫力
        7) 轴外力
    ② 数据采集
        1) 针对实时反馈数据可选择多达35项数据进行实时采集。

(5) 参数获取和设置

    ① 统一接口以参数名方式获取和设置所有参数。


# 二、接口介绍
## 接口快速全览见: c++_linux/MarvinSDK.h
## 所有左右臂相关接口都是后缀_A或_B表示， _A 为左臂 _B 为右臂

### (1) 连接和释放运行内存
bool OnLinkTo(FX_UCHAR ip1, FX_UCHAR ip2, FX_UCHAR ip3, FX_UCHAR ip4);

    DEMO:  
    OnLinkTo(192, 168,1,190)
    基于UDP 连接并不代表数据已经开始发送，只有在控制器接收到发送数据之后才会向上位机开始1000HZ的周期性状态数据发送。

bool OnRelease();

    释放内存后，要获取机器人的控制，需再次连接

### (2) 系统及系统更新相关
long OnGetSDKVersion();

    获取SDK版本


bool OnUpdateSystem(char* local_path);

    更新系统，更新文件为本机本机绝对路径

bool OnDownloadLog(char* local_path);

    获取系统日志，下载到本机绝对路径


### (3) 系统日志开关
void OnLogOn();

    全局日志开， 日志信息将全部打印，包括1000HZ频率日志以及清空待发送数据缓冲区日志信息
void OnLogOff();

    全局日志关
void OnLocalLogOn();

    主要日志开，打印显示主要指令接口信息
void OnLocalLogOff();

    主要日志关

### (4) 急停、获取错误码和清错
void OnEMG_A();

void OnEMG_B();

void OnEMG_AB();

    两条手臂开单独软急停也可同时软急停

void OnGetServoErr_A(long ErrCode[7]);

void OnGetServoErr_B(long ErrCode[7]);

    获取指定手臂的错误码，长度为7，十进制
    注意连接机器人小睡半秒后，应清错
    获取错误码不为0时，应清错
    订阅回来的机器人当前状态有错时候，应清错

    如果清错后仍然无法使能连接，则说明驱动器存在错误需要断电重启。

void OnClearErr_A();

void OnClearErr_B();

    清除指定手臂的错误/复位

### (5) 实时订阅机器人数据
bool OnGetBuf(DCSS * ret);


    DCSS结构体及信息细节，获取回来的数据都是双臂的数据，如果是单臂，数据索引第0位。
    typedef struct
    {
        FX_INT32   m_CurState;	///* 当前状态 */ 
        FX_INT32   m_CmdState;	///* 指令状态 */ 
        FX_INT32   m_ERRCode;	///* 错误码   */
    }StateCtr;
    

    typedef struct
    {
        FX_INT32 	m_OutFrameSerial;   	///* 输出帧序号   0 -  1000000 取模*/
        FX_FLOAT    m_FB_Joint_Pos[7];		///* 反馈关节位置 */							0-6
        FX_FLOAT    m_FB_Joint_Vel[7];		///* 反馈关节速度 */							10-16
        FX_FLOAT    m_FB_Joint_PosE[7];		///* 反馈关节位置(外编) */						20-26
        FX_FLOAT    m_FB_Joint_Cmd[7];		///* 位置关节指令 */							30-36
        FX_FLOAT    m_FB_Joint_CToq[7];		///* 反馈关节电流 */							40-46
        FX_FLOAT    m_FB_Joint_SToq[7];		///* 反馈关节扭矩 */							50-56
        FX_FLOAT    m_FB_Joint_Them[7];		///* 反馈关节温度 */
        FX_FLOAT    m_EST_Joint_Firc[7];	///* 关节摩檫力估计值 */						60-66
        FX_FLOAT    m_EST_Joint_Firc_Dot[7];	///* 关节力扰动估计值微分 */				70-76
        FX_FLOAT    m_EST_Joint_Force[7];	///* 关节力扰动估计值 */						80-86
        FX_FLOAT    m_EST_Cart_FN[6];		///* 末端扰动估计值 */							90-95
        FX_CHAR     m_TipDI;                ///* 是否按住拖动按钮信号 */	
        FX_CHAR     m_LowSpdFlag;			///* 机器人停止运动标志， 可用于判断是否运动到位。 */	
        FX_CHAR     m_pad[2];               ///* 填充，没有实义 */	
    }RT_OUT; ///* 机器人反馈数据*/
    
    typedef struct
    {
        FX_INT32 m_RtInSwitch;  	 	///* 实时输入开关 用户实时数据 进行开关设置 0 -  close rt_in ;1- open rt_in*/
        FX_INT32 m_ImpType;             ///*阻抗类型*/
        FX_INT32 m_InFrameSerial;    	///* 输入帧序号   0 -  1000000 取模*/
        FX_INT16 m_FrameMissCnt;    	///* 丢帧计数*/
        FX_INT16 m_MaxFrameMissCnt;		///* 开 启 后 最 大 丢 帧 计 数 */
    
        FX_INT32 m_SysCyc;    			///* 0 -  1000000 */
        FX_INT16 m_SysCycMissCnt;		///* 实 时 性  Miss 计 数*/
        FX_INT16 m_MaxSysCycMissCnt;	///* 开 启 后 最 大 实 时 性Miss 计 数 */
    
        FX_FLOAT m_ToolKine[6];			///* 工 具 运 动 学 参 数 */ 1
        FX_FLOAT m_ToolDyn[10];			///* 工 具 动 力 学 参 数 */ 1
    
        FX_FLOAT m_Joint_CMD_Pos[7];	///* 关 节 位 置 指 令 */         7     
        FX_INT16 m_Joint_Vel_Ratio;		///* 关 节 速 度 限 制 百分比*/        2
        FX_INT16 m_Joint_Acc_Ratio;		///* 关 节 加 速 度 限 制  百分比*/    2
    
        FX_FLOAT m_Joint_K[7]; 			///* 关节阻抗刚度K指令*///3
        FX_FLOAT m_Joint_D[7]; 			///* 关节阻抗阻尼D指令*///3
    
        FX_INT32 m_DragSpType; 			///* 零空间类型*///5
        FX_FLOAT m_DragSpPara[6]; 		///* 零空间参数类型*///5
        
        FX_INT32 m_Cart_KD_Type;		///* 坐标阻抗类型*/
        FX_FLOAT m_Cart_K[6]; 			///* 坐标阻抗刚度K指令*///4
        FX_FLOAT m_Cart_D[6]; 			///* 坐标阻抗阻尼D指令*///4

    
        FX_INT32  m_Force_FB_Type;		///* 力控反馈源类型*/
        FX_INT32  m_Force_Type;			///* 力控类型*///6
        FX_FLOAT  m_Force_Dir[6];		///* 力控方向6维空间方向*///6
        FX_FLOAT  m_Force_PIDUL[7];		///* 力控pid*///6
        FX_FLOAT  m_Force_AdjLmt;		///* 允许调节最大范围*///6
    
        FX_FLOAT  m_Force_Cmd;			///* 力控指令*///8
    
        FX_UCHAR m_SET_Tags[16];        ///* 设置TAG*///
        FX_UCHAR m_Update_Tags[16];     ///* 更新TAG*///
    
        FX_UCHAR m_PvtID;   ///* 设置的PVT号*///
        FX_UCHAR m_PvtID_Update;  ///* PVT号更新情况*///
        FX_UCHAR m_Pvt_RunID;    // 0: no pvt file; 1~99: 用户上传的PVT
        FX_UCHAR m_Pvt_RunState; // 0: idle空闲; 1: loading正在加载 ; 2: running正在运行; 3: error出错啦
    
    }RT_IN;  ///* 给机器人发送的最新指令*/
    
    typedef struct
    {
        StateCtr m_State[2]; //获取状态结构信息
        RT_IN    m_In[2]; //获取输入的最近历史指令信息
        RT_OUT	 m_Out[2]; //获取机器人当前反馈数据
    
        ///*获取机器人配置参数， 结合（6）配置机器人参数相关*/ 
        FX_CHAR m_ParaName[30]; // 参数名称
        FX_UCHAR m_ParaType; //参数的类型 0: FX_INT32; 1: FX_DOUBLE; 2: FX_STRING
        FX_UCHAR m_ParaIns;  // DCSS CfgOperationType
        FX_INT32 m_ParaValueI; // FX_INT32 value
        FX_FLOAT m_ParaValueF; // FX_FLOAT value
        FX_INT16 m_ParaCmdSerial; // from PC
        FX_INT16 m_ParaRetSerial; // working: 0; finish: cmd serial; error cmd_serial + 100
    }DCSS; 

    demo:设置指令后查看设定的指令，订阅机器人当前数据
              double K[7] = {2000,2000,2000,10,10,10,0}; //预设为参数最大上限，供参考。
              double D[7] = {0.1,0.1,0.1,0.3,0.3,1};//预设为参数最大上限，供参考。
              int type = 2; //type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
            
              OnClearSet();
              OnSetCartKD_A(K, D,type) ;
              OnSetSend();
              usleep(100000);
            
              OnClearSet();
              OnSetJointLmt_A(10, 10) ;
              OnSetSend();
              usleep(100000);

              OnClearSet();
              OnSetTargetState_A(3) ; //3:torque mode; 1:position mode; 
              OnSetImpType_A(2) ;//type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
              OnSetSend();
              usleep(100000);
            
              DCSS t;//实例化订阅数据结构体， 连接后只需实例化一次
              OnGetBuf(&t); //订阅数据
              //打印订阅A臂数据
              printf("current state of A arm:%d\n",t.m_State[0].m_CurState);
              printf("cmd state of A arm:%d\n",t.m_State[0].m_CmdState);
              printf("error code of A arms:%d\n",t.m_State[0].m_ERRCode);
              printf("CMD of impedance:%d\n",t.m_In[0].m_ImpType);
              printf("CMD of vel and acc:%d %d\n",t.m_In[0].m_Joint_Vel_Ratio,t.m_In[0].m_Joint_Acc_Ratio);
            
              printf("CMD of cart D=[%lf %lf %lf %lf %lf %lf %lf]\n",t.m_In[0].m_Cart_K[0],
                                                                      t.m_In[0].m_Cart_K[1],
                                                                      t.m_In[0].m_Cart_K[2],
                                                                      t.m_In[0].m_Cart_K[3],
                                                                      t.m_In[0].m_Cart_K[4],
                                                                      t.m_In[0].m_Cart_K[5],
                                                                      t.m_In[0].m_Cart_K[6]);
              printf("CMD of cart D=[%lf %lf %lf %lf %lf %lf %lf]\n",t.m_In[0].m_Cart_D[0],
                                                                      t.m_In[0].m_Cart_D[1],
                                                                      t.m_In[0].m_Cart_D[2],
                                                                      t.m_In[0].m_Cart_D[3],
                                                                      t.m_In[0].m_Cart_D[4],
                                                                      t.m_In[0].m_Cart_D[5],
                                                                      t.m_In[0].m_Cart_D[6]);
              printf("CMD of cart type=%d\n",t.m_In[0].m_Cart_KD_Type);
            
              // joints pose 
              double joints[7] = {10,20,30,40,50,60,70};
              OnClearSet();
              OnSetJointCmdPos_A(joints);
              OnSetSend();
              usleep(100000);
            
              //订阅刷新A臂数据
              OnGetBuf(&t);
             
              printf("CMD joints of arm A :%lf %lf %lf %lf %lf %lf %lf \n",t.m_In[0].m_Joint_CMD_Pos[0],
                                                                          t.m_In[0].m_Joint_CMD_Pos[1],
                                                                          t.m_In[0].m_Joint_CMD_Pos[2],
                                                                          t.m_In[0].m_Joint_CMD_Pos[3],
                                                                          t.m_In[0].m_Joint_CMD_Pos[4],
                                                                          t.m_In[0].m_Joint_CMD_Pos[5],
                                                                          t.m_In[0].m_Joint_CMD_Pos[6]);
              printf("current joints of arm A :%lf %lf %lf %lf %lf %lf %lf \n",t.m_Out[0].m_FB_Joint_Pos[0],
                                                                                t.m_Out[0].m_FB_Joint_Pos[1],
                                                                                t.m_Out[0].m_FB_Joint_Pos[2],
                                                                                t.m_Out[0].m_FB_Joint_Pos[3],
                                                                                t.m_Out[0].m_FB_Joint_Pos[4],
                                                                                t.m_Out[0].m_FB_Joint_Pos[5],
                                                                                t.m_Out[0].m_FB_Joint_Pos[6]);

              //查看B臂当前关节位置,获取回来的数据都是双臂的数据，B索引1，如：t.m_Out[1]， t.m_In[1], t.m_State[1]
              printf("current joints of arm B :%lf %lf %lf %lf %lf %lf %lf \n",t.m_Out[1].m_FB_Joint_Pos[0],
                                                                                t.m_Out[1].m_FB_Joint_Pos[1],
                                                                                t.m_Out[1].m_FB_Joint_Pos[2],
                                                                                t.m_Out[1].m_FB_Joint_Pos[3],
                                                                                t.m_Out[1].m_FB_Joint_Pos[4],
                                                                                t.m_Out[1].m_FB_Joint_Pos[5],
                                                                                t.m_Out[1].m_FB_Joint_Pos[6]);
    
    还有注意， 状态值的含义，参考：
    typedef enum
    {
        ARM_STATE_IDLE = 0,             //////// 下伺服
        ARM_STATE_POSITION = 1,			//////// 位置跟随
        ARM_STATE_PVT = 2,				//////// PVT
        ARM_STATE_TORQ = 3,				//////// 扭矩
    
        ARM_STATE_ERROR = 100, ////报错了，清错
        ARM_STATE_TRANS_TO_POSITION = 101,
        ARM_STATE_TRANS_TO_PVT = 102,
        ARM_STATE_TRANS_TO_TORQ = 103,
    }ArmState;
    

### (6) 配置机器人参数相关(参数名见robot.ini文件)
#### 读取整形和浮点参数信息：
long OnGetIntPara(char paraName[30],long * retValue);

    DEMO：获取左臂第一关节编码器单圈脉冲
    char name[30];
    long res;
    memset(name, 0, 30);
    sprintf(name, "R.A0.L%d.BASIC.EncRes", 0);
    if (OnGetIntPara(name, &res) != 0)
    {
        AfxMessageBox("Get K Err");
        return;
    }
long OnGetFloatPara(char paraName[30],double * retValue);

    DEMO：获取右臂7个关节位置上限
    char name[30];
    long i;
    for ( i = 0; i < 7; i++)
    {
        memset(name, 0, 30);
        sprintf(name, "R.A0.L%d.BASIC.LimitPos",i);
        OnGetFloatPara(name, &m_RunLmt[0].m_pos_u[i]);
    }


#### 设置整形和浮点参数信息：
long OnSetIntPara(char paraName[30],long setValue);

    设置整形配置参数
long OnSetFloatPara(char paraName[30], double setValue);

    设置浮点配置参数

#### 保存参数
long OnSavePara();

    返回值说明如下
    return -1/-2,               /////// 保存失败
    return ParaRetSerial,       /////// 保存参数的序号


### (7) 数据采集和保存相关

bool OnStartGather(long targetNum, long targetID[35], long recordNum);

    targetNum采集列数 （1-35列）
    targetID[35] 对应采集数据ID序号  
            左臂序号：
                0-6  	左臂关节位置 
                10-16 	左臂关节速度
                20-26   左臂外编位置
                30-36   左臂关节指令位置
                40-46	左臂关节电流（千分比）
                50-56   左臂关节传感器扭矩NM
                60-66	左臂摩擦力估计值
                70-76	左臂摩檫力速度估计值
                80-85   左臂关节外力估计值
                90-95	左臂末端点外力估计值
            右臂对应 + 100

    recordNum  采集行数 ，小于1000会采集1000行，设置大于一百万行会采集一百万行
    DEMO
    long targetNum = 2;
    long targetID[35] = {11, 31, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0}
    long recordNum = 500
    OnStartGather(targetNum, targetID, recordNum)

bool OnStopGather();

    中途停止采集
    在行数采集满后会自动停止采集,若需要中途停止采集调用本函数并等待1ms之后会停止采集。

bool OnSaveGatherData(char * path);

    path 保存到本机绝对路径
    在停止采集并且存在采集数据的情况下，可以将数据保存到文件
bool OnSaveGatherDataCSV(char* path);

    以CSV格式保存采集数据，保存到本机绝对路径
    在停止采集并且存在采集数据的情况下，可以将数据保存到文件

### (8) 末端通信模组指令收发相关
### 注意，之前无指令发送到末端模组，读取返回为0,使用逻辑为： 清缓存---发数据---读数据  或者： 读数据---清缓存---发数据---读数据
#### 清除指定手臂的缓存数据
bool OnClearChDataA();

bool OnClearChDataB();

#### 获取指定手臂的末端通信模块数据
long OnGetChDataA(unsigned char data_ptr[256], long* ret_ch);

long OnGetChDataB(unsigned char data_ptr[256], long* ret_ch);
    
    data_ptr[256]为数据，最长可收256长度字节
    ret_ch：信息来源。 1：‘C’端; 2：com1; 3:com2

#### 设置指定手臂的末端通信模块的指令数据
bool OnSetChDataA(unsigned char data_ptr[256], long size_int,long set_ch);

bool OnSetChDataB(unsigned char data_ptr[256], long size_int, long set_ch);

    data_ptr[256]：数据
    size_int：数据长度，不能超过256
    set_ch：发送通道。 1：‘C’端; 2：com1; 3:com2
    数据可发原始字节数据例如 b'0x00 0x101'  或者HEX数据"00 B1"


### (9) 上传PVT文件 
bool OnSendPVT_A(char* local_file, long serial);

bool OnSendPVT_B(char* local_file, long serial);

    local_file 本地文件绝对路径
    pvt_id     对应PVT路径号 （1-99）
    PVT文件格式见：c++_linux/DEMO_SRS_Left.fmv
    数据首行为行数和列数信息，“PoinType=9@9341 ”表示该PVT文件含9列数据，一共9341个点位。
    数据为什么是9列？ 首先前八列为关节角度， 为什么是8？ 我们预留了8关节，人形臂为7自由度，前7个有效值，第八列都填充0，
    好的，第九列，第九列是个标记列，全填0即可。


    

### (10) 运动相关指令发送  可以以1000HZ频率进行发送

bool OnClearSet();
    清空待发送数据缓冲区

    
bool OnSetSend();

    发送指令

    以下指令必须在OnClearSet()和中间OnSetSend()设置生效：
    ////×以下指令可以单条发送，也可以多条一起发送发×/////
    bool OnSetTargetState_A(int state);
    bool OnSetTool_A(double kinePara[6], double dynPara[10]);
    bool OnSetJointLmt_A(int velRatio, int AccRatio);
    bool OnSetJointKD_A(double K[7], double D[7]);
    bool OnSetCartKD_A(double K[7], double D[7], int type);
    bool OnSetDragSpace_A(int dgType);
    bool OnSetForceCtrPara_A(int fcType, double fxDir[6], double fcCtrlPara[7], double fcAdjLmt);
    bool OnSetJointCmdPos_A(double joint[7]);
    bool OnSetForceCmd_A(double force);
    bool OnSetPVT_A(int id);
    bool OnSetImpType_A(int type);
    bool OnSetTargetState_B(int state);
    bool OnSetTool_B(double kinePara[6], double dynPara[10]);
    bool OnSetJointLmt_B(int velRatio, int AccRatio);
    bool OnSetJointKD_B(double K[7], double D[7]);
    bool OnSetCartKD_B(double K[6], double D[6],int type);
    bool OnSetDragSpace_B(int dgType);
    bool OnSetForceCtrPara_B(int fcType, double fxDir[6], double fcCtrlPara[7], double fcAdjLmt);
    bool OnSetJointCmdPos_B(double joint[7]);
    bool OnSetForceCmd_B(double force);
    bool OnSetImpType_B(int type);
    bool OnSetPVT_B(int id);
    ////×以下指令可以单条发送，也可以多条一起发送发×/////

    DEMO:
    OnClearSet()   
    OnSetJointCmdPos_A(XXX) // 设置左臂目标关节位置
    OnSetJointCmdPos_B(XXX) // 设置右臂目标关节位置
    OnSetForceCmd_A(XXX)    // 设置左臂力控位置
    OnSetForceCmd_B(XXX)    // 设置右臂力控位置
    OnSetSend()

### (11) 设置指定手臂的目标状态
bool OnSetTargetState_A(int state);

bool OnSetTargetState_B(int state);
    
    state取值如下： 
    0,         //下伺服
    1,	       // 位置跟随
    2,		   // PVT
    3,		   // 扭矩


### (12) 设置指定手臂在扭矩模式下阻抗类型
bool OnSetImpType_A(int type);

bool OnSetImpType_B(int type);

    type取值如下：
    1,       // 关节阻抗
    2,       // 坐标阻抗
    3,       // 力控 
    需要在OnSetTargetState_A（3）状态

### （13）设置指定手臂的关节跟随速度/加速度
bool OnSetJointLmt_A(int velRatio, int AccRatio)

bool OnSetJointLmt_B(int velRatio, int AccRatio)

    velRatio 速度百分比， 全速100, 安全起见，调试期间设为10
    AccRatio 加速度百分比， 全速100, 安全起见，调试期间设为10

### （14）设置指定手臂的工具信息
bool OnSetTool_A(double kinePara[6], double dynPara[10]);

bool OnSetTool_B(double kinePara[6], double dynPara[10]);

    kinePara: 运动学参数 XYZABC 单位毫米和度
    dynPara:  动力学参数分别为 质量M  质心[3]:mx,my,mz 惯量I[6]:XX,XY,XZ,YY,YZ,ZZ

### （15）设置指定手臂的关节阻抗参数
bool OnSetJointKD_A(double K[7], double D[7])

bool OnSetJointKD_B(double K[7], double D[7])

    K 刚度 N*m/rad , 设置每个轴的的力为刚度系数。 如K=[2，2,2,1,1,1,1]，第1到3轴有2N作为刚度系数参与控制计算，第4到7轴有1N作为刚度系数参与控制计算。
    D 阻尼 N*m/（rad/s)，设置每个轴的的阻尼系数。

    #关节阻抗时，需更低刚度避免震动，且希望机械臂有顺从性，因此采用低刚度配低阻尼。
    1-7关节刚度不超过2
    1-7关节阻尼0-1之间

### （16）设置指定手臂的坐标阻抗参数
bool OnSetCartKD_A(double K[7], double D[7], int type)

bool OnSetCartKD_B(double K[7], double D[7], int type)

    K[0]-k[2] N*m        x,y,z 平移方向每米的控制力
    K[3]-k[5] N*m/rad    rx,ry,rz 旋转弧度的控制力
    K[6] 零空间总和刚度系数 N*m/rad  
    D[0]-D[5]  阻尼比例系数    
    D[6] 零空间总和阻尼比例系数  

    # 在笛卡尔阻抗模式下：
            刚度系数： 1-3平移方向刚度系数不超过3000, 4-6旋转方向不超过100。 零空间刚度系数不超过20
            阻尼系数： 平移和旋转阻尼系数0-1之间。 零空间阻尼系数不超过1

            零空间控制是保持末端固定不动，手臂角度运动的控制方式。接口未开放


### （17）设置指定手臂的力控参数和力控指令
bool OnSetForceCtrPara_A(int fcType, double fxDir[6], double fcCtrlPara[7], double fcAdjLmt)
bool OnSetForceCmd_A(double force)

    force目标力 单位N或者N×M

    fcType 力控类型 
        0- -坐标空间力控
        1- 工具空间力控(暂未实现)
    fxDir力控方向，需要控制方向设1，目前只支持 X,Y,Z控制方向。 如控制X方向{1,0,0,0,0,0}
    fcCtrlPara 控制参数, 目前全0
    fcAdjLmt 允许的调节范围, 厘米

    DEMO：
        set_force_control_params(arm='A',fcType=0, fxDirection=[0, 1, 0, 0, 0, 0], fcCtrlpara=[0, 0, 0, 0, 0, 0, 0],
                                        fcAdjLmt=5.)
        set_force_cmd(arm='A',f=10)
    #这两条指令搭配使用才有力控的效果
    #设置是在Y轴方向有个2斤的力一直拽着手臂提起5厘米， 上下拖动手臂试试， 手臂像弹簧一样会回到原来的位置。力控阻抗下更柔顺

bool OnSetForceCtrPara_B(int fcType, double fxDir[6], double fcCtrlPara[7], double fcAdjLmt)
bool OnSetForceCmd_B(double force)





### （18）设置指定手臂的关节跟踪指令值
bool OnSetJointCmdPos_A(double joint[7])

bool OnSetJointCmdPos_B(double joint[7])

    joint指令角度  
    在位置跟随和扭矩模式下均有效


### （19）设置指定手臂的设置运行PVT指令
bool OnSetPVT_A(int id)

bool OnSetPVT_B(int id)

    id   运行指定id号的pvt路径
    需要在 OnSetTargetState_A（2）状态状态



### （20） 设置指定手臂的拖动空间
bool OnSetDragSpace_A(int dgType);

bool OnSetDragSpace_B(int dgType);

    dgType取值如下
    0,       //退出拖动模式
    1,       //关节空间拖动
    2,       //笛卡尔空间X方向拖动
    3,       //笛卡尔空间Y方向拖动
    4,       //笛卡尔空间Z方向拖动
    5,       //笛卡尔空间旋转方向拖动




## 三、扭矩模式下刚度和阻尼的建议：
    刚度用来衡量物体抗变形的能力。刚度越大，形变越小力的传导率高，运动时感觉很脆很硬；反之，刚度越小，形变大，形状恢复慢，传递力效率低，运动时感觉比较柔软富有韧性。
    阻尼用来衡量物体耗散振动能量的能力。阻尼越大，物体振幅减小越快，但对力、位移的响应迟缓，运动时感觉阻力大，有粘滞感； 阻尼越小，减震效果减弱，但运动阻力小，更流畅，停止到位置时有余震感。

    在精密定位、点无接触式操作的应用下，需要高刚度，中高阻尼的配合。高刚度确保消除擦产生大力，快速到达精确位置，足够的阻尼能够抑制震荡。
    在刚性表面打磨、装配应用下，需要低中刚度，高阻尼的配合。低刚度避免与环境强对抗导致不稳定和过大冲击力，高阻尼消耗能量，抑制接触震荡，稳定接触力。
    生物组织操作、海绵打磨等柔性环境接触应用下，需要中刚度中阻尼的配合。中等刚度提供一定的位置跟随能力同时避免压坏柔性物体，中度阻尼平衡响应速度和平稳性。
    在人机协作、示教编程等安全接触应用下，需要极低刚度和中度阻尼的配合。极低刚度使得机械臂非常的顺从，接触力很小也能感知，中等的阻尼提供基本稳定。

    # 协作机器人关节柔性显著，当使用纯关节阻抗时，需更低刚度避免震动，且希望机械臂有顺从性，因此采用低刚度配低阻尼。
    1-7关节刚度系数不超过2
    1-7关节阻尼系数0-1之间

    # 在笛卡尔阻抗模式下：
    1-3平移方向刚度系数不超过3000, 4-6旋转方向不超过100。 零空间刚度系数不超过20
    平移和旋转阻尼系数0-1之间





## 四、案例脚本
### 4.1 C++开发的使用编译见：c++_win 和  c++_linux 下的 API_USAGE_MarvinSDK.txt
以下案例见 c++_win 和  c++_linux 下：
请注意：案例仅为参考使用，实地生产和业务逻辑需要您加油写~~~
### 位置模式：position_demo.cpp
### 拖动：drag_demo.cpp
### 扭矩关节阻抗：torque_joint_impedance_demo.cpp
### 扭矩迪卡尔阻抗：torque_cart_impedance_demo.cpp
### 扭矩力控阻抗：torque_force_impedance_demo.cpp
### 运行PVT和保存数据， 2ms 一个点 ： pvt_demo.cpp
### 末端 485 CAN 通信： eef_485_can_demo.cpp













