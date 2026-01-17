## APP已重构为MarvinPlatform

## 你好使用软件前，需要温馨提示您：

    软件同级config/文件夹内请放入独一无二的机型配置文件:ccs 6公斤的机型的计算配置文件为ccs_m6.MvKDCfg,ccs 3公斤的机型的计算配置文件为ccs_m3.MvKDCfg； srs机型为srs.MvKDCfg. 多个*.MvKDCfg会解析出错


## 安装APP:
    1. 我们测试并提供在WINDOWS 和UBUNTU24.04_X86下可执行的软件,如果与您的环境不一致,请下载源码后编译库,直接运行或者生成可执行APP运行
    2. MarvinPlatform基础环境:python3, pyinstaller, pillow
    3. 运行前请确认:
            3.1. MMarvinPlatform基础环境:python3(无特定版本要求), pyinstaller,pillow
            3.2. 确保在 ./contrlSDK  和 ./kinematicsSDK 下的动态库SO: libMarvinSDK.so 和 libKine.so 是当前上位机环境下重新编译生成的(最好是重新编译),SDK源码地址：[https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/master/](https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/tree/master)
            3.3. 将3.2生成的 libMarvinSDK.so 和 libKine.so 替换到./MarvinPlatform/python/ 下
    4. 运行APP:
            4.1. 源码运行 ui.py
            4.2. 生成可执行文件后运行,以便于分发到其他无PY环境的电脑上: python  setup.py



## MARVIN APP使用说明文档
    见同级目录下：天机Marvin系列MarvinPlatform软件使用说明2601.pptx








    

