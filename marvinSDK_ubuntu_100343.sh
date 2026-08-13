#!/bin/bash


echo "start compile & replace so ..."

# 编译 contrlSDK100343
cd contrlSDK100343
rm -f libMarvinSDK.so
make clean 2>/dev/null
make && chmod 777 libMarvinSDK.so
cd ..

# 编译 kinematicsSDK
cd kinematicsSDK
rm -f libKine.so
make clean 2>/dev/null
make && chmod 777 libKine.so
cd ..

# 复制文件
[ -d "SDK_PYTHON" ] && cp -v contrlSDK100343/libMarvinSDK.so SDK_PYTHON/ 2>/dev/null && chmod 777 SDK_PYTHON/libMarvinSDK.so 2>/dev/null
[ -d "SDK_PYTHON" ] && cp -v kinematicsSDK/libKine.so SDK_PYTHON/ 2>/dev/null && chmod 777 SDK_PYTHON/libKine.so 2>/dev/null
[ -d "DEMO_C++" ] && cp -v contrlSDK100343/libMarvinSDK.so DEMO_C++/ 2>/dev/null && chmod 777 DEMO_C++/libMarvinSDK.so 2>/dev/null
[ -d "DEMO_C++" ] && cp -v kinematicsSDK/libKine.so DEMO_C++/ 2>/dev/null && chmod 777 DEMO_C++/libKine.so 2>/dev/null

# 编译 interferenceCheck 干涉检测库 (so)
# 源文件: Interf/*.cpp + FxCfg/CfgBase.cpp + kinematicsSDK/*.cpp
# 依赖: ICBase.cpp 调用 kinematicsSDK/FXMatrix.cpp 中的 FX_PGMult / FX_VectNorm 等函数
echo "compile interferenceCheck SDK (so) ..."
rm -f interferenceCheck/Interf/libInterfCheck.so
cd interferenceCheck/Interf
g++ *.cpp ../FxCfg/CfgBase.cpp \
    -Wall -O2 -fPIC -shared \
    -o libInterfCheck.so
chmod 777 libInterfCheck.so
cd ../..

if [ ! -f interferenceCheck/Interf/libInterfCheck.so ]; then
    echo "[X] interferenceCheck so build failed."
    exit 1
fi

[ -d "DEMO_C++" ]   && cp -v interferenceCheck/Interf/libInterfCheck.so DEMO_C++/   && chmod 777 DEMO_C++/libInterfCheck.so
[ -d "SDK_PYTHON" ] && cp -v interferenceCheck/Interf/libInterfCheck.so SDK_PYTHON/ && chmod 777 SDK_PYTHON/libInterfCheck.so
echo "✓ interferenceCheck so finished"

echo "finished!"
ls -l SDK_PYTHON/*.so DEMO_C++/*.so 2>/dev/null
