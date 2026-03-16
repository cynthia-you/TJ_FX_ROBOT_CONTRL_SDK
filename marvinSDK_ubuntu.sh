#!/bin/bash


echo "start compile & replace so ..."

# 编译 contrlSDK
cd contrlSDK
make clean 2>/dev/null
make && chmod 777 libMarvinSDK.so
cd ..

# 编译 kinematicsSDK
cd kinematicsSDK
make clean 2>/dev/null
make && chmod 777 libKine.so
cd ..

# 复制文件
[ -d "SDK_PYTHON" ] && cp -v contrlSDK/libMarvinSDK.so SDK_PYTHON/ 2>/dev/null && chmod 777 SDK_PYTHON/libMarvinSDK.so 2>/dev/null
[ -d "SDK_PYTHON" ] && cp -v kinematicsSDK/libKine.so SDK_PYTHON/ 2>/dev/null && chmod 777 SDK_PYTHON/libKine.so 2>/dev/null
[ -d "DEMO_C++" ] && cp -v contrlSDK/libMarvinSDK.so DEMO_C++/ 2>/dev/null && chmod 777 DEMO_C++/libMarvinSDK.so 2>/dev/null
[ -d "DEMO_C++" ] && cp -v kinematicsSDK/libKine.so DEMO_C++/ 2>/dev/null && chmod 777 DEMO_C++/libKine.so 2>/dev/null

echo "finished!"
ls -l SDK_PYTHON/*.so DEMO_C++/*.so 2>/dev/null