#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SDK_DIR="$SCRIPT_DIR/C_SDK"
PYTHON_DIR="$SCRIPT_DIR/PYTHON_SDK"
C_SO_DIR="$SCRIPT_DIR/C_EXAMPLE_USE_DLL_SO"

CPP_FILES="
  ./L0Control/*.cpp
  ./FileClient/*.cpp
  ./L1Robot/*.cpp
  ./Kinematics/*.cpp
  ./Kinematics/ArmKinematics/*.cpp
  ./Kinematics/BaseMath/*.cpp
  ./Kinematics/DynaIdent/*.cpp
  ./Kinematics/KineCommon/*.cpp
  ./Kinematics/MotionPlanner/*.cpp
  ./Kinematics/SkyeBodyKinematics/*.cpp
"

INC_DIRS="
  -I./Common
  -I./Kinematics
  -I./Kinematics/ArmKinematics
  -I./Kinematics/BaseMath
  -I./Kinematics/DynaIdent
  -I./Kinematics/KineCommon
  -I./Kinematics/MotionPlanner
  -I./Kinematics/SkyeBodyKinematics
  -I./FileClient
  -I./L0Control
  -I./L1Robot
"

# 确保目标目录存在
mkdir -p "$PYTHON_DIR" "$C_SO_DIR"

cd "$SDK_DIR"

echo "============================================"
echo "[1/2] Building libGentoSDK.so (C/C++ link)"
echo "============================================"
g++ $CPP_FILES $INC_DIRS \
  -Wall -O2 -fPIC -shared \
  -o libGentoSDK.so \
  -lpthread -lrt -DCMPL_LIN
echo "[OK] libGentoSDK.so built."

echo ""
echo "============================================"
echo "[2/2] Building libGentoSDKPY.so (Python, glibc-compatible)"
echo "============================================"
g++ $CPP_FILES $INC_DIRS \
  -Wall -O2 -fPIC -shared \
  -Wl,-soname,libGentoSDKPY.so \
  -Wl,--hash-style=both \
  -o libGentoSDKPY.so \
  -DL1_SDK_EXPORTS -DCMPL_LIN \
  -static-libgcc -static-libstdc++ \
  -lpthread -ldl -lm
echo "[OK] libGentoSDKPY.so built."

cd "$SCRIPT_DIR"

echo ""
echo "============================================"
echo "Copying SOs to target folders..."
echo "============================================"

if [ -f "$SDK_DIR/libGentoSDK.so" ]; then
    cp -v "$SDK_DIR/libGentoSDK.so" "$C_SO_DIR/"
    echo "[OK] libGentoSDK.so -> C_EXAMPLE_USE_DLL_SO/"
else
    echo "[FAIL] Source file not found: $SDK_DIR/libGentoSDK.so"
    exit 1
fi

if [ -f "$SDK_DIR/libGentoSDKPY.so" ]; then
    cp -v "$SDK_DIR/libGentoSDKPY.so" "$PYTHON_DIR/"
    echo "[OK] libGentoSDKPY.so -> PYTHON_SDK/"
else
    echo "[FAIL] Source file not found: $SDK_DIR/libGentoSDKPY.so"
    exit 1
fi

echo ""
echo "============================================"
echo "All done!"
echo "============================================"
