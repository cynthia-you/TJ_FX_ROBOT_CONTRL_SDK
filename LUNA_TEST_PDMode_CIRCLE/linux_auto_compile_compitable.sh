#!/bin/bash
# ============================================================================
#  兼容性编译脚本 — 在容器内用旧版 glibc 编译，产物可在各类 Linux 上通用
#
#  背景:
#    Ubuntu 24.04 自带 glibc 2.39，直接编译出的 .so 要求 glibc >= 2.39，
#    在 Ubuntu 20.04 (glibc 2.31)、22.04 (glibc 2.35) 上会报错:
#      "version `GLIBC_2.xx' not found"
#  解决方案：
#     用 Docker 拉旧版 ubuntu 镜像在容器内编译，默认 ubuntu:20.04 (glibc 2.31)
#
#  用法:
#    ./linux_auto_compile_compitable.sh              # Docker 容器编译 (推荐)
#    ./linux_auto_compile_compitable.sh --native      # 本地直接编译
#    BASE_IMAGE=ubuntu:18.04 ./linux_auto_compile_compitable.sh  # 指定镜像
# ============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SDK_DIR="$SCRIPT_DIR/C_SDK"
PYTHON_DIR="$SCRIPT_DIR/PYTHON_SDK"
C_SO_DIR="$SCRIPT_DIR/C_EXAMPLE_USE_DLL_SO"

# 默认用 Ubuntu 20.04 编译（glibc 2.31，覆盖主流系统）
BASE_IMAGE="${BASE_IMAGE:-ubuntu:20.04}"

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

do_compile() {
    # 确保目标目录存在
    mkdir -p "$PYTHON_DIR" "$C_SO_DIR"

    cd "$SDK_DIR"

    echo "============================================"
    echo "[1/2] Building libGentoSDK.so (C/C++ link)"
    echo "============================================"
    g++ $CPP_FILES $INC_DIRS \
      -Wall -O2 -fPIC -shared \
      -Wl,-soname,libGentoSDK.so \
      -Wl,--hash-style=both \
      -o libGentoSDK.so \
      -DCMPL_LIN -DL1_SDK_EXPORTS \
      -static-libgcc -static-libstdc++ \
      -lpthread -lrt
    echo "[OK] libGentoSDK.so built."

    echo ""
    echo "============================================"
    echo "[2/2] Building libGentoSDKPY.so (Python, max compatibility)"
    echo "============================================"
    g++ $CPP_FILES $INC_DIRS \
      -Wall -O2 -fPIC -shared \
      -Wl,-soname,libGentoSDKPY.so \
      -Wl,--hash-style=both \
      -o libGentoSDKPY.so \
      -DCMPL_LIN -DL1_SDK_EXPORTS \
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
    echo "--- glibc 版本要求 (最高不应超过镜像的 glibc) ---"
    objdump -T "$PYTHON_DIR/libGentoSDKPY.so" 2>/dev/null | grep -oP 'GLIBC_\S+' | sort -Vu || true

    echo ""
    echo "============================================"
    echo "All done!"
    echo "============================================"
}

if [ "${1:-}" = "--native" ]; then
    echo "[INFO] 本地编译模式（产物仅适用当前系统）"
    do_compile
else
    echo "[INFO] Docker 兼容编译模式（镜像: $BASE_IMAGE）"
    echo "[INFO] 产物最低 glibc 取决于镜像，可兼容该版本及以上的所有 Linux"

    if ! command -v docker &>/dev/null; then
        echo "[ERROR] 未找到 Docker。请安装后重试，或用 --native 本地编译。"
        exit 1
    fi

    docker run --rm \
      -v "$SCRIPT_DIR":/work \
      -w /work \
      "$BASE_IMAGE" \
      bash -c "
        set -e
        echo '[INFO] 容器内 glibc: '\$(ldd --version 2>&1 | head -1)
        apt-get update -qq && apt-get install -y -qq --no-install-recommends g++ > /dev/null 2>&1
        echo '[INFO] 容器内 g++ 版本: '\$(g++ --version | head -1)
        bash linux_auto_compile_compitable.sh --native
      "
fi
