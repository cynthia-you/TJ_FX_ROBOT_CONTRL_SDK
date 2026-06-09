import PyInstaller.__main__
import shutil
import os
import sys
import glob

base_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(base_dir)

ui_entry = os.path.join(base_dir, 'ui_EN.py')
src_dir = os.path.join(base_dir, 'src')
sdk_dir = os.path.join(project_root, 'SDK_PYTHON')

if not os.path.exists(sdk_dir):
    print(f"Error: SDK_PYTHON directory not found: {sdk_dir}")
    sys.exit(1)


# ============================================================
# 工具函数
# ============================================================

def move_executable():
    """将可执行文件从 dist 拷贝到 MarvinPlatform_EN 目录下"""
    dist_dir = os.path.join(base_dir, 'dist')
    if not os.path.exists(dist_dir):
        print("Warning: dist folder not found, nothing to move")
        return

    if sys.platform == 'win32':
        pattern = '*.exe'
    else:
        pattern = '*'

    candidates = glob.glob(os.path.join(dist_dir, pattern))
    exec_files = [f for f in candidates if os.path.isfile(f)]

    if not exec_files:
        print("Warning: no executable found in dist/")
        return

    for src_path in exec_files:
        filename = os.path.basename(src_path)
        dst_path = os.path.join(base_dir, filename)
        print(f"Copying {src_path} -> {dst_path}")
        shutil.copy2(src_path, dst_path)

    print("Executable(s) copied to MarvinPlatform_EN/")


def clean_build_artifacts():
    """清理所有编译生成的垃圾：build / dist / spec / __pycache__"""
    print("\nCleaning build artifacts ...")

    dirs_to_remove = [
        os.path.join(base_dir, 'build'),
        os.path.join(base_dir, 'dist'),
        os.path.join(project_root, 'build'),
        os.path.join(project_root, 'dist'),
    ]
    for d in dirs_to_remove:
        if os.path.exists(d):
            print(f"  Removing {d}")
            shutil.rmtree(d, ignore_errors=True)

    for spec in glob.glob(os.path.join(base_dir, '*.spec')):
        print(f"  Removing {spec}")
        os.remove(spec)
    for spec in glob.glob(os.path.join(project_root, '*.spec')):
        print(f"  Removing {spec}")
        os.remove(spec)

    pycache_dirs = glob.glob(os.path.join(base_dir, '**', '__pycache__'), recursive=True)
    pycache_dirs.append(os.path.join(sdk_dir, '__pycache__'))
    for pc in pycache_dirs:
        if os.path.exists(pc):
            print(f"  Removing {pc}")
            shutil.rmtree(pc, ignore_errors=True)

    print("Cleanup done.")


# ============================================================
# 打包参数 — 双平台自适应
# ============================================================

# PyInstaller 的路径分隔符：Windows 用 ;  Linux/macOS 用 :
path_sep = ';' if sys.platform == 'win32' else ':'

# 可执行文件名 / 图标
if sys.platform == 'win32':
    exe_name = 'MarvinPlatform_win_100341'
    icon_ext = 'ico'
    bin_exts = ('.dll',)
else:
    exe_name = 'MarvinPlatform_linux_100341'
    icon_ext = 'png'
    bin_exts = ('.so',)

pack_args = [
    ui_entry,
    '--onefile',
    '--workpath', os.path.join(base_dir, 'build'),
    '--distpath', os.path.join(base_dir, 'dist'),
    '--specpath', base_dir,
    '--icon', os.path.join(src_dir, f'logo.{icon_ext}'),
    '--name', exe_name,
    '--paths', sdk_dir,
    '--hidden-import', 'ctypes',
]

# Linux 额外依赖
if sys.platform != 'win32':
    pack_args.extend([
        '--hidden-import', 'PIL._tkinter_finder',
        '--hidden-import', 'PIL.Image',
        '--hidden-import', 'PIL.ImageTk',
    ])

# ---------- 添加 SDK 的 .py 文件 (add-data) ----------
for py_file in glob.glob(os.path.join(sdk_dir, '*.py')):
    dest = py_file + path_sep + 'SDK_PYTHON'
    pack_args.extend(['--add-data', dest])
    print(f"  add-data: {py_file}")

# ---------- 添加 SDK 的二进制文件 (add-binary) ----------
for ext in bin_exts:
    for bin_file in glob.glob(os.path.join(sdk_dir, '*' + ext)):
        dest = bin_file + path_sep + '.'
        pack_args.extend(['--add-binary', dest])
        print(f"  add-binary: {bin_file}")

# ---------- 添加资源文件 ----------
icon_src = os.path.join(src_dir, f'logo.{icon_ext}')
dest = icon_src + path_sep + 'src'
pack_args.extend(['--add-data', dest])
print(f"  add-data: {icon_src}")

# ============================================================
# 执行
# ============================================================

print(f"\nBuilding for platform: {sys.platform} ...")
PyInstaller.__main__.run(pack_args)

move_executable()
clean_build_artifacts()

print("\n=== Packaging completed! Executable is in MarvinPlatform_EN/ ===")
