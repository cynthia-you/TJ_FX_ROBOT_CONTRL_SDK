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



def move_executable():
    """将生成的可执行文件从 dist 移动到项目根目录"""
    dist_dir = os.path.join(base_dir, 'dist')
    if not os.path.exists(dist_dir):
        print("警告: dist 文件夹不存在，无法移动可执行文件")
        return

    # 根据平台查找生成的可执行文件
    if sys.platform == 'win32':
        pattern = '*.exe'
    else:
        pattern = '*'  # Linux 下可执行文件无后缀，且只有一个文件

    # 获取 dist 目录下的所有匹配文件
    candidates = glob.glob(os.path.join(dist_dir, pattern))
    # 过滤掉目录，只保留文件
    exec_files = [f for f in candidates if os.path.isfile(f)]

    if not exec_files:
        print("警告: dist 目录中没有找到可执行文件")
        return

    # 如果有多个，取第一个（通常只有一个）
    src_path = exec_files[0]
    filename = os.path.basename(src_path)
    dst_path = os.path.join(project_root, filename)

    print(f"移动 {src_path} -> {dst_path}")
    shutil.move(src_path, dst_path)
    print("移动完成")

    # 移动完成后删除整个 dist 文件夹
    print("删除 dist 文件夹...")
    shutil.rmtree(dist_dir, ignore_errors=True)


# ========== 开始打包 ==========
if sys.platform == 'win32':
    pack_args = [
        'ui_EN.py',
        '--onefile',
        # '--windowed',          # 需要控制台则取消注释
        '--icon', os.path.join('src', 'logo.ico'),
        '--name', 'MarvinPlatform_win',
        '--add-binary', os.path.join(sdk_dir, 'libMarvinSDK.dll') + ';.',
        '--paths', sdk_dir,
        '--hidden-import', 'ctypes',
    ]
    pack_args.extend(['--add-binary', os.path.join(sdk_dir, '*.dll') + ';.'])
    pack_args.extend(['--add-data', os.path.join(sdk_dir, '*.py') + ';SDK_PYTHON'])
    pack_args.extend(['--add-data', os.path.join(src_dir, 'logo.ico') + ';src'])

else:  # Linux / macOS
    pack_args = [
        'ui_EN.py',
        '--onefile',
        '--windowed',  # 无控制台
        '--icon', os.path.join('src', 'logo.png'),
        '--name', 'MarvinPlatform_linux',
        '--add-binary', os.path.join(sdk_dir, 'libMarvinSDK.so') + ':.',
        '--paths', sdk_dir,
        '--hidden-import', 'ctypes',
        '--hidden-import', 'ctypes',
        '--hidden-import', 'PIL._tkinter_finder',
        '--hidden-import', 'PIL.Image',
        '--hidden-import', 'PIL.ImageTk',
    ]
    pack_args.extend(['--add-binary', os.path.join(sdk_dir, '*.so') + ':.'])
    pack_args.extend(['--add-data', os.path.join(sdk_dir, '*.py') + ':PYTHON_SDK'])
    pack_args.extend(['--add-data', os.path.join(src_dir, 'logo.png') + ':src'])
# 执行打包
PyInstaller.__main__.run(pack_args)

print("Cleaning temporary files...")
shutil.rmtree(os.path.join(base_dir, 'build'), ignore_errors=True)
shutil.rmtree(os.path.join(base_dir, '__pycache__'), ignore_errors=True)
sdk_pycache = os.path.join(sdk_dir, '__pycache__')
shutil.rmtree(sdk_pycache, ignore_errors=True)

exe_files = glob.glob(os.path.join(base_dir, 'dist', '*.exe'))
if exe_files:
    exe_path = exe_files[0]
    target_path = os.path.join(base_dir, os.path.basename(exe_path))
    print(f"Moving {exe_path} -> {target_path}")
    shutil.move(exe_path, target_path)
else:
    print("Warning: No .exe file found in dist directory")

print("Deleting dist folder...")
shutil.rmtree(os.path.join(base_dir, 'dist'), ignore_errors=True)

spec_files = glob.glob(os.path.join(base_dir, '*.spec'))
for spec_file in spec_files:
    print(f"Deleting {spec_file}")
    os.remove(spec_file)

print("Packaging completed! Executable file is in the project root directory.")
