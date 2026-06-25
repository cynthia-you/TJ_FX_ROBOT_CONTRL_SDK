import PyInstaller.__main__
import shutil
import os
import sys
import glob

base_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(base_dir)

ui_entry = os.path.join(base_dir, 'UI.py')
src_dir = os.path.join(base_dir, 'src')
sdk_dir = os.path.join(project_root, 'PYTHON_SDK')

if not os.path.exists(sdk_dir):
    print(f"Error: PYTHON_SDK directory not found: {sdk_dir}")
    sys.exit(1)

sdk_py_files = glob.glob(os.path.join(sdk_dir, '*.py'))
sdk_dll_files = glob.glob(os.path.join(sdk_dir, '*.dll'))
sdk_so_files = glob.glob(os.path.join(sdk_dir, '*.so'))

build_dir = os.path.join(base_dir, 'build')
dist_dir = os.path.join(base_dir, 'dist')

if sys.platform == 'win32':
    pack_args = [
        ui_entry,
        '--onefile',
        # '--windowed',
        '--icon', os.path.join(src_dir, 'logo.ico'),
        '--name', 'FXPlatform',
        '--workpath', build_dir,
        '--distpath', dist_dir,
        '--specpath', base_dir,
        '--paths', sdk_dir,
        '--hidden-import', 'ctypes',
    ]
    for dll in sdk_dll_files:
        pack_args.extend(['--add-binary', dll + ';.'])
    for py_file in sdk_py_files:
        pack_args.extend(['--add-data', py_file + ';PYTHON_SDK'])
    pack_args.extend(['--add-data', os.path.join(src_dir, 'logo.ico') + ';src'])
else:
    pack_args = [
        ui_entry,
        '--onefile',
        '--windowed',
        '--icon', os.path.join(src_dir, 'logo.png'),
        '--name', 'FXPlatform',
        '--workpath', build_dir,
        '--distpath', dist_dir,
        '--specpath', base_dir,
        '--paths', sdk_dir,
        '--hidden-import', 'ctypes',
        '--hidden-import', 'PIL._tkinter_finder',
        '--hidden-import', 'PIL.Image',
        '--hidden-import', 'PIL.ImageTk',
    ]
    for so in sdk_so_files:
        pack_args.extend(['--add-binary', so + ':.'])
    for py_file in sdk_py_files:
        pack_args.extend(['--add-data', py_file + ':PYTHON_SDK'])
    pack_args.extend(['--add-data', os.path.join(src_dir, 'logo.png') + ':src'])

PyInstaller.__main__.run(pack_args)

print("Cleaning temporary files...")
shutil.rmtree(build_dir, ignore_errors=True)
shutil.rmtree(os.path.join(base_dir, '__pycache__'), ignore_errors=True)
sdk_pycache = os.path.join(sdk_dir, '__pycache__')
shutil.rmtree(sdk_pycache, ignore_errors=True)

if sys.platform == 'win32':
    exe_pattern = os.path.join(dist_dir, '*.exe')
else:
    exe_pattern = os.path.join(dist_dir, 'FXPlatform')

exe_files = glob.glob(exe_pattern)
if exe_files:
    exe_path = exe_files[0]
    target_path = os.path.join(project_root, os.path.basename(exe_path))
    print(f"Moving {exe_path} -> {target_path}")
    shutil.move(exe_path, target_path)
else:
    print("Warning: No executable file found in dist directory")

print("Deleting dist folder...")
shutil.rmtree(dist_dir, ignore_errors=True)

for spec_file in glob.glob(os.path.join(base_dir, '*.spec')):
    print(f"Deleting {spec_file}")
    os.remove(spec_file)

print("Cleaning project root residuals...")
shutil.rmtree(os.path.join(project_root, 'build'), ignore_errors=True)
shutil.rmtree(os.path.join(project_root, 'dist'), ignore_errors=True)
for spec_file in glob.glob(os.path.join(project_root, '*.spec')):
    print(f"Deleting {spec_file}")
    os.remove(spec_file)

print("Packaging completed! Executable file is in the project root directory.")
