import PyInstaller.__main__
import shutil
import os
import sys

base_dir = os.path.dirname(os.path.abspath(__file__))

if sys.platform=='win32':
    pack_args = [
        'UI_L1.py',
        '--onefile',
        # '--windowed',  # 窗口应用程序(无控制台)
        '--icon', os.path.join('src', 'logo.ico'),
        '--name', 'GentoPlatformV1004',
        '--add-binary', f'PythonSdk/libGentoSDKPY.dll;.',
    ]

    # 添加python文件夹中的所有.dll文件作为二进制文件（排除已单独添加的）
    # 可以根据需要决定是否保留这一行
    pack_args.extend(['--add-binary', f'PythonSdk/*.dll;.'])

    pack_args.extend(['--add-data', f'PythonSdk/*.py;PythonSdk'])

    pack_args.extend(['--add-data', f'src/logo.ico;src'])

    pack_args.extend(['--paths', 'PythonSdk'])

    PyInstaller.__main__.run(pack_args)

    print("清理临时文件...")
    shutil.rmtree('build', ignore_errors=True)
    shutil.rmtree('__pycache__', ignore_errors=True)

    # 清理python文件夹中的pycache
    python_pycache = os.path.join('PythonSdk', '__pycache__')
    shutil.rmtree(python_pycache, ignore_errors=True)

else:

    pack_args = [
        'UI_L1.py',
        '--onefile',
        '--windowed',
        '--icon', os.path.join('src', 'logo.png'),
        '--name', 'GentoPlatformV1004',
        '--add-binary', f'PythonSdk/libGentoSDKPY.so:.',
        '--hidden-import', 'PIL._tkinter_finder',
        '--hidden-import', 'PIL.Image',
        '--hidden-import', 'PIL.ImageTk',
    ]

    pack_args.extend(['--add-binary', f'PythonSdk/*.so:.'])

    pack_args.extend(['--add-data', f'PythonSdk/*.py:PythonSdk'])

    pack_args.extend(['--add-data', f'src/logo.png:src'])

    pack_args.extend(['--paths', 'PythonSdk'])

    PyInstaller.__main__.run(pack_args)

    print("清理临时文件...")
    shutil.rmtree('build', ignore_errors=True)
    shutil.rmtree('__pycache__', ignore_errors=True)

    python_pycache = os.path.join('PythonSdk', '__pycache__')
    shutil.rmtree(python_pycache, ignore_errors=True)

print("打包完成！可执行文件在 dist 文件夹中")
