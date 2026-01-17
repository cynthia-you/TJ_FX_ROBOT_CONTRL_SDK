import PyInstaller.__main__
import shutil
import os
import sys

# 获取当前目录
base_dir = os.path.dirname(os.path.abspath(__file__))

if sys.platform=='win32':

    # 打包参数列表
    pack_args = [
        'ui.py',  # 主入口文件
        '--onefile',  # 打包成单个exe文件
        # '--windowed',  # 窗口应用程序(无控制台)
        '--icon', os.path.join('src', 'logo.ico'),  # 设置exe图标
        '--name', 'MarvinPlatform0117',  # 生成的exe名称
        '--add-binary', f'python/libMarvinSDK.dll;.',  # 关键修改：DLL放在根目录
    ]

    # 添加python文件夹中的所有.dll文件作为二进制文件（排除已单独添加的）
    # 可以根据需要决定是否保留这一行
    pack_args.extend(['--add-binary', f'python/*.dll;.'])

    # 添加python文件夹中的所有.py文件作为数据文件
    pack_args.extend(['--add-data', f'python/*.py;python'])

    # 添加src文件夹中的资源文件

    pack_args.extend(['--add-data', f'src/logo.ico;src'])

    # 添加python文件夹到搜索路径
    pack_args.extend(['--paths', 'python'])

    # 运行打包
    PyInstaller.__main__.run(pack_args)

    # 清理临时文件
    print("清理临时文件...")
    shutil.rmtree('build', ignore_errors=True)
    shutil.rmtree('__pycache__', ignore_errors=True)

    # 清理python文件夹中的pycache
    python_pycache = os.path.join('python', '__pycache__')
    shutil.rmtree(python_pycache, ignore_errors=True)

else:
    # 打包参数列表
    pack_args = [
        'ui.py',                    # 主入口文件
        '--onefile',                   # 打包成单个可执行文件
        '--windowed',                  # 窗口应用程序(无控制台)
        '--icon', os.path.join('src', 'logo.png'),  # 设置图标 - 使用PNG格式
        '--name', 'MarvinPlatform-ubuntu2404-0117',        # 生成的执行文件名称
        '--add-binary', f'python/libMarvinSDK.so:.',  # 特别添加SO文件，使用Linux路径分隔符
        '--hidden-import', 'PIL._tkinter_finder',  # 添加隐藏导入
        '--hidden-import', 'PIL.Image',            # 可以同时添加其他可能缺失的PIL模块
        '--hidden-import', 'PIL.ImageTk',
    ]

    # 添加python文件夹中的所有.so文件作为二进制文件
    pack_args.extend(['--add-binary', f'python/*.so:.'])  # 使用冒号作为路径分隔符

    # 添加python文件夹中的所有.py文件作为数据文件
    pack_args.extend(['--add-data', f'python/*.py:python'])  # 使用冒号作为路径分隔符

    # 添加src文件夹中的资源文件
    pack_args.extend(['--add-data', f'src/logo.png:src'])  # 使用PNG格式图标

    # 添加python文件夹到搜索路径
    pack_args.extend(['--paths', 'python'])

    # 运行打包
    PyInstaller.__main__.run(pack_args)

    # 清理临时文件
    print("清理临时文件...")
    shutil.rmtree('build', ignore_errors=True)
    shutil.rmtree('__pycache__', ignore_errors=True)

    # 清理python文件夹中的pycache
    python_pycache = os.path.join('python', '__pycache__')
    shutil.rmtree(python_pycache, ignore_errors=True)

print("打包完成！可执行文件在 dist 文件夹中")
