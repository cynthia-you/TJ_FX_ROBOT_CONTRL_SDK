# -*- mode: python ; coding: utf-8 -*-


a = Analysis(
    ['d:\\working\\TJ_FX_ROBOT_CONTRL_SDK\\MarvinPlatform_EN\\ui_EN.py'],
    pathex=['d:\\working\\TJ_FX_ROBOT_CONTRL_SDK\\SDK_PYTHON'],
    binaries=[('d:\\working\\TJ_FX_ROBOT_CONTRL_SDK\\SDK_PYTHON\\libKine.dll', '.'), ('d:\\working\\TJ_FX_ROBOT_CONTRL_SDK\\SDK_PYTHON\\libMarvinSDK.dll', '.')],
    datas=[('d:\\working\\TJ_FX_ROBOT_CONTRL_SDK\\SDK_PYTHON\\fx_kine.py', 'SDK_PYTHON'), ('d:\\working\\TJ_FX_ROBOT_CONTRL_SDK\\SDK_PYTHON\\fx_robot.py', 'SDK_PYTHON'), ('d:\\working\\TJ_FX_ROBOT_CONTRL_SDK\\SDK_PYTHON\\__ini__.py', 'SDK_PYTHON'), ('d:\\working\\TJ_FX_ROBOT_CONTRL_SDK\\MarvinPlatform_EN\\src\\logo.ico', 'src')],
    hiddenimports=['ctypes'],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='MarvinPlatform_win_100341',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=['d:\\working\\TJ_FX_ROBOT_CONTRL_SDK\\MarvinPlatform_EN\\src\\logo.ico'],
)
