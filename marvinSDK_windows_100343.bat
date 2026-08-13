@echo off
chcp 65001 >nul
echo compile c++ SDK for DEMO_C++...
del /F /Q contrlSDK100343\libMarvinSDK.dll 2>nul
del /F /Q kinematicsSDK\libKine.dll 2>nul
cd contrlSDK100343 && g++ *.cpp -Wall -O2 -shared -o libMarvinSDK.dll -lws2_32 -lwinmm -DCMPL_WIN && cd ..
cd kinematicsSDK && g++ *.cpp -Wall -O2 -fPIC -shared -o libKine.dll && cd ..
copy /Y contrlSDK100343\libMarvinSDK.dll "DEMO_C++\" >nul
copy /Y kinematicsSDK\libKine.dll "DEMO_C++\" >nul
echo ✓ c++ finished


echo compile SDK_PYTHON...
del /F /Q contrlSDK100343\libMarvinSDK.dll 2>nul
del /F /Q kinematicsSDK\libKine.dll 2>nul
cd contrlSDK100343 && g++ *.cpp -Wall -O2 -shared -o libMarvinSDK.dll -DBUILDING_DLL -D_WIN32 -DCMPL_WIN -fPIC -static -static-libgcc -static-libstdc++ -lws2_32 -lwinmm && cd ..
cd kinematicsSDK && g++ *.cpp -Wall -O2 -shared -o libKine.dll -DBUILDING_DLL -D_WIN32 -fPIC -static -static-libgcc -static-libstdc++ -lws2_32 -lwinmm && cd ..
copy /Y contrlSDK100343\libMarvinSDK.dll SDK_PYTHON\ >nul
copy /Y kinematicsSDK\libKine.dll SDK_PYTHON\ >nul
echo ✓ Python SDK finished

echo compile interferenceCheck SDK (dll) ...
del /F /Q interferenceCheck\Interf\libInterfCheck.dll 2>nul

echo [1/2] building libInterfCheck.dll for DEMO_C++ ...
cd interferenceCheck\Interf && g++ *.cpp ..\FxCfg\CfgBase.cpp -Wall -O2 -fPIC -shared -o libInterfCheck.dll -DCMPL_WIN && cd ..\..
if errorlevel 1 (
    echo [X] DEMO_C++ dll build failed.
    pause
    exit /b 1
)
if exist "DEMO_C++\" copy /Y interferenceCheck\Interf\libInterfCheck.dll "DEMO_C++\" >nul

echo [2/2] building libInterfCheck.dll for SDK_PYTHON ...
del /F /Q interferenceCheck\Interf\libInterfCheck.dll 2>nul
cd interferenceCheck\Interf && g++ *.cpp ..\FxCfg\CfgBase.cpp  -Wall -O2 -shared -o libInterfCheck.dll -DBUILDING_DLL -D_WIN32 -DCMPL_WIN -fPIC -static -static-libgcc -static-libstdc++ && cd ..\..
if errorlevel 1 (
    echo [X] SDK_PYTHON dll build failed.
    pause
    exit /b 1
)
if exist "SDK_PYTHON\" copy /Y interferenceCheck\Interf\libInterfCheck.dll "SDK_PYTHON\" >nul
echo ✓ interferenceCheck dll finished

echo.
dir "DEMO_C++\*.dll" SDK_PYTHON\*.dll 2>nul
pause
