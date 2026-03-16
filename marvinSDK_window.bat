@echo off
chcp 65001 >nul
echo complie c++ SDK for DEMO_C++...
cd contrlSDK && g++ *.cpp -Wall -w -O2 -shared -o libMarvinSDK.dll -lws2_32 -lwinmm -DCMPL_WIN && cd ..
cd kinematicsSDK && g++ *.cpp -Wall -w -O2 -fPIC -shared -o libKine.dll && cd ..
copy /Y contrlSDK\libMarvinSDK.dll "DEMO_C++\" >nul
copy /Y kinematicsSDK\libKine.dll "DEMO_C++\" >nul
echo ✓ c++ finished

echo complie SDK_PYTHON...
del /F /Q contrlSDK\libMarvinSDK.dll 2>nul
del /F /Q kinematicsSDK\libKine.dll 2>nul
cd contrlSDK && g++ *.cpp -Wall -w -O2 -shared -o libMarvinSDK.dll -DBUILDING_DLL -D_WIN32 -DCMPL_WIN -fPIC -static -static-libgcc -static-libstdc++ -lws2_32 -lwinmm && cd ..
cd kinematicsSDK && g++ *.cpp -Wall -w -O2 -shared -o libKine.dll -DBUILDING_DLL -D_WIN32 -fPIC -static -static-libgcc -static-libstdc++ -lws2_32 -lwinmm && cd ..
copy /Y contrlSDK\libMarvinSDK.dll SDK_PYTHON\ >nul
copy /Y kinematicsSDK\libKine.dll SDK_PYTHON\ >nul
echo ✓ Python SDK finished

echo.
dir "DEMO_C++\*.dll" SDK_PYTHON\*.dll 2>nul
pause