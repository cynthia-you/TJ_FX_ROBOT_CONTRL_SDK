cmake_minimum_required(VERSION 3.8)
project(marvin_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ArmClearErr.srv"
  "srv/ArmSoftStop.srv"
  "srv/CollectData.srv"
  "srv/Connect.srv"
  "srv/DownloadLog.srv"
  "srv/GetPara.srv"
  "srv/SetPara.srv"
  "srv/GetServoErrCode.srv"
  "srv/ReceiveFile.srv"
  "srv/Release.srv"
  "srv/SaveCollectData.srv"
  "srv/SaveCollectDataAsCsv.srv"
  "srv/SaveParam.srv"
  "srv/SdkVersion.srv"
  "srv/SetForceCmd.srv"
  "srv/SetForceCtrlPara.srv"
  "srv/SendFile.srv"
  "srv/SendPvt.srv"
  "srv/SetArmState.srv"
  "srv/SetArmVelAcc.srv"
  "srv/SetCardKD.srv"
  "srv/SetDragSpace.srv"
  "srv/SetImpedanceType.srv"
  "srv/SetJointCmdPos.srv"
  "srv/SetJointKD.srv"
  "srv/SetPvtId.srv"
  "srv/SetToolPara.srv"
  "srv/StopCollectData.srv"
  "srv/UpdateSdk.srv"
  "srv/InitialKine.srv"
  "srv/FK.srv"
  "srv/IK.srv"
  "srv/IkNsp.srv"
  "srv/IkRangeCross67.srv"
  "srv/JointsToJacobMatrix.srv"
  "srv/LogSwitch.srv"
  "srv/RobotData.srv"
  "msg/State.msg"
  "msg/Output.msg"
  "msg/Input.msg"
  "msg/MarvinResponse.msg"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



ament_package()
