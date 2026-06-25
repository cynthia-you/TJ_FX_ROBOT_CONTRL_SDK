Test Configuration:
Robot: Old Luna
IP: 6.6.7.190
SDK version: 00040402
Controller version: 00040405
Platform: Win11
Language: Python


Run with one KD on arm A(left)
python TEST_SCRIPT/CARTESIAN_CIRCLE_GentoArmOnlyPD_200hz_IK.py --arm A  --tag default_kd

Run a group of test sweeping different kds
python TEST_SCRIPT/CARTESIAN_CIRCLE_GentoArmOnlyPD_200hz_IK.py --arm A  --sweep_kd --tag kd_sweep