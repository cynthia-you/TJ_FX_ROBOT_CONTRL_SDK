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


If you'd like to run in Linux/WSL,
recommended form:

```bash
python3 TEST_SCRIPT/CARTESIAN_CIRCLE_GentoArmOnlyPD_200hz_IK_cross_platform.py \
  --arm A \
  --tag default_kd_linux \
  --linux_timer_slack_ns 50000 \
  --cpu_affinity 2 \
  --linux_rt_priority 0 \
  --spin_threshold_s 0.0002
```

Sweep mode:

```bash
python3 TEST_SCRIPT/CARTESIAN_CIRCLE_GentoArmOnlyPD_200hz_IK_cross_platform.py \
  --arm A \
  --sweep_kd \
  --tag kd_sweep_linux \
  --linux_timer_slack_ns 50000 \
  --cpu_affinity 2 \
  --linux_rt_priority 0 \
  --spin_threshold_s 0.0002 \
  --continue_on_trial_failure
```

If you want to request Linux real-time scheduling, `--linux_rt_priority` normally requires `sudo` or `CAP_SYS_NICE`:

```bash
sudo -E python3 TEST_SCRIPT/CARTESIAN_CIRCLE_GentoArmOnlyPD_200hz_IK_cross_platform.py \
  --arm A \
  --tag default_kd_linux_rt \
  --linux_timer_slack_ns 50000 \
  --cpu_affinity 2 \
  --linux_rt_priority 50 \
  --spin_threshold_s 0.0002
```
