3 — Arduino: arm controller (Mega) — responsibilities

- run A4988 step & dir for 5 stepper joints
- read homing switches, perform homing/calibration
- expose JointState and accept joint angle targets via rosserial
- optionally perform microstepping config pins and current limiting via hardware
