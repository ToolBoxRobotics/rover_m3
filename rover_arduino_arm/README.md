3 — Arduino: arm controller (Mega) — responsibilities

- run A4988 step & dir for 5 stepper joints
- read homing switches, perform homing/calibration
- expose JointState and accept joint angle targets via rosserial
- optionally perform microstepping config pins and current limiting via hardware

Notes:
The above does blocking step loops — fine for simple rigs. If you want non-blocking multi-joint coordinated moves, implement Bresenham / motion planner or use stepper libraries with interrupts.
Implement homing routine triggered by a ROS service or message: move until homePins[i] reads LOW, set step count to zero.

