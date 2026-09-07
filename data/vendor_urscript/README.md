# Vendor and community URScript examples (non-LLM comparator)

Third-party programs scored by `scripts/review/comparator_urscript.py` as a
reference point for the rule checker (manuscript revision item R2-1). They are
redistributed unchanged under their own licences:

| File | Source | Licence |
|------|--------|---------|
| admittance_control.script, joint_impedance_cartesian_circle_motion.script, dmgmori_full.script, dmgmori_simple.script, festo_io_library.script, festo_servo_library.script, read-digital.script | UniversalRobots/URScript_Examples, commit 5762d0e (2025-11-17) | BSD-3-Clause (LICENSE-BSD-3-Clause-UniversalRobots) |
| rparak_PickAndPlace_UR10e.script, rparak_Move_Circle.script | rparak/URScript_Examples | MIT (LICENSE-MIT-rparak) |

Only programs that contain motion commands carry information for the motion-related
checks; library-style files (Modbus, Ethernet/IP) are kept because the checker is
applied to whatever a model emits, including non-motion code.
