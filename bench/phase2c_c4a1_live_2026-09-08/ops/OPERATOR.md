# C4A.1 Operator actions (required for matrix close)
BOOT A: real dock; charging or docked true; touch PLACE_BOOT_A_CHARGING
BOOT D: place similar_corridor ~(-8.94, 1.60); touch PLACE_BOOT_D_SIMILAR
BOOT E: place open ~(-0.76, 9.21); touch PLACE_BOOT_E_OPEN
LOST B: Follow at known-good pose; induce LOST without /initialpose; touch PLACE_LOST_B_INDUCED
LOST C: manual carry; touch PLACE_LOST_C_CARRY_DONE
Script: python3 /ros2_ws/src/xw_phase2c/scripts/phase2c_c4a1_live_matrix.py
