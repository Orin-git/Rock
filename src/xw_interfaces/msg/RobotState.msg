# Latched global snapshot for clients
builtin_interfaces/Time stamp
# Modes: 0 IDLE 1 MAPPING 2 NAVIGATING 3 FOLLOWING 4 FALL_DETECT
uint8 mode
string mode_name
# run_mode: 0 production 1 developer
uint8 run_mode
bool emergency_stop  # latched from /xw/chassis/motor_disabled (MCU Flag_Stop)
bool safety_ok
bool localization_ok
string active_map
string profile
xw_interfaces/PowerState power
string detail
