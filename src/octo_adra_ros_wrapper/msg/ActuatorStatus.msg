# TODO: Add enums for some of the variables

string[] uuid
string[] sw_version
string[] hw_version
string[] multi_version

float64[] mech_ratio
float64[] elec_ratio
uint8[] motion_dir

int8[] temp_limit_min
int8[] temp_limit_max
uint8[] volt_limit_min
uint8[] volt_limit_max
float64[] curr_limit

uint8[] motion_mode
uint8[] motion_enable
uint8[] brake_enable

float64[] temp_driver
float64[] temp_motor
float64[] bus_volt
float64[] bus_curr
float64[] multi_volt

uint8[] error_code

float64[] pos_limit_min
float64[] pos_limit_max
float64[] pos_limit_diff
float64[] vel_limit_min
float64[] vel_limit_max
float64[] vel_limit_diff
float64[] tau_limit_min
float64[] tau_limit_max
float64[] tau_limit_diff

float64[] pos_target
float64[] pos_current
float64[] vel_target
float64[] vel_current
float64[] tau_target
float64[] tau_current

float64[] pos_pidp
float64[] vel_pidp
float64[] vel_pidi
float64[] tau_pidp
float64[] tau_pidi

uint8[] pos_smooth_cyc
uint8[] vel_smooth_cyc
uint8[] tau_smooth_cyc