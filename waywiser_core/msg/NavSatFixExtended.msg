# File: waywiser_core/msg/NavSatFixExtended.msg

uint8 FIX_TYPE_NO_FIX=0
uint8 FIX_TYPE_DEAD_RECKONING_ONLY_FIX=1
uint8 FIX_TYPE_2D_FIX=2
uint8 FIX_TYPE_3D_FIX=3
uint8 FIX_TYPE_GNSS_DR_COMBINED_FIX=4
uint8 FIX_TYPE_TIME_ONLY_FIX=5

std_msgs/Header header
float64 latitude
float64 longitude
float64 altitude
float32 heading

float32 horizontal_accuracy
float32 vertical_accuracy
float32 heading_accuracy
uint8 last_rtcm_correction_age
uint8 num_satellites
uint8 fix_type
bool is_fused_on_chip