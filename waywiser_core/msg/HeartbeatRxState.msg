builtin_interfaces/Time stamp
builtin_interfaces/Time last_rx_stamp

uint8 DISABLED = 0
uint8 NO_HEARTBEAT = 1
uint8 ACTIVE = 2
uint8 STALE = 3
uint8 TIMEOUT = 4

uint8 state
string state_str
float32 age_s
float32 timeout_s
uint32 received_count
