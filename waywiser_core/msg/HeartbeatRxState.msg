builtin_interfaces/Time stamp
# Generation timestamp copied from the most recently accepted heartbeat Header.
builtin_interfaces/Time last_heartbeat_stamp

uint8 NO_HEARTBEAT = 0
uint8 ACTIVE = 1
uint8 TIMEOUT = 2

uint8 state
# Seconds elapsed since last_heartbeat_stamp; -1 when no stamped heartbeat was accepted.
float32 age_s
