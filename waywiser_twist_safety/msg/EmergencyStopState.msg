# File: waywiser_twist_safety/msg/EmergencyStopState.msg

uint8 UNKNOWN = 0
uint8 CLEAR = 1
uint8 ACTIVE = 2

string sender_id
builtin_interfaces/Time stamp
uint8 state
string reason
