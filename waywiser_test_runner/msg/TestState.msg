# File: waywiser_test_runner/msg/TestState.msg

int8 IDLE = 0
int8 WAITING_FOR_VEHICLE_INIT = 1
int8 TEST_INIT = 2
int8 CONFIG_LOADED = 3
int8 SETUP_INIT = 4
int8 SETUP_ONGOING = 5
int8 SETUP_STOPPED = 6
int8 SETUP_COMPLETED = 7
int8 EXECUTION_INIT = 8
int8 EXECUTION_ONGOING = 9
int8 EXECUTION_STOPPED = 10
int8 EXECUTION_COMPLETED = 11

# Field to hold the current state
int8 state
