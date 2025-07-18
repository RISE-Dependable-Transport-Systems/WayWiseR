# File: waywiser_test_runner/msg/SetupState.msg

int8 IDLE = 0
int8 SETUP_INIT = 4
int8 SETUP_ONGOING = 5
int8 SETUP_STOPPED = 6
int8 SETUP_COMPLETED = 7

# Field to hold the current state
int8 state
