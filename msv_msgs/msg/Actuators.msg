# Reports the state of the msv_01 actuators (traction system for now).
Header header           # timestamp in the header is the time the data is received from the joystick
float32[] values        # the msv_01 actuators speeds
bool[] alarms           # the msv_01 main actuators states
