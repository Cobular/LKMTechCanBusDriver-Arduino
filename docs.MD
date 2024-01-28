# CAN Controller

## Serial Commands

Send these commands over serial to control the motor:

1. STOP: Stops the motor. No parameters required.
2. ON: Turns the motor on. No parameters required.
3. OFF: Turns the motor off. No parameters required.
4. TORQUE: Runs the motor with a set torque. Takes one parameter:
- TORQUE<torque_value>: The torque value should be a 2-digit integer.
5. SPEED: Runs the motor with a set speed. Takes one parameter:
- SPEED<speed_value>: The speed value should be an 8-digit integer.
6. ABS_ANGLE_DEG: Sets the absolute angle of the motor. Takes one parameter:
- ABS_ANGLE_DEG<angle_value>: The angle value should be an 8-digit integer.
7. REL_ANGLE_CW: Sets the relative angle of the motor in the clockwise direction. Takes one parameter:
- REL_ANGLE_CW<angle_value>: The angle value should be an 8-digit integer.
8. REL_ANGLE_CCW: Sets the relative angle of the motor in the counter-clockwise direction. Takes one parameter:
- REL_ANGLE_CCW<angle_value>: The angle value should be an 8-digit integer.

The commands are case-sensitive and should be followed by a newline character (\n). The parameters should be directly appended to the command without any spaces. For example, to set the torque to 50, the command would be TORQUE50\n.