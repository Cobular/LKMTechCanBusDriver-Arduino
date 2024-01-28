
#ifndef __LKMTECH_MOTOR_CONTROLLER_H__
#define __LKMTECH_MOTOR_CONTROLLER_H__

#include <Adafruit_MCP2515.h>

typedef void (*ResponseParser)(uint8_t*);

class LKMTechMotorController {
public:
  LKMTechMotorController();
  LKMTechMotorController(Adafruit_MCP2515 *mcp);

  void initalize(Adafruit_MCP2515 *mcp);

  /**
 * Sends a command to switch the motor from the on state to the off state.
 * Clears motor turns and earlier commands. The motor LED changes from always on to slow flashing.
 * The motor can still reply to commands but will not perform actions.
 *
 * @param id The unique identifier for the motor.
 */
  ResponseParser sendMotorOff(int id);

  /**
 * Sends a command to switch the motor from the off state to the on state.
 * The motor LED changes from slow flashing to always on. The motor is then ready to receive and execute commands.
 *
 * @param id The unique identifier for the motor.
 */
  ResponseParser sendMotorOn(int id);

  /**
 * Sends a command to stop the motor but does not clear the motor state.
 * The motor can receive and execute new commands following this command.
 *
 * @param id The unique identifier for the motor.
 */
  ResponseParser sendMotorStop(int id);

  /**
 * Sends a command for open loop control, applicable to MS series motors.
 * This command controls the open-loop voltage, and the PowerControl value determines the motor current and torque.
 *
 * @param id The unique identifier for the motor.
 * @param powerControl The power control value, ranging from -850 to 850.
 */
  ResponseParser sendOpenLoopControl(int id, int16_t powerControl);

  /**
 * Sends a command for torque closed loop control, applicable to MFMHMG series motors.
 * This command controls the torque current output (iqControl), affecting the actual torque of the motor.
 *
 * @param id The unique identifier for the motor.
 * @param iqControl The torque current control value, ranging from -2048 to 2048.
 */
  ResponseParser sendTorqueClosedLoopControl(int id, int16_t iqControl);

  /**
 * Sends a command for speed closed loop control.
 * This command controls the motor speed, and the SpeedControl value specifies the desired speed.
 *
 * @param id The unique identifier for the motor.
 * @param speedControl The speed control value as an int32_t.
 */
  ResponseParser sendSpeedClosedLoopControl(int id, int32_t speedControl);

  /**
 * Sends a command for multi-loop angle control (command 1).
 * This command sets the motor to a specific angle in a multi-turn scenario.
 *
 * @param id The unique identifier for the motor.
 * @param angleControl The angle control value as an int32_t.
 */
  ResponseParser sendAbsoluteAngleControl(int id, int32_t angleControl);

  /**
 * Sends a command for multi-loop angle control (command 2).
 * This command sets the motor to a specific angle in a multi-turn scenario with a maximum speed limit.
 *
 * @param id The unique identifier for the motor.
 * @param angleControl The angle control value as an int32_t.
 * @param maxSpeed The maximum speed limit as a uint16_t.
 */
  ResponseParser sendAbsoluteAngleControlSpeedLimit(int id, int32_t angleControl, uint16_t maxSpeed);

  /**
 * Sends a command for single-loop angle control (command 1).
 * This command sets the motor to a specific angle in a single-turn scenario.
 *
 * @param id The unique identifier for the motor.
 * @param spinDirection The spin direction (0 for clockwise, 1 for counterclockwise).
 * @param angleControl The angle control value as an int32_t.
 */
  ResponseParser sendRelativeAngleControl(int id, uint8_t spinDirection, int32_t angleControl);

  /**
 * Sends a command for single-loop angle control (command 2).
 * This command sets the motor to a specific angle in a single-turn scenario with a maximum speed limit.
 *
 * @param id The unique identifier for the motor.
 * @param spinDirection The spin direction (0 for clockwise, 1 for counterclockwise).
 * @param angleControl The angle control value as an int32_t.
 * @param maxSpeed The maximum speed limit as a uint16_t.
 */
  ResponseParser sendRelativeAngleControlSpeedLimit(int id, uint8_t spinDirection, int32_t angleControl, uint16_t maxSpeed);

  /**
 * Sends a command for increment angle control (command 1).
 * This command controls the incremental angle of the motor.
 *
 * @param id The unique identifier for the motor.
 * @param angleIncrement The angle increment value as an int32_t.
 */
  ResponseParser sendIncrementAngleControl1(int id, int32_t angleIncrement);

  /**
 * Sends a command for increment angle control (command 2).
 * This command controls the incremental angle of the motor with a maximum speed limit.
 *
 * @param id The unique identifier for the motor.
 * @param angleIncrement The angle increment value as an int32_t.
 * @param maxSpeed The maximum speed limit as a uint16_t.
 */
  ResponseParser sendIncrementAngleControl2(int id, int32_t angleIncrement, uint16_t maxSpeed);

  /**
 * Sends a command to read the current PID parameters of the motor.
 * The response includes the PID parameters for position, speed, and torque loops.
 *
 * @param id The unique identifier for the motor.
 */
  ResponseParser sendReadPIDParameter(int id);

  /**
 * Sends a command to write PID parameters to the motor's RAM.
 * These parameters are temporary and will be lost upon power off.
 *
 * @param id The unique identifier for the motor.
 * @param anglePidKp The position loop P parameter.
 * @param anglePidKi The position loop I parameter.
 * @param speedPidKp The speed loop P parameter.
 * @param speedPidKi The speed loop I parameter.
 * @param iqPidKp The torque loop P parameter.
 * @param iqPidKi The torque loop I parameter.
 */
  ResponseParser writePIDParamsToRAM(int id, uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi);

  /**
 * Sends a command to write PID parameters to the motor's ROM.
 * These parameters are permanent and will be retained upon power off.
 *
 * @param id The unique identifier for the motor.
 * @param anglePidKp The position loop P parameter.
 * @param anglePidKi The position loop I parameter.
 * @param speedPidKp The speed loop P parameter.
 * @param speedPidKi The speed loop I parameter.
 * @param iqPidKp The torque loop P parameter.
 * @param iqPidKi The torque loop I parameter.
 */
  ResponseParser writePIDParamsToROM(int id, uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi);

  /**
 * Sends a command to read the current acceleration parameter of the motor.
 * The response includes the acceleration data as an int32_t, with units of 1 dps/s.
 *
 * @param id The unique identifier for the motor.
 */
  ResponseParser readAcceleration(int id);

  /**
 * Sends a command to write the acceleration parameter to the motor's RAM.
 * These settings are temporary and will be lost upon power off.
 *
 * @param id The unique identifier for the motor.
 * @param Accel The acceleration value as an int32_t, with units of 1 dps/s.
 */
  ResponseParser writeAccelerationToRAM(int id, int32_t Accel);

  /**
 * Sends a command to read the current encoder position of the motor.
 * The response includes the encoder value, raw encoder position, and encoder offset.
 *
 * @param id The unique identifier for the motor.
 */
  ResponseParser readEncoder(int id);

  /**
 * Sends a command to set the encoder offset to the motor's ROM.
 * This command sets the current encoder value as the motor's zero point.
 *
 * @param id The unique identifier for the motor.
 * @param encoderOffset The encoder offset as a uint16_t.
 */
  ResponseParser writeEncoderValToROM(int id, uint16_t encoderOffset);

  /**
 * Sends a command to write the motor's current encoder position to ROM as the initial position.
 * This command sets the current position as the zero point and is valid only after a reset.
 *
 * @param id The unique identifier for the motor.
 */
  ResponseParser writeCurrentPositionToROM(int id);

  /**
 * Sends a command to read the current motor multi-angle absolute value.
 * The response includes the motor angle as an int64_t, with positive values for clockwise and negative for counterclockwise.
 *
 * @param id The unique identifier for the motor.
 */
  ResponseParser readMultiAngleLoop(int id);

  /**
 * Sends a command to read the motor's current single-angle value.
 * The response includes the single loop angle value as a uint32_t.
 *
 * @param id The unique identifier for the motor.
 */
  ResponseParser readSingleAngleLoop(int id);

  /**
 * Sends a command to clear the motor's multi-turn and single-turn data.
 * This command sets the current position as the motor's zero point but is invalid when power is off.
 *
 * @param id The unique identifier for the motor.
 */
  ResponseParser clearMotorAngleLoop(int id);

  /**
 * Sends a command to read the current motor state, including temperature, voltage, and error state.
 * The response includes details about the motor's temperature, voltage, and specific error states.
 *
 * @param id The unique identifier for the motor.
 */
  ResponseParser readMotorState1AndErrorState(int id);

  /**
 * Sends a command to clear the motor's current error state.
 * This command is effective only when the motor state returns to normal.
 *
 * @param id The unique identifier for the motor.
 */
  ResponseParser clearMotorErrorState(int id);

  /**
 * Sends a command to read the motor's current state, including temperature, voltage, speed, and encoder position.
 * The response includes detailed information about each parameter.
 *
 * @param id The unique identifier for the motor.
 */
  ResponseParser readMotorState2(int id);

  /**
 * Sends a command to read the motor's current state, including temperature and phase currents.
 * The response includes detailed information about the temperature and A, B, C phase currents.
 *
 * @param id The unique identifier for the motor.
 */
  ResponseParser readMotorState3(int id);

private:
  Adafruit_MCP2515 *mcp;
};

#endif
