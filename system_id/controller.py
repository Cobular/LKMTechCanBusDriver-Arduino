import serial

class ArduinoCANController:
    def __init__(self, port: str, baudrate: int = 115200):
        self.serial_connection = serial.Serial(port, baudrate, timeout=1)

    def send_command(self, command: str):
        if not command.endswith('\n'):
            command += '\n'
        self.serial_connection.write(command.encode())

    def stop_motor(self):
        self.send_command('STOP')

    def turn_motor_on(self):
        self.send_command('ON')

    def turn_motor_off(self):
        self.send_command('OFF')

    def set_motor_torque(self, torque_value: int):
        self.send_command(f'TORQUE{torque_value:02d}')

    def set_motor_speed(self, speed_value: int):
        self.send_command(f'SPEED{speed_value:08d}')

    def set_absolute_angle_deg(self, angle_value: int):
        self.send_command(f'ABS_ANGLE_DEG{angle_value:08d}')

    def set_relative_angle_cw(self, angle_value: int):
        self.send_command(f'REL_ANGLE_CW{angle_value:08d}')

    def set_relative_angle_ccw(self, angle_value: int):
        self.send_command(f'REL_ANGLE_CCW{angle_value:08d}')

    def close(self):
        self.serial_connection.close()

# Example usage:
# controller = ArduinoCANController('/dev/ttyUSB0')
# controller.turn_motor_on()
# controller.set_motor_speed(12345678)
# controller.close()