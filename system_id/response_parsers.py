from dataclasses import dataclass


@dataclass
class OpenLoopResponse:
    temp: int
    power: int
    speed: int
    pos: int

    @property
    def temp_celsius(self):
        return self.temp

    @property
    def temp_fahrenheit(self):
        return self.temp * 9 / 5 + 32

    @property
    def motor_speed_dps(self):
        return self.speed

class ClosedLoopResponse:
    __temp: int
    __current: int
    __speed: int
    __pos: int

    def __init__(self, temp: int, current: int, speed: int, pos: int) -> None:
        self.__temp = temp
        self.__current = current
        self.__speed = speed
        self.__pos = pos

    @property
    def temp_celsius(self):
        return self.__temp

    @property
    def temp_fahrenheit(self):
        return self.__temp * 9 / 5 + 32

    @property
    def motor_current(self):
        """The value internally is:

        Torque current value iq,int16_t, range is -2048~2048,
          corresponding MG motor actual torque current range is-33A~33A.
        """

        return self.__current / 62.0606060606
