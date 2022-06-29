#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys
import logging
import math
import time
import enum
import struct
import board
import RPi.GPIO as GPIO

logger = logging.getLogger()
FORMAT = '%(levelname)s %(asctime)s [%(filename)s:%(lineno)d]: %(message)s'
logging.basicConfig(format=FORMAT, stream=sys.stdout, level=logging.DEBUG)
logger = logging.getLogger()


class Pulse:
    def __init__(self, min=750, max=2250):
        self.min = min
        self.max = max

    def __repr__(self):
        return str(self.__dict__)


class Duty:
    def __init__(self, pulse=Pulse(), frequency=50):
        self.frequency = frequency
        self.min = self._from_pulse_(pulse.min)
        self.max = self._from_pulse_(pulse.max)
        self.scale = self.max - self.min

    # 占空比
    def cycle(self, value):
        return self.min + int(value * self.scale)

    def _from_pulse_(self, pulse_value):
        return int((pulse_value * self.frequency) / 1000000 * 0xFFFF)

    def __repr__(self):
        return str(self.__dict__)


class Register:
    def __init__(self, i2c, device_address, register_address, struct_format, is_array=False, index=0):
        self.i2c = i2c
        self.device_address = device_address
        self.struct_format = struct_format
        self.register_address = register_address
        self.index = index
        self.is_array = is_array
        self.buffer = self._create_buffer_()

    def write(self, value):
        if self.is_array:
            struct.pack_into(self.struct_format, self.buffer, 1, *value)
        else:
            struct.pack_into(self.struct_format, self.buffer, 1, value)
        logger.debug(f'{self.register_address} write {self.buffer}')
        self.i2c.writeto(self.device_address, self.buffer)

    def read(self):
        buf = self._create_buffer_()
        self.i2c.writeto_then_readfrom(
            self.device_address, buf, buf, out_start=0, out_end=1, in_start=1, in_end=None)
        return struct.unpack_from(self.struct_format, buf, 1)[0]

    def _create_buffer_(self):
        size = struct.calcsize(self.struct_format)
        buffer = bytearray(1 + size)
        if self.is_array:
            buffer[0] = self.register_address + size * self.index
        else:
            buffer[0] = self.register_address
        return buffer

    def __repr__(self):
        return str(self.__dict__)


class Engine:
    def __init__(self, device_address=0x40, clock_speed=25000000, frequency=50):
        GPIO.cleanup()
        print(GPIO.JETSON_INFO)
        self.i2c = board.I2C()
        self.device_address = device_address
        self.clock_speed = clock_speed
        self.frequency = frequency
        self.prescale_value = math.floor(
            self.clock_speed / 4096.0 / self.frequency + 0.5)
        self._alive_()

    def _alive_(self):
        while not self.i2c.try_lock():
            pass
        try:
            self.i2c.writeto(self.device_address, b'')
        except OSError:
            try:
                result = bytearray(1)
                self.i2c.readfrom_into(self.device_address, result)
                logger.debug(f'probe device result: {result}')
            except OSError:
                raise ValueError("No I2C device at address: 0x%x" %
                                 self.device_address)
        finally:
            self.i2c.unlock()

    def start(self):
        mode_reg = Register(self.i2c, self.device_address, 0x00, '<B')
        prescale_reg = Register(self.i2c, self.device_address, 0xFE, '<B')
        mode_val = mode_reg.read()
        mode_reg.write(0x00)
        mode_reg.write((mode_val & 0x7F) | 0x10)
        prescale_reg.write(math.floor(self.clock_speed /
                           4096.0 / self.frequency + 0.5))
        mode_reg.write(mode_val)
        time.sleep(0.005)
        mode_reg.write(mode_val | 0xA0)

    def __repr__(self):
        return str(self.__dict__)


class Contoller:
    def __init__(self, engine, index=0):
        self.engine = engine
        self.duty = Duty()
        self.pwm_reg = Register(
            engine.i2c, engine.device_address, 0x06, '<HH', True, index)

    def _send_(self, value):
        cycle = self.duty.cycle(value)
        cycle = (cycle + 1) >> 4
        self.pwm_reg.write((0, cycle))


class Steering(Contoller):
    def __init__(self, engine):
        super().__init__(engine)

    def left(self, angle):
        if angle < 0 or angle > 90:
            raise ValueError("Angle must be between 0 and 90")
        angle += 90.0
        self._turn_(angle)

    def right(self, angle):
        if angle < 0 or angle > 90:
            raise ValueError("Angle must be between 0 and 90")
        self._turn_(angle)

    def neutral(self):
        self._turn_(90.0)

    def _turn_(self, angle):
        self._send_(angle / 180.0)


class Pedal(Contoller):
    def __init__(self, engine):
        super().__init__(engine, 1)

    def forward(self, force=0):
        if force < 0 or force > 1.0:
            raise ValueError('pedal force must be between 0.0 and 1.0')
        force += 0.006
        self._pedal_(force)

    def backward(self, force=0):
        if force < 0 or force > 1.0:
            raise ValueError('pedal force must be between 0.0 and 1.0')
        force = -force - 0.13
        self._pedal_(force)

    def brake(self):
        self._pedal_(0.0)

    def _pedal_(self, force):
        self._send_((force + 1.0) * 0.5)


class Direction(enum.Enum):
    LEFT = 'left'
    RIGHT = 'right'
    MIDDLE = 'middle'


class Vehicle:
    def __init__(self):
        self.engine = Engine()
        self.steering = Steering(self.engine)
        self.pedal = Pedal(self.engine)

    def start(self):
        self.engine.start()

    def turn(self, direction, angle, speed=0):
        self.pedal.forward(speed)
        if direction == Direction.LEFT:
            self.steering.left(angle)
        if direction == Direction.RIGHT:
            self.steering.right(angle)

    def straight(self, speed=0):
        self.steering.neutral()
        self.pedal.forward(speed)

    def stop(self):
        self.steering.neutral()
        self.pedal.brake()

    def reverse(self, direction, angle, speed=0):
        self.pedal.backward(speed)
        if direction == Direction.LEFT:
            self.steering.left(angle)
        if direction == Direction.RIGHT:
            self.steering.right(angle)
