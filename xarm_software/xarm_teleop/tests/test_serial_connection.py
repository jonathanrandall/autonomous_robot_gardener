import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))

import pytest

from xarm_remote_control.bus_servo_serial import BusServoSerial, print_open_ports


def test_list_all_serial_connections():
    print_open_ports()

def test_init_remote_control():
    with pytest.raises(Exception):
        BusServoSerial('/dev/wrong/port')
    BusServoSerial('/dev/ttyUSB0')

def test_read_position():
    servo = BusServoSerial()
    id = 1
    position = servo.get_position(id=id)
    assert int(position) > 0

def test_get_positions():
    servo = BusServoSerial()
    servo_ids = [1, 2, 3, 4, 5, 6]
    positions = servo.get_positions()
    for n, id in enumerate(servo_ids):
        position = servo.get_position(id=id)
        assert abs(position == positions[n]) < 5

def test_set_positions():
    servo = BusServoSerial()
    positions = servo.get_positions()
    if positions[0] > 350:
        positions[0] = 100
    else:
        positions[0] = 550
    servo.set_positions(positions, 100)


def test_close_or_open_gripper():
    servo = BusServoSerial()
    id = 1
    position = int(servo.get_position(id=id))
    if position > 350:
        new_position = 100
    else:
        new_position = 550
    servo.run(id,new_position,10)

def test_default_position():
    l =  [125, 0, 0, 0, 0, 0]
    servo = BusServoSerial()
    for n, v in  enumerate(l):
        angel = v
        angel_factor = 1000/ 240
        cmnd = int(angel * angel_factor) + 500
        print(n+1,cmnd)
        servo.run(n+1, cmnd, 500)

def test_default_position1():
    l = [120, 500, 500, 500, 500, 500]
    servo = BusServoSerial()
    for n, v in  enumerate(l):
        servo.run(n+1, v, 500)

# test_list_all_serial_connections()
test_init_remote_control()
# test_read_position()
# test_set_positions()
# test_close_or_open_gripper()
# test_default_position()
# test_default_position1()
# test_get_positions()
# test_default_position1()
