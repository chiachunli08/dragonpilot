#!/usr/bin/env python3
import unittest
import numpy as np
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common

class TestVolvoSafety(common.CarSafetyTest, common.AngleSteeringSafetyTest):
  ARG_SPEED = 0x130
  ARG_BUTTONS = 0x010

  # Volvo constants
  DEG_TO_CAN = 1.0 / 0.04395
  MAX_ANGLE = 8189
  MIN_ANGLE = -8190

  ANGLE_RATE_BP = [7., 17., 36.]
  ANGLE_RATE_UP = [2., 0.25, 0.1]
  ANGLE_RATE_DOWN = [2., 0.25, 0.1]

  MAX_RATE_UP = 2.0
  MAX_RATE_DOWN = 2.0

  ANGLE_MEAS_TOLERANCE = 20.0

  TX_MSGS = [[0x0d0, 0], [0x010, 0], [0x280, 0]]

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volvoC1, 0)
    self.safety.init_tests()
    # Enable giraffe forwarding by simulating camera message
    self._rx(self._pcm_status_msg(False))

  def _angle_meas_msg(self, angle):
    raw = int(angle * self.DEG_TO_CAN) + 32768
    data = bytearray(8)
    data[5] = (raw >> 8) & 0xFF
    data[6] = raw & 0xFF
    return libsafety_py.make_CANPacket(0x125, 0, bytes(data))

  def _angle_cmd_msg(self, angle, enabled):
    raw = int(angle * self.DEG_TO_CAN) + 8192
    data = bytearray(8)
    data[4] = (raw >> 8) & 0x3F
    data[5] = raw & 0xFF
    if enabled:
      data[7] = 0x01
    else:
      data[7] = 0x00
    return libsafety_py.make_CANPacket(0x0d0, 0, bytes(data))

  def _speed_msg(self, speed):
    value = int(speed * 3.6 / 0.01)
    data = bytearray(8)
    data[3] = (value >> 8) & 0xFF
    data[4] = value & 0xFF
    return libsafety_py.make_CANPacket(0x130, 0, bytes(data))

  def _user_brake_msg(self, brake):
    return libsafety_py.make_CANPacket(0x0, 0, b'\x00'*8)

  def _user_gas_msg(self, gas):
    val = 100 if gas else 0
    data = bytearray(8)
    data[1] = (val >> 8) & 0x03
    data[2] = val & 0xFF
    return libsafety_py.make_CANPacket(0x055, 0, bytes(data))

  def _pcm_status_msg(self, enable):
    data = bytearray(8)
    if enable:
      data[7] = 0x04
    return libsafety_py.make_CANPacket(0x030, 2, bytes(data))

  def test_acc_buttons(self):
    self.safety.set_controls_allowed(0)
    # Cancel button (allowed)
    data = bytearray(8)
    data[7] = 0x10
    self.assertTrue(self._tx(libsafety_py.make_CANPacket(0x010, 0, bytes(data))))
    # Other button
    data = bytearray(8)
    data[7] = 0x01
    self.assertFalse(self._tx(libsafety_py.make_CANPacket(0x010, 0, bytes(data))))
    self.safety.set_controls_allowed(1)
    data = bytearray(8)
    data[7] = 0x01
    self.assertTrue(self._tx(libsafety_py.make_CANPacket(0x010, 0, bytes(data))))

  def test_angle_cmd_offset_limit(self):
    self.safety.set_controls_allowed(1)
    self._rx(self._angle_meas_msg(0))

    current_angle = 0.0
    for i in range(15):
      current_angle += 1.9
      msg = self._angle_cmd_msg(current_angle, True)
      allowed = self._tx(msg)

      if current_angle <= self.ANGLE_MEAS_TOLERANCE:
        self.assertTrue(allowed, f"Step {i}, angle {current_angle} blocked! Should be allowed.")
      else:
        self.assertFalse(allowed, f"Step {i}, angle {current_angle} allowed! Should be blocked by offset limit.")

  def test_angle_cmd_last_reset(self):
    self.safety.set_controls_allowed(1)
    self._rx(self._angle_meas_msg(0))
    self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))

    self.safety.set_controls_allowed(0)
    self._rx(self._angle_meas_msg(20))
    # Send disabled command to trigger hook
    self._tx(self._angle_cmd_msg(0, False))

    self.safety.set_controls_allowed(1)
    msg = self._angle_cmd_msg(20, True)
    allowed = self._tx(msg)
    self.assertTrue(allowed, "Engaging at current angle blocked! desired_angle_last not reset.")

if __name__ == "__main__":
  unittest.main()
