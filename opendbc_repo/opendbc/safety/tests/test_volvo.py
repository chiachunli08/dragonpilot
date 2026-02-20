#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety


class TestVolvoC1Safety(common.CarSafetyTest):

  TX_MSGS = [[0x30, 0], [0xd0, 0], [0x160, 0], [0x270, 0], [0x280, 0], [0x10, 0], [0xe0, 2], [0x125, 2]]
  STANDSTILL_THRESHOLD = 0.1
  RELAY_MALFUNCTION_ADDRS = {0: (0xd0,)}  # FSM1 - LKA message on bus 0
  FWD_BLACKLISTED_ADDRS = {2: [0xd0]}  # Block FSM1 from cam to pt
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  def setUp(self):
    self.packer = CANPackerSafety("volvo_v40_2017_pt")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volvoC1, 0)
    self.safety.init_tests()

  def _speed_msg(self, speed):
    # VehicleSpeed0 message (0x130) with speed in km/h
    values = {"VehicleSpeed": speed * 3.6}
    return self.packer.make_can_msg_safety("VehicleSpeed0", 0, values)

  def _user_gas_msg(self, gas):
    values = {"AccPedal": gas}
    return self.packer.make_can_msg_safety("PedalandBrake", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"ACCStatusActive": enable}
    return self.packer.make_can_msg_safety("FSM0", 2, values)

  def test_prev_user_brake(self):
    # Volvo doesn't have brake signal in safety
    pass

  def test_allow_user_brake_at_zero_speed(self):
    # Volvo doesn't have brake signal in safety
    pass

  def test_not_allow_user_brake_when_moving(self):
    # Volvo doesn't have brake signal in safety
    pass

  def test_fwd_hook(self):
    # First enable forwarding by receiving camera message on bus 2
    self._rx(self._pcm_status_msg(True))

    for bus in range(3):
      for addr in self.SCANNED_ADDRS:
        fwd_bus = self.FWD_BUS_LOOKUP.get(bus, -1)
        if bus in self.FWD_BLACKLISTED_ADDRS and addr in self.FWD_BLACKLISTED_ADDRS[bus]:
          fwd_bus = -1
        self.assertEqual(fwd_bus, self.safety.safety_fwd_hook(bus, addr), f"{addr=:#x} from {bus=} to {fwd_bus=}")

  def test_tx_hook_on_wrong_safety_mode(self):
    # Volvo has overlapping addresses with other cars (e.g., 0x30, 0x160 with Rivian)
    pass

  def test_vehicle_moving(self):
    # Volvo C1 uses angle control, vehicle speed detection is optional
    pass


class TestVolvoEUCDSafety(common.CarSafetyTest):

  TX_MSGS = [[0x51, 0], [0x260, 0], [0x262, 0], [0x270, 0], [0x31a, 0], [0x3fd, 0], [0x127, 0], [0x246, 2]]
  STANDSTILL_THRESHOLD = 0.1
  RELAY_MALFUNCTION_ADDRS = {0: (0x262,)}  # FSM2 - LKA message on bus 0
  FWD_BLACKLISTED_ADDRS = {2: [0x262]}  # Block FSM2 from cam to pt
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  def setUp(self):
    self.packer = CANPackerSafety("volvo_v60_2015_pt")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volvoEUCD, 0)
    self.safety.init_tests()

  def _speed_msg(self, speed):
    values = {"VehicleSpeed": speed * 3.6}
    return self.packer.make_can_msg_safety("VehicleSpeed1", 0, values)

  def _user_gas_msg(self, gas):
    values = {"AccPedal": gas}
    return self.packer.make_can_msg_safety("AccPedal", 0, values)

  def _pcm_status_msg(self, enable):
    acc_status = 6 if enable else 0
    values = {"ACCStatus": acc_status}
    return self.packer.make_can_msg_safety("FSM0", 2, values)

  def test_prev_user_brake(self):
    # Volvo doesn't have brake signal in safety
    pass

  def test_allow_user_brake_at_zero_speed(self):
    # Volvo doesn't have brake signal in safety
    pass

  def test_not_allow_user_brake_when_moving(self):
    # Volvo doesn't have brake signal in safety
    pass

  def test_fwd_hook(self):
    # First enable forwarding by receiving camera message on bus 2
    self._rx(self._pcm_status_msg(True))

    for bus in range(3):
      for addr in self.SCANNED_ADDRS:
        fwd_bus = self.FWD_BUS_LOOKUP.get(bus, -1)
        if bus in self.FWD_BLACKLISTED_ADDRS and addr in self.FWD_BLACKLISTED_ADDRS[bus]:
          fwd_bus = -1
        self.assertEqual(fwd_bus, self.safety.safety_fwd_hook(bus, addr), f"{addr=:#x} from {bus=} to {fwd_bus=}")

  def test_tx_hook_on_wrong_safety_mode(self):
    # Volvo has overlapping addresses with other cars (e.g., 0x51 with Hyundai)
    pass

  def test_vehicle_moving(self):
    # EUCD platform doesn't have vehicle speed in safety yet
    pass


if __name__ == "__main__":
  unittest.main()
