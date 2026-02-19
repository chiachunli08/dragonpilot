import numpy as np
from opendbc.car import Bus, rate_limit, structs
from opendbc.car.volvo.values import PLATFORM_C1, PLATFORM_EUCD, DBC, CarControllerParams as CCP
from opendbc.car.volvo import volvocan
from opendbc.car.can_definitions import CanData
from opendbc.car.interfaces import CarControllerBase
from opendbc.can import CANPacker
from collections import deque


class SteerCommand:
  angle_request = 0
  steer_direction = 0
  trqlim = 0


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP):
    super().__init__(dbc_name, CP)
    self.last_blinker_on = False
    self.blinker_end_frame = 0.
    self.acc_enabled_prev = 0
    self.angle_request_prev = 0

    self.UNBLOCKED = 0
    self.BLOCKED = 1
    self.BLOCK_LEN = CCP.BLOCK_LEN

    self.dir_state = 0
    self.block_steering = 0
    self.steer_direction_bf_block = 0
    self.des_steer_direction_prev = 0

    self.SteerCommand = SteerCommand()
    self.trq_fifo = deque([])
    self.fault_frame = -200

    self.doDTCRequests = True
    self.checkPN = False
    self.clearDtcs = False
    self.timeout = 0
    self.diagRequest = {"byte0": 0x03, "byte1": 0x19, "byte2": 0x02, "byte3": 0x02}
    self.flowControl = {"byte0": 0x30, "byte1": 0x00, "byte2": 0x00, "byte3": 0x00}
    self.clearDTC = {"byte0": 0x04, "byte1": 0x14, "byte2": 0xFF, "byte3": 0xFF, "byte4": 0xFF}

    self.cnt = 0
    self.sndNxtFrame = 0
    self.dictKeys = ["byte" + str(x) for x in range(8)]
    startdid = 0xf1a1
    self.dids = [x for x in range(startdid, startdid + 9)]

    self.packer = CANPacker(DBC[CP.carFingerprint][Bus.pt])

  def dir_change(self, steer_direction, error):
    dzError = 0 if abs(error) < CCP.DEADZONE else error
    tState = -1

    self.des_steer_direction_prev = steer_direction if not self.acc_enabled_prev else self.des_steer_direction_prev

    if self.dir_state == self.UNBLOCKED:
      tState = self.BLOCKED if (steer_direction != self.des_steer_direction_prev and dzError != 0) else tState
    elif self.dir_state == self.BLOCKED:
      if (steer_direction == self.steer_direction_bf_block) or (self.block_steering <= 0) or (dzError == 0):
        tState = self.UNBLOCKED

    if tState == self.UNBLOCKED:
      self.dir_state = self.UNBLOCKED
    elif tState == self.BLOCKED:
      self.steer_direction_bf_block = self.des_steer_direction_prev
      self.block_steering = self.BLOCK_LEN
      self.dir_state = self.BLOCKED

    if self.dir_state == self.UNBLOCKED:
      if dzError == 0:
        steer_direction = self.des_steer_direction_prev
    if self.dir_state == self.BLOCKED:
      self.block_steering -= 1
      steer_direction = CCP.STEER_NO

    return steer_direction

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    enabled = CC.enabled

    can_sends = []
    frame = self.frame

    if (frame % 2 == 0):
      fingerprint = self.CP.carFingerprint

      if enabled and CS.out.vEgo > self.CP.minSteerSpeed:
        current_steer_angle = CS.out.steeringAngleDeg
        self.SteerCommand.angle_request = actuators.steeringAngleDeg

        if self.angle_request_prev * self.SteerCommand.angle_request > 0. and abs(self.SteerCommand.angle_request) > abs(self.angle_request_prev):
          angle_rate_lim = np.interp(CS.out.vEgo, CCP.ANGLE_DELTA_BP, CCP.ANGLE_DELTA_V)
        else:
          angle_rate_lim = np.interp(CS.out.vEgo, CCP.ANGLE_DELTA_BP, CCP.ANGLE_DELTA_VU)

        self.SteerCommand.angle_request = np.clip(self.SteerCommand.angle_request, self.angle_request_prev - angle_rate_lim, self.angle_request_prev + angle_rate_lim)

        self.SteerCommand.trqlim = 0
        if fingerprint in PLATFORM_C1:
          self.SteerCommand.steer_direction = CCP.STEER
        elif fingerprint in PLATFORM_EUCD:
          self.SteerCommand.steer_direction = CCP.STEER_RIGHT if current_steer_angle > self.SteerCommand.angle_request else CCP.STEER_LEFT
          self.SteerCommand.steer_direction = self.dir_change(self.SteerCommand.steer_direction, current_steer_angle - self.SteerCommand.angle_request)

      else:
        self.SteerCommand.steer_direction = CCP.STEER_NO
        self.SteerCommand.trqlim = 0
        if fingerprint in PLATFORM_C1:
          self.SteerCommand.angle_request = np.clip(CS.out.steeringAngleDeg, -359.95, 359.90)
        else:
          self.SteerCommand.angle_request = 0

      if fingerprint in PLATFORM_C1:
        if enabled and CS.out.vEgo > self.CP.minSteerSpeed:
          self.trq_fifo.append(CS.PSCMInfo.LKATorque)
          if len(self.trq_fifo) > CCP.N_ZERO_TRQ:
            self.trq_fifo.popleft()
        else:
          self.trq_fifo.clear()
          self.fault_frame = -200

        if (self.trq_fifo.count(0) >= CCP.N_ZERO_TRQ) and (self.fault_frame == -200):
          self.fault_frame = frame + 100

        if enabled and (frame < self.fault_frame):
          self.SteerCommand.steer_direction = CCP.STEER_NO

        if frame > self.fault_frame + 8:
          self.fault_frame = -200

      self.acc_enabled_prev = enabled
      self.angle_request_prev = self.SteerCommand.angle_request
      if self.SteerCommand.steer_direction == CCP.STEER_RIGHT or self.SteerCommand.steer_direction == CCP.STEER_LEFT:
        self.des_steer_direction_prev = self.SteerCommand.steer_direction

      can_sends.append(volvocan.manipulateServo(self.packer, self.CP.carFingerprint, CS))
      can_sends.append(volvocan.create_steering_control(self.packer, frame, self.CP.carFingerprint, self.SteerCommand, CS.FSMInfo))

    if not enabled and CS.out.cruiseState.enabled:
      can_sends.append(volvocan.cancelACC(self.packer, self.CP.carFingerprint, CS))

    if self.doDTCRequests:
      if (frame % 100 == 0) and (not self.clearDtcs):
        can_sends.append(self.packer.make_can_msg("diagFSMReq", 2, self.diagRequest))
        can_sends.append(self.packer.make_can_msg("diagGlobalReq", 0, self.diagRequest))
        self.timeout = frame + 5

      if frame > self.timeout and self.timeout > 0:
        self.timeout = 0
        if (CS.diag.diagFSMResp & 0x10000000):
          can_sends.append(self.packer.make_can_msg("diagFSMReq", 2, self.flowControl))
        if (CS.diag.diagCEMResp & 0x10000000):
          can_sends.append(self.packer.make_can_msg("diagCEMReq", 0, self.flowControl))
        if (CS.diag.diagPSCMResp & 0x10000000):
          can_sends.append(self.packer.make_can_msg("diagPSCMReq", 0, self.flowControl))
        if (CS.diag.diagCVMResp & 0x10000000):
          can_sends.append(self.packer.make_can_msg("diagCVMReq", 0, self.flowControl))

      if self.checkPN and frame > 100 and frame > self.sndNxtFrame:
        if self.cnt < len(self.dids):
          did = [0x03, 0x22, (self.dids[self.cnt] & 0xff00) >> 8, self.dids[self.cnt] & 0x00ff]
          did.extend([0] * (8 - len(did)))
          diagReq = dict(zip(self.dictKeys, did))
          can_sends.append(self.packer.make_can_msg("diagFSMReq", 2, diagReq))
          can_sends.append(self.packer.make_can_msg("diagCEMReq", 0, diagReq))
          can_sends.append(self.packer.make_can_msg("diagPSCMReq", 0, diagReq))
          can_sends.append(self.packer.make_can_msg("diagCVMReq", 0, diagReq))
          self.cnt += 1
          self.timeout = frame + 5
          self.sndNxtFrame = self.timeout + 5
        else:
          self.checkPN = False

      if self.clearDtcs and (frame > 0) and (frame % 500 == 0):
        can_sends.append(self.packer.make_can_msg("diagGlobalReq", 0, self.clearDTC))
        can_sends.append(self.packer.make_can_msg("diagFSMReq", 2, self.clearDTC))
        self.clearDtcs = False

    new_actuators = structs.CarControl.Actuators()
    new_actuators.steeringAngleDeg = self.SteerCommand.angle_request

    self.frame += 1
    return new_actuators, can_sends
