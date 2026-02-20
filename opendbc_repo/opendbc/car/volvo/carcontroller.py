from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volvo import volvocan
from opendbc.car.volvo.values import CAR, PLATFORM, DBC, CarControllerParams as CCP


class SteerCommand:
  angle_request = 0
  steer_direction = 0
  trqlim = 0


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
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
    self.packer = CANPacker(dbc_names[Bus.pt])

  def dir_change(self, steer_direction, error):
    dessd = steer_direction
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
    can_sends = []

    enabled = CC.enabled
    actuators = CC.actuators

    if self.frame % 2 == 0:
      fingerprint = self.CP.carFingerprint

      if enabled and CS.out.vEgo > self.CP.minSteerSpeed:
        current_steer_angle = CS.out.steeringAngleDeg
        self.SteerCommand.angle_request = actuators.steeringAngleDeg

        if self.angle_request_prev * self.SteerCommand.angle_request > 0. and abs(self.SteerCommand.angle_request) > abs(self.angle_request_prev):
          angle_rate_lim = CCP.ANGLE_DELTA_V[0]
          for i, bp in enumerate(CCP.ANGLE_DELTA_BP):
            if CS.out.vEgo > bp:
              angle_rate_lim = CCP.ANGLE_DELTA_V[i]
        else:
          angle_rate_lim = CCP.ANGLE_DELTA_VU[0]
          for i, bp in enumerate(CCP.ANGLE_DELTA_BP):
            if CS.out.vEgo > bp:
              angle_rate_lim = CCP.ANGLE_DELTA_VU[i]

        max_angle = self.angle_request_prev + angle_rate_lim
        min_angle = self.angle_request_prev - angle_rate_lim
        self.SteerCommand.angle_request = max(min(self.SteerCommand.angle_request, max_angle), min_angle)

        self.SteerCommand.trqlim = 0
        if fingerprint in PLATFORM.C1:
          self.SteerCommand.steer_direction = CCP.STEER
        elif fingerprint in PLATFORM.EUCD:
          self.SteerCommand.steer_direction = CCP.STEER_RIGHT if current_steer_angle > self.SteerCommand.angle_request else CCP.STEER_LEFT
          self.SteerCommand.steer_direction = self.dir_change(self.SteerCommand.steer_direction, current_steer_angle - self.SteerCommand.angle_request)

      else:
        self.SteerCommand.steer_direction = CCP.STEER_NO
        self.SteerCommand.trqlim = 0
        if fingerprint in PLATFORM.C1:
          max_angle = 359.90
          min_angle = -359.95
          self.SteerCommand.angle_request = max(min(CS.out.steeringAngleDeg, max_angle), min_angle)
        else:
          self.SteerCommand.angle_request = 0

      self.acc_enabled_prev = enabled
      self.angle_request_prev = self.SteerCommand.angle_request
      if self.SteerCommand.steer_direction == CCP.STEER_RIGHT or self.SteerCommand.steer_direction == CCP.STEER_LEFT:
        self.des_steer_direction_prev = self.SteerCommand.steer_direction

      can_sends.append(volvocan.manipulateServo(self.packer, self.CP.carFingerprint, CS))
      can_sends.append(volvocan.create_steering_control(self.packer, self.frame, self.CP.carFingerprint, self.SteerCommand, CS.FSMInfo))

    if not enabled and CS.out.cruiseState.enabled:
      can_sends.append(volvocan.cancelACC(self.packer, self.CP.carFingerprint, CS))

    new_actuators = CC.actuators.as_builder()
    new_actuators.steeringAngleDeg = self.SteerCommand.angle_request

    self.frame += 1
    return new_actuators, can_sends
