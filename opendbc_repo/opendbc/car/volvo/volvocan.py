from opendbc.car.volvo.values import PLATFORM_C1, PLATFORM_EUCD


def cancelACC(packer, car_fingerprint, CS):
  msg = {}

  if car_fingerprint in PLATFORM_C1:
    msg["ACCStopBtn"] = 1

  elif car_fingerprint in PLATFORM_EUCD:
    msg["ACCOnOffBtn"] = 1
    msg["ACCOnOffBtnInv"] = 0

  return packer.make_can_msg("CCButtons", 0, msg)


def manipulateServo(packer, car_fingerprint, CS):
  msg = {
    "LKATorque": 0,
    "SteeringAngleServo": CS.PSCMInfo.SteeringAngleServo,
    "byte0": CS.PSCMInfo.byte0,
    "byte4": CS.PSCMInfo.byte4,
    "byte7": CS.PSCMInfo.byte7,
  }

  if car_fingerprint in PLATFORM_C1:
    msg["LKAActive"] = CS.PSCMInfo.LKAActive & 0xFD
    msg["byte3"] = CS.PSCMInfo.byte3
  elif car_fingerprint in PLATFORM_EUCD:
    msg["LKAActive"] = CS.PSCMInfo.LKAActive & 0xF5
    msg["SteeringWheelRateOfChange"] = CS.PSCMInfo.SteeringWheelRateOfChange

  return packer.make_can_msg("PSCM1", 2, msg)


def create_chksum(dat, car_fingerprint):
  if car_fingerprint in PLATFORM_C1:
    steer_angle_request = ((dat[4] & 0x3F) << 8) + dat[5]
    steering_direction_request = dat[7] & 0x03
    trqlim = dat[3]
  elif car_fingerprint in PLATFORM_EUCD:
    steer_angle_request = ((dat[3] & 0x3F) << 8) + dat[4]
    steering_direction_request = dat[5] & 0x03
    trqlim = dat[2]

  s = (trqlim + steering_direction_request + steer_angle_request + (steer_angle_request >> 8)) & 0xFF
  return s ^ 0xFF


def create_steering_control(packer, frame, car_fingerprint, SteerCommand, FSMInfo):
  values = {
    "LKAAngleReq": SteerCommand.angle_request,
    "LKASteerDirection": SteerCommand.steer_direction,
    "TrqLim": SteerCommand.trqlim,
  }

  if car_fingerprint in PLATFORM_C1:
    values_static = {
      "SET_X_E3": 0xE3,
      "SET_X_B4": 0xB4,
      "SET_X_08": 0x08,
      "SET_X_02": 0x02,
      "SET_X_25": 0x25,
    }
  elif car_fingerprint in PLATFORM_EUCD:
    values_static = {
      "SET_X_22": 0x25,
      "SET_X_02": 0,
      "SET_X_10": 0x10,
      "SET_X_A4": 0xa7,
    }

  values.update(values_static)

  if car_fingerprint in PLATFORM_C1:
    dat = packer.make_can_msg("FSM1", 0, values)[2]
  elif car_fingerprint in PLATFORM_EUCD:
    dat = packer.make_can_msg("FSM2", 0, values)[2]

  values["Checksum"] = create_chksum(dat, car_fingerprint)

  if car_fingerprint in PLATFORM_C1:
    return packer.make_can_msg("FSM1", 0, values)
  elif car_fingerprint in PLATFORM_EUCD:
    return packer.make_can_msg("FSM2", 0, values)
