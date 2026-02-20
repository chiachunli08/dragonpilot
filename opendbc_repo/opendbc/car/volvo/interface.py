#!/usr/bin/env python3
from opendbc.car import get_safety_config, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.volvo.carcontroller import CarController
from opendbc.car.volvo.carstate import CarState
from opendbc.car.volvo.values import CAR, PLATFORM


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, dp_params, docs) -> structs.CarParams:
    ret.brand = "volvo"
    ret.radarUnavailable = True

    if candidate in PLATFORM.C1:
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.volvoC1)]

    elif candidate in PLATFORM.EUCD:
      ret.dashcamOnly = True
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.volvoEUCD)]

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.2
    ret.minSteerSpeed = 1.0 * CV.KPH_TO_MS

    ret.lateralTuning.pid.kpBP = [0.]
    ret.lateralTuning.pid.kiBP = [0.]
    ret.lateralTuning.pid.kf = 0.0
    ret.lateralTuning.pid.kpV = [0.0]
    ret.lateralTuning.pid.kiV = [0.0]

    ret.transmissionType = structs.CarParams.TransmissionType.automatic

    ret.centerToFront = ret.wheelbase * 0.44

    return ret
