from opendbc.car import Bus, structs, get_safety_config
from opendbc.car.volvo.carstate import CarState
from opendbc.car.volvo.carcontroller import CarController
from opendbc.car.volvo.radar_interface import RadarInterface
from opendbc.car.volvo.values import CAR, PLATFORM_C1, PLATFORM_EUCD
from opendbc.car.interfaces import CarInterfaceBase

SteerControlType = structs.CarParams.SteerControlType


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, dp_params, docs) -> structs.CarParams:
    ret.brand = "volvo"

    if candidate in PLATFORM_C1:
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.volvoC1)]

    elif candidate in PLATFORM_EUCD:
      ret.dashcamOnly = True

    ret.radarUnavailable = True
    ret.steerControlType = SteerControlType.angle
    ret.steerActuatorDelay = 0.2
    ret.steerRateCost = 1.

    ret.lateralTuning.init('pid')
    ret.lateralTuning.pid.kpBP = [0.]
    ret.lateralTuning.pid.kiBP = [0.]
    ret.lateralTuning.pid.kf = 0.0
    ret.lateralTuning.pid.kpV = [0.0]
    ret.lateralTuning.pid.kiV = [0.0]

    ret.transmissionType = structs.CarParams.TransmissionType.automatic

    return ret
