from dataclasses import dataclass, field

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu

"""
Volvo Electronic Control Units abbreviations and network topology
Platforms C1/EUCD

Three main CAN network buses
  1. Powertrain
  2. Chassis (also called MS* CAN) *MS=Medium Speed
  3. Extended
Only mentioning control units of interest on the network buses.

Powertrain CAN
  BCM - Brake Control Module
  CEM - Central Electronic Module
  CVM - Closing Velocity Module (low speed auto emergency braking <30kph)
  FSM - Forward Sensing Module (camera mounted in windscreen)
  PPM - Pedestrian Protection Module (controls pedestrian airbag under the engine hood)
  PSCM - Power Steering Control Module (EPS - Electronic Power Steering)
  SAS - Steering Angle Sensor Module
  SRS - Supplemental Restraint System Module (seatbelts, airbags...)
  TCM - Transmission Control Module

Chassis CAN
  CEM - Central Electronic Module
  DIM - Driver Information Module (the instrument cluster with odo and speedometer, relayed thru CEM)
  PAM - Parking Assistance Module (automatic parking, relayed thru CEM)

Extended CAN
  CEM - Central Electronic Module
  SODL - Side Object Detection Left (relayed thru CEM)
  SODR - Side Object Detection Right (relayed thru CEM)
"""


class CarControllerParams:
  STEER_NO = 0
  STEER_RIGHT = 1
  STEER_LEFT = 2
  STEER = 3

  ANGLE_DELTA_BP = [0., 8.33, 13.89, 19.44, 25., 30.55, 36.1]
  ANGLE_DELTA_V = [2., 1.2, .25, .20, .15, .10, .10]
  ANGLE_DELTA_VU = [2., 1.2, .25, .20, .15, .10, .10]

  N_ZERO_TRQ = 12

  BLOCK_LEN = 8
  DEADZONE = 0.1

  def __init__(self, CP):
    pass


BUTTON_STATES = {
  "altButton1": False,
  "setCruise": False,
  "resumeCruise": False,
  "accelCruise": False,
  "decelCruise": False,
  "gapAdjustCruise": False,
}


@dataclass
class VolvoCarDocs(CarDocs):
  package: str = "All"


@dataclass(frozen=True, kw_only=True)
class VolvoCarSpecs(CarSpecs):
  tireStiffnessFactor: float = 0.7


@dataclass
class VolvoPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'volvo_v40_2017_pt'})


class CAR(Platforms):
  V40 = VolvoPlatformConfig(
    [VolvoCarDocs("Volvo V40 2014-17")],
    VolvoCarSpecs(mass=1610, wheelbase=2.647, steerRatio=14.7)
  )
  V60 = VolvoPlatformConfig(
    [VolvoCarDocs("Volvo V60 2015")],
    VolvoCarSpecs(mass=1750, wheelbase=2.776, steerRatio=15.0),
    dbc_dict={Bus.pt: 'volvo_v60_2015_pt'},
  )


class PLATFORM:
  C1 = [CAR.V40]
  EUCD = [CAR.V60]


ECU_ADDRESS = {
  CAR.V40: {"BCM": 0x760, "ECM": 0x7E0, "DIM": 0x720, "CEM": 0x726, "FSM": 0x764, "PSCM": 0x730, "TCM": 0x7E1, "CVM": 0x793},
}


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    ),
  ],
)


FW_VERSIONS = {
  CAR.V40: {
    (Ecu.unknown, ECU_ADDRESS[CAR.V40]["CEM"], None): [b'31453061 AA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'],
    (Ecu.eps, ECU_ADDRESS[CAR.V40]["PSCM"], None): [b'31288595 AE\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'],
    (Ecu.fwdCamera, ECU_ADDRESS[CAR.V40]["FSM"], None): [b'31400454 AA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'],
  }
}


FINGERPRINTS = {
  CAR.V40: [
    {8: 8, 16: 8, 48: 8, 64: 8, 85: 8, 101: 8, 112: 8, 114: 8, 117: 8, 128: 8, 176: 8, 192: 8, 208: 8, 224: 8, 240: 8, 245: 8, 256: 8, 272: 8, 288: 8, 291: 8, 293: 8, 304: 8, 325: 8, 336: 8, 352: 8, 424: 8, 432: 8, 437: 8, 464: 8, 472: 8, 480: 8, 528: 8, 608: 8, 624: 8, 640: 8, 648: 8, 652: 8, 656: 8, 657: 8, 681: 8, 693: 8, 704: 8, 707: 8, 709: 8, 816: 8, 832: 8, 848: 8, 853: 8, 864: 8, 880: 8, 912: 8, 928: 8, 943: 8, 944: 8, 968: 8, 970: 8, 976: 8, 992: 8, 997: 8, 1024: 8, 1029: 8, 1061: 8, 1072: 8, 1409: 8},
    {8: 8, 16: 8, 64: 8, 85: 8, 101: 8, 112: 8, 114: 8, 117: 8, 128: 8, 176: 8, 192: 8, 224: 8, 240: 8, 245: 8, 256: 8, 272: 8, 288: 8, 291: 8, 293: 8, 304: 8, 325: 8, 336: 8, 424: 8, 432: 8, 437: 8, 464: 8, 472: 8, 480: 8, 528: 8, 608: 8, 648: 8, 652: 8, 656: 8, 657: 8, 681: 8, 693: 8, 704: 8, 707: 8, 709: 8, 816: 8, 832: 8, 864: 8, 880: 8, 912: 8, 928: 8, 943: 8, 944: 8, 968: 8, 970: 8, 976: 8, 992: 8, 997: 8, 1024: 8, 1029: 8, 1061: 8, 1072: 8, 1409: 8},
    {8: 8, 16: 8, 64: 8, 85: 8, 101: 8, 112: 8, 114: 8, 117: 8, 128: 8, 176: 8, 192: 8, 224: 8, 240: 8, 245: 8, 256: 8, 272: 8, 288: 8, 291: 8, 293: 8, 304: 8, 325: 8, 336: 8, 424: 8, 432: 8, 437: 8, 464: 8, 472: 8, 480: 8, 528: 8, 608: 8, 648: 8, 652: 8, 657: 8, 681: 8, 693: 8, 704: 8, 707: 8, 709: 8, 816: 8, 864: 8, 880: 8, 912: 8, 928: 8, 943: 8, 944: 8, 968: 8, 970: 8, 976: 8, 992: 8, 997: 8, 1024: 8, 1029: 8, 1072: 8, 1409: 8},
  ],
  CAR.V60: [
    {0: 8, 16: 8, 32: 8, 81: 8, 99: 8, 104: 8, 112: 8, 144: 8, 277: 8, 295: 8, 298: 8, 307: 8, 320: 8, 328: 8, 336: 8, 343: 8, 352: 8, 359: 8, 384: 8, 465: 8, 511: 8, 522: 8, 544: 8, 565: 8, 582: 8, 608: 8, 609: 8, 610: 8, 612: 8, 613: 8, 624: 8, 626: 8, 635: 8, 648: 8, 665: 8, 673: 8, 704: 8, 706: 8, 708: 8, 750: 8, 751: 8, 778: 8, 788: 8, 794: 8, 797: 8, 802: 8, 803: 8, 805: 8, 807: 8, 819: 8, 820: 8, 821: 8, 913: 8, 923: 8, 978: 8, 979: 8, 1006: 8, 1021: 8, 1024: 8, 1029: 8, 1039: 8, 1042: 8, 1045: 8, 1137: 8, 1141: 8, 1152: 8, 1174: 8, 1187: 8, 1198: 8, 1214: 8, 1217: 8, 1226: 8, 1240: 8, 1409: 8},
  ],
}


DBC = CAR.create_dbc_map()
