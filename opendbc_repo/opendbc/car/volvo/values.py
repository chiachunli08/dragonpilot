from dataclasses import dataclass, field

from opendbc.car import Bus, CarSpecs, PlatformConfig, Platforms
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarDocs, SupportType, CarParts, CarHarness
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


@dataclass
class VolvoCarDocs(CarDocs):
  package: str = "All"


@dataclass
class VolvoDashcamCarDocs(VolvoCarDocs):
  support_type: SupportType = SupportType.DASHCAM
  support_link: str = "#dashcam"


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


class VolvoFlags:
  pass


def dbc_dict(pt, radar):
  return {Bus.pt: pt, Bus.radar: radar}


class CAR(Platforms):
  V40 = PlatformConfig(
    car_docs=[VolvoCarDocs("Volvo V40 2017")],
    specs=CarSpecs(mass=1610., wheelbase=2.647, steerRatio=14.7, centerToFrontRatio=0.44, minSteerSpeed=1. * CV.KPH_TO_MS),
    dbc_dict=dbc_dict('volvo_v40_2017_pt', None),
  )
  V60 = PlatformConfig(
    car_docs=[VolvoDashcamCarDocs("Volvo V60 2015")],
    specs=CarSpecs(mass=1750., wheelbase=2.776, steerRatio=15., centerToFrontRatio=0.44, minSteerSpeed=1. * CV.KPH_TO_MS),
    dbc_dict=dbc_dict('volvo_v60_2015_pt', None),
  )


PLATFORM_C1 = {CAR.V40}
PLATFORM_EUCD = {CAR.V60}

ECU_ADDRESS = {
  CAR.V40: {"BCM": 0x760, "ECM": 0x7E0, "DIM": 0x720, "CEM": 0x726, "FSM": 0x764, "PSCM": 0x730, "TCM": 0x7E1, "CVM": 0x793},
}

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

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[Request(
    [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.SUPPLIER_SOFTWARE_VERSION_REQUEST],
    [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.SUPPLIER_SOFTWARE_VERSION_RESPONSE],
    bus=0,
  )]
)
