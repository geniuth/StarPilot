import time

from opendbc.car import Bus, get_safety_config, structs, uds
from opendbc.car.carlog import carlog
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.isotp_parallel_query import IsoTpParallelQuery
from opendbc.car.volkswagen.carcontroller import CarController
from opendbc.car.volkswagen.carstate import CarState
from opendbc.car.volkswagen.radar_interface import RadarInterface
from opendbc.car.volkswagen.values import (CanBus, CAR, DBC, NetworkLocation, RADAR_DISABLE_STATE, TransmissionType, VOLKSWAGEN_RX_OFFSET, VolkswagenFlags, VolkswagenSafetyFlags)


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate: CAR, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "volkswagen"
    ret.radarUnavailable = True

    if ret.flags & VolkswagenFlags.PQ:
      # Set global PQ35/PQ46/NMS parameters
      safety_configs = [get_safety_config(structs.CarParams.SafetyModel.volkswagenPq)]
      ret.enableBsm = 0x3BA in fingerprint[0]  # SWA_1

      if 0x440 in fingerprint[0] or docs:  # Getriebe_1
        ret.transmissionType = TransmissionType.automatic
      else:
        ret.transmissionType = TransmissionType.manual

      if any(msg in fingerprint[1] for msg in (0x1A0, 0xC2)):  # Bremse_1, Lenkwinkel_1
        ret.networkLocation = NetworkLocation.gateway
      else:
        ret.networkLocation = NetworkLocation.fwdCamera

      # The PQ port is in dashcam-only mode due to a fixed six-minute maximum timer on HCA steering. An unsupported
      # EPS flash update to work around this timer, and enable steering down to zero, is available from:
      #   https://github.com/pd0wm/pq-flasher
      # It is documented in a four-part blog series:
      #   https://blog.willemmelching.nl/carhacking/2022/01/02/vw-part1/
      # Panda ALLOW_DEBUG firmware required.
      ret.dashcamOnly = True

    elif ret.flags & VolkswagenFlags.MLB:
      # Set global MLB parameters
      safety_configs = [get_safety_config(structs.CarParams.SafetyModel.volkswagenMlb)]
      ret.enableBsm = 0x30F in fingerprint[0]  # SWA_01
      ret.networkLocation = NetworkLocation.gateway
      ret.dashcamOnly = True  # Pending HCA timeout fix, safety validation, harness termination, install procedure

    elif ret.flags & VolkswagenFlags.MEB:
      safety_configs = [get_safety_config(structs.CarParams.SafetyModel.volkswagenMeb)]
      if ret.flags & VolkswagenFlags.MEB_GEN2:
        safety_configs[0].safetyParam |= VolkswagenSafetyFlags.MEB_ALT_CRC.value

      ret.transmissionType = TransmissionType.direct
      ret.steerControlType = structs.CarParams.SteerControlType.curvatureDEPRECATED
      ret.steerAtStandstill = True

      ret.lateralTuning.init('pid')
      ret.lateralTuning.pid.kpBP = [10., 40.]
      ret.lateralTuning.pid.kpV = [0., 1.45]
      ret.lateralTuning.pid.kiBP = [10., 40.]
      ret.lateralTuning.pid.kiV = [0., 0.12]
      ret.lateralTuning.pid.kf = 1.

      if docs or any(msg in fingerprint[1] for msg in (0x520, 0x86, 0xFD, 0x13D)):
        ret.networkLocation = NetworkLocation.gateway
        ret.radarUnavailable = Bus.radar not in DBC[candidate]
      else:
        ret.networkLocation = NetworkLocation.fwdCamera

      ret.enableBsm = 0x24C in fingerprint[0]
      if 0x25D in fingerprint[0]:
        ret.flags |= VolkswagenFlags.STOCK_KLR_PRESENT.value
      if all(msg in fingerprint[2] for msg in (0x1A4, 0x1F0)):  # EA_01, EA_02
        ret.flags |= VolkswagenFlags.STOCK_EA_PRESENT.value
        # Panda only blocks the stock EA HUD from being forwarded when openpilot relays its own
        safety_configs[0].safetyParam |= VolkswagenSafetyFlags.MEB_EA_RELAY.value
      if 0x3DC in fingerprint[0]:
        ret.flags |= VolkswagenFlags.ALT_GEAR.value

      # Camera-harness longitudinal: with MebDisableRadar set, openpilot traps the stock radar
      # in a programming session and replaces its messages, trading stock AEB/FCW/EA for
      # longitudinal without the J533 gateway harness. Unverified on a car.
      if ret.networkLocation == NetworkLocation.fwdCamera and not docs:
        try:
          from openpilot.common.params import Params
          if Params().get_int("MebDisableRadar") > 0:
            ret.flags |= VolkswagenFlags.DISABLE_RADAR.value
        except Exception:
          pass

      # MEB support requires the J533 gateway harness; camera installations remain passive
      # unless the driver has opted into the radar knockout.
      ret.dashcamOnly = (ret.networkLocation == NetworkLocation.fwdCamera
                         and not ret.flags & VolkswagenFlags.DISABLE_RADAR)

    else:
      # Set global MQB parameters
      safety_configs = [get_safety_config(structs.CarParams.SafetyModel.volkswagen)]
      ret.enableBsm = 0x30F in fingerprint[0]  # SWA_01

      if 0xAD in fingerprint[0] or docs:  # Getriebe_11
        ret.transmissionType = TransmissionType.automatic
      elif 0x187 in fingerprint[0]:  # Motor_EV_01
        ret.transmissionType = TransmissionType.direct
      else:
        ret.transmissionType = TransmissionType.manual

      if any(msg in fingerprint[1] for msg in (0x40, 0x86, 0xB2, 0xFD)):  # Airbag_01, LWI_01, ESP_19, ESP_21
        ret.networkLocation = NetworkLocation.gateway
      else:
        ret.networkLocation = NetworkLocation.fwdCamera

      if 0x126 in fingerprint[2]:  # HCA_01
        ret.flags |= VolkswagenFlags.STOCK_HCA_PRESENT.value
      if 0x6B8 in fingerprint[0]:  # Kombi_03
        ret.flags |= VolkswagenFlags.KOMBI_PRESENT.value

    # Global lateral tuning defaults, can be overridden per-vehicle

    ret.steerLimitTimer = 0.4
    if ret.flags & VolkswagenFlags.PQ or ret.flags & VolkswagenFlags.MLB:
      ret.steerActuatorDelay = 0.2
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
    elif ret.flags & VolkswagenFlags.MEB:
      ret.steerActuatorDelay = 0.3
    else:
      ret.steerActuatorDelay = 0.1
      ret.lateralTuning.pid.kpBP = [0.]
      ret.lateralTuning.pid.kiBP = [0.]
      ret.lateralTuning.pid.kf = 0.00006
      ret.lateralTuning.pid.kpV = [0.6]
      ret.lateralTuning.pid.kiV = [0.2]

    # Global longitudinal tuning defaults, can be overridden per-vehicle

    if ret.flags & VolkswagenFlags.MEB:
      ret.longitudinalActuatorDelay = 0.5
      ret.longitudinalTuning.kiBP = [0., 30.]
      ret.longitudinalTuning.kiV = [0.4, 0.]

    ret.alphaLongitudinalAvailable = (ret.networkLocation == NetworkLocation.gateway
                                      or bool(ret.flags & VolkswagenFlags.DISABLE_RADAR) or docs)
    if alpha_long and (not ret.flags & VolkswagenFlags.MEB or ret.alphaLongitudinalAvailable):
      # Panda ALLOW_DEBUG firmware is required for Volkswagen longitudinal control.
      ret.openpilotLongitudinalControl = True
      safety_configs[0].safetyParam |= VolkswagenSafetyFlags.LONG_CONTROL.value
      if ret.flags & VolkswagenFlags.DISABLE_RADAR:
        safety_configs[0].safetyParam |= VolkswagenSafetyFlags.MEB_DISABLE_RADAR.value
      if ret.transmissionType == TransmissionType.manual:
        ret.minEnableSpeed = 4.5

    # Per-vehicle overrides

    if candidate == CAR.PORSCHE_MACAN_MK1:
      ret.steerActuatorDelay = 0.07
    elif candidate == CAR.VOLKSWAGEN_TAOS_MK1:
      # Logged Taos braking response aligns about 0.1 s later than the MQB default.
      ret.longitudinalActuatorDelay = 0.25

    ret.pcmCruise = not ret.openpilotLongitudinalControl
    ret.stopAccel = -0.55
    ret.vEgoStarting = 0.1
    ret.vEgoStopping = 0.5
    ret.autoResumeSng = ret.minEnableSpeed == -1

    CAN = CanBus(fingerprint=fingerprint)
    if CAN.pt >= 4:
      safety_configs.insert(0, get_safety_config(structs.CarParams.SafetyModel.noOutput))
    ret.safetyConfigs = safety_configs

    return ret

  # **** Radar knockout for camera-harness longitudinal (VolkswagenFlags.DISABLE_RADAR) **** #
  # Ported from carrot-wip, which took it from infiniteCable2. At startup the stock radar at
  # 0x757 is pushed into a programming session so it stops transmitting, and carcontroller
  # takes over its AEB and object messages. Stock AEB, FCW and EA are lost while this is on.
  # Unverified on a car.

  @staticmethod
  def init(CP, can_recv, can_send):
    if not (CP.openpilotLongitudinalControl and CP.flags & VolkswagenFlags.DISABLE_RADAR
            and CP.flags & VolkswagenFlags.MEB):
      return

    RADAR_DISABLE_STATE["error"] = False
    # A programming session is refused with the engine on, and the radar cannot be revived
    # afterwards, so do not even try unless the car is in a state that accepts it.
    if not CarInterface._is_engine_state_allowed_meb(can_recv):
      RADAR_DISABLE_STATE["error"] = True
      carlog.warning("MEB radar disable skipped: engine is on")
      return

    carlog.warning("Trying to disable the radar")
    if not CarInterface._radar_communication_control(CP, can_recv, can_send):
      RADAR_DISABLE_STATE["error"] = True

  @staticmethod
  def _radar_communication_control(CP, can_recv, can_send):
    bus = CanBus(CP).pt
    addr_radar, addr_diag, rx_offset = 0x757, 0x700, VOLKSWAGEN_RX_OFFSET
    retry, timeout = 3, 0.5

    tp_req = bytes([uds.SERVICE_TYPE.TESTER_PRESENT, 0x00])
    tp_resp = bytes([uds.SERVICE_TYPE.TESTER_PRESENT + 0x40, 0x00])
    ext_diag_req = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, uds.SESSION_TYPE.EXTENDED_DIAGNOSTIC])
    ext_diag_resp = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL + 0x40, uds.SESSION_TYPE.EXTENDED_DIAGNOSTIC])
    flash_req = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, uds.SESSION_TYPE.PROGRAMMING])

    for i in range(retry):
      try:
        query = IsoTpParallelQuery(can_send, can_recv, bus, [(addr_radar, None)], [tp_req], [tp_resp],
                                   rx_offset, functional_addrs=[addr_diag])
        if not query.get_data(timeout):
          carlog.warning(f"Tester Present returned no data on attempt {i + 1}")
          continue

        query = IsoTpParallelQuery(can_send, can_recv, bus, [(addr_radar, None)], [ext_diag_req],
                                   [ext_diag_resp], rx_offset)
        if not query.get_data(timeout):
          carlog.warning(f"Radar extended session returned no data on attempt {i + 1}")
          continue

        # Don't wait for the programming session response: the replacement messages need to
        # start immediately or the cruise ECUs fault on the gap.
        query = IsoTpParallelQuery(can_send, can_recv, bus, [(addr_radar, None)], [flash_req], [b''], rx_offset)
        query.get_data(0)
        carlog.warning(f"Radar disabled by programming session on attempt {i + 1}")
        return True
      except Exception as e:
        carlog.error(f"Radar disable exception on attempt {i + 1}: {repr(e)}")
        continue

    carlog.error("Radar disable failed")
    return False

  @staticmethod
  def _is_engine_state_allowed_meb(can_recv, timeout: float = 0.5) -> bool:
    # Motor_54.Engine_On, read straight off the wire since carstate isn't up yet
    end_time = time.monotonic() + timeout
    while time.monotonic() < end_time:
      for packet in can_recv(wait_for_one=True) or []:
        for msg in packet:
          if msg.address != 0x14C:
            continue
          if bool((msg.dat[9] >> 5) & 0x01):
            carlog.warning("Engine state is not allowed: Engine_On=True")
            return False
          return True
    carlog.warning("Engine state unknown")
    return True
