from cereal import car
from opendbc.can.packer import CANPacker
from openpilot.common.numpy_fast import clip
from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car import apply_driver_steer_torque_limits
from openpilot.selfdrive.car.volkswagen import mqbcan, pqcan
from openpilot.selfdrive.car.volkswagen.bap import Bap
from openpilot.selfdrive.car.volkswagen.values import CANBUS, PQ_CARS, CarControllerParams, VolkswagenFlags

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState


class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.CCP = CarControllerParams(CP)
    self.CCS = pqcan if CP.carFingerprint in PQ_CARS else mqbcan
    self.packer_pt = CANPacker(dbc_name)
    self.bap = Bap()

    self.apply_steer_last = 0
    self.gra_acc_counter_last = None
    self.frame = 0
    self.eps_timer_soft_disable_alert = False
    self.hca_frame_timer_running = 0
    self.hca_frame_same_torque = 0
    self.hca_steer_step = self.CCP.STEER_STEP_INACTIVE
    self.hca_standby_timer = 0
    self.hca_enabled = False

    self.gra_send_up = False
    self.gra_send_down = False
    self.gra_speed = 0
    self.target_speed = 0
    self.gra_button_timer = 20
    self.gra_button_frame = 0

    self.bap_ldw_mode = 0
    self.test = False

  def update(self, CC, CS, ext_bus, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

    # **** Steering Controls ************************************************ #

    if self.frame % self.hca_steer_step == 0:
      # Logic to avoid HCA state 4 "refused":
      #   * Don't steer unless HCA is in state 3 "ready" or 5 "active"
      #   * Don't steer at standstill
      #   * Don't send > 3.00 Newton-meters torque
      #   * Don't send the same torque for > 6 seconds
      #   * Don't send uninterrupted steering for > 360 seconds
      # MQB racks reset the uninterrupted steering timer after a single frame
      # of HCA disabled; this is done whenever output happens to be zero.

      if CC.latActive and not CS.out.accFaulted:
        self.hca_enabled = True
        self.hca_standby_timer = 0
        new_steer = int(round(actuators.steer * self.CCP.STEER_MAX))
        apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.CCP)
        self.hca_frame_timer_running += self.CCP.STEER_STEP
        if self.apply_steer_last == apply_steer:
          self.hca_frame_same_torque += self.CCP.STEER_STEP
          if self.hca_frame_same_torque > self.CCP.STEER_TIME_STUCK_TORQUE / DT_CTRL:
            apply_steer -= (1, -1)[apply_steer < 0]
            self.hca_frame_same_torque = 0
        else:
          self.hca_frame_same_torque = 0
        hca_request = abs(apply_steer) > 0
      else:
        hca_request = False
        apply_steer = 0

        if self.hca_enabled and self.hca_standby_timer <= 10:
          self.hca_standby_timer += 1
        else:
          self.hca_enabled = False
          self.hca_standby_timer = 0

      if not hca_request:
        self.hca_frame_timer_running = 0

      self.eps_timer_soft_disable_alert = self.hca_frame_timer_running > self.CCP.STEER_TIME_ALERT / DT_CTRL
      self.apply_steer_last = apply_steer
      can_sends.append(self.CCS.create_steering_control(self.packer_pt, CANBUS.pt, apply_steer, self.hca_enabled, hca_request))

    # set steer command frequency to satisfy eps (no lasting perm. fault)
    if self.hca_enabled:
      self.hca_steer_step = self.CCP.STEER_STEP
    else:
      self.hca_steer_step = self.CCP.STEER_STEP_INACTIVE

      if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
        # Pacify VW Emergency Assist driver inactivity detection by changing its view of driver steering input torque
        # to the greatest of actual driver input or 2x openpilot's output (1x openpilot output is not enough to
        # consistently reset inactivity detection on straight level roads). See commaai/openpilot#23274 for background.
        ea_simulated_torque = clip(apply_steer * 2, -self.CCP.STEER_MAX, self.CCP.STEER_MAX)
        if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
          ea_simulated_torque = CS.out.steeringTorque
        can_sends.append(self.CCS.create_eps_update(self.packer_pt, CANBUS.cam, CS.eps_stock_values, ea_simulated_torque))

    # **** Acceleration Controls ******************************************** #

    if self.CP.openpilotLongitudinalControl and CS.out.cruiseState.enabled and not CS.out.accFaulted and CC.longActive:
      if self.frame % 20 == 0:
        target_accel = actuators.accel
        if target_accel > 0:
          scaling = 40
        elif target_accel < 0:
          scaling = 10
        else:
          scaling = 1
        #target_speed = int(actuators.speed * CV.MS_TO_KPH * ((CS.out.vEgo * CV.MS_TO_KPH) / CS.clu_speed))
        self.target_speed = int(max(CS.clu_speed + (target_accel * scaling), 0))
        self.gra_speed = int(CS.gra_speed)
        speed_diff = abs(self.target_speed - self.gra_speed)
        self.gra_button_timer = max(int(200 / speed_diff) if speed_diff != 0 else 200, 20)

      if self.gra_button_frame >= self.gra_button_timer:
        if self.target_speed > self.gra_speed:
          self.gra_send_up = True
          self.gra_send_down = False
        elif self.target_speed < self.gra_speed:
          self.gra_send_up = False
          self.gra_send_down = True
        else:
          self.gra_send_up = False
          self.gra_send_down = False

        self.gra_button_frame = 0
      else:
        self.gra_button_frame += 1

    else:
      self.gra_send_up = False
      self.gra_send_down = False

    # **** HUD Controls ***************************************************** #

    if CS.bap_ldw_01 is not None or CS.bap_ldw_01_rec != 0:
      self.test = True

    if self.frame % self.CCP.LDW_STEP == 0:
      hud_alert = 0
      if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw) or self.test:
        self.test = False
        if not CS.steering_recovered:
          hud_alert = self.CCP.LDW_MESSAGES["none"]
        else:
          hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOverUrgent"]
      can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, CANBUS.cam, CS.ldw_stock_values, CC.enabled,
                                                       CS.out.steeringPressed, hud_alert, hud_control))

    #self.handle_bap_ldw_01(can_sends, CS.bap_ldw_01)
    #if self.frame % 100 == 0:
    #  self.send_bap_ldw(can_sends)

    # **** Stock ACC Button Controls **************************************** #

    gra_send_ready = CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if gra_send_ready:
      if self.CP.pcmCruise and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
        can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, CANBUS.pt, CS.gra_stock_values,
                                                             cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))

      elif self.CP.openpilotLongitudinalControl and CC.longActive and not CS.out.cruiseState.enabled:
        can_sends.append(self.CCS.create_gra_buttons_control(self.packer_pt, CANBUS.pt, CS.gra_stock_values, up=self.gra_send_up, down=self.gra_send_down))
        self.gra_send_up = False
        self.gra_send_down = False

    new_actuators = actuators.copy()
    new_actuators.steer = self.apply_steer_last / self.CCP.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last

    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends, self.eps_timer_soft_disable_alert

  def send_bap_ldw(self, can_sends):
    bap_dest_dbc = "BAP_LDW_10"
    bap_dest_hex = 0x17331910

    if self.bap_ldw_mode == 0:
      can_frames = self.bap.send(bap_dest_hex, 0, 25, 2, bytes.fromhex("030019000401"))
      self.send_bap(can_sends, bap_dest_dbc, can_frames)
      self.bap_ldw_mode = 1

    elif self.bap_ldw_mode == 1:
      can_frames = self.bap.send(bap_dest_hex, 4, 25, 3, bytes.fromhex("3807E000040108003807E0"))
      self.send_bap(can_sends, bap_dest_dbc, can_frames)
      self.bap_ldw_mode = 2

    elif self.bap_ldw_mode == 2:
      can_frames = self.bap.send(bap_dest_hex, 4, 25, 1, bytes.fromhex("03001900040108003807E000000000000A00020001000200000000"))
      self.send_bap(can_sends, bap_dest_dbc, can_frames)
      self.bap_ldw_mode = 0

  def send_bap(self, can_sends, can_dest, can_frames):
    for (id, data) in can_frames:
      can_sends.append(self.CCS.create_bap(self.packer_pt, CANBUS.cam, can_dest, int.from_bytes(data)))

  def handle_bap_ldw_01(self, can_sends, bap_ldw_01):
    bap_dest_dbc = "BAP_LDW_10"
    bap_dest_hex = 0x17331910

    if bap_ldw_01 is not None:
      can_id, opcode, lsg_id, fct_id, bap_data = bap_ldw_01

      if lsg_id == 25: # LDW
        if opcode == 1: # get
          if fct_id == 2: # configuration
            can_frames = self.bap.send(bap_dest_hex, 0, lsg_id, fct_id, bytes.fromhex("030019000401"))
            for (id, data) in can_frames:
              #can_sends.append([bap_dest_hex, 0, data, CANBUS.cam])
              can_sends.append(self.CCS.create_bap(self.packer_pt, CANBUS.cam, bap_dest_dbc, int.from_bytes(data)))

          elif fct_id == 3: # functions
            can_frames = self.bap.send(bap_dest_hex, 4, lsg_id, fct_id, bytes.fromhex("3807E000040108003807E0"))
            for (id, data) in can_frames:
            #  can_sends.append([bap_dest_hex, 0, data, CANBUS.cam])
              can_sends.append(self.CCS.create_bap(self.packer_pt, CANBUS.cam, bap_dest_dbc, int.from_bytes(data)))

          elif fct_id == 1: # properties
            can_frames = self.bap.send(bap_dest_hex, 4, lsg_id, fct_id, bytes.fromhex("03001900040108003807E000000000000A00020001000200000000"))
            for (id, data) in can_frames:
             # can_sends.append([bap_dest_hex, 0, data, CANBUS.cam])
              can_sends.append(self.CCS.create_bap(self.packer_pt, CANBUS.cam, bap_dest_dbc, int.from_bytes(data)))

