from cereal import car
from opendbc.can.packer import CANPacker
from openpilot.common.numpy_fast import clip
from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car import apply_driver_steer_torque_limits
from openpilot.selfdrive.car.volkswagen import mqbcan, pqcan
from openpilot.selfdrive.car.volkswagen.values import CANBUS, PQ_CARS, CarControllerParams

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState


class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.CCP = CarControllerParams(CP)
    self.CCS = pqcan if CP.carFingerprint in PQ_CARS else mqbcan
    self.packer_pt = CANPacker(dbc_name)

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

      if CC.latActive:
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

    # **** Acceleration Controls ******************************************** #

    if self.CP.openpilotLongitudinalControl and CS.out.cruiseState.enabled and not CS.out.accFaulted and CC.longActive:
      if self.frame % 20 == 0:
        target_accel = actuators.accel
        if target_accel > 0:
          scaling = 65
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

    if self.frame % self.CCP.LDW_STEP == 0:
      hud_alert = 0
      if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw):
        hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOverUrgent"]
      can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, CANBUS.cam, CS.ldw_stock_values, CC.enabled,
                                                       CS.out.steeringPressed, hud_alert, hud_control))

    #self.handle_bap_ldw_01(self, can_sends, CS.bap_ldw_01)

    # **** Stock ACC Button Controls **************************************** #

    gra_send_ready = CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if gra_send_ready:
      if self.CP.pcmCruise and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
        can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, CANBUS.pt, CS.gra_stock_values,
                                                             cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))

      elif self.CP.openpilotLongitudinalControl and CC.longActive:
        can_sends.append(self.CCS.create_gra_buttons_control(self.packer_pt, CANBUS.pt, CS.gra_stock_values, up=self.gra_send_up, down=self.gra_send_down))
        self.gra_send_up = False
        self.gra_send_down = False

    new_actuators = actuators.copy()
    new_actuators.steer = self.apply_steer_last / self.CCP.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last

    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends, self.eps_timer_soft_disable_alert

  #def handle_bap_ldw_01(self, can_sends, bap_ldw_01):
  #  op = bap_ldw_01["Op"]
  #  log_id = bap_ldw_01["LogID"]
  #  func = bap_ldw_01["Func"]

  #  if log_id == 25: # LDW
  #    if op == 1: # get
  #      if func == 2: # configuration
  #        data = [0x03, 0x00, 0x19, 0x00, 0x04, 0x01]
  #        can_sends.append(self.CCS.create_bap_short(self.packer_pt, CANBUS.cam, "BAP_LDW_10_S", 0, log_id, func, data))
          
  #      elif func == 3: # functions
  #        data = [0x38, 0x07, 0xE0, 0x00]
  #        can_sends.append(self.CCS.create_bap_long_1(self.packer_pt, CANBUS.cam, "BAP_LDW_10_L1", 4, 0x08, log_id, func, data))
  #        data = [0x04, 0x01, 0x08, 0x00, 0x38, 0x07, 0xE0]
  #        can_sends.append(self.CCS.create_bap_long_n(self.packer_pt, CANBUS.cam, "BAP_LDW_10_LN", 0, data))
                            
  #      elif func == 1: # properties
  #        data = [0x03,0x00, 0x19, 0x00]
  #        can_sends.append(self.CCS.create_bap_long_1(self.packer_pt, CANBUS.cam, "BAP_LDW_10_L1", 4, 0x1A, log_id, func, data))
  #        data = [0x04, 0x01, 0x08, 0x00, 0x38, 0x07, 0xE0]
  #        can_sends.append(self.CCS.create_bap_long_n(self.packer_pt, CANBUS.cam, "BAP_LDW_10_LN", 0, data))
  #        data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00]
  #        can_sends.append(self.CCS.create_bap_long_n(self.packer_pt, CANBUS.cam, "BAP_LDW_10_LN", 1, data))
  #        data = [0x02, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00]
  #        can_sends.append(self.CCS.create_bap_long_n(self.packer_pt, CANBUS.cam, "BAP_LDW_10_LN", 2, data))
  #        data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
  #        can_sends.append(self.CCS.create_bap_long_n(self.packer_pt, CANBUS.cam, "BAP_LDW_10_LN", 3, data))
