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

    if self.frame % 30 == 0 and not self.gra_send_up and not self.gra_send_down:
      if self.CP.openpilotLongitudinalControl and CS.out.cruiseState.enabled and not CS.out.accFaulted and CC.longActive:
        target_accel = actuators.accel
        target_speed = max(CS.out.vEgo + (target_accel * 3), 0)
        gra_speed = int(round(CS.gra_speed))

          if target_speed > gra_speed:
            self.gra_send_up = True
          elif target_speed < gra_speed:
            self.gra_send_down = True
    
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

    if self.frame % self.CCP.ACC_HUD_STEP == 0 and self.CP.openpilotLongitudinalControl:
      lead_distance = 0
      if hud_control.leadVisible and self.frame * DT_CTRL > 1.0:  # Don't display lead until we know the scaling factor
        lead_distance = 512 if CS.upscale_lead_car_signal else 8
      acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
      # FIXME: follow the recent displayed-speed updates, also use mph_kmh toggle to fix display rounding problem?
      set_speed = hud_control.setSpeed * CV.MS_TO_KPH
      can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, CANBUS.cam, acc_hud_status, set_speed,
                                                       lead_distance))

    # **** Stock ACC Button Controls **************************************** #

    gra_send_ready = CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if gra_send_ready:
      if self.CP.pcmCruise and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
        can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, CANBUS.pt, CS.gra_stock_values,
                                                             cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))

      elif self.CP.openpilotLongitudinalControl and (self.gra_send_up or self.gra_send_down):
        can_sends.append(self.CCS.create_gra_buttons_control(self.packer_pt, CANBUS.pt, CS.gra_stock_values, self.gra_send_up, self.gra_send_down))
        self.gra_send_up = False
        self.gra_send_down = False

    new_actuators = actuators.copy()
    new_actuators.steer = self.apply_steer_last / self.CCP.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last

    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends, self.eps_timer_soft_disable_alert
