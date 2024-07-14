from cereal import car
from opendbc.can.packer import CANPacker
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car import apply_driver_steer_torque_limits, apply_std_steer_angle_limits
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.volkswagen import mqbcan, pqcan, mebcan
from openpilot.selfdrive.car.volkswagen.values import CANBUS, CarControllerParams, VolkswagenFlags

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.CCP = CarControllerParams(CP)

    if CP.flags & VolkswagenFlags.PQ:
      self.CCS = pqcan
    elif CP.flags & VolkswagenFlags.MEB:
      self.CCS = mebcan
    else:
      self.CCS = mqbcan
    
    self.packer_pt = CANPacker(dbc_name)
    self.ext_bus = CANBUS.pt if CP.networkLocation == car.CarParams.NetworkLocation.fwdCamera else CANBUS.cam

    self.apply_steer_last = 0
    self.apply_angle_last = 0
    self.gra_acc_counter_last = None
    self.frame = 0
    self.eps_timer_soft_disable_alert = False
    self.hca_frame_timer_running = 0
    self.hca_frame_same_torque = 0
    self.lat_active_prev = False
    self.torque_wind_down = 0
    self.long_active = False
    self.acc_control = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      if self.CP.flags & VolkswagenFlags.MEB:
        # Logic to avoid HCA refused state
        #   * angle change torque as counter near zero before standstill OP lane assist deactivation
        # MEB rack can be used continously without found time limits yet
        # Angle change counter is used to:
        #   * prevent sudden fluctuations at low speeds
        #   * avoid HCA refused
        #   * easy user intervention
        #   * keep it near maximum regarding speed to get full torque in shortest time
        
        if CC.latActive:
          hca_enabled          = True
          self.lat_active_prev = True
          apply_angle          = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw, self.CCP)

          # torque wind down as lazy counter
          torque_wind_down_min_by_speed = interp(CS.out.vEgoRaw, [0, 20], [self.CCP.TORQUE_WIND_DOWN_MIN, self.CCP.TORQUE_WIND_DOWN_MAX])
          torque_wind_down_by_angle     = self.CCP.TORQUE_WIND_DOWN_MAX * abs(apply_angle) / 10 # maximum angle change torque is reached with 10 degrees
          torque_wind_down_target       = clip(torque_wind_down_by_angle, torque_wind_down_min_by_speed, self.CCP.TORQUE_WIND_DOWN_MAX)

          if self.torque_wind_down < self.CCP.TORQUE_WIND_DOWN_MIN:  # OP lane assist just activated
            self.torque_wind_down += 1
          elif CS.out.steeringPressed and self.torque_wind_down > self.CCP.TORQUE_WIND_DOWN_MIN: # user action results in decreasing the angle change torque
            self.torque_wind_down = max(self.torque_wind_down - 4, self.CCP.TORQUE_WIND_DOWN_MIN)
          elif self.torque_wind_down < self.CCP.TORQUE_WIND_DOWN_MAX: # following angle change target
            if self.torque_wind_down < torque_wind_down_target:
              self.torque_wind_down = min(self.torque_wind_down + 4, torque_wind_down_target)
            elif self.torque_wind_down > torque_wind_down_target:
              self.torque_wind_down -= 1

          #if abs(apply_angle) > 45:
          #  new_steer = self.CCP.STEER_MAX - 
          #  apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.CCP) 
        
        else:
          if self.lat_active_prev and self.torque_wind_down > 0: # decrement angle change torque to zero before disabling lane assist to prevent EPS fault
            hca_enabled            = True
            apply_angle            = CS.out.steeringAngleDeg
            self.torque_wind_down -= 1
          else:
            hca_enabled           = False
            self.lat_active_prev  = False
            self.torque_wind_down = 0
            apply_angle           = 0

        self.apply_angle_last = clip(apply_angle, -self.CCP.ANGLE_MAX, self.CCP.ANGLE_MAX)
        can_sends.append(self.CCS.create_steering_control_angle(self.packer_pt, CANBUS.pt, apply_angle, hca_enabled, self.torque_wind_down))

      else:
        # Logic to avoid HCA state 4 "refused":
        #   * Don't steer unless HCA is in state 3 "ready" or 5 "active"
        #   * Don't steer at standstill
        #   * Don't send > 3.00 Newton-meters torque
        #   * Don't send the same torque for > 6 seconds
        #   * Don't send uninterrupted steering for > 360 seconds
        # MQB racks reset the uninterrupted steering timer after a single frame
        # of HCA disabled; this is done whenever output happens to be zero.
        
        if CC.latActive:
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
          hca_enabled = abs(apply_steer) > 0
        else:
          hca_enabled = False
          apply_steer = 0

        if not hca_enabled:
          self.hca_frame_timer_running = 0

        self.eps_timer_soft_disable_alert = self.hca_frame_timer_running > self.CCP.STEER_TIME_ALERT / DT_CTRL
        self.apply_steer_last = apply_steer
        can_sends.append(self.CCS.create_steering_control(self.packer_pt, CANBUS.pt, apply_steer, hca_enabled))

      if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
        # Pacify VW Emergency Assist driver inactivity detection by changing its view of driver steering input torque
        # to the greatest of actual driver input or 2x openpilot's output (1x openpilot output is not enough to
        # consistently reset inactivity detection on straight level roads). See commaai/openpilot#23274 for background.
        ea_simulated_torque = clip(apply_steer * 2, -self.CCP.STEER_MAX, self.CCP.STEER_MAX)
        if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
          ea_simulated_torque = CS.out.steeringTorque
        can_sends.append(self.CCS.create_eps_update(self.packer_pt, CANBUS.cam, CS.eps_stock_values, ea_simulated_torque))

    # **** Acceleration Controls ******************************************** #

    if self.frame % self.CCP.ACC_CONTROL_STEP == 0 and self.CP.openpilotLongitudinalControl:
      accel = clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.longActive else 0
      stopping = actuators.longControlState == LongCtrlState.stopping
      starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < self.CP.vEgoStopping)
     
      if self.CP.flags & VolkswagenFlags.MEB:
        self.long_active = CC.longActive #and not CS.out.gasPressed
        acc_control_disable = CS.out.gasPressed and self.long_active
        self.acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, self.long_active, acc_control_disable)
        can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, CANBUS.pt, CS.acc_type, self.long_active, accel,
                                                           self.acc_control, stopping, starting, CS.esp_hold_confirmation,
                                                           CS.meb_acc_02_values))

      else:
        acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
        can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, CANBUS.pt, CS.acc_type, CC.longActive, accel,
                                                           acc_control, stopping, starting, CS.esp_hold_confirmation))

    # **** HUD Controls ***************************************************** #

    if self.frame % self.CCP.LDW_STEP == 0:
      hud_alert = 0
      if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw):
        hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOverUrgent"]
      can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, CANBUS.pt, CS.ldw_stock_values, CC.latActive,
                                                       CS.out.steeringPressed, hud_alert, hud_control))

    if self.frame % self.CCP.ACC_HUD_STEP == 0 and self.CP.openpilotLongitudinalControl:
      if self.CP.flags & VolkswagenFlags.MEB:
        lead_distance = 0
        if hud_control.leadVisible and self.frame * DT_CTRL > 1.0:  # Don't display lead until we know the scaling factor
          lead_distance = 512
        acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, self.long_active)
        set_speed = hud_control.setSpeed * CV.MS_TO_KPH
        can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, CANBUS.pt, acc_hud_status, self.acc_control, set_speed,
                                                         lead_distance, hud_control.leadDistanceBars, CS.meb_acc_01_values))
        
      else:
        lead_distance = 0
        if hud_control.leadVisible and self.frame * DT_CTRL > 1.0:  # Don't display lead until we know the scaling factor
          lead_distance = 512 if CS.upscale_lead_car_signal else 8
        acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
        # FIXME: follow the recent displayed-speed updates, also use mph_kmh toggle to fix display rounding problem?
        set_speed = hud_control.setSpeed * CV.MS_TO_KPH
        can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, CANBUS.pt, acc_hud_status, set_speed,
                                                         lead_distance, hud_control.leadDistanceBars))

    # **** Stock ACC Button Controls **************************************** #

    gra_send_ready = self.CP.pcmCruise and CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if gra_send_ready and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
      can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, self.ext_bus, CS.gra_stock_values,
                                                           cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))

    new_actuators = actuators.as_builder()
    new_actuators.steer = self.apply_steer_last / self.CCP.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends
