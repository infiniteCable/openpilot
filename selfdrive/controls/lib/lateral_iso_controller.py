import numpy as np
from cereal import log
from opendbc.car.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import MIN_SPEED, MAX_CURVATURE, MAX_LATERAL_JERK, MAX_LATERAL_ACCEL_NO_ROLL

SOFT_LIMIT_TIME = 2.0
SOFT_LIMIT_STEPS = int(SOFT_LIMIT_TIME / DT_CTRL)

# OP Original Safety ISO Limiting Problems:
# 1) Constant Resistance Against the Driver
# 2) Harsh Transition When the Driver Releases Steering Input (Abrupt Jerk)

# Result of this controller: A More Natural and Intuitive Experience
# The system works with the driver, not against them.
# If the driver overrides, the system gradually adapts instead of forcing a hard constraint.
# The soft limit transitions feel human-like, improving comfort, control, and safety.
# This approach ensures that the ISO regulations are respected, but in a way that maintains smooth vehicle dynamics and a more natural driving experience.


class LateralISOController:
  def __init__(self):
    self.prev_curvature = 0.0
    self.soft_limit_active = False
    self.soft_limit_counter = 0
    self.soft_limit_start_curvature = 0.0
    self.override_last = False

  def reset(self):
    self.prev_curvature = 0.0
    self.soft_limit_active = False
    self.soft_limit_counter = 0
    self.soft_limit_start_curvature = 0.0
    self.override_last = False
  
  def update(self, v_ego, new_curvature, current_curvature, roll, lateral_user_override):
    v_ego = max(v_ego, MIN_SPEED)
    max_curvature_rate = MAX_LATERAL_JERK / (v_ego ** 2)
    
    new_curvature = np.clip(new_curvature,
                            self.prev_curvature - max_curvature_rate * DT_CTRL,
                            self.prev_curvature + max_curvature_rate * DT_CTRL)
    
    roll_compensation = roll * ACCELERATION_DUE_TO_GRAVITY
    max_lat_accel = MAX_LATERAL_ACCEL_NO_ROLL + roll_compensation
    min_lat_accel = -MAX_LATERAL_ACCEL_NO_ROLL + roll_compensation

    iso_limit = max_lat_accel / (v_ego ** 2)  # Berechnung des ISO-Limits
    iso_limit_exceeded = abs(new_curvature) > iso_limit

    if lateral_user_override:
      # Während Override: Nutze die tatsächliche Krümmung, aber begrenze sie maximal auf `new_curvature`
      adjusted_curvature = min(abs(current_curvature), abs(new_curvature)) * np.sign(new_curvature)
      new_curvature = np.clip(adjusted_curvature, -iso_limit, iso_limit)
      self.soft_limit_active = False  # Soft Limit deaktivieren
      self.override_last = True # Override hat stattgefunden
    else:
      # Falls kein Override mehr -> sanft auf ISO-Limit zurückfahren, aber nur wenn das Limit überschritten wurde
      if self.override_last:
        if iso_limit_exceeded:
          if not self.soft_limit_active:
            self.soft_limit_active = True
            self.soft_limit_counter = 0
            self.soft_limit_start_curvature = self.prev_curvature

          self.soft_limit_counter += 1
          alpha = min(1.0, self.soft_limit_counter / SOFT_LIMIT_STEPS)
          target_curvature = (1 - alpha) * self.soft_limit_start_curvature + alpha * np.clip(new_curvature, -iso_limit, iso_limit)
          new_curvature = target_curvature

          if self.soft_limit_counter >= SOFT_LIMIT_STEPS:
            self.soft_limit_active = False
            self.override_last = False
        else:
          # Falls keine Limitüberschreitung mehr -> Soft Limit deaktivieren
          self.soft_limit_active = False
          self.override_last = False

    new_curvature, limited_max_curv = np.clip(new_curvature, -MAX_CURVATURE, MAX_CURVATURE), abs(new_curvature) > MAX_CURVATURE

    self.prev_curvature = new_curvature
    return float(new_curvature), limited_max_curv or iso_limit_exceeded
