import math
import numpy as np
from collections import deque

from cereal import log
from openpilot.common.pid import PIDController
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.latcontrol import LatControl

ALPHA_MIN = 0.05
ALPHA_MAX = 0.4

# model curvature does not project correctly into the real world
# 1) calculate the current state of curvature with 3DOF for better disturbance detection
# 2) use a runtime dynamic scaled filter to try to divide disturbance (outer error) and car reaction from control (inner error)
# 3) include steering actuator delay to adapt to physical car reaction time offset
# 4) build error from model curv, delayed car reaction, disturbance and roll
# 4.1) disturbance and roll fully apply / are corrected with PID kp to effectivly are scaled to kp = 1
# 4.2) delayed car reaction is scaled with true PID kp to correct quasi static model error behaviour

class LatControlCurvaturePID(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.pid = PIDController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                             (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                             k_f=CP.lateralTuning.pid.kf, pos_limit=0.2, neg_limit=-0.2)
    self.kpBP = CP.lateralTuning.pid.kpBP
    self.kpV = CP.lateralTuning.pid.kpV
    self.curvature_hist = deque([0.0], maxlen=int(round(CP.steerActuatorDelay / DT_CTRL))+1)
    self.desired_curvature_prev = 0.
    self.lowpass_filtered = 0.0
    self.alpha_prev = ALPHA_MIN

  def compute_dynamic_alpha(self, desired_curvature, desired_curvature_prev, alpha_prev, dt=DT_CTRL, alpha_min=ALPHA_MIN, alpha_max=ALPHA_MAX, A=0.02, n=2.0, beta=3.0, k=2.0):
    d_desired = abs(desired_curvature - desired_curvature_prev) / dt
    alpha_reactive = d_desired**n / (k * A) if A > 0 else 0.0
    alpha = np.clip(alpha_prev * np.exp(-beta * dt) + alpha_reactive, alpha_min, alpha_max)
    return alpha

  def lowpass_filter(self, current_value, alpha, alpha_min=ALPHA_MIN, alpha_max=ALPHA_MAX):
    alpha = min(alpha, alpha_max)
    if alpha >= alpha_max * 0.9:
        reset_factor = (alpha - alpha_min) / (alpha_max - alpha_min)
        self.lowpass_filtered = (1 - reset_factor) * self.lowpass_filtered + reset_factor * current_value
    else:    
      self.lowpass_filtered = (1 - alpha) * self.lowpass_filtered + alpha * current_value
    return self.lowpass_filtered

  def highpass_filter(self, current_value, lowpass_value):
    return current_value - lowpass_value

  def update(self, active, CS, VM, params, steer_limited, desired_curvature, calibrated_pose):
    curvature_log = log.ControlsState.LateralCurvatureState.new_message()
    if not active:
      output_curvature = 0.0
      curvature_log.active = False
      self.pid.reset()
      self.curvature_hist.clear()
      self.curvature_hist.append(0.0)
      self.lowpass_filtered = 0.0
      self.desired_curvature_prev = desired_curvature
      self.alpha_prev = ALPHA_MIN
    else:
      curvature_log.active = True
      roll_compensation = -VM.roll_compensation(params.roll, CS.vEgo)
      actual_curvature_vm = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, 0.)
      assert calibrated_pose is not None
      actual_curvature_3dof = -VM.calc_curvature_3dof(calibrated_pose.acceleration.y, calibrated_pose.acceleration.x, calibrated_pose.angular_velocity.yaw,
                                                      CS.vEgo, math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), 0.)
      actual_curvature = np.interp(CS.vEgo, [2.0, 5.0], [actual_curvature_vm, actual_curvature_3dof])

      alpha = self.compute_dynamic_alpha(desired_curvature, self.desired_curvature_prev, self.alpha_prev)
      reaction = self.lowpass_filter(actual_curvature, alpha)
      self.curvature_hist.append(reaction)
      disturbance = self.highpass_filter(actual_curvature, reaction)
      
      correction_factor = 1 / (np.interp(CS.vEgo, self.kpBP, self.kpV) or 1)

      error = desired_curvature - (self.curvature_hist[0] + (roll_compensation + disturbance) * correction_factor)
      output_curvature = self.pid.update(error, feedforward=desired_curvature, speed=CS.vEgo)

      self.desired_curvature_prev = desired_curvature
      self.alpha_prev = alpha

      curvature_log.p = float(np.float32(self.pid.p))
      curvature_log.i = float(np.float32(self.pid.i))
      curvature_log.f = float(np.float32(self.pid.f))
      curvature_log.saturated = bool(self._check_saturation(abs(desired_curvature - output_curvature) < 1e-5, CS, False))
      curvature_log.error = float(np.float32(error))
      curvature_log.desiredCurvature = float(np.float32(desired_curvature))
      curvature_log.actualCurvature = float(np.float32(actual_curvature))
      curvature_log.output = float(np.float32(output_curvature))

    return 0, 0.0, float(output_curvature), curvature_log
