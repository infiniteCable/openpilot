import math
import numpy as np
from collections import deque

from cereal import log
from openpilot.common.pid import PIDController
from openpilot.common.numpy_fast import interp
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.latcontrol import LatControl


class LatControlCurvaturePID(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.pid = PIDController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                             (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                             k_f=CP.lateralTuning.pid.kf, pos_limit=0.2, neg_limit=-0.2)
    self.kpBP = CP.lateralTuning.pid.kpBP
    self.kpV = CP.lateralTuning.pid.kpV
    self.curvature_hist = deque([0.0], maxlen=int(round(CP.steerActuatorDelay / DT_CTRL))+1)
    self.alpha_range = [0.2, 0.05]
    self.v_alpha_range = [0., 28.]
    self.lowpass_filtered = 0.0

  def adaptive_alpha(self, u):
    return interp(u, self.v_alpha_range, self.alpha_range)

  def lowpass_filter(self, current_value, u):
    alpha = self.adaptive_alpha(u)
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
    else:
      curvature_log.active = True
      roll_compensation = -VM.roll_compensation(params.roll, CS.vEgo)
      actual_curvature_vm = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, 0.)
      assert calibrated_pose is not None
      actual_curvature_3dof = -VM.calc_curvature_3dof(calibrated_pose.acceleration.y, calibrated_pose.acceleration.x, calibrated_pose.angular_velocity.yaw,
                                                      CS.vEgo, math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), 0.)
      actual_curvature = interp(CS.vEgo, [2.0, 5.0], [actual_curvature_vm, actual_curvature_3dof])

      reaction = self.lowpass_filter(actual_curvature, CS.vEgo)
      disturbance = self.highpass_filter(actual_curvature, reaction)

      self.curvature_hist.append(reaction)
      curvature_result = self.curvature_hist[0] + disturbance
      
      roll_factor = 1 / (interp(CS.vEgo, self.kpBP, self.kpV) or 1)

      error = desired_curvature - (curvature_result + roll_compensation * roll_factor)
      output_curvature = self.pid.update(error, feedforward=desired_curvature, speed=CS.vEgo)

      curvature_log.p = float(np.float32(self.pid.p))
      curvature_log.i = float(np.float32(self.pid.i))
      curvature_log.f = float(np.float32(self.pid.f))
      curvature_log.saturated = self._check_saturation(abs(desired_curvature - output_curvature) < 1e-5, CS, False)
      curvature_log.error = float(np.float32(error))
      curvature_log.desiredCurvature = float(np.float32(desired_curvature))
      curvature_log.actualCurvature = float(np.float32(actual_curvature))
      curvature_log.output = float(np.float32(output_curvature))

    return 0, 0.0, float(output_curvature), curvature_log
