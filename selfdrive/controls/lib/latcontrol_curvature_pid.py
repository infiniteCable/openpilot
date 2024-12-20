import math
import numpy as np

from cereal import log
from openpilot.common.pid import PIDController
from openpilot.common.numpy_fast import interp
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY


class LatControlCurvaturePID(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.pid = PIDController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                             (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                             k_f=CP.lateralTuning.pid.kf, pos_limit=0.195, neg_limit=-0.195)

  def update(self, active, CS, VM, params, steer_limited, desired_curvature, calibrated_pose, modelV2):
    curvature_log = log.ControlsState.LateralCurvatureState.new_message()
    if not active:
      output_curvature = 0.0
      curvature_log.active = False
      self.pid.reset()
    else:
      curvature_log.active = True
      actual_curvature_vm = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
      roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
      assert calibrated_pose is not None
      actual_curvature_pose = calibrated_pose.angular_velocity.yaw / CS.vEgo
      actual_curvature = interp(CS.vEgo, [2.0, 5.0], [actual_curvature_vm, actual_curvature_pose])

      gravity_adjusted_curvature = desired_curvature + (roll_compensation / (CS.vEgo ** 2))
      error = desired_curvature - actual_curvature
      output_curvature = self.pid.update(error, feedforward=gravity_adjusted_curvature, speed=CS.vEgo)

      curvature_log.saturated = self._check_saturation(abs(desired_curvature - output_curvature) < 1e-5, CS, False)
      curvature_log.error = float(np.float32(error))
      curvature_log.desiredCurvature = float(np.float32(output_curvature))

    return 0, 0.0, float(output_curvature), curvature_log