import math

from cereal import log
from openpilot.selfdrive.controls.lib.latcontrol import LatControl

STEER_ANGLE_SATURATION_THRESHOLD = 2.5  # Degrees


class LatControlCurvature(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.sat_check_min_speed = 5.

  def update(self, active, CS, VM, params, steer_limited, desired_curvature, calibrated_pose, modelV2):
    curvature_log = log.ControlsState.LateralCurvatureState.new_message()

    if not active:
      curvature_log.active = False
      curvature_desired = 0.0
      angle_steers_des = float(CS.steeringAngleDeg)
    else:
      curvature_log.active = True
      #curvature_desired = VM.calc_curvature_correction_3dof(modelV2, CS.latAccel, CS.longAccel, CS.yawRate, CS.vEgo, math.radians(CS.steeringAngleDeg))
      curvature_desired = VM.calc_curvature_correction_dbm(modelV2, math.radians(CS.steeringAngleDeg), CS.vEgo, params)
      angle_steers_des = math.degrees(VM.get_steer_from_curvature(-curvature_desired, CS.vEgo, params.roll))
      angle_steers_des += params.angleOffsetDeg

    angle_control_saturated = abs(angle_steers_des - CS.steeringAngleDeg) > STEER_ANGLE_SATURATION_THRESHOLD
    curvature_log.saturated = self._check_saturation(angle_control_saturated, CS, False)
    curvature_log.desiredCurvature = float(curvature_desired)
    return 0, 0.0, float(curvature_desired), curvature_log
