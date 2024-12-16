import math

from cereal import log
from openpilot.common.pid import PIDController
from openpilot.common.numpy_fast import interp
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY


class LatControlCurvaturePID(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.curvature_params = CP.lateralTuning.torque.as_builder()
    self.pid = PIDController(kp=self.curvature_params.kp, ki=self.curvature_params.ki,
                             k_f=self.curvature_params.kf, pos_limit=0.195, neg_limit=-0.195)
    self.use_steering_angle = self.curvature_params.useSteeringAngle

  def update(self, active, CS, VM, params, steer_limited, desired_curvature, calibrated_pose, modelV2):
    curvature_log = log.ControlsState.LateralCurvatureState.new_message()
    if not active:
      corrected_curvature = 0.0
      curvature_log.active = False
      self.pid.reset()
    else:
      curvature_log.active = True
      actual_curvature_vm = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
      roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
      if self.use_steering_angle:
        actual_curvature = actual_curvature_vm
        curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
      else:
        assert calibrated_pose is not None
        actual_curvature_pose = calibrated_pose.angular_velocity.yaw / CS.vEgo
        actual_curvature = interp(CS.vEgo, [2.0, 5.0], [actual_curvature_vm, actual_curvature_pose])
        curvature_deadzone = 0.0

      desired_lateral_accel = desired_curvature * CS.vEgo**2
      error = desired_curvature - actual_curvature
      ff = desired_lateral_accel
      freeze_integrator = steer_limited or CS.steeringPressed or CS.vEgo < 5
      corrected_curvature = self.pid.update(error, feedforward=ff, speed=CS.vEgo, freeze_integrator=freeze_integrator)

      curvature_log.error = error
      curvature_log.desiredCurvature = corrected_curvature

    return 0, 0.0, float(corrected_curvature), curvature_log
