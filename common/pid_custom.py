import numpy as np
from numbers import Number

from openpilot.common.numpy_fast import clip, interp


class PIDControllerCustom:
  def __init__(self, k_p, k_i, k_f=0., k_d=0.,
               k_p_pos=0., k_i_pos=0., k_d_pos=0.,
               k_p_neg=0., k_i_neg=0., k_d_neg=0.,
               pos_limit=1e308, neg_limit=-1e308, rate=100):
                 
    self.k_f = k_f

    self._k_p_pos = k_p_pos if self._is_nonzero(k_p_pos) else k_p
    self._k_i_pos = k_i_pos if self._is_nonzero(k_i_pos) else k_i
    self._k_d_pos = k_d_pos if self._is_nonzero(k_d_pos) else k_d
    self._k_p_neg = k_p_neg if self._is_nonzero(k_p_neg) else k_p
    self._k_i_neg = k_i_neg if self._is_nonzero(k_i_neg) else k_i
    self._k_d_neg = k_d_neg if self._is_nonzero(k_d_neg) else k_d

    if isinstance(self._k_p_pos, (float, int)):
      self._k_p_pos = [[0], [self._k_p_pos]]
    if isinstance(self._k_i_pos, (float, int)):
      self._k_i_pos = [[0], [self._k_i_pos]]
    if isinstance(self._k_d_pos, (float, int)):
      self._k_d_pos = [[0], [self._k_d_pos]]

    if isinstance(self._k_p_neg, (float, int)):
      self._k_p_neg = [[0], [self._k_p_neg]]
    if isinstance(self._k_i_neg, (float, int)):
      self._k_i_neg = [[0], [self._k_i_neg]]
    if isinstance(self._k_d_neg, (float, int)):
      self._k_d_neg = [[0], [self._k_d_neg]]

    self.pos_limit = pos_limit
    self.neg_limit = neg_limit

    self.i_unwind_rate = 0.3 / rate
    self.i_rate = 1.0 / rate
    self.speed = 0.0

    self.reset()

  def _is_nonzero(self, value):
    if isinstance(value, (list, np.ndarray)):
      return np.any(np.array(value) != 0)
    return value != 0

  @property
  def k_p(self):
    if self.f >= 0:
      return interp(self.speed, self._k_p_pos[0], self._k_p_pos[1])
    else:
      return interp(self.speed, self._k_p_neg[0], self._k_p_neg[1])

  @property
  def k_i(self):
    if self.f >= 0:
      return interp(self.speed, self._k_i_pos[0], self._k_i_pos[1])
    else:
      return interp(self.speed, self._k_i_neg[0], self._k_i_neg[1])

  @property
  def k_d(self):
    if self.f >= 0:
      return interp(self.speed, self._k_d_pos[0], self._k_d_pos[1])
    else:
      return interp(self.speed, self._k_d_neg[0], self._k_d_neg[1])

  @property
  def error_integral(self):
    return self.i / self.k_i

  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.d = 0.0
    self.f = 0.0
    self.control = 0

  def update(self, error, error_rate=0.0, speed=0.0, override=False, feedforward=0., freeze_integrator=False):
    self.speed = speed

    self.p = float(error) * self.k_p
    self.f = feedforward * self.k_f
    self.d = error_rate * self.k_d

    if override:
      self.i -= self.i_unwind_rate * float(np.sign(self.i))
    else:
      if not freeze_integrator:
        self.i = self.i + error * self.k_i * self.i_rate

        # Clip i to prevent exceeding control limits
        control_no_i = self.p + self.d + self.f
        control_no_i = clip(control_no_i, self.neg_limit, self.pos_limit)
        self.i = clip(self.i, self.neg_limit - control_no_i, self.pos_limit - control_no_i)

    control = self.p + self.i + self.d + self.f

    self.control = clip(control, self.neg_limit, self.pos_limit)
    return self.control
