from opendbc.can.parser import CANParser
from cereal import car
from openpilot.selfdrive.car.volkswagen.values import DBC, VolkswagenFlags
from openpilot.selfdrive.car.interfaces import RadarInterfaceBase

RADAR_ADDR = 0x24F

def get_radar_can_parser(CP):
  if CP.flags & VolkswagenFlags.MEB:
    messages = [("MEB_Distance_01", 25)]
  else:
    return None

  return CANParser(DBC[CP.carFingerprint]['radar'], messages, 2)
  

class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.updated_messages = set()
    self.trigger_msg = RADAR_ADDR
    self.track_id = 0

    self.radar_off_can = CP.radarUnavailable
    self.rcp = get_radar_can_parser(CP)

  def update(self, can_strings):
    if self.radar_off_can or (self.rcp is None):
      return super().update(None)

    vls = self.rcp.update_strings(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    ret = car.RadarData.new_message()
    if self.rcp is None:
      return ret

    errors = []

    if not self.rcp.can_valid:
      errors.append("canError")
    ret.errors = errors

    addr = RADAR_ADDR
    msg = self.rcp.vl["MEB_Distance_01"]

    if addr not in self.pts:
      self.pts[addr] = car.RadarData.RadarPoint.new_message()
      self.pts[addr].trackId = self.track_id
      self.track_id += 1

    valid = msg['Same_Lane_01_Detection'] > 0
    if valid:
      self.pts[addr].measured = True
      self.pts[addr].dRel = msg['Same_Lane_01_Long_Distance']
      self.pts[addr].yRel = msg['Same_Lane_01_Lat_Distance']
      self.pts[addr].vRel = float('nan')
      self.pts[addr].aRel = float('nan')
      self.pts[addr].yvRel = float('nan')

    else:
      del self.pts[addr]

    ret.points = list(self.pts.values())
    return ret
