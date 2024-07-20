def create_steering_control_angle(packer, bus, apply_angle, lkas_enabled, torque_wind_down):
  values = {
    "Steering_Angle": abs(apply_angle),
    "Active": lkas_enabled,
    "VZ": 1 if apply_angle < 0 and lkas_enabled == 1 else 0,
    "Active_02": lkas_enabled,
    "Inactive": not lkas_enabled,
    "Torque_Wind_Down": torque_wind_down if lkas_enabled else 0,
  }
  return packer.make_can_msg("HCA_03", bus, values)


def create_steering_control(packer, bus, apply_steer, lkas_enabled):
  values = {
    "HCA_01_LM_Offset": abs(apply_steer),
    "HCA_01_Request": lkas_enabled,
    "HCA_01_LM_OffSign": 1 if apply_steer < 0 and lkas_enabled == 1 else 0,
    "HCA_01_Enable": lkas_enabled,
    "HCA_01_Standby": not lkas_enabled,
    "HCA_01_Available": 1,
  }
  return packer.make_can_msg("HCA_01", bus, values)


def create_lka_hud_control(packer, bus, ldw_stock_values, lat_active, steering_pressed, hud_alert, hud_control):
  values = {}
  if len(ldw_stock_values):
    values = {s: ldw_stock_values[s] for s in [
      "LDW_SW_Warnung_links",   # Blind spot in warning mode on left side due to lane departure
      "LDW_SW_Warnung_rechts",  # Blind spot in warning mode on right side due to lane departure
      "LDW_Seite_DLCTLC",       # Direction of most likely lane departure (left or right)
      "LDW_DLC",                # Lane departure, distance to line crossing
      "LDW_TLC",                # Lane departure, time to line crossing
    ]}

  values.update({
    "LDW_Status_LED_gelb": 1 if lat_active and steering_pressed else 0,
    "LDW_Status_LED_gruen": 1 if lat_active and not steering_pressed else 0,
    "LDW_Lernmodus_links": 3 if hud_control.leftLaneDepart else 1 + hud_control.leftLaneVisible,
    "LDW_Lernmodus_rechts": 3 if hud_control.rightLaneDepart else 1 + hud_control.rightLaneVisible,
    "LDW_Texte": hud_alert,
  })
  return packer.make_can_msg("LDW_02", bus, values)
  

def create_acc_buttons_control(packer, bus, gra_stock_values, cancel=False, resume=False):
  values = {s: gra_stock_values[s] for s in [
    "GRA_Hauptschalter",           # ACC button, on/off
    "GRA_Typ_Hauptschalter",       # ACC main button type
    "GRA_Codierung",               # ACC button configuration/coding
    "GRA_Tip_Stufe_2",             # unknown related to stalk type
    "GRA_ButtonTypeInfo",          # unknown related to stalk type
  ]}

  values.update({
    "COUNTER": (gra_stock_values["COUNTER"] + 1) % 16,
    "GRA_Abbrechen": cancel,
    "GRA_Tip_Wiederaufnahme": resume,
  })
  return packer.make_can_msg("GRA_ACC_01", bus, values)
  

def acc_control_value(main_switch_on, acc_faulted, long_active, just_disabled):
  if acc_faulted:
    acc_control = 6
  elif just_disabled:
    acc_control = 5
  elif long_active:
    acc_control = 3
  elif main_switch_on:
    acc_control = 2
  else:
    acc_control = 0

  return acc_control
  

def acc_hold_type(main_switch_on, acc_faulted, long_active, just_disabled, starting, stopping, esp_hold, just_started):
  if acc_faulted or not main_switch_on:
    acc_hold_type = 0
  elif just_disabled: # disabling of acc control
    acc_hold_type = 5
  elif starting:
    acc_hold_type = 4  # hold release and startup
  elif esp_hold or stopping:
    acc_hold_type = 1  # hold or hold request
  elif just_started: # signal right after starting
    acc_hold_type = 5
  else:
    acc_hold_type = 0

  return acc_hold_type
  

def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, acc_hold_type, stopping, starting, just_started, esp_hold, speed, reversing, meb_acc_02_values):
  commands = []

  values = {
    "ACC_Typ": acc_type,
    "ACC_Status_ACC": acc_control,
    "ACC_StartStopp_Info": acc_enabled,
    "ACC_Sollbeschleunigung_02": accel if acc_enabled else 3.01,
    "ACC_zul_Regelabw_unten": 0.2,  # TODO: dynamic adjustment of comfort-band
    "ACC_zul_Regelabw_oben": 0.2,  # TODO: dynamic adjustment of comfort-band
    "ACC_neg_Sollbeschl_Grad_02": 4.0 if acc_enabled else 0,  # TODO: dynamic adjustment of jerk limits
    "ACC_pos_Sollbeschl_Grad_02": 4.0 if acc_enabled else 0,  # TODO: dynamic adjustment of jerk limits
    "ACC_Anfahren": starting,
    "ACC_Anhalten": stopping,
    "ACC_Anhalteweg": 20.46,
    "ACC_Anforderung_HMS": acc_hold_type,
    "ACC_AKTIV_regelt": 1 if acc_control == 3 else 0,
    #"SET_ME_0XFE": 0xFE,
    #"SET_ME_0X1": 0x1,
    #"SET_ME_0X9": 0x9,
    "Speed": speed,
    "Reversing": reversing,
    #"Accel_Boost": 1 if speed != 0 else 0, 
  }

  values.update({
    "SET_ME_0XFE": meb_acc_02_values["SET_ME_0XFE"],
    "SET_ME_0X1": meb_acc_02_values["SET_ME_0X1"],
    "SET_ME_0X9": meb_acc_02_values["SET_ME_0X9"],
    "SET_ME_0XFE": meb_acc_02_values["SET_ME_0XFE"],
    "SET_ME_0X1": meb_acc_02_values["SET_ME_0X1"],
    "SET_ME_0X9": meb_acc_02_values["SET_ME_0X9"],
    #"Accel_Boost": meb_acc_02_values["Accel_Boost"],
  })
  
  commands.append(packer.make_can_msg("MEB_ACC_02", bus, values))
  
  return commands


def acc_hud_status_value(main_switch_on, acc_faulted, long_active, override):
  if acc_faulted:
    acc_hud_control = 6
  elif long_active:
    acc_hud_control = 4 if override else 3
  elif main_switch_on:
    acc_hud_control = 2
  else:
    acc_hud_control = 0

  return acc_hud_control
  

def create_acc_hud_control(packer, bus, acc_control, set_speed, lead_distance, distance, heartbeat, esp_hold, meb_acc_01_values, distance_stock_values):  
  zeitluecke_3 = 0
  zeitluecke_4 = 0
  zeitluecke_5 = 0

  if distance == 3:
    zeitluecke_3 = 50
  elif distance == 4:
    zeitluecke_4 = 50
  elif distance == 5:
    zeitluecke_5 = 50

  lead_distance = distance_stock_values["Same_Lane_01_Long_Distance"]
  
  values = {
    #"STA_Primaeranz": acc_hud_status,
    "ACC_Status_ACC": acc_control,
    "ACC_Wunschgeschw_02": set_speed if set_speed < 250 else 327.36,
    "ACC_Gesetzte_Zeitluecke": distance + 2,
    "ACC_Display_Prio": 3,
    "ACC_Abstandsindex_02": lead_distance,
    "ACC_EGO_Fahrzeug": 1 if acc_control == 3 else 0,
    #"SET_ME_0X3FF": 0x3FF,
    "Heartbeat": heartbeat,
    #"SET_ME_0XFFFF": 0xFFFF,
    #"SET_ME_0X7FFF": 0x7FFF,
    #"SET_ME_0X1": 1,
    "Lead_Type_Detected": 1 if lead_distance > 0 else 0,
    "Lead_Type": 3 if lead_distance > 0 else 0,
    "Lead_Distance": lead_distance if lead_distance > 0 else 0,
    "ACC_Enabled": 1 if acc_control == 3 else 0,
    "ACC_Standby_Override": 1 if acc_control != 3 else 0,
    "ACC_AKTIV_regelt": 1 if acc_control == 3 else 0,
    "ACC_Limiter_Mode": 0,
    #"ACC_Driving_Type": 3 if lead_distance > 0 else 0,
    "Unknown_03": 106,
    "Unknown_01": 0,
    "Unknown_08": 0,
    "ACC_Special_Events": 3 if esp_hold and acc_control == 3 else 0,
    "Zeitluecke_3_Signal": zeitluecke_3,
    "Zeitluecke_4_Signal": zeitluecke_4,
    "Zeitluecke_5_Signal": zeitluecke_5,
  }

  values.update({
    #"STA_Primaeranz": meb_acc_01_values["STA_Primaeranz"],
    "Unknown_Area_01": meb_acc_01_values["Unknown_Area_01"],
    "SET_ME_0X1": meb_acc_01_values["SET_ME_0X1"],
    "SET_ME_0X3FF": meb_acc_01_values["SET_ME_0X3FF"],
    #"Heartbeat": meb_acc_01_values["Heartbeat"],
    "SET_ME_0XFFFF": meb_acc_01_values["SET_ME_0XFFFF"],
    "SET_ME_0X7FFF": meb_acc_01_values["SET_ME_0X7FFF"],
    #"ACC_Enabled": meb_acc_01_values["ACC_Enabled"],
    #"Unknown_02": meb_acc_01_values["Unknown_03"],
    #"Unknown_03": meb_acc_01_values["Unknown_03"],
    "Unknown_04": meb_acc_01_values["Unknown_04"],
    "Unknown_05": meb_acc_01_values["Unknown_05"],
    "Unknown_06": meb_acc_01_values["Unknown_06"],
    "Unknown_07": meb_acc_01_values["Unknown_07"],
    #"Unknown_08": meb_acc_01_values["Unknown_07"],
    #"Lead_Type_Detected": meb_acc_01_values["Lead_Type_Detected"],
    #"ACC_Standby_Override": meb_acc_01_values["ACC_Standby_Override"],
    #"ACC_AKTIV_regelt": meb_acc_01_values["ACC_AKTIV_regelt"],
    #"ACC_Limiter_Mode": meb_acc_01_values["ACC_Limiter_Mode"],
    "ACC_Driving_Type": meb_acc_01_values["ACC_Driving_Type"],
    #"Lead_Type": meb_acc_01_values["Lead_Type"],
    #"ACC_Special_Events": meb_acc_01_values["ACC_Special_Events"],
    #"Zeitluecke_1_Signal": meb_acc_01_values["Zeitluecke_1_Signal"],
    #"Zeitluecke_2_Signal": meb_acc_01_values["Zeitluecke_2_Signal"],
    #"Zeitluecke_3_Signal": meb_acc_01_values["Zeitluecke_3_Signal"],
    #"Zeitluecke_4_Signal": meb_acc_01_values["Zeitluecke_4_Signal"],
    #"Zeitluecke_5_Signal": meb_acc_01_values["Zeitluecke_5_Signal"],
  })

  return packer.make_can_msg("MEB_ACC_01", bus, values)
