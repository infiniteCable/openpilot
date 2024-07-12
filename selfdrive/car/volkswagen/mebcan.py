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

def acc_control_value(main_switch_on, acc_faulted, long_active):
  if acc_faulted:
    acc_control = 6
  elif long_active:
    acc_control = 3
  elif main_switch_on:
    acc_control = 2
  else:
    acc_control = 0

  return acc_control

def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, stopping, starting, esp_hold, just_started, speed, reversing):
  commands = []

  if starting:
    acc_hold_type = 4  # hold release / startup
  elif just_started:
    acc_hold_type = 5  # hold release quit
  elif esp_hold or stopping:
    acc_hold_type = 1  # hold standby
  else:
    acc_hold_type = 0

  if stopping:
    anhalteweg = 1
  else:
    anhalteweg = 20
  
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
    
    "ACC_Anhalteweg": anhalteweg if acc_enabled else 0,
    "ACC_Anforderung_HMS": acc_hold_type if acc_enabled and acc_control != 4 else 0,
    "Unknown_01": 25 if stopping else 31,
    "Unknown_02": 0 if stopping else 15,
    "ACC_Active": 0,
    "Constant_1_1": 1,
    "Constant_1_2": 1,
    "Constant_1_3": 1,
    "Constant_1_4": 1,
    "Constant_FE": 0xFE,
    "Speed": speed,
    "Reversing": reversing,
  }
  commands.append(packer.make_can_msg("MEB_ACC_02", bus, values))
  
  return commands

def create_acc_hud_control(packer, bus, acc_hud_status, set_speed, lead_distance, distance, meb_acc_01_values):
  values = {s: meb_acc_01_values[s] for s in [
    "NEW_SIGNAL_22",
    "NEW_SIGNAL_9",
    "NEW_SIGNAL_6",
    "NEW_SIGNAL_7",
    "NEW_SIGNAL_5",
    "NEW_SIGNAL_4",
    "NEW_SIGNAL_3",
    "FCW_Active",
    "Constant_F",
    "Constant_63",
    "Heartbeat",
    "Constant_FFFF",
    "NEW_SIGNAL_20",
    "NEW_SIGNAL_21",
    "NEW_SIGNAL_17",
    "ACC_Status_ACC",
    "NEW_SIGNAL_18",
    "NEW_SIGNAL_19",
    "NEW_SIGNAL_11",
    "NEW_SIGNAL_12",
    "NEW_SIGNAL_13",
    "NEW_SIGNAL_14",
    "NEW_SIGNAL_16",
    "NEW_SIGNAL_15",
    "NEW_SIGNAL_1",
  ]}

  values.update({
    "ACC_Status_ACC": acc_hud_status,
    "Lead_Distance": lead_distance,
  })
  
  #values = {
  #  "ACC_Status_Anzeige": acc_hud_status,
  #  "ACC_Wunschgeschw_02": set_speed if set_speed < 250 else 327.36,
  #  "ACC_Gesetzte_Zeitluecke": distance + 2,
  #  "ACC_Display_Prio": 3,
  #  "ACC_Abstandsindex": lead_distance,
  #}

  return packer.make_can_msg("MEB_ACC_01", bus, values)
