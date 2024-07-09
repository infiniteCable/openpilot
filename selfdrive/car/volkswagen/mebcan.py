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

def acc_control_value(main_switch_on, acc_faulted, long_active, user_overriding, disabling, enabling):
  if acc_faulted:
    acc_control = 7
  elif long_active:
    if enabling:
      acc_control = 4
    elif disabling:
      acc_control = 5
    elif user_overriding:
      acc_control = 12
    else:
      acc_control = 3
  elif main_switch_on:
    acc_control = 2
  else:
    acc_control = 0

  return acc_control

def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, stopping, starting, esp_hold, disabling, enabling, speed, reversing, user_overriding):
  commands = []
  
  values = {
    "ACC_State": acc_control,
    "ACC_Active": acc_enabled,
    "Regulating_Strength": 5 if acc_enabled and not esp_hold else 0,
    "Accel": accel if acc_enabled else 6,
    "Starting": starting,
    "Disabling_Starting": starting or disabling,
    "Stopping": stopping,
    "Stopping_02": stopping,
    "Anti_Stopping": 0 if stopping else 1,
    "Anti_Stopping_02": 0 if stopping else 1,
    "Anti_Stopping_03": 0 if stopping else 1,
    "Anti_Stopping_04": 0 if stopping else 7,
    "Anti_Stopping_05": 0 if stopping else 1,
    "Anti_Stopping_06": 0 if stopping else 1,
    "Anti_Stopping_07": 0 if stopping else 1,
    "Disabling_or_Active_Hold": esp_hold and acc_enabled or disabling,
    "Constant_1_1": 1,
    "Constant_1_2": 1,
    "Constant_1_3": 1,
    "Constant_1_5": 1,
    "Constant_1_6": 1,
    "Constant_FE": 0xFE,
    "Speed": speed,
    "Secondary_Accel": 0, #if esp_hold else 1,
    "Secondary_Accel_02": 20 if acc_enabled and not esp_hold else 0,
    "Secondary_Accel_03": 10 if acc_enabled and not esp_hold else 0,
    "Reversing": reversing,
    "User_Override_State": 3, # if user_overriding else 0,
    "Stopping_Stronger": 0,    
  }
  commands.append(packer.make_can_msg("MEB_ACC_02", bus, values))
  
  return commands
