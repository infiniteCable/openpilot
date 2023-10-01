def create_steering_control(packer, bus, apply_steer, lkas_enabled, lkas_request):
  values = {
    "SET_ME_0XF": 0xF,
    "HCA_01_LM_Offset": abs(apply_steer),
    "HCA_01_Request": lkas_request,
    "HCA_01_LM_OffSign": 1 if apply_steer < 0 and lkas_enabled == 1 else 0,
    "HCA_01_Enabled": lkas_enabled,
    "HCA_01_Standby": not lkas_enabled,
    "SET_ME_0XFE": 0xFE,
    "SET_ME_0X1": 0x1,
  }
  return packer.make_can_msg("HCA_01", bus, values)


def create_lka_hud_control(packer, bus, ldw_stock_values, enabled, steering_pressed, hud_alert, hud_control):
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
    "LDW_Status_LED_gelb": 1 if enabled and steering_pressed else 0,
    "LDW_Status_LED_gruen": 1 if enabled and not steering_pressed else 0,
    "LDW_Lernmodus_links": 3 if hud_control.leftLaneDepart else 1 + hud_control.leftLaneVisible,
    "LDW_Lernmodus_rechts": 3 if hud_control.rightLaneDepart else 1 + hud_control.rightLaneVisible,
    "LDW_Texte": hud_alert,
  })
  return packer.make_can_msg("LDW_02", bus, values)


def create_acc_buttons_control(packer, bus, gra_stock_values, cancel=False, resume=False):
  values = {s: gra_stock_values[s] for s in [
    "GRA_Hauptschalter",           # GRA button, on/off
    #"GRA_Abbrechen",               # GRA button cancel
    "GRA_Typ_Hauptschalter",       # GRA main button type
    #"GRA_Limiter",                 # GRA Limiter
    #"GRA_Tip_Setzen",              # GRA set
    #"GRA_Tip_Hoch",                # GRA up
    #"GRA_Tip_Runter",              # GRA down
    #"GRA_Tip_Wiederaufnahme"       # GRA resume
    #"GRA_Verstellung_Zeitlücke",   # GRA change time gap
    "GRA_Codierung",               # GRA button configuration/coding
    #"GRA_Fehler",                  # GRA error
    #"GRA_Typ468",                  # GRA Typ468
    "GRA_Tip_Stufe_2",             # unknown related to stalk type
    "GRA_ButtonTypeInfo",          # unknown related to stalk type
  ]}

  values.update({
    "COUNTER": (gra_stock_values["COUNTER"] + 1) % 16,
    "GRA_Abbrechen": cancel,
    "GRA_Tip_Wiederaufnahme": resume,
  })

  return packer.make_can_msg("GRA_ACC_01", bus, values)
