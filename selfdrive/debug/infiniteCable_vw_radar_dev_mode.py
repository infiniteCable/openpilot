#!/usr/bin/env python3

import argparse
import struct
import time
from enum import IntEnum
from opendbc.car.uds import UdsClient, MessageTimeoutError, NegativeResponseError, SESSION_TYPE, DATA_IDENTIFIER_TYPE, ACCESS_TYPE
from opendbc.safety import Safety
from panda import Panda

# Radar ECU-Adresse
RADAR_CAN_ADDR = 0x757  # Dein Radar
RX_OFFSET = 0x6A  # Antwort-Offset kann variieren

# Bekannte Security Access Passphrase
SECURITY_ACCESS_KEY = 0x4E37  # Hex für 20103 (sofern statisch)

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Activates raw radar data (grid points) on Continental radar (MEB).")
  parser.add_argument("--debug", action="store_true", help="Enable UDS debugging")
  parser.add_argument("action", choices={"enable", "disable"}, help="Enable or disable raw radar grid data")
  args = parser.parse_args()

  panda = Panda()
  panda.set_safety_mode(Safety.SAFETY_ELM327)
  bus = 1 if panda.has_obd() else 0  # Wähle den richtigen CAN-Bus

  uds_client = UdsClient(panda, RADAR_CAN_ADDR, RADAR_CAN_ADDR + RX_OFFSET, bus, timeout=0.2)

  try:
    # 1. Entwicklermodus aktivieren (Extended Diagnostic Session)
    print("[INFO] Activating Extended Diagnostic Mode...")
    uds_client.diagnostic_session_control(SESSION_TYPE.EXTENDED_DIAGNOSTIC)
    time.sleep(0.1)

    # 2. Security Access (falls nötig)
    print("[INFO] Requesting Security Access...")
    seed = uds_client.security_access(ACCESS_TYPE.REQUEST_SEED)
    key = SECURITY_ACCESS_KEY  # Falls statisch, sonst müsste der Key berechnet werden
    uds_client.security_access(ACCESS_TYPE.SEND_KEY, struct.pack("!H", key))
    time.sleep(0.1)

    # 3. Prüfen, ob Radar Rohdaten unterstützt (Grid-Punkte)
    print("[INFO] Checking if raw data is supported...")
    raw_data_support = uds_client.read_data_by_identifier(0xF190)  # Alternativ 0xF1A0, 0xF1B0, 0x2000 testen

    print(f"[INFO] Raw Data Support Response: {raw_data_support.hex()}")

    #if args.action == "enable":
    #  print("[INFO] Enabling raw radar grid data...")
    #  uds_client.write_data_by_identifier(0xF190, b"\x01")  # 01 = Aktivieren
    #else:
    #  print("[INFO] Disabling raw radar grid data...")
    #  uds_client.write_data_by_identifier(0xF190, b"\x00")  # 00 = Deaktivieren

    # 4. Prüfen, ob Änderung übernommen wurde
    #time.sleep(0.2)
    #new_status = uds_client.read_data_by_identifier(0xF190)
    #print(f"[INFO] New raw data status: {new_status.hex()}")

    # 5. Session beenden
    uds_client.diagnostic_session_control(SESSION_TYPE.DEFAULT)
    print("[INFO] Diagnostic session closed.")

  except (NegativeResponseError, MessageTimeoutError) as e:
    print(f"[ERROR] UDS request failed: {e}")
