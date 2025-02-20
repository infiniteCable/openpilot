#!/usr/bin/env python3

import struct
import time
from opendbc.car.uds import UdsClient, MessageTimeoutError, NegativeResponseError
from opendbc.safety import Safety  # Import für Safety-Modi
from panda import Panda

# Radar ECU-Adresse mit gefundenem RX Offset
RADAR_CAN_ADDR = 0x757  # Dein Radar
RX_OFFSET = 0x6A  # Gefundene Antwort-Adresse
BUS = 1  # Falls notwendig, auf 0 wechseln

# Teste DIDs von 0x0000 bis 0xFFFF
DID_START = 0x0000
DID_END = 0xFFFF

if __name__ == "__main__":
  print("[INFO] Starting UDS Scan on Continental Radar...")

  panda = Panda()
  panda.set_safety_mode(Safety.SAFETY_ELM327)  # Freier Zugriff auf UDS

  uds_client = UdsClient(panda, RADAR_CAN_ADDR, RADAR_CAN_ADDR + RX_OFFSET, BUS, timeout=0.2)

  try:
    # 1. Entwicklermodus aktivieren (Extended Diagnostic Session)
    print("[INFO] Activating Extended Diagnostic Mode...")
    uds_client.diagnostic_session_control(0x03)
    time.sleep(0.1)

    # 2. Security Access (falls nötig)
    print("[INFO] Requesting Security Access...")
    try:
      seed = uds_client.security_access(0x01)
      key = 0x4E37  # Falls statisch, sonst müsste der Key berechnet werden
      uds_client.security_access(0x02, struct.pack("!H", key))
      print("[INFO] Security Access successful.")
    except NegativeResponseError as e:
      print(f"[WARNING] Security Access not required or failed: {e}")

    # 3. UDS-Scan durchführen
    print("[INFO] Scanning for valid Data Identifiers (DIDs)...")
    found_dids = {}

    for did in range(DID_START, DID_END + 1):
      try:
        response = uds_client.read_data_by_identifier(did)
        hex_response = response.hex()

        # Falls Antwort Text enthält, dekodieren
        try:
          text_response = response.decode("utf-8").strip()
          print(f"[FOUND] DID 0x{did:04X}: {text_response} ({hex_response})")
        except UnicodeDecodeError:
          print(f"[FOUND] DID 0x{did:04X}: {hex_response}")

        found_dids[did] = hex_response
      except NegativeResponseError as e:
        if "request out of range" in str(e):
          continue  # DID wird nicht unterstützt, weitermachen
        print(f"[ERROR] Negative Response for DID 0x{did:04X}: {e}")
      except MessageTimeoutError:
        continue  # Keine Antwort, weitermachen

    print(f"[INFO] Scan complete. {len(found_dids)} valid DIDs found.")

    # 4. Session beenden
    uds_client.diagnostic_session_control(0x01)
    print("[INFO] Diagnostic session closed.")

  except (NegativeResponseError, MessageTimeoutError) as e:
    print(f"[ERROR] UDS request failed: {e}")
