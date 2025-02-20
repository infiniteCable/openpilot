#!/usr/bin/env python3

import struct
import time
from opendbc.car.uds import UdsClient, MessageTimeoutError, NegativeResponseError
from opendbc.safety import Safety
from panda import Panda

# Radar ECU-Adresse
RADAR_CAN_ADDR = 0x757  # Dein Radar
RX_OFFSET = 0x8A  # Antwort-Offset kann variieren

# Scanbereich (Testet alle DIDs von 0x0000 bis 0xFFFF)
DID_START = 0x0000
DID_END = 0xFFFF

# Optional: Liste bekannter DIDs (falls vorher getestet)
KNOWN_DIDS = {
  0xF190: "Software Version",
  0xF1A0: "Radar Debug Data",
  0xF1B0: "Extended Sensor Data",
  0x2000: "Engineering Mode Parameter",
  0x3000: "Raw Radar Grid Activation",
}

if __name__ == "__main__":
  print("[INFO] Starting UDS Scan on Continental Radar...")

  panda = Panda()
  panda.set_safety_mode(Safety.SAFETY_ELM327)
  bus = 0 #1 if panda.has_obd() else 0  # Wähle den richtigen CAN-Bus

  uds_client = UdsClient(panda, RADAR_CAN_ADDR, RADAR_CAN_ADDR + RX_OFFSET, bus, timeout=0.2)

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
    except NegativeResponseError:
      print("[WARNING] Security Access not required or failed.")

    # 3. UDS-Scan durchführen
    print("[INFO] Scanning for valid Data Identifiers (DIDs)...")
    found_dids = {}

    for did in range(DID_START, DID_END + 1):
      try:
        response = uds_client.read_data_by_identifier(did)
        hex_response = response.hex()

        # Prüfen, ob das DID bekannt ist
        did_name = KNOWN_DIDS.get(did, "Unknown DID")

        # Falls Antwort Text enthält, dekodieren
        try:
          text_response = response.decode("utf-8").strip()
          print(f"[FOUND] DID 0x{did:04X}: {did_name} → {text_response} ({hex_response})")
        except UnicodeDecodeError:
          print(f"[FOUND] DID 0x{did:04X}: {did_name} → {hex_response}")

        found_dids[did] = hex_response
      except NegativeResponseError as e:
        # Falls Antwort `7F 22 31` ist, wird die DID nicht unterstützt
        if e.response_code == 0x31:
          continue
      except MessageTimeoutError:
        continue  # Keine Antwort erhalten, überspringen

    print(f"[INFO] Scan completed. {len(found_dids)} valid DIDs found.")

    # 4. Session beenden
    uds_client.diagnostic_session_control(0x01)
    print("[INFO] Diagnostic session closed.")

  except (NegativeResponseError, MessageTimeoutError) as e:
    print(f"[ERROR] UDS request failed: {e}")

