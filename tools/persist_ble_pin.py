#!/usr/bin/env python3
"""Bluetooth PIN that survives restarts.
Stock MeshCore picks a NEW random pairing PIN every time a companion radio with a screen starts, so any restart breaks the phone's saved pairing.
This patch works the PIN out from the chip's own unique ID, so it is the same at every restart and nothing is written to flash at boot ("set pin" still overrides it).
Usage: python3 tools/persist_ble_pin.py --meshcore <MeshCore checkout>.  Safe to run twice; builds without Bluetooth or a screen are left alone."""
import argparse, os, sys

OLD = ("      StdRNG rng;\n"
       "      the_mesh.setBLEPin(rng.nextInt(100000, 999999)); // random pin each session\n")
NEW = ("      StdRNG rng;\n"
       "#if defined(ESP32)\n"
       "      // LODESTAR: the PIN comes from the chip's own unique ID, so it is the same at every restart and nothing has to be written to flash at boot\n"
       "      uint64_t ls_mac = ESP.getEfuseMac(); uint32_t ls_h = 2166136261u;\n"
       "      for (int ls_i = 0; ls_i < 6; ls_i++) { ls_h ^= (uint8_t)(ls_mac >> (8 * ls_i)); ls_h *= 16777619u; }\n"
       "      the_mesh.setBLEPin(100000 + ls_h % 900000);\n"
       "#else\n"
       "      the_mesh.setBLEPin(rng.nextInt(100000, 999999)); // random pin each session\n"
       "#endif\n")

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--meshcore", required=True); a = ap.parse_args()
    p = os.path.join(a.meshcore, "examples", "companion_radio", "main.cpp")
    if not os.path.isfile(p):
        print("persist_ble_pin: no companion_radio/main.cpp here, nothing to do"); return
    t = open(p).read()
    if "chip's own unique ID" in t:
        print("persist_ble_pin: already applied"); return
    if t.count(OLD) != 1:
        print("persist_ble_pin: WARNING - upstream MeshCore changed the PIN code; the PIN will still change at every restart. Update tools/persist_ble_pin.py.", file=sys.stderr); return
    open(p, "w").write(t.replace(OLD, NEW)); print("persist_ble_pin: Bluetooth PIN now comes from the chip ID")

if __name__ == "__main__":
    main()
