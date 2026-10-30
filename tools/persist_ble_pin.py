#!/usr/bin/env python3
"""Bluetooth PIN that survives restarts.
Stock MeshCore picks a NEW random pairing PIN every time a companion radio with a screen starts, so any restart breaks the phone's saved pairing.
This patch makes one random PIN the first time the radio starts, saves it in the radio's settings and keeps it ("set pin" still overrides it).
Usage: python3 tools/persist_ble_pin.py --meshcore <MeshCore checkout>.  Safe to run twice; builds without Bluetooth or a screen are left alone."""
import argparse, os, sys

OLD = ("      StdRNG rng;\n"
       "      the_mesh.setBLEPin(rng.nextInt(100000, 999999)); // random pin each session\n")
NEW = ("      StdRNG rng;\n"
       "      uint32_t ls_pin = rng.nextInt(100000, 999999);   // LODESTAR: made once for this radio, then kept\n"
       "      the_mesh.getNodePrefs()->ble_pin = ls_pin;\n"
       "      the_mesh.savePrefs();\n"
       "      the_mesh.setBLEPin(ls_pin);\n")

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--meshcore", required=True); a = ap.parse_args()
    p = os.path.join(a.meshcore, "examples", "companion_radio", "main.cpp")
    if not os.path.isfile(p):
        print("persist_ble_pin: no companion_radio/main.cpp here, nothing to do"); return
    t = open(p).read()
    if "made once for this radio" in t:
        print("persist_ble_pin: already applied"); return
    if t.count(OLD) != 1:
        print("persist_ble_pin: WARNING - upstream MeshCore changed the PIN code; the PIN will still change at every restart. Update tools/persist_ble_pin.py.", file=sys.stderr); return
    open(p, "w").write(t.replace(OLD, NEW)); print("persist_ble_pin: Bluetooth PIN is now made once and kept")

if __name__ == "__main__":
    main()
