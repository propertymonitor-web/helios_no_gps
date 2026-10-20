#!/usr/bin/env python3
"""
apply_helios.py - build LODESTAR HELIOS on top of a MeshCore checkout.

  python tools/apply_helios.py --meshcore MeshCore --variant gps      (SenseCAP Solar Node P1-Pro)
  python tools/apply_helios.py --meshcore MeshCore --variant nogps    (SenseCAP Solar Node P1, no GPS)

1. runs apply_lodestar.py (the PULSAR uplink-scoring patch) on the checkout,
2. adds helios/Helios.h (battery and solar watch) and wires it into the repeater:
   a battery reading once a minute and a `helios` command,
3. adds the build environment HELIOS_GPS_repeater or HELIOS_NoGps_repeater to the
   SenseCAP Solar variant. The no-GPS one removes ENV_INCLUDE_GPS so the GPS driver is not built.

Every edit must match exactly once; if upstream MeshCore moved something the script stops and says which
anchor it could not find, instead of leaving a half-patched tree.
"""
import argparse
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
HELIOS_H = os.path.join(HERE, "..", "helios", "Helios.h")


def die(msg):
    print(f"\nERROR: {msg}\n", file=sys.stderr)
    sys.exit(1)


def once(text, old, new, what):
    if text.count(old) != 1:
        die(f"Upstream MeshCore changed: expected exactly 1 match for '{what}', found {text.count(old)}. "
            f"tools/apply_helios.py needs a new anchor for it.")
    return text.replace(old, new, 1)


ENV_TEMPLATE = """
[env:{env}]
extends = SenseCap_Solar
build_flags =
  ${{SenseCap_Solar.build_flags}}
  -D ADVERT_NAME='"LODESTAR HELIOS"'
  -D ADVERT_LAT=0.0
  -D ADVERT_LON=0.0
  -D ADMIN_PASSWORD='"password"'
  -D MAX_NEIGHBOURS=50
build_src_filter = ${{SenseCap_Solar.build_src_filter}}
  +<../examples/simple_repeater/*.cpp>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--meshcore", required=True)
    ap.add_argument("--variant", required=True, choices=["gps", "nogps"])
    args = ap.parse_args()
    root = args.meshcore

    r = subprocess.run([sys.executable, os.path.join(HERE, "apply_lodestar.py"), "--meshcore", root])
    if r.returncode != 0:
        die("apply_lodestar.py failed (see above).")

    rep = os.path.join(root, "examples", "simple_repeater")
    h_path, c_path = os.path.join(rep, "MyMesh.h"), os.path.join(rep, "MyMesh.cpp")
    ini_path = os.path.join(root, "variants", "sensecap_solar", "platformio.ini")
    for p in (h_path, c_path, ini_path, HELIOS_H):
        if not os.path.isfile(p):
            die(f"Cannot find {p}.")
    shutil.copyfile(HELIOS_H, os.path.join(root, "src", "helpers", "Helios.h"))

    hdr = open(h_path).read()
    hdr = once(hdr, "#include <helpers/Lodestar.h>\n", "#include <helpers/Lodestar.h>\n#include <helpers/Helios.h>\n",
               "Lodestar include in MyMesh.h")
    hdr = once(hdr, "  lodestar::Lodestar lodestar;   // uplink scoring (Lodestar)\n",
               "  lodestar::Lodestar lodestar;   // uplink scoring (Lodestar)\n  heliosfw::Helios helios;       // battery and solar watch (Helios)\n",
               "Lodestar member in MyMesh.h")

    cpp = open(c_path).read()
    cpp = once(cpp, "  lodestar.loop(millis());\n",
               "  lodestar.loop(millis());\n"
               "  if (helios.due(millis())) helios.sample(board.getBattMilliVolts(), millis());   // Helios: one battery reading a minute\n",
               "Lodestar loop call in MyMesh.cpp")
    label = "alpha4 (P1-Pro, GPS)" if args.variant == "gps" else "alpha4 (no GPS)"
    cpp = once(cpp, '} else if (memcmp(command, "lodestar", 8) == 0 && (command[8] == 0 || command[8] == \' \')) {',
               '} else if (strcmp(command, "firmware") == 0) {\n'
               '    strcpy(reply, "LODESTAR HELIOS ' + label + '");\n'
               '  } else if (strcmp(command, "fear") == 0 || strcmp(command, "FEAR") == 0) {\n'
               '    strcpy(reply, "First Encounter And Response\\nLODESTAR HELIOS\\nCreated by: Aries Thorne and Todd Steinhauer\\nFor: Wolf Reactive Systems");\n'
               '  } else if (memcmp(command, "helios", 6) == 0 && (command[6] == 0 || command[6] == \' \')) {\n'
               '    helios.runCommand(command + 6, reply, 150, millis(), lodestarSerialPrint);\n'
               '  } else if (memcmp(command, "lodestar", 8) == 0 && (command[8] == 0 || command[8] == \' \')) {',
               "lodestar command branch in MyMesh.cpp")
    open(h_path, "w").write(hdr)
    open(c_path, "w").write(cpp)

    ini = open(ini_path).read()
    env = "HELIOS_GPS_repeater" if args.variant == "gps" else "HELIOS_NoGps_repeater"
    if args.variant == "nogps":
        # no GPS: do not build the GPS driver at all (target.cpp tests #ifdef, so it must be undefined, not 0)
        ini = once(ini, "  -D ENV_INCLUDE_GPS=1\n", "", "ENV_INCLUDE_GPS define in the SenseCap_Solar base")
    ini = ini.rstrip("\n") + "\n" + ENV_TEMPLATE.format(env=env)
    open(ini_path, "w").write(ini)

    print(f"Helios applied ({args.variant}):")
    print("  + src/helpers/Helios.h")
    print("  ~ examples/simple_repeater/MyMesh.h, MyMesh.cpp  (battery reading, helios command)")
    print(f"  ~ variants/sensecap_solar/platformio.ini  (new environment {env})")


if __name__ == "__main__":
    main()
