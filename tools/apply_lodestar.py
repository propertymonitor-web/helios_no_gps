#!/usr/bin/env python3
"""
apply_lodestar.py - add Lodestar (mobile-repeater uplink scoring) to a MeshCore checkout.

  python tools/apply_lodestar.py --meshcore MeshCore

Copies lodestar/Lodestar.h into MeshCore/src/helpers and patches
examples/simple_repeater (MyMesh.h / MyMesh.cpp). Every edit must match exactly
once; if upstream MeshCore moved something the script stops and says which
anchor it could not find, instead of producing a half-patched tree.

Run tools/make_splash.py afterwards (or before) for the boot screen; the two
tools touch different files.
"""
import argparse
import os
import re
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
LODESTAR_H = os.path.join(HERE, "..", "lodestar", "Lodestar.h")


def die(msg):
    print(f"\nERROR: {msg}\n", file=sys.stderr)
    sys.exit(1)


def patch_once(text, pattern, repl, what, flags=0):
    """Apply one regex replacement; it must match exactly one place."""
    found = re.findall(pattern, text, flags)
    if len(found) != 1:
        die(f"Upstream MeshCore changed: expected exactly 1 match for '{what}', found {len(found)}. "
            f"tools/apply_lodestar.py anchor for this edit needs updating.")
    return re.sub(pattern, lambda m: repl(m) if callable(repl) else repl, text, count=1, flags=flags)


# --------------------------------------------------------------------------
# C++ snippets inserted into MyMesh.cpp
# --------------------------------------------------------------------------
OBSERVE_RX = r'''  // --- Lodestar: learn from every flood packet we hear ---
  if (pkt->isRouteFlood()) {
    uint32_t ls_now = millis();
    lodestar.cfg.sf = _prefs.sf;
    uint8_t ls_n = pkt->getPathHashCount();
    uint8_t ls_sz = pkt->getPathHashSize();
    if (ls_n > 0) {
      // the last hop in the path is the repeater we just heard directly
      int ls_from = lodestar.onHeardFrom(&pkt->path[(ls_n - 1) * ls_sz], ls_sz, pkt->getSNR(),
                                         _radio->getLastRSSI(), ls_now);
      if (pkt->getPayloadType() == PAYLOAD_TYPE_ADVERT && ls_n >= 2) lodestar.noteReach(ls_from, ls_n - 1);
      uint8_t ls_hash[MAX_HASH_SIZE];
      pkt->calculatePacketHash(ls_hash);
      // implicit ACK for anything we sent, and: has a strong uplink already forwarded this?
      if (lodestar.onFloodCopy(ls_hash, pkt->path, ls_n, ls_sz, ls_from, ls_now)) {
        // drop our own queued retransmit of the same packet (airtime saved, uplink covers it)
        for (int ls_i = _mgr->getOutboundTotal() - 1; ls_i >= 0; ls_i--) {
          mesh::Packet* ls_q = _mgr->getOutboundByIdx(ls_i);
          if (!ls_q || !ls_q->isRouteFlood()) continue;
          uint8_t ls_qn = ls_q->getPathHashCount();
          uint8_t ls_qs = ls_q->getPathHashSize();
          if (ls_qn == 0 || !self_id.isHashMatch(&ls_q->path[(ls_qn - 1) * ls_qs], ls_qs)) continue;
          uint8_t ls_qh[MAX_HASH_SIZE];
          ls_q->calculatePacketHash(ls_qh);
          if (memcmp(ls_qh, ls_hash, MAX_HASH_SIZE) == 0) {
            mesh::Packet* ls_rm = _mgr->removeOutboundByIdx(ls_i);
            if (ls_rm) {
              _mgr->free(ls_rm);
              lodestar.noteCancel();
            }
          }
        }
      }
    }
  }
'''

OBSERVE_TX = r'''  // --- Lodestar: remember flood packets we send, to watch for their echo ---
  if (pkt->isRouteFlood()) {
    uint8_t ls_hash[MAX_HASH_SIZE];
    pkt->calculatePacketHash(ls_hash);
    uint32_t ls_win = _radio->getEstAirtimeFor(len) * 12;
    if (ls_win < 4000) ls_win = 4000;
    if (ls_win > 15000) ls_win = 15000;
    lodestar.onTx(ls_hash, pkt->path, pkt->getPathHashCount(), pkt->getPathHashSize(), millis(), ls_win);
  }
'''

GPS_FEED = r'''#if ENV_INCLUDE_GPS == 1
  {  // --- Lodestar: hand the GPS fix (if any) to Lodestar; read-only, changes no forwarding ---
    LocationProvider* ls_gps = sensors.getLocationProvider();
    if (ls_gps != NULL && ls_gps->isEnabled()) {
      lodestar.onPosition((int32_t)ls_gps->getLatitude(), (int32_t)ls_gps->getLongitude(),
                          (uint8_t)ls_gps->satellitesCount(), ls_gps->isValid(), millis());
    }
  }
#endif
'''

LOAD_SETTINGS = r'''  {  // --- Lodestar: restore saved settings (if any); anything invalid is ignored ---
    char ls_buf[128];
    File ls_f = _fs->open("/lodestar.cfg");
    if (ls_f) {
      int ls_n = ls_f.read((uint8_t*)ls_buf, sizeof(ls_buf) - 1);
      ls_f.close();
      if (ls_n > 0) { ls_buf[ls_n] = 0; lodestar.loadSettings(ls_buf); }
    }
  }
'''

SAVE_SETTINGS = r'''  if (lodestar.settings_dirty) {   // --- Lodestar: a setting changed, so write it to flash (rare) ---
    lodestar.settings_dirty = false;
    char ls_buf[128];
    size_t ls_n = lodestar.saveSettings(ls_buf, sizeof(ls_buf));
    if (ls_n > 0) {
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
      _fs->remove("/lodestar.cfg");
      File ls_f = _fs->open("/lodestar.cfg", FILE_O_WRITE);
#elif defined(RP2040_PLATFORM)
      File ls_f = _fs->open("/lodestar.cfg", "w");
#else
      File ls_f = _fs->open("/lodestar.cfg", "w", true);
#endif
      if (ls_f) { ls_f.write((const uint8_t*)ls_buf, ls_n); ls_f.close(); }
    }
  }
'''

SERIAL_PRINT = r'''static void lodestarSerialPrint(const char* line) {
  Serial.println(line);
}

'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--meshcore", required=True, help="path to a MeshCore checkout")
    args = ap.parse_args()

    root = args.meshcore
    rep = os.path.join(root, "examples", "simple_repeater")
    h_path = os.path.join(rep, "MyMesh.h")
    c_path = os.path.join(rep, "MyMesh.cpp")
    for p in (h_path, c_path, LODESTAR_H):
        if not os.path.isfile(p):
            die(f"Cannot find {p}. Is --meshcore pointing at a MeshCore checkout?")

    with open(c_path) as f:
        cpp = f.read()
    with open(h_path) as f:
        hdr = f.read()
    if "lodestar" in cpp or "lodestar" in hdr:
        die("This MeshCore checkout already has Lodestar in it. Use a fresh clone.")

    # ---- library file ----------------------------------------------------
    dest = os.path.join(root, "src", "helpers", "Lodestar.h")
    shutil.copyfile(LODESTAR_H, dest)

    # ---- MyMesh.h --------------------------------------------------------
    hdr = patch_once(hdr, r"#include <helpers/RoutingPolicy\.h>\n",
                     "#include <helpers/RoutingPolicy.h>\n#include <helpers/Lodestar.h>\n",
                     "RoutingPolicy include")
    hdr = patch_once(hdr, r"(class MyMesh : public mesh::Mesh, public CommonCLICallbacks \{\n)",
                     lambda m: m.group(1) + "  lodestar::Lodestar lodestar;   // uplink scoring (Lodestar)\n",
                     "class MyMesh declaration")

    # ---- MyMesh.cpp ------------------------------------------------------
    # 1. watch every received packet
    cpp = patch_once(
        cpp, r"(mesh::DispatcherAction MyMesh::onRecvPacket\(mesh::Packet\* pkt\) \{\n)",
        lambda m: m.group(1) + OBSERVE_RX, "onRecvPacket")

    # 2. zero-hop repeater adverts feed the table
    cpp = patch_once(
        cpp, r"(      putNeighbour\(id, timestamp, packet->getSNR\(\)\);\n)",
        lambda m: m.group(1) + "      lodestar.onAdvert(id.pub_key, packet->getSNR(), _radio->getLastRSSI(), millis());\n",
        "putNeighbour call in onAdvertRecv")

    # 3. note flood packets we transmit
    cpp = patch_once(
        cpp, r"(void MyMesh::logTx\(mesh::Packet \*pkt, int len\) \{\n)",
        lambda m: m.group(1) + OBSERVE_TX, "logTx")

    # 4. scale the flood retransmit delay by uplink score (direct delay untouched)
    cpp = patch_once(
        cpp,
        r"(uint32_t MyMesh::getRetransmitDelay\(const mesh::Packet \*packet\) \{.*?)"
        r"return getRNG\(\)->nextInt\(0, 5\*t \+ 1\);",
        lambda m: m.group(1) + "uint32_t ls_d = getRNG()->nextInt(0, 5*t + 1);\n"
                  "  return (uint32_t)(ls_d * lodestar.delayFactor(millis()));   // Lodestar: best-placed goes first",
        "getRetransmitDelay", flags=re.S)

    # 5. housekeeping, and the GPS fix (sensors.loop() in main.cpp keeps the GPS parsed)
    cpp = patch_once(
        cpp, r"(  mesh::Mesh::loop\(\);\n)",
        lambda m: m.group(1) + "  lodestar.loop(millis());\n" + SAVE_SETTINGS + GPS_FEED, "Mesh::loop call in MyMesh::loop")

    # 5b. keep Lodestar's settings across reboots: load once at start-up, save when changed
    cpp = patch_once(
        cpp, r"(  _cli\.loadPrefs\(_fs\);\n)",
        lambda m: m.group(1) + LOAD_SETTINGS, "loadPrefs call in MyMesh::begin")

    # 6. serial / remote CLI:  lodestar [table|gps|weights r l h|floor f|cancel on|off|f|on|off|defaults|reset]
    cpp = patch_once(
        cpp, r"(void MyMesh::handleCommand\(uint32_t sender_timestamp, char \*command, char \*reply\) \{\n)",
        lambda m: SERIAL_PRINT + m.group(1), "handleCommand")
    cpp = patch_once(
        cpp, r"\} else\s*\{\s*\n(\s*)_cli\.handleCommand\(sender_timestamp, command, reply\);",
        lambda m: ('} else if (memcmp(command, "lodestar", 8) == 0 && (command[8] == 0 || command[8] == \' \')) {\n'
                   '    lodestar.runCommand(command + 8, reply, 150, millis(), lodestarSerialPrint);\n'
                   '  } else {\n' + m.group(1) + '_cli.handleCommand(sender_timestamp, command, reply);'),
        "common CLI fallthrough in handleCommand")

    with open(c_path, "w") as f:
        f.write(cpp)
    with open(h_path, "w") as f:
        f.write(hdr)

    print("Lodestar applied:")
    print(f"  + {os.path.relpath(dest, root)}")
    print(f"  ~ {os.path.relpath(h_path, root)}  (include + member)")
    print(f"  ~ {os.path.relpath(c_path, root)}  (rx/tx observe, advert feed, delay scaling, loop, saved settings, GPS, CLI)")


if __name__ == "__main__":
    main()
