#!/usr/bin/env python3
"""
Molten MOSFET: generate the custom-config XML descriptor + signature for
mm_config (the dyno bus-clamp knobs surfaced in stock VESC Tool).

This replaces the VESC Tool "generate C code" step for our small hand-authored
config. It emits:
  * the settings XML that VESC Tool parses to render the page (with tooltips),
  * the config signature (crc32c over the serialize order), and
  * the length-prefixed, zlib-compressed byte blob embedded in mm_config.c.

The signature algorithm was reverse-engineered from VESC Tool
(configparams.cpp::getSignature + utility.cpp::crc32c) and validated: it
reproduces BALANCE_CONFIG_SIGNATURE (32903057) exactly from the balance
example's settings.xml. VESC Tool computes the same value from the XML it
receives and prepends it to COMM_SET_CUSTOM_CONFIG payloads; confparser
(mm_config.c) rejects any mismatch, so the firmware constant MUST equal this.

Run:  python3 documentation/mm_config_xml_gen.py
Paste the emitted MM_CONFIG_SIGNATURE and mm_config_xml[] into mm_config.c/.h.

Keep this file and mm_config.c in lockstep: the <SerOrder> here and the
serialize order in mm_config.c's get_cfg/set_cfg must match field-for-field.
"""

import zlib

# --- VESC Tool type codes (from settings.xml) ---
T_INT = 0        # section header / read-only label when transmittable=0
T_DOUBLE = 1     # float
T_INT_EDIT = 2   # integer
T_CFG_NAME = 3   # config identifier (not serialized)
T_ENUM = 4
T_BOOL = 5

# vTx transmit-type codes. 9 = float32_auto, 3 = uint16. Bool/enum omit <vTx>
# (VESC Tool defaults them to 0; validated against the balance signature).
VTX_FLOAT32_AUTO = 9

XML_HEADER = '<?xml version="1.0" encoding="UTF-8"?>\n'


def esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


class Param:
    def __init__(self, name, ptype, longName, tx=1, vTx=0, desc="",
                 valDouble=0.0, minDouble=0.0, maxDouble=0.0, stepDouble=1.0,
                 decimals=2, suffix="", valInt=0, enumNames=None, valString=""):
        self.name = name
        self.ptype = ptype
        self.longName = longName
        self.tx = tx
        self.vTx = vTx
        self.desc = desc
        self.valDouble = valDouble
        self.minDouble = minDouble
        self.maxDouble = maxDouble
        self.stepDouble = stepDouble
        self.decimals = decimals
        self.suffix = suffix
        self.valInt = valInt
        self.enumNames = enumNames or []
        self.valString = valString

    def xml(self):
        L = [f"        <{self.name}>"]
        L.append(f"            <longName>{esc(self.longName)}</longName>")
        L.append(f"            <type>{self.ptype}</type>")
        L.append(f"            <transmittable>{self.tx}</transmittable>")
        L.append(f"            <description>{esc(self.desc)}</description>")
        L.append("            <cDefine></cDefine>")
        if self.ptype == T_CFG_NAME:
            L.append(f"            <valString>{esc(self.valString)}</valString>")
            L.append("            <maxLen>0</maxLen>")
        elif self.ptype == T_DOUBLE:
            L.append(f"            <editorDecimalsDouble>{self.decimals}</editorDecimalsDouble>")
            L.append("            <editorScale>1</editorScale>")
            L.append("            <editAsPercentage>0</editAsPercentage>")
            L.append(f"            <maxDouble>{self.maxDouble:g}</maxDouble>")
            L.append(f"            <minDouble>{self.minDouble:g}</minDouble>")
            L.append("            <showDisplay>0</showDisplay>")
            L.append(f"            <stepDouble>{self.stepDouble:g}</stepDouble>")
            L.append(f"            <valDouble>{self.valDouble:g}</valDouble>")
            L.append("            <vTxDoubleScale>1000</vTxDoubleScale>")
            L.append(f"            <suffix>{esc(self.suffix)}</suffix>")
            L.append(f"            <vTx>{self.vTx}</vTx>")
        elif self.ptype in (T_ENUM, T_BOOL):
            L.append(f"            <valInt>{self.valInt}</valInt>")
            for e in self.enumNames:
                L.append(f"            <enumNames>{esc(e)}</enumNames>")
        L.append(f"        </{self.name}>")
        return "\n".join(L)


# --- Parameter definitions -------------------------------------------------
# NOTE: order of the transmittable params here defines the serialize order and
# MUST match get_cfg/set_cfg in mm_config.c.
PARAMS = [
    Param("config_name", T_CFG_NAME, "none", tx=0, valString="mm_config"),
    # hw_name is the custom-config page title. VESC Tool's config page looks it
    # up by name (getParam("hw_name")) and dereferences the result without a
    # null check, so a config XML that omits it segfaults VESC Tool on connect.
    Param("hw_name", T_INT, "Molten MOSFET Dyno", tx=0,
          desc="Molten MOSFET dyno fork configuration: the DC-link capacitance "
               "(which sets the bus-clamp gains) and the optional boot auto-arm "
               "for the d-axis bus clamp. See documentation/mm_d_axis_dissipation.md."),

    Param("c_dc_uf", T_DOUBLE, "DC-link capacitance", vTx=VTX_FLOAT32_AUTO,
          suffix=" uF", minDouble=0.0, maxDouble=200000.0, stepDouble=10.0,
          decimals=1, valDouble=0.0,
          desc="Total DC-link bus capacitance in microfarads. The firmware "
               "derives the voltage-clamp PI gains from this (target damping "
               "1.1, natural frequency 71 Hz) so the clamp response stays "
               "invariant as the bus bank changes. 0 keeps the compiled 2 mF "
               "bench defaults. Measure the real bank (sum the bus cans) before "
               "the high-voltage rig: larger C with unscaled gains is the slow, "
               "underdamped, dangerous direction."),

    Param("autoarm_en", T_BOOL, "Arm the bus clamp at boot", valInt=0,
          desc="When on, the stored clamp configuration below is applied at "
               "every boot (equivalent to the arm command). Default off keeps "
               "the armed-only posture — nothing burns unless commanded. "
               "Boot arming is skipped if the sensor mode is HFI or the other "
               "motor already owns the bus."),
    Param("autoarm_v_clamp", T_DOUBLE, "Clamp voltage setpoint",
          vTx=VTX_FLOAT32_AUTO, suffix=" V", minDouble=0.0, maxDouble=450.0,
          stepDouble=0.5, decimals=1, valDouble=0.0,
          desc="Bus voltage the clamp holds at or below. Firmware limits it to "
               "[l_min_vin+2, l_max_vin-1]. Set it 2-3 V below the stock "
               "battery-regen-cut start and well under l_max_vin so the "
               "degradation chain is clamp -> regen-cut -> over-voltage fault."),
    Param("autoarm_i_floor", T_DOUBLE, "Bus-current floor", vTx=VTX_FLOAT32_AUTO,
          suffix=" A", minDouble=-1000.0, maxDouble=1000.0, stepDouble=0.5,
          decimals=2, valDouble=0.0,
          desc="Minimum (filtered) DC input current. 0 burns all regen as "
               "winding heat and never backfeeds the supply. A small positive "
               "floor keeps a series diode conducting."),
    Param("autoarm_i_max", T_DOUBLE, "Injected id ceiling", vTx=VTX_FLOAT32_AUTO,
          suffix=" A", minDouble=0.0, maxDouble=1000.0, stepDouble=1.0,
          decimals=1, valDouble=0.0,
          desc="Per-feature ceiling on injected d-axis current so an unattended "
               "clamp cannot silently own the full motor current limit. 0 uses "
               "the motor limit. Burnable clamp power ~= 1.5*Rs*i_max^2; size it "
               ">= expected regen power."),
    Param("autoarm_clamp_en", T_BOOL, "Enable voltage clamp", valInt=1,
          desc="Enable the v_bus <= v_clamp protection loop when auto-armed."),
    Param("autoarm_floor_en", T_BOOL, "Enable current floor", valInt=1,
          desc="Enable the i_bus >= i_floor operating loop when auto-armed."),
    Param("autoarm_allow_start", T_BOOL, "Allow modulation restart", valInt=0,
          desc="Allow the clamp to (re)start modulation when the bus is pumped "
               "by an externally-spun motor (body-diode rectification at speed). "
               "Off by default."),
]

GROUPING = """        <group>
            <groupName>Molten MOSFET Dyno</groupName>
            <subgroup>
                <subgroupName>Bus Clamp</subgroupName>
                <subgroupParams>
                    <param>::sep::DC link</param>
                    <param>c_dc_uf</param>
                    <param>::sep::Auto-arm at boot</param>
                    <param>autoarm_en</param>
                    <param>autoarm_v_clamp</param>
                    <param>autoarm_i_floor</param>
                    <param>autoarm_i_max</param>
                    <param>autoarm_clamp_en</param>
                    <param>autoarm_floor_en</param>
                    <param>autoarm_allow_start</param>
                </subgroupParams>
            </subgroup>
        </group>"""


def crc32c(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            mask = -(crc & 1) & 0xFFFFFFFF
            crc = (crc >> 1) ^ (0x82F63B78 & mask)
    return (~crc) & 0xFFFFFFFF


def build_xml():
    ser = [p for p in PARAMS if p.tx == 1]
    parts = [XML_HEADER, "<ConfigParams>\n", "    <Params>\n"]
    parts.append("\n".join(p.xml() for p in PARAMS))
    parts.append("\n    </Params>\n")
    parts.append("    <SerOrder>\n")
    parts.append("\n".join(f"        <ser>{p.name}</ser>" for p in ser))
    parts.append("\n    </SerOrder>\n")
    parts.append("    <Grouping>\n")
    parts.append(GROUPING)
    parts.append("\n    </Grouping>\n")
    parts.append("</ConfigParams>\n")
    return "".join(parts), ser


def signature(ser):
    sig = ""
    for p in ser:
        sig += p.name + str(p.ptype) + str(p.vTx)
        for e in p.enumNames:
            sig += e
    return crc32c(sig.encode("utf-8"))


def c_array(name, data: bytes):
    lines = [f"static const uint8_t {name}[{len(data)}] = {{"]
    for i in range(0, len(data), 16):
        chunk = data[i:i + 16]
        lines.append("\t" + " ".join(f"0x{b:02x}," for b in chunk))
    lines.append("};")
    return "\n".join(lines)


def main():
    xml, ser = build_xml()
    sig = signature(ser)
    raw = xml.encode("utf-8")
    blob = len(raw).to_bytes(4, "big") + zlib.compress(raw, 9)

    print("=" * 70)
    print(f"MM_CONFIG_SIGNATURE = {sig}   (0x{sig:08X})")
    print(f"serialize params: {[p.name for p in ser]}")
    print(f"XML bytes: {len(raw)}   compressed blob: {len(blob)}")
    print("=" * 70)
    # self-check: round-trip the blob
    assert zlib.decompress(blob[4:]) == raw
    assert int.from_bytes(blob[:4], "big") == len(raw)
    print("\n// --- paste into mm_config.h ---")
    print(f"#define MM_CONFIG_SIGNATURE\t\t{sig}u")
    print("\n// --- paste into mm_config.c (descriptor blob) ---")
    print(c_array("mm_config_xml", blob))

    out = __file__.rsplit("/", 1)[0] + "/mm_config_settings.xml"
    with open(out, "w", encoding="utf-8") as f:
        f.write(xml)
    print(f"\n// full XML written to {out}")


if __name__ == "__main__":
    main()
