# d-axis dissipation braking (Molten MOSFET fork feature)

Dump braking energy as controlled winding heat: inject d-axis current under
FOC so the absorber motor's copper becomes the load, cooled by its water
jacket. Replaces a resistor bank + brake chopper on the Molten MOSFET EUC
dynamometer (decision record §10). Design discussion: torque (q-axis) always
keeps priority; the injection is watchdogged so a dead host can never leave
it latched on.

## The primitive

`mc_interface_set_id_dissipate(current, off_delay)` → `mcpwm_foc_set_id_dissipate()`

- `current` — dissipation magnitude in A, applied as **negative id** (the
  field-weakening direction: it *lowers* back-EMF, buying modulation headroom
  at speed instead of costing it). Clamped to `l_current_max × scale`.
- `off_delay` — the command's validity window, clamped to **[0.05, 5.0] s**.
  Expiry ramps the injection to zero. This is the watchdog: refresh
  continuously (e.g. 10 Hz with 0.5 s) to sustain a dump.

In the FOC loop (`mcpwm_foc.c`, current-limit section):

- Slew-limited both directions at `ID_DISS_RAMP_A_PER_S` (500 A/s).
- Combined with MTPA/field-weakening id via `max_abs` — concurrent FW is
  subsumed, never stacked.
- **Torque priority:** the injection takes only
  `sqrt(current_max² − iq²)` — the budget torque leaves over. Stock FW
  truncation is the opposite order (id first); dissipation must never clip a
  braking setpoint mid-measurement.
- `lo_current_max` is temp-foldback-derated upstream (`l_temp_fet/motor_*`),
  so the injection inherits the existing FET/winding thermal backstop.
- Cleared on: off-delay expiry, motor release, any fault / motor-stopped
  state, and comms timeout (`timeout.c` zeroes it next to the timeout brake).
- Standalone use (no torque command) starts CURRENT mode with iq = 0 and
  keeps modulation alive until the injection decays.

## What the firmware does NOT do (host responsibility)

- **Power regulation** — the host closes the dissipation-power loop (the
  copper-loss telemetry below is its feedback). Thermal dynamics are
  seconds-scale; a 100 Hz CAN loop is ample.
- **Magnet-temperature protection** — the binding limit for sustained dumps
  is rotor/magnet temp, which no sensor sees (KTY reads windings). The host
  magnet-temp model (decision record item #2) must limit dump duty; the
  firmware winding-temp foldback is only the backstop.
- Note for salient motors (EM57): `T = 1.5·p·[λ·iq + (Ld−Lq)·id·iq]` — the
  injection adds reluctance torque whenever iq ≠ 0. Load cell is torque
  truth; the host loop trims it. A surface-PM bench motor will NOT show this.

## Wire protocol (private blocks, collision-proof vs upstream)

| Surface | ID | Payload |
|---|---|---|
| CAN command `CAN_PACKET_MM_SET_ID_DISSIPATE` | 200 | `[current i32 ×1e3 A][off_delay i16 ×1e3 s]` — off-delay mandatory, short frames ignored |
| CAN status `CAN_PACKET_MM_STATUS_DISSIPATION` | 201 | `[id_meas i16 ×10][iq_meas i16 ×10][id_diss_now i16 ×10][p_copper u16 W]` |
| COMM `COMM_MM_SET_ID_DISSIPATE` | 240 | same fields as CAN command |

The status frame broadcasts alongside STATUS_1 **only while armed** (idle
feature = silent bus). `p_copper = 1.5·Rs·(id² + iq²)` — total copper loss,
the quantity a thermal model integrates. Host driver: `moltenmosfet/PyVESC`
(`pyvesc.can` `set_id_dissipate` / `StatusDissipation`, COMM `SetIdDissipate`).

VESC Tool terminal: `mm_diss [current] [off_delay]` injects one watchdogged
shot; bare `mm_diss` prints id/iq/diss/p_copper state.

## Bench validation ladder

Hardware: Mini FSESC 6.7 Pro (CAN id 100) + surface-PM bench motor.

1. **Flash**: confirm the hardware name in VESC Tool (Firmware page shows the
   connected hw string — expected `60`) matches the build target, keep the
   current firmware .bin for rollback, then upload `build/60/60.bin` as a
   custom file. Re-run `can_bench_check.py` (py-vesc repo) — expect 11/11,
   proving the fork didn't disturb stock behavior.
2. **Ladder**: run `can_diss_check.py` (py-vesc repo) — fork detection,
   standstill injection, torque priority under a tight envelope,
   refresh-or-decay, ride-along with spin. Small currents (3 A default), but
   the motor warms — that is the feature working.
3. **Power accounting** (manual): at standstill with N amps injected, DC
   input power ≈ p_copper + converter losses. This sanity check is the
   foundation the host power loop stands on.

Anything above bench currents (real dumps, magnet-temp model validation)
waits for the EM57 + water jacket + load cell rig.
