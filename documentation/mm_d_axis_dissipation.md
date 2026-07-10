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

---

# The d-axis bus clamp — dissipative braking without a battery

Second feature on this branch. The dissipation primitive above is
host-commanded; the bus clamp is a **firmware-local controller** that uses the
same injector to solve a different problem: braking when the DC bus has no
sink (bench PSU that can't sink current; later, a full pack that must not be
overcharged). Braking regen then charges the DC-link caps and runs the bus to
the overvoltage fault. A host loop cannot help — the bus slews ~2.5 V/ms at
5 A into ~2 mF, ~1000× too fast for a 100 Hz CAN loop — so this lives in the
15 kHz fast loop.

## Two layers, one injector

1. **Bus-current floor** (operating mode): PI regulates extra id so the
   filtered DC input current stays ≥ `i_floor`. With `i_floor = 0`, regen is
   burned in the windings *as it is produced* and never backfeeds the supply
   — braking with the d axis while iq keeps making torque. A small positive
   floor keeps a series protection diode conducting.
2. **Voltage clamp** (protection): positive-part PI on `v_bus − v_clamp`
   catches transients, estimate error, and the coast case. With
   `allow_start_modulation`, an armed clamp may (re)start modulation when an
   externally-spun motor pumps the bus through the body diodes — gated on
   enough speed for the observer phase to be trustworthy (at a garbage angle
   "id" leaks into the real q axis as torque; a standstill motor can't pump
   the bus anyway).

Both PIs work in **bus-amps** and the winning demand is linearized through
the square-law copper plant (`id = sqrt(u·v_bus / (1.5·Rs))`, Rs
temperature-tracked) so loop gain doesn't depend on operating point. The
result merges with the host dissipation command via `max` into the same
iq-priority budget — torque is never clipped by the clamp.

Gains are compile-time (`foc_math.c`, `BC_*`): derived for the ~2 mF bench
DC link (clamp ωn ≈ 71 Hz, ζ ≈ 1.1). **They scale with C** — re-derive
before a larger DC link (planned: a conf frame carrying C_dc).

## Arming model

RAM-only, armed-only, **not watchdogged**: protection survives faults,
motor release, and comms loss; it is cleared only by an explicit disarm
(flags = 0) or a reboot — re-arm each power-up as part of run setup. Arming
is **rejected** (returns false / terminal prints REJECTED) when the sensor
mode is any HFI variant (HFI owns the d axis — the clamp would be silently
inert) or when the other motor instance on a dual-drive board already owns
the bus (two PIs on one bus fight).

## Degradation chain and voltage margins

When the iq-priority budget or `i_max` caps the burn, the `saturated` status
flag asserts and the bus keeps rising into the stock machinery, stacked:

```
PSU/pack voltage < v_clamp < regen-cut band (start→end) < l_max_vin fault
   e.g. 44 V         48 V         50 → 53 V                  57 V
```

- **`l_battery_regen_cut_start/end`** folds back regen iq (brake torque
  tapers). Defaults are 1000/1100 V = inert; **there is no CAN setter — set
  it in VESC Tool at commissioning**, band ≥ 2–3 V (a narrow band
  limit-cycles against its ~48 Hz filter).
- **`l_max_vin` OV fault** is the final backstop: hard PWM stop + ~500 ms
  restart lockout. During that window a spinning motor back-feeds through
  the body diodes toward rectified back-EMF and software can do nothing —
  the protection there is the design rule *max speed × kV keeps rectified
  BEMF below the cap/FET rating* (the rig: ~297 V BEMF vs 450 V-rated bus;
  the bench: no prime mover, so BEMF can't exceed the bus that spun it).

## Wire protocol

| Surface | ID | Payload |
|---|---|---|
| CAN conf `CAN_PACKET_MM_CONF_BUS_CLAMP` | 202 | `[v_clamp i16 ×10 V][i_floor i16 ×100 A][i_max i16 ×10 A][flags u8]` — flags: bit0 clamp_en, bit1 floor_en, bit2 allow_start; 0 = disarm |
| CAN status `CAN_PACKET_MM_STATUS_BUS_CLAMP` | 203 | `[v_bus i16 ×10][i_bus_filtered i16 ×100][id_clamp_now i16 ×10][flags u8: armed·clamp_active·floor_active·saturated·started_modulation]` — broadcast with STATUS_1 **while armed** |
| COMM `COMM_MM_CONF_BUS_CLAMP` | 241 | same fields as the CAN conf frame |

`i_bus` in the status frame is the **filtered** value the floor loop
regulates on (the raw estimate is per-cycle noisy). Host driver:
`moltenmosfet/PyVESC` — `node.conf_bus_clamp()` / `disarm_bus_clamp()` /
`StatusBusClamp`; COMM `SetBusClamp`. VESC Tool terminal: `mm_clamp`
(`mm_diss`/`mm_clamp` are registered callbacks in `mm_commands.c` — they
must NOT live in `terminal.c`, whose code sits in the ~full 16 kB `.text2`
flash region).

## Build note

The stock hw60 image sits within a few kB of the app-flash ceiling. The
dyno build drops LispBM (unused — the CAN-native architecture retired the
Lisp escape hatch) for ~160 kB of headroom:

```
make 60 USE_LISPBM=0
```

Plain `make 60` (LispBM in) still fits and links; use whichever, but the
lisp-less build is the dyno standard.

## Bench validation ladder

Setup: current-limited PSU ≤ 48 V, **inline series diode** (heatsinked),
regen-cut configured per the margin stack, kinetic energy from spin-up only.
Guided script: `can_clamp_check.py` (py-vesc repo) covers arm/disarm + status
visibility, standstill-burn sanity, clamp-alone brake pulse, floor-alone
brake pulse, and the standstill restart-gate refusal. Manual steps:

3. Baseline pump, everything disarmed: measure the v_bus rise rate under a
   known brake current → effective C (validates the compiled gains); let the
   OV fault trip once deliberately to verify lockout + recovery.
6b. Coast restart at speed: spin, release, drop `v_clamp` below PSU voltage
   with `allow_start` on → clamp must restart modulation (iq = 0) and burn;
   `started_modulation` flag asserts.
7. Degradation chain order with `i_max` ≈ 3 A and a hard brake:
   `saturated` → regen-cut torque taper → only then the OV fault; verify
   auto-restart stays blocked during the lockout.
8. Endurance: repeated spin/brake at max planned burn — temp foldback should
   shrink the budget (`saturated` earlier); disarm mid-burn; kill the host
   mid-burn (clamp must persist).
