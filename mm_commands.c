/*
	Copyright 2026 Molten MOSFET

	This file is part of the Molten MOSFET fork of the VESC firmware
	(dyno features — see the repo README).

	The VESC firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The VESC firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Molten MOSFET terminal commands for bench work. Registered through the
// terminal callback registry rather than added to terminal.c: terminal.c's
// code lives in the small 16 kB `.text2` flash region (already ~full), while
// this file compiles into the main app region.

#include "mm_commands.h"

#include "commands.h"
#include "mc_interface.h"
#include "mcpwm_foc.h"
#include "mm_config.h"
#include "terminal.h"
#include "timeout.h"
#include "utils_math.h"

#include <stdio.h>
#include <string.h>

// One-shot d-axis dissipation. The off-delay watchdog (capped at 5 s) ramps
// it out on its own, so a single terminal command can never leave the
// injection latched.
static void mm_diss_cmd(int argc, const char **argv) {
	if (argc == 3) {
		float current = -1.0;
		float off_delay = -1.0;
		sscanf(argv[1], "%f", &current);
		sscanf(argv[2], "%f", &off_delay);
		if (current >= 0.0 && current <= mc_interface_get_configuration()->l_current_max && off_delay > 0.0) {
			timeout_reset();
			mc_interface_set_id_dissipate(current, off_delay);
			commands_printf("Dissipating %.1f A on the d axis for up to %.2f s (5 s cap)",
					(double)current, (double)off_delay);
			int fault = mc_interface_get_fault();
			if (fault != FAULT_CODE_NONE) {
				commands_printf("Fault occured: %s", mc_interface_fault_to_string(fault));
			}
		} else {
			commands_printf("Invalid argument(s). Current 0.0..%.2f, off_delay > 0.",
					(double)mc_interface_get_configuration()->l_current_max);
		}
	} else if (argc == 1) {
		commands_printf("diss_now : %.2f A", (double)mcpwm_foc_get_id_dissipate_now());
		commands_printf("diss_set : %.2f A", (double)mcpwm_foc_get_id_dissipate_set());
		commands_printf("id       : %.2f A", (double)mcpwm_foc_get_id_filter());
		commands_printf("iq       : %.2f A", (double)mcpwm_foc_get_iq_filter());
		commands_printf("p_copper : %.1f W\n", (double)(1.5 *
				mc_interface_get_configuration()->foc_motor_r *
				(SQ(mcpwm_foc_get_id_filter()) + SQ(mcpwm_foc_get_iq_filter()))));
	} else {
		commands_printf("Usage: mm_diss [current] [off_delay], or mm_diss to print state\n");
	}
}

// Bus-clamp arm/disarm/state.
static void mm_clamp_cmd(int argc, const char **argv) {
	if (argc == 5) {
		float v_clamp = -1.0, i_floor = 0.0, i_max = 0.0;
		int flags = -1;
		sscanf(argv[1], "%f", &v_clamp);
		sscanf(argv[2], "%f", &i_floor);
		sscanf(argv[3], "%f", &i_max);
		sscanf(argv[4], "%d", &flags);
		if (flags >= 0 && flags <= 7) {
			bool ok = mc_interface_conf_bus_clamp(v_clamp, i_floor, i_max, (uint8_t)flags);
			commands_printf(ok ? "Bus clamp %s" : "REJECTED (HFI sensor mode, or other motor owns the bus)",
					flags == 0 ? "disarmed" : "armed");
		} else {
			commands_printf("flags must be 0..7 (bit0 clamp, bit1 floor, bit2 allow_start; 0 = disarm)");
		}
	} else if (argc == 2 && strcmp(argv[1], "off") == 0) {
		mc_interface_conf_bus_clamp(0.0, 0.0, 0.0, 0);
		commands_printf("Bus clamp disarmed");
	} else if (argc == 1) {
		mm_bus_clamp_state bc;
		mcpwm_foc_get_bus_clamp(&bc);
		commands_printf("armed    : %d (clamp %d, floor %d, allow_start %d)",
				bc.armed, bc.clamp_en, bc.floor_en, bc.allow_start);
		commands_printf("v_clamp  : %.1f V   (v_bus now %.1f V)",
				(double)bc.v_clamp, (double)mc_interface_get_input_voltage_filtered());
		commands_printf("i_floor  : %.2f A   (i_bus filt %.2f A)",
				(double)bc.i_floor, (double)bc.ibus_filter);
		commands_printf("i_max    : %.1f A", (double)bc.i_max);
		commands_printf("id_now   : %.2f A", (double)bc.id_now);
		commands_printf("flags    : clamp_active %d, floor_active %d, saturated %d, started_mod %d\n",
				bc.clamp_active, bc.floor_active, bc.saturated, bc.started_modulation);
	} else {
		commands_printf("Usage: mm_clamp [v_clamp] [i_floor] [i_max] [flags], mm_clamp off, or mm_clamp for state\n");
	}
}

// Print the stored dyno config + live clamp gains, or re-apply it. "apply"
// re-pushes the C_dc-derived gains and re-evaluates auto-arm — run it after the
// motor-detection wizard, which resets the clamp gains to their bench defaults
// along with the rest of the protection stack.
static void mm_config_cmd(int argc, const char **argv) {
	if (argc == 2 && strcmp(argv[1], "apply") == 0) {
		mm_config_apply();
		commands_printf("mm_config: re-applied (gains pushed, auto-arm evaluated)\n");
		return;
	}
	if (argc == 1) {
		const mm_config_t *c = mm_config_get();
		mm_bus_clamp_state bc;
		mcpwm_foc_get_bus_clamp(&bc);
		commands_printf("c_dc_uf  : %.1f uF%s", (double)c->c_dc_uf,
				c->c_dc_uf < 1.0 ? "  (0 -> compiled 2 mF clamp gains)" : "");
		commands_printf("autoarm  : %d (clamp %d, floor %d, allow_start %d)",
				c->autoarm_en, c->autoarm_clamp_en, c->autoarm_floor_en,
				c->autoarm_allow_start);
		commands_printf("v_clamp  : %.1f V", (double)c->autoarm_v_clamp);
		commands_printf("i_floor  : %.2f A", (double)c->autoarm_i_floor);
		commands_printf("i_max    : %.1f A", (double)c->autoarm_i_max);
		commands_printf("clamp PI : Kp %.4f  Ki %.1f  (live in motor)\n",
				(double)bc.clamp_kp, (double)bc.clamp_ki);
		return;
	}
	commands_printf("Usage: mm_config to print state, or mm_config apply\n");
}

void mm_commands_init(void) {
	terminal_register_command_callback(
			"mm_diss",
			"Molten MOSFET: inject d-axis dissipation current; no args prints state",
			"[current] [off_delay]",
			mm_diss_cmd);

	terminal_register_command_callback(
			"mm_clamp",
			"Molten MOSFET: arm the d-axis bus clamp (flags bit0 clamp, bit1 floor, bit2 allow_start; 0/off = disarm)",
			"[v_clamp] [i_floor] [i_max] [flags]",
			mm_clamp_cmd);

	terminal_register_command_callback(
			"mm_config",
			"Molten MOSFET: print the stored dyno config + live clamp gains; 'apply' re-applies after motor detection",
			"[apply]",
			mm_config_cmd);
}
