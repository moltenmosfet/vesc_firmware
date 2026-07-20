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

// Molten MOSFET: persistent dyno configuration surfaced as a custom-config
// page in stock VESC Tool (with tooltips), no forked GUI required. Holds the
// DC-link capacitance (from which the bus-clamp voltage gains are derived) and
// an optional auto-arm block for the d-axis bus clamp. See mm_config.c and
// documentation/mm_d_axis_dissipation.md.

#ifndef MM_CONFIG_H_
#define MM_CONFIG_H_

#include <stdbool.h>

// VESC Tool config signature — a crc32c over the serialize order of the XML
// descriptor. MUST equal what VESC Tool computes from the embedded XML, or
// COMM_SET_CUSTOM_CONFIG payloads are rejected. Regenerate both together with
// documentation/mm_config_xml_gen.py whenever the parameter set changes.
#define MM_CONFIG_SIGNATURE		3482054794u

typedef struct {
	float c_dc_uf;              // DC-link capacitance, uF (0 = keep BC_* defaults)
	bool  autoarm_en;           // apply the clamp config below at boot
	float autoarm_v_clamp;      // V
	float autoarm_i_floor;      // bus A
	float autoarm_i_max;        // injected id ceiling, A
	bool  autoarm_clamp_en;
	bool  autoarm_floor_en;
	bool  autoarm_allow_start;
} mm_config_t;

// Load stored config from custom EEPROM, push derived clamp gains into the
// motor(s), auto-arm the bus clamp if enabled, and register the VESC Tool
// page. Call once at boot, AFTER mc_interface_init().
void mm_config_init(void);

// Re-derive and re-push the clamp gains (and, when armed-at-boot is enabled,
// re-arm). Call after anything that re-runs mcpwm_foc_init — most notably the
// motor-detection wizard, which resets the clamp gains to their compiled
// defaults along with the rest of the protection stack.
void mm_config_apply(void);

const mm_config_t *mm_config_get(void);

#endif /* MM_CONFIG_H_ */
