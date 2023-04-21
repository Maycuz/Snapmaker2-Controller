/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "../gcode.h"
#include "../../module/planner.h"
#include "../../module/shaper/AxisInputShaper.h"
#include "../../../../snapmaker/src/module/module_base.h"

void report_M593(void) {
 axis_mng.log_xy_shpaer();
}

/**
 * M593: Set axis inputshaper
 *      example M593 X F50 P1 D0.1
 *      F: specify the frequency
 *      P: inputshaper type
 *        0: "none",
 *        1: "ei",
 *        2: "ei2",
 *        3: "ei3",
 *        4: "mzv",
 *        5: "zv",
 *        6: "zvd",
 *        7: "zvdd",
 *        8: "zvddd"
 *      D: damping ration
 */
void GcodeSuite::M593() {

  bool x = parser.seen('X');
  bool y = parser.seen('Y');
  bool p = parser.seen('P');
  bool f = parser.seen('F');
  bool d = parser.seen('D');
  bool r = parser.seen('R');

  if (!p && !f && !d && !r) {
    report_M593();
    return;
  }

  if (parser.seen('R')) {
    axis_mng.req_reset_shaper();
    report_M593();
    return;
  }

  if (!x && !y) {
    x = true;
    y = true;
  }

  if (x) {
    AxisInputShaper *axis_input_shaper = axis_mng.x_sp;
    float frequency = parser.floatval('F', axis_input_shaper->frequency);
    float zeta = parser.floatval('D', axis_input_shaper->zeta);
    int type = parser.byteval('P', (int)axis_input_shaper->type);
    axis_mng.input_shaper_set(X_AXIS, type, frequency, zeta);
  }

  if (y) {
    AxisInputShaper *axis_input_shaper = axis_mng.y_sp;
    float frequency = parser.floatval('F', axis_input_shaper->frequency);
    float zeta = parser.floatval('D', axis_input_shaper->zeta);
    int type = parser.byteval('P', (int)axis_input_shaper->type);
    axis_mng.input_shaper_set(Y_AXIS, type, frequency, zeta);
  }
  axis_mng.req_update_shaper();
  report_M593();
}
