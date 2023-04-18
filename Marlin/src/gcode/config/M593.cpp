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
  // 747 TODO
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

    bool update = false;
    bool x = parser.seen('X');
    bool y = parser.seen('Y');
    if (!x && !y) {
      x = true;
      y = true;
    }

    if (x) {
      AxisInputShaper* axis_input_shaper = &axis_mng.axes[X_AXIS];
      float frequency = parser.floatval('F', axis_input_shaper->frequency);
      float zeta = parser.floatval('D', axis_input_shaper->zeta);
      int type = parser.byteval('P', (int)axis_input_shaper->type);
      if (frequency != axis_input_shaper->frequency || zeta != axis_input_shaper->zeta || type != (int)axis_input_shaper->type) {
        update = true;
      }
      if (update) {
        axis_input_shaper->setConfig(type, frequency, zeta);
      }
      // LOG
    }
    if (y) {
      AxisInputShaper* axis_input_shaper = &axis_mng.axes[Y_AXIS];
      float frequency = parser.floatval('F', axis_input_shaper->frequency);
      float zeta = parser.floatval('D', axis_input_shaper->zeta);
      int type = parser.floatval('P', (int)axis_input_shaper->type);
      if (frequency != axis_input_shaper->frequency || zeta != axis_input_shaper->zeta || type != (int)axis_input_shaper->type) {
          update = true;
      }
      if (update) {
        axis_input_shaper->setConfig(type, frequency, zeta);
      }
    }

    if (update) {
        // planner.synchronize();
        // axisManager.initAxisShaper();
        // axisManager.abort();
    }
}
