/*
 * Snapmaker2-Controller Firmware
 * Copyright (C) 2019-2020 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Controller
 * (see https://github.com/Snapmaker/Snapmaker2-Controller)
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
 */
#ifndef SNAPMAKER_TOOLHEAD_DUAL_EXTRUDER_H_
#define SNAPMAKER_TOOLHEAD_DUAL_EXTRUDER_H_

#include "module_base.h"
#include "toolhead_3dp.h"
#include "can_host.h"
#include "../common/config.h"

typedef struct {
  uint8_t model;
  float diameter;
}hotend_type_info_t;

typedef enum {
  SINGLE_EXTRUDER_MODULE_FAN       = 0,
  SINGLE_EXTRUDER_NOZZLE_FAN       = 1,
  DUAL_EXTRUDER_LEFT_MODULE_FAN    = 0,
  DUAL_EXTRUDER_RIGHT_MODULE_FAN   = 1,
  DUAL_EXTRUDER_NOZZLE_FAN         = 2,
}fan_e;

typedef struct {
  int16_t current;
  int16_t target;
}hotend_temp_t;

#define HOTEND_INFO_MAX 22
const hotend_type_info_t hotend_info[HOTEND_INFO_MAX] = {{.model = 2, .diameter = 0.4}, \
                                                         {.model = 1, .diameter = 0.6}, \
                                                         {.model = 1, .diameter = 0.8}, \
                                                         {.model = 1, .diameter = 0.4},\
                                                         {.model = 1, .diameter = 0.2},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 2, .diameter = 0.4},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0xff, .diameter = 0},\
                                                         {.model = 0, .diameter = 0.4}, \
                                                         {.model = 0, .diameter = 0.8}, \
                                                        };

class ToolHeadDualExtruder: public ToolHead3DP {
  public:
    ToolHeadDualExtruder(ModuleDeviceID id): ToolHead3DP(id) {
      for (int i = 0; i < EXTRUDERS; i++) {
        hotend_type_[i] = 0xff;
        target_temp_[0] = 0;
      }
    }

    //
    ErrCode Init(MAC_t &mac, uint8_t mac_index);
    ErrCode ToolChange(uint8_t new_extruder, bool use_compensation = true);
    bool probe_state();
    bool probe_state(probe_sensor_t sensor);
    bool filament_state();
    bool filament_state(uint8_t e);
    void SelectProbeSensor(probe_sensor_t sensor);
    void SetZCompensation(float &left_val, float &right_val);
    void GetZCompensation(float &left_z_compensation, float &right_z_compensation);

    // for hmi interface
    ErrCode HmiGetHotendType(SSTP_Event_t &event);
    ErrCode HmiGetFilamentState(SSTP_Event_t &event);
    ErrCode HmiGetHotendTemp(SSTP_Event_t &event);
    ErrCode HmiRequestToolChange(SSTP_Event_t &event);
    ErrCode HmiSetFanSpeed(SSTP_Event_t &event);
    ErrCode HmiSetHotendOffset(SSTP_Event_t &event);
    ErrCode HmiGetHotendOffset(SSTP_Event_t &event);
    ErrCode HmiRequestGetActiveExtruder(SSTP_Event_t &event);

    // module report callback
    void ReportProbeState(uint8_t state[]);
    void ReportTemperature(uint8_t *data);
    void ReportPID(uint8_t *data);
    void ReportFilamentState(uint8_t state[]);
    void ReportHotendType(uint8_t *data);
    void ReportExtruderInfo(uint8_t *data);
    void ReportHotendOffset(uint8_t *data);
    void ReportProbeSensorCompensation(uint8_t *data);
    void ReportRightExtruderPos(uint8_t *data);

    // set module
    ErrCode ModuleCtrlProximitySwitchPower(uint8_t state);
    ErrCode SetFan(uint8_t fan_index, uint8_t speed, uint8_t delay_time = 0);
    ErrCode SetHeater(uint16_t target_temp, uint8_t extrude_index = 0);
    ErrCode ModuleCtrlProbeStateSync();
    ErrCode ModuleCtrlPidSync();
    ErrCode ModuleCtrlHotendTypeSync();
    ErrCode ModuleCtrlFilamentStateSync();
    ErrCode ModuleCtrlHotendOffsetSync();
    ErrCode ModuleCtrlZProbeSensorCompensationSync();
    ErrCode ModuleCtrlRightExtruderPosSync();
    ErrCode ModuleCtrlSetPid(float p, float i, float d);
    ErrCode ModuleCtrlToolChange(uint8_t new_extruder);
    ErrCode ModuleCtrlSaveHotendOffset(float offset, uint8_t axis);
    ErrCode ModuleCtrlSaveZCompensation(float *val);
    ErrCode ModuleCtrlRightExtruderMove(move_type_e type, float destination = 0);
    ErrCode ModuleCtrlSetRightExtruderPosition(float raise_for_home_pos, float z_max_pos);

    uint8_t GetHWVersion();
    void ShowInfo();

  protected:
    void CheckLevelingData();

  protected:
    probe_sensor_t active_probe_sensor_ = PROBE_SENSOR_PROXIMITY_SWITCH;
    uint8_t hotend_type_[EXTRUDERS];
    uint16_t target_temp_[EXTRUDERS];
    float hotend_diameter_[EXTRUDERS];
    float z_compensation_[EXTRUDERS];

  private:
    uint8_t mac_index_ = MODULE_MAC_INDEX_INVALID;
    bool hotend_type_initialized_ = false;
    uint8_t hw_version_ = 0xff;
};

extern ToolHeadDualExtruder printer_dualextruder;

#endif