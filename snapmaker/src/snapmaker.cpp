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
#include "snapmaker.h"

#include "common/debug.h"
#include "hmi/event_handler.h"
#include "module/can_host.h"
#include "module/linear.h"
#include "service/system.h"
#include "service/upgrade.h"
#include "service/power_loss_recovery.h"

// marlin headers
#include "src/module/endstops.h"
#include "src/feature/runout.h"
#include "src/gcode/gcode.h"
#include "flash_stm32.h"
#include "hmi/gcode_result_handler.h"

// Shapper
#include "../../Marlin/src/module/planner.h"
#include "../../Marlin/src/module/shaper/AxisInputShaper.h"
#include "../../Marlin/src/module/shaper/CircularBuffer.h"
#include "../../Marlin/src/module/shaper/MoveQueue.h"
#include "../../Marlin/src/module/shaper/TimeGenFunc.h"
#include "../../Marlin/src/module/shaper/StepsSeq.h"


SnapmakerHandle_t sm2_handle;

SemaphoreHandle_t plan_buffer_lock = NULL;

extern void enqueue_hmi_to_marlin();


uint8_t action_ban = 0;
void enable_action_ban(uint8_t ab) {
  action_ban |= ab;
}

void disable_action_ban(uint8_t ab) {
  action_ban &= (~ab);
}

// default all power domain is available
uint8_t power_ban = 0;
void enable_power_ban(uint8_t pd) {
  power_ban |= pd;
}

void disable_power_ban(uint8_t pd) {
  power_ban &= (~pd);
}

void enable_power_domain(uint8_t pd) {
  pd &= ~power_ban;
  #if PIN_EXISTS(POWER0_SUPPLY)
    if (pd & POWER_DOMAIN_0) WRITE(POWER0_SUPPLY_PIN, POWER0_SUPPLY_ON);
  #endif

  #if PIN_EXISTS(POWER1_SUPPLY)
    if (pd & POWER_DOMAIN_1) WRITE(POWER1_SUPPLY_PIN, POWER1_SUPPLY_ON);
  #endif

  #if PIN_EXISTS(POWER2_SUPPLY)
    if (pd & POWER_DOMAIN_2) WRITE(POWER2_SUPPLY_PIN, POWER2_SUPPLY_ON);
  #endif
}

void disable_power_domain(uint8_t pd) {
  #if PIN_EXISTS(POWER0_SUPPLY)
    if (pd & POWER_DOMAIN_0) WRITE(POWER0_SUPPLY_PIN, POWER0_SUPPLY_OFF);
  #endif

  #if PIN_EXISTS(POWER1_SUPPLY)
    if (pd & POWER_DOMAIN_1) WRITE(POWER1_SUPPLY_PIN, POWER1_SUPPLY_OFF);
  #endif

  #if PIN_EXISTS(POWER2_SUPPLY)
    if (pd & POWER_DOMAIN_2) WRITE(POWER2_SUPPLY_PIN, POWER2_SUPPLY_OFF);
  #endif
}


void HeatedBedSelfCheck(void) {
  enable_power_domain(POWER_DOMAIN_BED);
  // disable heated bed firstly
  OUT_WRITE(HEATER_BED_PIN, LOW);
  // and set input for the detect pin
  SET_INPUT_PULLUP(HEATEDBED_ON_PIN);
  vTaskDelay(pdMS_TO_TICKS(10));
  // if we get LOW, indicate the NMOS is breakdown
  // we need to disable its power supply immediately
  if(READ(HEATEDBED_ON_PIN) == LOW) {
    disable_power_domain(POWER_DOMAIN_BED);
    enable_power_ban(POWER_DOMAIN_BED);
    systemservice.ThrowException(EHOST_MC, ETYPE_PORT_BAD);
  }
}


/**
 * Check Update Flag
 */
void CheckUpdateFlag(void)
{
  uint32_t Address;
  uint32_t Flag;
  Address = FLASH_UPDATE_CONTENT_INFO;
  Flag = *((uint32_t*)Address);
  if(Flag != 0xffffffff)
  {
    FLASH_Unlock();
    FLASH_ErasePage(Address);
    FLASH_Lock();
  }
}


/**
 * main task
 */
static void main_loop(void *param) {
  struct DispatcherParam dispather_param;

  millis_t cur_mills;

  dispather_param.owner = TASK_OWN_MARLIN;

  dispather_param.event_buff = (uint8_t *)pvPortMalloc(SSTP_RECV_BUFFER_SIZE);
  configASSERT(dispather_param.event_buff);

  dispather_param.event_queue = ((SnapmakerHandle_t)param)->event_queue;

  HeatedBedSelfCheck();

  systemservice.SetCurrentStatus(SYSTAT_IDLE);

  // waiting for initializing modules
  xEventGroupWaitBits(((SnapmakerHandle_t)param)->event_group, EVENT_GROUP_MODULE_READY, pdFALSE, pdTRUE, pdMS_TO_TICKS(10000));

  // to reduce log covered by can events
  vTaskDelay(pdMS_TO_TICKS(100));
  // init power-loss recovery after initializing modules
  // because we need to check if current toolhead is same with previous
  pl_recovery.Init();

  SERIAL_ECHOLN("Finish init\n");

  cur_mills = millis() - 3000;

  for (;;) {

    // receive and execute one command, or push Gcode into Marlin queue
    DispatchEvent(&dispather_param);

    enqueue_hmi_to_marlin();
    if (commands_in_queue < BUFSIZE) get_available_commands();

    advance_command_queue();
    quickstop.Process();
    endstops.event_handler();
    idle();

    // avoid module proactive reply failure, loop query
    // case 1: unexpected faliment runout trigger if we startup withou toolhead loaded.
    // case 2: Z axis hit boundary when we run G28.
    // case 3: Z_MIN_Probe error, when we do z probe, the triggered message didn't arrive main controller

    if (cur_mills + 2500 <  millis()) {
      cur_mills = millis();
      // TODO: poll filament sensor state
    }
  }
}


static void planner_task(void *param) {

  steps_flag.reset();
  steps_seq.reset();
  move_queue.init(STEPPER_TIMER_TICKS_PER_MS);
  axis_mng.init(&move_queue, STEPPER_TIMER_TICKS_PER_MS);
  axis_mng.load_shaper_setting();

  // LOG_I("System start, adding a empty move for shaper\r\n");
  // move_queue.addEmptyMove(2 * axis_mng.max_shaper_window_tick);
  // axis_mng.prepare(move_queue.move_tail);
  // steps_seq.buf_tick_test(1000);

  while(1) {
    planner.shaped_loop();
  }
}


static void hmi_task(void *param) {
  SnapmakerHandle_t    task_param;
  struct DispatcherParam dispather_param;

  ErrCode ret = E_FAILURE;

  uint8_t  count = 0;

  configASSERT(param);
  task_param = (SnapmakerHandle_t)param;

  dispather_param.owner = TASK_OWN_HMI;

  dispather_param.event_buff = (uint8_t *)pvPortMalloc(SSTP_RECV_BUFFER_SIZE);
  configASSERT(dispather_param.event_buff);

  dispather_param.event_queue = task_param->event_queue;
  event_handler_init();

  for (;;) {
    if(READ(SCREEN_DET_PIN)) {
      xTaskNotifyStateClear(task_param->heartbeat);

      if (systemservice.GetCurrentStatus() == SYSTAT_WORK && count) {
        // if we lost screen in working for 100ms, stop current work
        systemservice.StopTrigger(TRIGGER_SOURCE_SC_LOST);
        LOG_E("stop cur work because screen lost!\n");
      }

      if (++count)
        count = 0;

      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    else
      count = 0;

    ret = hmi.CheckoutCmd(dispather_param.event_buff, dispather_param.size);

    systemservice.CheckIfSendWaitEvent();

    if (ret != E_SUCCESS) {
      // no command, sleep 10ms for next command
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // execute or send out one command
    DispatchEvent(&dispather_param);

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void motion_info_log(void) {

  static uint32_t _1_last_milliseconds = 0 ;
  if (ELAPSED(millis(), _1_last_milliseconds+10)) {
    _1_last_milliseconds = millis();
    struct step_seq_statistics_info _1_sssi, _2_sssi;
    if (axis_mng.step_seq_statistics_rb.pop(_1_sssi)) {
      LOG_I("%d: StepsSeq's use rate: %.1f%%, prepare time %f ms\n", _1_sssi.sys_time_ms, _1_sssi.use_rate, _1_sssi.prepare_time_ms);
    }
    if (axis_mng.step_seq_statistics_rb.pop(_2_sssi)) {
      LOG_I("%d: StepsSeq's use rate: %.1f%%, prepare time %f ms\n", _2_sssi.sys_time_ms, _2_sssi.use_rate, _2_sssi.prepare_time_ms);
      LOG_I("Each step's caculation task %.1f us\n\n",
      (float)(_2_sssi.sys_time_ms - _1_sssi.sys_time_ms) * 100 * 1000 / (StepsSeq::SIZE * (_2_sssi.use_rate - _1_sssi.use_rate)));
    }
  }

  static uint32_t _2_last_milliseconds = 0;
  if (ELAPSED(millis(), _2_last_milliseconds+10)) {
    _2_last_milliseconds = millis();
    struct step_runout step_runout;
    if (stepper.step_runout_rb.pop(step_runout)) {
      LOG_I("%d: ### NOTE! ### Step runout\n", step_runout.sys_time_ms);
    }
  }

  static uint32_t _3_last_milliseconds = 0;
  if (ELAPSED(millis(), _3_last_milliseconds+10)) {
    _3_last_milliseconds = millis();
    // struct motion_info mi;
    // if (axis_mng.motion_info_rb.pop(mi)) {
    //   LOG_I("\n%d: ====== motion info from %s ======\n", mi.sys_time_ms, mi.tag);
    //   LOG_I("block count: %u, optimally planned: %u\n", mi.block_count, mi.block_planned_count);
    //   LOG_I("move count and use rate: %u,\t%.1f\n", mi.move_count, mi.move_use_rate);
    //   LOG_I("steps count and use rate: %u,\t%.1f\n", mi.step_count, mi.step_use_rate);
    //   float msp_ms = (float)(mi.move_head_tick - mi.move_tail_tick) / STEPPER_TIMER_TICKS_PER_MS;
    //   float mmsp_ms = (float)(mi.move_head_tick - mi.current_print_tick) / STEPPER_TIMER_TICKS_PER_MS;
    //   float mnmsp_ms = (float)(mi.current_print_tick - mi.move_tail_tick) / STEPPER_TIMER_TICKS_PER_MS;
    //   LOG_I("move tick: head %u, tail %u, move time spand %fms\n", mi.move_head_tick, mi.move_tail_tick, msp_ms);
    //   LOG_I("move motion time spand %f ms, move no motion time spand %f ms\n", mmsp_ms, mnmsp_ms);
    //   // LOG_I("move can print tick %u, current print tick %u\n", mmsp_ms, mnmsp_ms);
    //   LOG_I("step prepare time(ms): %f\n\n", mi.step_prepare_time);
    // }
  }

  static uint32_t _4_last_milliseconds = 0;
  if (ELAPSED(millis(), _4_last_milliseconds+10)) {
    _4_last_milliseconds = millis();
    // if (planner.step_generating && steps_seq.getBufMilliseconds() < 15.0) {
    //   LOG_I("%.1f\n", steps_seq.getBufMilliseconds());
    //   LOG_I("move head: %u, tail: %u, count: %u\n", move_queue.move_head, move_queue.move_tail, move_queue.getMoveSize());
    //   LOOP_SHAPER_AXES(i) {
    //     if (axis_mng.axes[i].no_more_move) {
    //       uint32_t idx = axis_mng.axes[i].shaper_window.t_cls_pls;
    //       LOG_I("Axis %u no move, move index%u\n", i, axis_mng.axes[i].shaper_window.pluse[idx].m_idx);
    //     }
    //   }
    // }

    //   LOG_I("Planner sleep ms %u, %u ~ %u, now %u, schedule cnt %u\n",
    //   planner_sch_info.sleep_ms,
    //   planner_sch_info.start_sleep_ms,
    //   planner_sch_info.end_sleep_ms,
    //   millis(),
    //   planner_sch_info.entry_cnt
    //   );
    // }
    // LOG_I("EDA %f\n", axis_mng.e_sp->delta_e);

    extern uint32_t statistics_slowdown_cnt;
    static uint32_t last_statistics_slowdown_cnt;

    if (last_statistics_slowdown_cnt != statistics_slowdown_cnt) {
      LOG_I("statistics_slowdown_cnt %d\r\n", statistics_slowdown_cnt);
      last_statistics_slowdown_cnt = statistics_slowdown_cnt;
    }

    uint32_t file_pos;
    if (Stepper::file_pos_rb.pop(file_pos)) {
      LOG_I("### Pause file position: %u\n", file_pos);
    }
  }

}

static void heartbeat_task(void *param) {
  //SSTP_Event_t   event = {EID_SYS_CTRL_ACK, SYSCTL_OPC_GET_STATUES};

  int counter = 0;

  for (;;) {
    // do following every 10ms without being blocked
    #if HAS_FILAMENT_SENSOR
      runout.run();
    #endif

    systemservice.CheckException();

    if (++counter > 100) {
      counter = 0;

      // do following every 1s
      upgrade.Check();
      canhost.SendHeartbeat();
    }

    motion_info_log();

    // sleep for 10ms
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


void SnapmakerSetupEarly() {

  systemservice.Init();
  // init serial for HMI
  hmi.Init(&MSerial2, HMI_SERIAL_IRQ_PRIORITY);
}


/**
 * Check App Valid Flag
 */
void CheckAppValidFlag(void)
{
  uint32_t Value;
  uint32_t Address;
  Address = FLASH_BOOT_PARA;
  Value = *((uint32_t*)Address);
  if(Value != 0xaa55ee11) {
    FLASH_Unlock();
    FLASH_ErasePage(Address);
    FLASH_ProgramWord(Address, 0xaa55ee11);
    FLASH_Lock();
  }
}


static void TaskEventHandler(void *p) {
  canhost.EventHandler(p);
}


static void TaskReceiveHandler(void *p) {
  canhost.ReceiveHandler(p);
}


void SnapmakerSetupPost() {
  // init the power supply pins
  OUT_WRITE(POWER0_SUPPLY_PIN, POWER0_SUPPLY_ON);
  OUT_WRITE(POWER1_SUPPLY_PIN, POWER1_SUPPLY_OFF);
  OUT_WRITE(POWER2_SUPPLY_PIN, POWER2_SUPPLY_OFF);

  SET_INPUT_PULLUP(SCREEN_DET_PIN);

  if(READ(SCREEN_DET_PIN)) {
    disable_power_domain(POWER_DOMAIN_SCREEN);
    SERIAL_ECHOLN("Screen doesn't exist!\n");
  }
  else {
    enable_power_domain(POWER_DOMAIN_SCREEN);
    SERIAL_ECHOLN("Screen exists!\n");
  }

  // forced update of speed parameters
  // Comment by 747
  // process_cmd_imd("M201 X1000 Y1000");
  // process_cmd_imd("M203 X100 Y100 Z40 E40");
  // process_cmd_imd("M204 S1000");

  // power on the modules by default
  enable_all_steppers();

  BreathLightInit();

  CheckAppValidFlag();

  CheckUpdateFlag();

  // to disable heartbeat if module need to be upgraded
  upgrade.CheckIfUpgradeModule();

  canhost.Init();

  enable_power_domain(POWER_DOMAIN_LINEAR);
  enable_power_domain(POWER_DOMAIN_ADDON);

  sm2_handle = (SnapmakerHandle_t)pvPortMalloc(sizeof(struct SnapmakerHandle));
  sm2_handle->event_queue = xMessageBufferCreate(1024);
  configASSERT(sm2_handle->event_queue);

  sm2_handle->event_group = xEventGroupCreate();
  configASSERT(sm2_handle->event_group);

  // create marlin task
  BaseType_t ret;
  ret = xTaskCreate((TaskFunction_t)main_loop, "Marlin_task", MARLIN_LOOP_STACK_DEPTH,
        (void *)sm2_handle, MARLIN_LOOP_TASK_PRIO, &sm2_handle->marlin);
  if (ret != pdPASS) {
    LOG_E("Failed to create marlin task!\n");
    while(1);
  }
  else {
    LOG_I("Created marlin task!\n");
  }

  ret = xTaskCreate((TaskFunction_t)hmi_task, "HMI_task", HMI_TASK_STACK_DEPTH,
        (void *)sm2_handle, HMI_TASK_PRIO, &sm2_handle->hmi);
  if (ret != pdPASS) {
    LOG_E("Failed to create HMI task!\n");
    while(1);
  }
  else {
    LOG_I("Created HMI task!\n");
  }


  ret = xTaskCreate((TaskFunction_t)heartbeat_task, "HB_task", HB_TASK_STACK_DEPTH,
        (void *)sm2_handle, HB_TASK_PRIO, &sm2_handle->heartbeat);
  if (ret != pdPASS) {
    LOG_E("Failed to create heartbeat task!\n");
    while(1);
  }
  else {
    LOG_I("Created heartbeat task!\n");
  }

  ret = xTaskCreate((TaskFunction_t)TaskReceiveHandler, "can_recv_handler", CAN_RECEIVE_HANDLER_STACK_DEPTH,
        (void *)sm2_handle, CAN_RECEIVE_HANDLER_PRIORITY,  &sm2_handle->can_recv);
  if (ret != pdPASS) {
    LOG_E("Failed to create receiver task!\n");
    while(1);
  }
  else {
    LOG_I("Created can receiver task!\n");
  }

  ret = xTaskCreate((TaskFunction_t)TaskEventHandler, "can_event_handler", CAN_EVENT_HANDLER_STACK_DEPTH,
        (void *)sm2_handle, CAN_EVENT_HANDLER_PRIORITY, &sm2_handle->can_event);
  if (ret != pdPASS) {
    LOG_E("Failed to create can event task!\n");
    while(1);
  }
  else {
    LOG_I("Created can event task!\n");
  }

  plan_buffer_lock = xSemaphoreCreateMutex();
  if (!plan_buffer_lock) {
    LOG_E("Can not create plan buffer lock\n");
  }

  ret = xTaskCreate((TaskFunction_t)planner_task, "planner_task", PLANNER_TASK_STACK_DEPTH,
        (void *)nullptr, PLANNER_TASK_PRIO, &sm2_handle->planner);
  if (ret != pdPASS) {
    LOG_E("Failed to create planner task!\n");
    while(1);
  }
  else {
    LOG_I("Created planner task!\n");
  }

  vTaskStartScheduler();
}


extern "C" {

void vApplicationMallocFailedHook( void ) {
  LOG_E("RTOS malloc failed");
}

}
