#include "AxisInputShaper.h"
#include "MoveQueue.h"
#include "TimeGenFunc.h"
#include "DiagnoseLog.h"
#include "../../../../snapmaker/src/common/debug.h"
#include "../../../../snapmaker/src/snapmaker.h"


AxisMng axis_mng;
#ifdef LOG_PRINT_POS
struct pos_trace pt;
#endif
const char* input_shaper_type_name[] = {"none", "ei", "ei2", "ei3", "mzv", "zv", "zvd", "zvdd", "zvddd"};
circular_buffer<struct motion_info> AxisMng::motion_info_rb;

void AxisInputShaper::init(int axis, MoveQueue *mq, InputShaperType type, float freq, float zeta, uint32_t ms2t) {
  this->axis = axis;
  this->mq = mq;
  this->ms2tick = ms2t;
  this->mm_per_step = 1.0;
  this->mm_per_half_step = 0.5;
  this->backup_type = this->type = type;
  this->frequency = freq;
  this->zeta = zeta;
  this->const_dist_hold = false;
  this->tgf_1.flag = this->tgf_2.flag = false;
  this->no_more_move = true;
  shaper_init(this->type, this->frequency, this->zeta);
}

void AxisInputShaper::shaper_init(void) {
  return shaper_init(type, frequency, zeta);
}

void AxisInputShaper::shaper_init(InputShaperType type, float frequency, float zeta) {
  origin_pluse.n = 0;
  switch (type) {
    case InputShaperType::none: {
      origin_pluse.n = 1;
      origin_pluse.A[0] = 1;
      origin_pluse.T[0] = 0;
      break;
    }

    case InputShaperType::zv: {
      float df = SQRT(1. - sq(zeta));
      float K = expf(-zeta * M_PI / df);
      float t_d = 1. / (frequency * df);
      origin_pluse.n = 2;
      origin_pluse.A[0] = 1;
      origin_pluse.A[1] = K;
      origin_pluse.T[0] = 0;
      origin_pluse.T[1] = 0.5 * t_d;
      break;
    }

    case InputShaperType::zvd: {
      float df = SQRT(1. - sq(zeta));
      float K = expf(-zeta * M_PI / df);
      float t_d = 1. / (frequency * df);
      origin_pluse.n = 3;
      origin_pluse.A[0] = 1;
      origin_pluse.A[1] = 2 * K;
      origin_pluse.A[2] = sq(K);
      origin_pluse.T[0] = 0;
      origin_pluse.T[1] = 0.5 * t_d;
      origin_pluse.T[2] = t_d;
      break;
    }

    case InputShaperType::zvdd: {
      float df = SQRT(1. - sq(zeta));
      float K = expf(-zeta * M_PI / df);
      float t_d = 1. / (frequency * df);
      origin_pluse.n = 4;

      origin_pluse.A[0] = 1;
      origin_pluse.A[1] = 3 * K;
      origin_pluse.A[2] = 3 * sq(K);
      origin_pluse.A[3] = K * K * K;

      origin_pluse.T[0] = 0;
      origin_pluse.T[1] = 0.5 * t_d;
      origin_pluse.T[2] = t_d;
      origin_pluse.T[3] = 1.5 * t_d;
      break;
    }

    case InputShaperType::zvddd: {
      float df = SQRT(1. - sq(zeta));
      float K = expf(-zeta * M_PI / df);
      float t_d = 1. / (frequency * df);
      origin_pluse.n = 4;

      origin_pluse.A[0] = 1;
      origin_pluse.A[1] = 4 * K;
      origin_pluse.A[2] = 6 * sq(K);
      origin_pluse.A[3] = 4 * K * K * K;
      origin_pluse.A[4] = K * K * K * K;

      origin_pluse.T[0] = 0;
      origin_pluse.T[1] = 0.5 * t_d;
      origin_pluse.T[2] = t_d;
      origin_pluse.T[3] = 1.5 * t_d;
      origin_pluse.T[4] = 2 * t_d;
      break;
    }

    case InputShaperType::mzv: {
      float df = SQRT(1. - sq(zeta));
      float K = expf(-.75 * zeta * M_PI / df);
      float t_d = 1. / (frequency * df);

      float a1 = 1. - 1. / SQRT(2.);
      float a2 = (SQRT(2.) - 1.) * K;
      float a3 = a1 * K * K;

      origin_pluse.n = 3;
      origin_pluse.A[0] = a1;
      origin_pluse.A[1] = a2;
      origin_pluse.A[2] = a3;

      origin_pluse.T[0] = 0.;
      origin_pluse.T[1] = .375*t_d;
      origin_pluse.T[2] = .75*t_d;
      break;
    }

    case InputShaperType::ei: {
      float v_tol = 1. / SHAPER_VIBRATION_REDUCTION; // vibration tolerance
      float df = SQRT(1. - sq(zeta));
      float K = expf(-zeta * M_PI / df);
      float t_d = 1. / (frequency * df);

      float a1 = .25 * (1. + v_tol);
      float a2 = .5 * (1. - v_tol) * K;
      float a3 = a1 * K * K;

      origin_pluse.n = 3;
      origin_pluse.A[0] = a1;
      origin_pluse.A[1] = a2;
      origin_pluse.A[2] = a3;

      origin_pluse.T[0] = 0;
      origin_pluse.T[1] = 0.5 * t_d;
      origin_pluse.T[2] = t_d;
      break;
    }

    case InputShaperType::ei2: {
      float v_tol = 1. / SHAPER_VIBRATION_REDUCTION; // vibration tolerance
      float df = SQRT(1. - sq(zeta));
      float K = expf(-zeta * M_PI / df);
      float t_d = 1. / (frequency * df);

      float V2 = sq(v_tol);
      float X = pow(V2 * (SQRT(1. - V2) + 1.), 1./3.);
      float a1 = (3.*X*X + 2.*X + 3.*V2) / (16.*X);
      float a2 = (.5 - a1) * K;
      float a3 = a2 * K;
      float a4 = a1 * K * K * K;

      origin_pluse.n = 4;
      origin_pluse.A[0] = a1;
      origin_pluse.A[1] = a2;
      origin_pluse.A[2] = a3;
      origin_pluse.A[3] = a4;

      origin_pluse.T[0] = 0;
      origin_pluse.T[1] = 0.5 * t_d;
      origin_pluse.T[2] = t_d;
      origin_pluse.T[3] = 1.5 * t_d;
      break;
    }

    case InputShaperType::ei3: {
      float v_tol = 1. / SHAPER_VIBRATION_REDUCTION;
      float df = SQRT(1. - sq(zeta));
      float K = expf(-zeta * M_PI / df);
      float t_d = 1. / (frequency * df);

      float K2 = K * K;
      float a1 = 0.0625 * (1. + 3. * v_tol + 2. * SQRT(2. * (v_tol + 1.) * v_tol));
      float a2 = 0.25 * (1. - v_tol) * K;
      float a3 = (0.5 * (1. + v_tol) - 2. * a1) * K2;
      float a4 = a2 * K2;
      float a5 = a1 * K2 * K2;

      origin_pluse.n = 5;
      origin_pluse.A[0] = a1;
      origin_pluse.A[1] = a2;
      origin_pluse.A[2] = a3;
      origin_pluse.A[3] = a4;
      origin_pluse.A[4] = a5;

      origin_pluse.T[0] = 0;
      origin_pluse.T[1] = 0.5 * t_d;
      origin_pluse.T[2] = t_d;
      origin_pluse.T[3] = 1.5 * t_d;
      origin_pluse.T[4] = 2 * t_d;
      break;
    }

    default: {
      origin_pluse.n = 1;
      origin_pluse.A[0] = 1;
      origin_pluse.T[0] = 0;
      break;
    }
  }

  shiftPulses();
  shift_pluse.log();
}

void AxisInputShaper::reset() {
  tgf_1.flag = tgf_2.flag = 0;
  const_dist_hold = false;
  g1.valid = g2.valid = false;
  print_tick = 0;
  print_pos = 0.0;
  delta_e = 0.0;
  shaper_window.reset();
}

void AxisInputShaper::setConfig(int type, float frequency, float zeta) {
  this->type = (InputShaperType)type;
  this->frequency = frequency;
  this->zeta = zeta;
};

bool AxisInputShaper::prepare(int m_idx) {
  if (!alignToStartMove(m_idx))
    return false;

  calcShaperWindowEndPosAndTime();
  if (!generateShapedFuncParams()){
    print_tick = shaper_window.tick;
    print_pos = shaper_window.pos;
  }

  // return getStep();
  return genNextStep();
}

void AxisInputShaper::logShaperWindow() {
  shaper_window.log(axis, ms2tick);
}

uint32_t AxisInputShaper::getShaperWindown() {
  return (right_delta + left_delta) * ms2tick;
}

void AxisInputShaper::enable() {
  type = backup_type;
}

void AxisInputShaper::disable() {
  backup_type = type;
  type = InputShaperType::none;
}

void AxisInputShaper::shiftPulses() {
  float sum_a = 0.;
  for (int i = 0; i < origin_pluse.n; ++i)
    sum_a += origin_pluse.A[i];

  float inv_a = 1. / sum_a;
  for (int i = 0; i < origin_pluse.n; ++i) {
    shift_pluse.A[origin_pluse.n - i - 1] = origin_pluse.A[i] * inv_a;
    shift_pluse.T[origin_pluse.n - i - 1] = -origin_pluse.T[i];
  }
  shift_pluse.n = origin_pluse.n;

  float ts = 0.;
  for (int i = 0; i < shift_pluse.n; ++i)
    ts += shift_pluse.A[i] * shift_pluse.T[i];

  for (int i = 0; i < shift_pluse.n; ++i)
    shift_pluse.T[i] -= ts;

  left_delta = origin_pluse.n == 0 ? 0 : ABS(shift_pluse.T[0]) * 1000.0f;
  right_delta = origin_pluse.n == 0 ? 0 : ABS(shift_pluse.T[shift_pluse.n - 1]) * 1000.0f;
}

bool AxisInputShaper::alignToStartMove(int m_idx) {
  Move *move;
  int t_m_idx = m_idx;
  tgf_coef_a_sum = 0.0;
  tgf_1.flag = tgf_2.flag = 0;

  shaper_window.pls_cnt = shift_pluse.n;
  shaper_window.tick = mq->moves[m_idx].start_tick + LROUND(left_delta * ms2tick);

  for (int i = 0; i < shaper_window.pls_cnt; i++) {
    CalcPluseInfo &p = shaper_window.pluse[i];
    p.A = shift_pluse.A[i];
    p.T = LROUND(shift_pluse.T[i] * 1000.0f * ms2tick);   // To tick offset
    do {
      move = &mq->moves[t_m_idx];
      uint32_t pluse_tick = p.T + shaper_window.tick;
      if (move->start_tick <= pluse_tick && pluse_tick <= move->end_tick) {
        break;
      }
      t_m_idx++;
      if (!mq->isBetween(t_m_idx)) {
        LOG_E("Can not align the start move as move's time is not enough\r\n");
        return false;
      }
    } while(1);
    p.m_idx = t_m_idx;
    tgf_coef_a_sum += 0.5 * move->axis_r[axis] * p.A * move->accelerate;
  }
  shaper_window.lpos = shaper_window.pos = calcPosition();
  shaper_window.ltick = shaper_window.tick;

  print_pos = shaper_window.lpos;
  print_tick = shaper_window.ltick;
  have_gen_step_tick = false;

  return true;
}

void AxisMng::init(MoveQueue *mq, uint32_t ms2t) {
  reqAbort = false;
  ms2tick = ms2t;

  endisable = true;
  req_update_shaper_flag = false;
  req_reset_shaper_flag = false;
  req_endisable_shaper_flag = false;
  this->mq = mq;

  x_sp = &axes[X_AXIS];
  y_sp = &axes[Y_AXIS];
  z_sp = &axes[Z_AXIS];
  b_sp = &axes[B_AXIS];
  e_sp = &axes[E_AXIS];

  x_sp->init(X_AXIS, mq, SP_DEFT_TYPE, SP_DEFT_FREQ, SP_DEFT_ZETA, ms2tick);
  y_sp->init(Y_AXIS, mq, SP_DEFT_TYPE, SP_DEFT_FREQ, SP_DEFT_ZETA, ms2tick);
  z_sp->init(Z_AXIS, mq, InputShaperType::none, SP_DEFT_FREQ, SP_DEFT_ZETA, ms2t);
  b_sp->init(B_AXIS, mq, InputShaperType::none, SP_DEFT_FREQ, SP_DEFT_ZETA, ms2t);
  e_sp->init(E_AXIS, mq, InputShaperType::none, SP_DEFT_FREQ, SP_DEFT_ZETA, ms2t);

  is_init = true;
}

// Call in planner task, can NOT delay as planner must generate the steps info
void AxisMng::loop(void) {

  if (req_endisable_shaper_flag) {
    if (endisable_shaper(req_shaper_status))
      req_endisable_shaper_flag = false;
  }

  if (req_update_shaper_flag) {
    if (update_shaper())
      req_update_shaper_flag = false;
  }

  if (req_reset_shaper_flag) {
    if (reset_shaper())
      req_reset_shaper_flag = false;
  }

}

bool AxisMng::planner_sync(void) {
  if (
    Planner::has_blocks_queued() || Planner::cleaning_buffer_counter
    #if ENABLED(EXTERNAL_CLOSED_LOOP_CONTROLLER)
      || (READ(CLOSED_LOOP_ENABLE_PIN) && !READ(CLOSED_LOOP_MOVE_COMPLETE_PIN))
    #endif
    || Planner::has_motion_queue()
    || axis_mng.reqAbort
  ) return false;

  return true;
}

void AxisMng::load_shaper_setting(void) {
  if (  (int)InputShaperType::none <= is_setting[0].type && is_setting[0].type <= (int)InputShaperType::zvddd &&
        10 <= is_setting[0].freq && is_setting[0].freq <= 200.0 &&
        0.0 <= is_setting[0].dampe && is_setting[0].dampe < 1.0
      )
  {
    x_sp->init(X_AXIS, mq, (InputShaperType)is_setting[0].type, is_setting[0].freq, is_setting[0].dampe, ms2tick);
  }
  else {
    x_sp->init(X_AXIS, mq, SP_DEFT_TYPE, SP_DEFT_FREQ, SP_DEFT_ZETA, ms2tick);
  }

  if (  (int)InputShaperType::none <= is_setting[1].type && is_setting[1].type <= (int)InputShaperType::zvddd &&
        10 <= is_setting[1].freq && is_setting[1].freq <= 200.0 &&
        0.0 <= is_setting[1].dampe && is_setting[1].dampe < 1.0
      )
  {
    y_sp->init(Y_AXIS, mq, (InputShaperType)is_setting[1].type, is_setting[1].freq, is_setting[1].dampe, ms2tick);
  }
  else {
    y_sp->init(Y_AXIS, mq, SP_DEFT_TYPE, SP_DEFT_FREQ, SP_DEFT_ZETA, ms2tick);
  }
  update_shaper();
}

bool AxisMng::update_shaper(void) {
  if (!planner_sync())
    return false;

  abort();
  x_sp->init(X_AXIS, mq, x_sp->type, x_sp->frequency, x_sp->zeta, ms2tick);
  y_sp->init(Y_AXIS, mq, y_sp->type, y_sp->frequency, y_sp->zeta, ms2tick);
  z_sp->init(Z_AXIS, mq, z_sp->type, z_sp->frequency, z_sp->zeta, ms2tick);
  b_sp->init(B_AXIS, mq, b_sp->type, b_sp->frequency, b_sp->zeta, ms2tick);
  e_sp->init(E_AXIS, mq, e_sp->type, e_sp->frequency, e_sp->zeta, ms2tick);

  endisable = false;
  LOOP_SHAPER_AXES(i) {
    if (axes[i].type != InputShaperType::none) {
      endisable = true;
      break;
    }
  }

  LOG_I("Update_shaper, adding a empty move after update\r\n");
  move_queue.addEmptyMove(EMPTY_MOVE_TIME_TICK);
  axis_mng.prepare(move_queue.move_tail);

  return true;
}

bool AxisMng::input_shaper_set(int axis, int type, float freq, float dampe) {
  if (X_AXIS <= axis && axis <= E_AXIS) {
    AxisInputShaper* axis_input_shaper = &axes[axis];
    axis_input_shaper->setConfig(type, freq, dampe);
    LOG_I("setting: axis: %d type: %s, frequency: %lf, zeta: %lf\n", axis, input_shaper_type_name[type], freq, dampe);
    return true;
  }
  else {
    return false;
  }
}

bool AxisMng::input_shaper_get(int axis, int &type, float &freq, float &dampe) {
  if (X_AXIS <= axis && axis <= E_AXIS) {
    AxisInputShaper* axis_input_shaper = &axes[axis];
    type = (int)axis_input_shaper->type;
    freq = axis_input_shaper->frequency;
    dampe = axis_input_shaper->zeta;
    return true;
  }
  else {
    return false;
  }
}

bool AxisMng::endisable_shaper(bool endisable) {
  if (!planner_sync())
    return false;

  abort();

  LOG_I("All axis %s\n", endisable ? "enbale" : "disable");
  LOOP_SHAPER_AXES(i) {
    if (endisable)
      axes[i].enable();
    else
      axes[i].disable();
  }

  // Re-init axis shaper
  x_sp->init(X_AXIS, mq, x_sp->type, x_sp->frequency, x_sp->zeta, ms2tick);
  y_sp->init(Y_AXIS, mq, y_sp->type, y_sp->frequency, y_sp->zeta, ms2tick);
  z_sp->init(Z_AXIS, mq, z_sp->type, z_sp->frequency, z_sp->zeta, ms2tick);
  b_sp->init(B_AXIS, mq, b_sp->type, b_sp->frequency, b_sp->zeta, ms2tick);
  e_sp->init(E_AXIS, mq, e_sp->type, e_sp->frequency, e_sp->zeta, ms2tick);

  LOG_I("Endisable shaper, adding a empty move\r\n");
  move_queue.addEmptyMove(EMPTY_MOVE_TIME_TICK);
  axis_mng.prepare(move_queue.move_tail);

  this->endisable = endisable;
  return true;
}

bool AxisMng::reset_shaper(void) {
  if (!planner_sync())
    return false;

  abort();
  init(mq, ms2tick);

  LOG_I("reset shaper, Add empty move\n");
  move_queue.addEmptyMove(EMPTY_MOVE_TIME_TICK);
  axis_mng.prepare(move_queue.move_tail);

  return true;
}

void AxisMng::log_xy_shpaer(void) {
  // SERIAL_ECHOPAIR("X type: ", input_shaper_type_name[int(x_sp->type)]);
  // SERIAL_ECHOPAIR(" frequency: ", x_sp->frequency);
  // SERIAL_ECHOLNPAIR(" zeta: ", x_sp->zeta);
  // SERIAL_ECHOPAIR("Y type: ", input_shaper_type_name[int(y_sp->type)]);
  // SERIAL_ECHOPAIR(" frequency: ", y_sp->frequency);
  // SERIAL_ECHOLNPAIR(" zeta: ", y_sp->zeta);
  LOG_I("X type: %s, frequency: %lf, zeta: %lf\n", input_shaper_type_name[int(x_sp->type)], x_sp->frequency, x_sp->zeta);
  LOG_I("Y type: %s, frequency: %lf, zeta: %lf\n", input_shaper_type_name[int(y_sp->type)], y_sp->frequency, y_sp->zeta);
}

bool AxisMng::prepare(uint8_t m_idx) {

  uint32_t sw;
  max_shaper_window_tick = 0;
  max_shaper_window_right_delta_tick = 0;
  LOOP_SHAPER_AXES(i) {
    if (axes[i].prepare(m_idx)) {
      #ifdef LOG_PRINT_POS
      pt.axis[i] = axes[i].print_pos;
      #endif
    }

    sw = axes[i].getShaperWindown();
    if (max_shaper_window_tick < sw)
      max_shaper_window_tick = sw;
    sw = axes[i].right_delta * ms2tick;
    if (max_shaper_window_right_delta_tick < sw)
      max_shaper_window_right_delta_tick = sw;
  }

  // LOG_I("max_shaper_window_tick %d, max_shaper_window_right_delta_tick %d\r\n", max_shaper_window_tick, max_shaper_window_right_delta_tick);
  mq->update_shaper_param(max_shaper_window_tick, max_shaper_window_right_delta_tick);

  AxisInputShaper *dm = findNearestPrintTickAxis();
  if (dm)
    cur_print_tick = dm->print_tick;
  else
    cur_print_tick = START_TICK;

  return true;
}

void AxisMng::logShaperWindows() {
  LOOP_SHAPER_AXES(i) {
    axes[i].logShaperWindow();
  }
}

bool AxisMng::tgfValid() {
  if (!is_init)
    return false;

  LOOP_SHAPER_AXES(i) {
    if (axes[i].tgf_1.flag || axes[i].tgf_2.flag)
      return true;
  }

  return false;
}

void AxisMng::abort() {
  // LOG_I("Axes mng aborted\r\n");
  steps_seq.reset();
  steps_flag.reset();
  mq->reset();
  LOOP_SHAPER_AXES(i) {
    axes[i].reset();
  }
}

void AxisMng::updateOldestPluesTick() {
  if (!is_init)
    return;
  uint32_t shaper_window_tick = axes[0].shaper_window.tick;
  uint32_t left_plues_tick = shaper_window_tick + LROUND((-axes[0].left_delta * ms2tick));
  uint32_t opt = left_plues_tick;
  LOOP_SHAPER_AXES(i) {
    shaper_window_tick = axes[i].shaper_window.tick;
    left_plues_tick = shaper_window_tick + LROUND((-axes[i].left_delta * ms2tick));
    if (PENDING(left_plues_tick, opt)) {
      opt = left_plues_tick;
    }
  }
  oldest_plues_tick = opt;
}

bool AxisMng::req_endisable_shaper(bool endisable) {
  uint32_t wait;

  wait = 100;
  while(req_endisable_shaper_flag && wait--) vTaskDelay(pdMS_TO_TICKS(10));
  if (req_endisable_shaper_flag) {
    LOG_E("Can not request endisable shaper as other thread doing\n");
    return false;
  }

  req_endisable_shaper_flag = true;
  req_shaper_status = endisable;
  // Planner task request endisable shaper, just mark this flag and do it in AixsMng::loop().
  if (xTaskGetCurrentTaskHandle() == sm2_handle->planner) {
    return true;
  }
  // Other task request endisable shaper, mark and wait.
  else {
    wait = 300;
    while(req_endisable_shaper_flag && wait--) vTaskDelay(pdMS_TO_TICKS(10));
    if (req_endisable_shaper_flag) {
      LOG_E("Endisable shaper timeout\n");
      return false;
    }
    req_endisable_shaper_flag = false;
    return true;
  }
}

bool AxisMng::req_update_shaper(void) {
  uint32_t wait;

  wait = 100;
  while(req_update_shaper_flag && wait--) vTaskDelay(pdMS_TO_TICKS(10));
  if (req_update_shaper_flag) {
    LOG_E("Can not request reset shaper as other thread doing\n");
    return false;
  }

  req_update_shaper_flag = true;
  // Planner task request update shaper and do it in AxisMng::loop
  if (xTaskGetCurrentTaskHandle() == sm2_handle->planner) {
    return true;
  }
  // Other task request update shpaer, mark a flag and wait
  else {
    wait = 300;
    while(req_update_shaper_flag && wait--) vTaskDelay(pdMS_TO_TICKS(10));
    if (req_update_shaper_flag) {
      LOG_E("Can reset shaper timeout\n");
      return false;
    }
    req_update_shaper_flag = false;
    return true;
  }
}

bool AxisMng::req_reset_shaper(void) {
  uint32_t wait;

  wait = 100;
  while(req_reset_shaper_flag && wait--) vTaskDelay(pdMS_TO_TICKS(10));
  if (req_reset_shaper_flag) {
    LOG_E("Can not request reset shaper as other thread doing\n");
    return false;
  }

  req_reset_shaper_flag = true;
  // Planner task request to reset shaper, just mark and do it in AxisMng::loop()
  if (xTaskGetCurrentTaskHandle() == sm2_handle->planner) {
    return true;
  }
  // Other task request to reset shaper, mark flag and wait
  else {
    wait = 300;
    while(req_reset_shaper_flag && wait--) vTaskDelay(pdMS_TO_TICKS(10));
    if (req_reset_shaper_flag) {
      LOG_E("Can reset shaper timeout\n");
      return false;
    }
    req_reset_shaper_flag = false;
    return true;
  }
}