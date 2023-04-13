#include "AxisInputShaper.h"
#include "MoveQueue.h"
#include "TimeGenFunc.h"
#include "DiagnoseLog.h"
#include "../../../../snapmaker/src/common/debug.h"


#define CALC_OF_BINOMIAL(tgf)     ((tgf.coef_a * tgf.time_wind + tgf.coef_b) * tgf.time_wind)


AxisMng axis_mng;
struct pos_trace pt;


void AxisInputShaper::init(int axis, MoveQueue *mq, InputShaperType type, uint32_t ms2t) {
  this->axis = axis;
  this->axis_bit_mask = 1<<axis;
  this->mq = mq;
  this->type = type;
  this->ms2tick = ms2t;
  this->mm_per_step = 1.0;
  this->mm_per_half_step = 0.5;
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
  last_print_tick = print_pos = 0.0;
  last_print_tick = 0;
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
    last_print_tick = print_tick = shaper_window.tick;
    print_pos = shaper_window.pos;
  }

  return genNextStepTime();
}

bool AxisInputShaper::genNextStepTime() {
  if (have_gen_step_tick) {
    return true;
  }

  if (tgf_1.flag) {
    if (getTimeFromTgf(tgf_1)){
      return true;
    }
    else {
      tgf_1.flag = 0;
      return genNextStepTime();
    }
  }
  else if(tgf_2.flag) {
    if (getTimeFromTgf(tgf_2)){
      return true;
    }
    else {
      tgf_2.flag = 0;
      return genNextStepTime();
    }
  }
  else {
    for (;;) {
      if (moveShaperWindowToNext()) {
        calcShaperWindowEndPosAndTime();
        if (generateShapedFuncParams()) {
          return genNextStepTime();
        }
        else {
          // current print tick and positoin must update to shapper window
          last_print_tick = print_tick = shaper_window.tick;
          print_pos = shaper_window.pos;
        }
      }
      else {
        return false;
      }
    }
  }
}

void AxisInputShaper::logShaperWindow() {
  shaper_window.log(axis, ms2tick);
}

uint32_t AxisInputShaper::getShaperWindown() {
  return (right_delta + left_delta) * ms2tick;
}

bool AxisInputShaper::alignToMoveHead() {
  // Shaper windown align to the newest move. No move have
  return mq->nextMoveIndex(shaper_window.pluse[shaper_window.pls_cnt-1].m_idx) == mq->move_head;
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

float AxisInputShaper::calcPosition(void) {
  float res = 0;
  for (int i = 0; i < shaper_window.pls_cnt; i++) {
    CalcPluseInfo &pluse = shaper_window.pluse[i];
    float pd = pluse.A * mq->getAxisPosition(pluse.m_idx, axis, pluse.T + shaper_window.tick);
    res += pd;
  }
  return res;
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
  last_print_tick = print_tick = shaper_window.ltick;
  have_gen_step_tick = false;

  return true;
}

void AxisInputShaper::calcShaperWindowEndPosAndTime() {
  tgf_coef_a_sum = 0.0;

  // Fine next time closest pluse and calculate the shaper windown's time spand
  shaper_window.wind_tick = 0xFFFFFFFF;
  for (int i = 0; i < shaper_window.pls_cnt; i++) {
    CalcPluseInfo &p = shaper_window.pluse[i];
    uint32_t pluse_tick = p.T + shaper_window.tick;
    uint32_t min_next_time = mq->moves[p.m_idx].end_tick - pluse_tick;
    if (min_next_time < shaper_window.wind_tick) {
      shaper_window.wind_tick = min_next_time;
      shaper_window.t_cls_pls = i;
    }
    tgf_coef_a_sum += 0.5 * mq->moves[p.m_idx].axis_r[axis] * p.A * mq->moves[p.m_idx].accelerate;
  }

  #ifdef SHAPER_LOG_ENABLE
  // LOG_I("Axis %d cloesest move index update to %d in plues\r\n", axis, shaper_window.pluse[shaper_window.t_cls_pls].m_idx, shaper_window.t_cls_pls);
  #endif

  // Update shaper window time and calculate shaper windown's position.
  shaper_window.tick += shaper_window.wind_tick;
  shaper_window.pos = calcPosition();

  #if ENABLED(LIN_ADVANCE)
  if (E_AXIS == axis) {
    float eda = 0.0;
    Move *move = &(mq->moves[shaper_window.pluse[0].m_idx]);
    if (move->use_advance) {
      eda = move->delta_v * ((float)shaper_window.wind_tick/ms2tick) * move->axis_r[axis];
      #ifdef SHAPER_LOG_ENABLE
      // LOG_I("delta_v %f delta_e %d, Eda %d\r\n", move->delta_v, delta_e, eda);
      #endif
    }
    delta_e += eda;
    shaper_window.pos += delta_e;
  }
  #endif

  #ifdef SHAPER_LOG_ENABLE
  if (axis == 0) logShaperWindow();
  #endif
}

bool AxisInputShaper::moveShaperWindowToNext() {

  shaper_window.lpos = shaper_window.pos;
  shaper_window.ltick = shaper_window.tick;

  // The closest pluse move to next move
  CalcPluseInfo &cls_p = shaper_window.pluse[shaper_window.t_cls_pls];
  uint8_t cls_p_m_idx = mq->nextMoveIndex(cls_p.m_idx);

  if (InputShaperType::none != type) {
    // Shaper axis do NOT support sync in moving as position is not in the correction. Skip the sync move
    do {
      if (!mq->isBetween(cls_p_m_idx)) {
        #ifdef SHAPER_LOG_ENABLE
        LOG_I("Axis %d move queue is empty, cls_p_m_idx %d\r\n", axis, cls_p_m_idx);
        #endif
        return false;
      }
      if (mq->moves[cls_p_m_idx].flag & BLOCK_FLAG_SYNC_POSITION) {
        cls_p_m_idx = mq->nextMoveIndex(cls_p_m_idx);
      }
      else {
        break;
      }
    } while(1);
  }
  else {
    if (!mq->isBetween(cls_p_m_idx)) {
      #ifdef SHAPER_LOG_ENABLE
      LOG_I("Axis %d move queue is empty, cls_p_m_idx %d\r\n", axis, cls_p_m_idx);
      #endif
      return false;
    }
    // Push the sync's target position
    if (mq->moves[cls_p_m_idx].flag & BLOCK_FLAG_SYNC_POSITION) {
      sync_pos_rb.push(mq->moves[cls_p_m_idx].sync_target_pos[axis]);
    }
  }

  cls_p.m_idx = cls_p_m_idx;
  return true;
}

bool AxisInputShaper::generateShapedFuncParams() {
  tgf_1.flag = tgf_2.flag = 0;
  tgf_1.coef_a = tgf_coef_a_sum;

  // A sync
  if (0 == shaper_window.wind_tick && InputShaperType::none == type) {
    tgf_1.flag = TimeGenFunc::TGF_SYNC_FLAG;
    return true;
  }

  float s1 = shaper_window.lpos;
  float s2 = shaper_window.pos;
  float ds = s2 - s1;
  float dt = (float)shaper_window.wind_tick / ms2tick;

  if (dt < EPSILON || fabs(ds) < EPSILON) {
    #ifdef SHAPER_LOG_ENABLE
    LOG_I("ds %f, dt %f\r\n", ds, dt);
    #endif
    return false;
  }

  tgf_1.coef_b = ds / dt - tgf_1.coef_a * dt;
  if (IS_ZERO(tgf_1.coef_a)) {
    if (IS_ZERO(tgf_1.coef_b)) {
      LOG_E("#e# Remove a const tgf\r\n");
      return false;
    }
    tgf_1.monotone = tgf_1.coef_b > 0.0 ? 1 : -1;
    tgf_1.start_tick = shaper_window.ltick;
    tgf_1.end_tick = shaper_window.tick;
    tgf_1.time_wind = dt;
    tgf_1.start_pos = s1;
    tgf_1.end_pos = s2;
    tgf_1.flag = TimeGenFunc::TGF_VALID_FLAG;
    #ifdef SHAPER_LOG_ENABLE
    tgf_1.log(1);
    #endif
  }
  else {
    /*
    middle_t: set derivative of S(ds = 2*a*t + b = 0)
    t = -b / (2*a)
    */
    float middle_t = -tgf_1.coef_b / (2 * tgf_1.coef_a);
    if (0 < middle_t && middle_t < dt) {
      tgf_1.monotone = tgf_1.coef_a > 0.0 ? -1 : 1;
      tgf_1.start_tick = shaper_window.ltick;
      tgf_1.end_tick = shaper_window.ltick + middle_t * ms2tick;
      tgf_1.time_wind = middle_t;
      tgf_1.start_pos = s1;
      float middle_p = CALC_OF_BINOMIAL(tgf_1);
      tgf_1.end_pos = s1 + middle_p;
      if(axis == X_AXIS)
        axis_mng.tgf_middle_pos_rb.push(fabs(middle_p));
      tgf_1.flag = TimeGenFunc::TGF_VALID_FLAG;
      #ifdef SHAPER_LOG_ENABLE
      tgf_1.log(1);
      #endif

      tgf_2.monotone = -tgf_1.monotone;
      tgf_2.start_tick = tgf_1.start_tick;            // Use the same start tick
      tgf_2.end_tick = shaper_window.tick;
      tgf_2.coef_a = tgf_1.coef_a;
      tgf_2.coef_b = tgf_1.coef_b;
      tgf_2.time_wind = dt;
      tgf_2.start_pos = s1;                           // Use the same start position
      tgf_2.end_pos = s2;
      tgf_2.flag = TimeGenFunc::TGF_VALID_FLAG;
      #ifdef SHAPER_LOG_ENABLE
      tgf_2.log(2);
      #endif
    }
    else {
      tgf_1.monotone = tgf_1.coef_a > 0.0 ? (middle_t <= 0 ? 1 : -1) : (middle_t <= 0 ? -1 : 1);
      tgf_1.start_tick = shaper_window.ltick;
      tgf_1.end_tick = shaper_window.tick;
      tgf_1.time_wind = dt;
      tgf_1.start_pos = s1;
      tgf_1.end_pos = s2;
      tgf_1.flag = TimeGenFunc::TGF_VALID_FLAG;
      #ifdef SHAPER_LOG_ENABLE
      tgf_1.log(1);
      #endif
    }
  }

  #ifdef LOG_MOTION_INFO
  if (tgf_1.flag) tgf_rb.push(tgf_1);
  if (tgf_2.flag) tgf_rb.push(tgf_2);
  #endif

  return true;
}

bool AxisInputShaper::getTimeFromTgf(TimeGenFunc &tgf) {
  float np;
  float ns;

  if (tgf.flag & TimeGenFunc::TGF_SYNC_FLAG) {
    sync_trigger_flag = true;
    have_gen_step_tick = true;
    tgf.flag = false;
    return true;
  }

  if (tgf.monotone < 0) {
    np = print_pos - mm_per_step;
    ns = print_pos - mm_per_half_step;
    if (ns < tgf.end_pos - EPSILON) {
      #ifdef SHAPER_LOG_ENABLE
      LOG_I("Axis %d, PP %u,  SP %u tgf.end_pos %d(%d)\r\n", axis, np, ns, tgf.end_pos, tgf.monotone);
      #endif
      return false;
    }
  }
  else if (tgf.monotone > 0) {
    np = print_pos + mm_per_step;
    ns = print_pos + mm_per_half_step;
    if (ns > tgf.end_pos + EPSILON) {
      #ifdef SHAPER_LOG_ENABLE
      LOG_I("Axis %d, PP %u,  SP %u tgf.end_pos %d(%d)\r\n", axis, np, ns, tgf.end_pos, tgf.monotone);
      #endif
      return false;
    }
  }
  else {
    return false;
  }

  float delta_dist = (float)(tgf.start_pos - ns);
  #ifdef SHAPER_LOG_ENABLE
  LOG_I("Axis %d delta_dist %f\r\n", axis, delta_dist);
  #endif
  float t_ms = tgf.dist2time(delta_dist);
  if (t_ms < 0.0) {
    // When we can make sure this will NOT hanppen?
    LOG_E("### ERROR ###: t_ms(%f) < 0.0)\r\n", t_ms);
    t_ms = 0;
  }

  #ifdef SHAPER_LOG_ENABLE
  uint32_t lt = print_tick;
  #endif
  print_tick = tgf.start_tick + (uint32_t)LROUND((t_ms * ms2tick));
  dir = tgf.monotone;
  print_pos = np;
  have_gen_step_tick = true;
  last_print_tick = print_tick;

  #ifdef SHAPER_LOG_ENABLE
  float itv = ((float)print_tick - lt) / ms2tick;
  LOG_I("Axis %d, print_pos %u,  sample_pos %u tgf's start_tick %d ", axis, np, ns, tgf.start_tick);
  LOG_I("tgf return t %.3fms, last tick %d, cur tick %d, itv %.3fms, dir %d\n", t_ms, lt, print_tick, itv, dir);
  #endif

  return true;
}

void AxisMng::init(MoveQueue *mq, uint32_t ms2t) {
  reqAbort = false;
  ms2tick = ms2t;
  this->mq = mq;

  x_sp = &axes[0];
  y_sp = &axes[1];
  z_sp = &axes[2];
  e_sp = &axes[3];

  x_sp->init(X_AXIS, mq, InputShaperType::ei, ms2t);
  y_sp->init(Y_AXIS, mq, InputShaperType::ei, ms2t);
  z_sp->init(Z_AXIS, mq, InputShaperType::none, ms2t);
  e_sp->init(E_AXIS, mq, InputShaperType::none, ms2t);

  uint32_t sw;
  max_shaper_window_tick = 0;
  max_shaper_window_right_delta_tick = 0;
  LOOP_SHAPER_AXES(i) {
    sw = axes[i].getShaperWindown();
    if (max_shaper_window_tick < sw)
      max_shaper_window_tick = sw;
    sw = axes[i].right_delta * ms2tick;
    if (max_shaper_window_right_delta_tick < sw)
      max_shaper_window_right_delta_tick = sw;
  }

  LOG_I("max_shaper_window_tick %d, max_shaper_window_right_delta_tick %d\r\n", max_shaper_window_tick, max_shaper_window_right_delta_tick);
}

bool AxisMng::prepare(uint8_t m_idx) {

  max_shaper_window_tick = 0;
  LOOP_SHAPER_AXES(i) {
    if (axes[i].prepare(m_idx)) {
      pt.axis[i] = axes[i].print_pos;
    }
    uint32_t sw;
    sw = axes[i].getShaperWindown();
    if (max_shaper_window_tick < sw)
      max_shaper_window_tick = sw;
  }

  AxisInputShaper *dm = findMinPrintTickAxis();
  if (dm)
    cur_print_tick = dm->print_tick;
  else
    cur_print_tick = 0;

  return true;
}

void AxisMng::logShaperWindows() {
  LOOP_SHAPER_AXES(i) {
    axes[i].logShaperWindow();
  }
}

bool AxisMng::getNextStep(StepInfo &step_info) {
  AxisInputShaper *dm = findMinPrintTickAxis();
  // ActiveDM = dm;
  if (dm) {
    if (PENDING(dm->print_tick, cur_print_tick)) {
      LOG_E("### ERROR ####: cur print tick < last print tick %d\r\n", int(cur_print_tick - dm->print_tick));
      dm->print_tick = cur_print_tick;
    }
    if (!dm->have_gen_step_tick) {
      LOG_E("### ERROR ####: got a in-have gen step tick\r\n");
    }

    step_info.time_dir.itv = (uint16_t)(dm->print_tick - cur_print_tick);
    step_info.time_dir.dir = dm->dir > 0 ? 1 : 0;
    step_info.time_dir.move_bits = dm->axis_bit_mask;
    step_info.time_dir.axis = dm->axis;
    if (!dm->sync_trigger_flag) {
      step_info.time_dir.sync = 0;
      #ifdef SHAPER_LOG_ENABLE
      // LOG_I("Axis %d trigger pos %f, print_pos %f, got sync target pos %f\r\n",
      // dm->axis, dm->sync_trigger_pos, dm->print_pos, dm->sync_target_pos);
      #endif
    }
    else {
      step_info.time_dir.sync = 1;
      dm->sync_pos_rb.pop(step_info.flag_data.sync_pos);
    }

    dm->sync_trigger_flag = false;
    cur_print_tick = dm->print_tick;
    dm->have_gen_step_tick = false;

    return true;
  }
  else {
    return false;
  }

}

bool AxisMng::tgfValid() {
  LOOP_SHAPER_AXES(i) {
    if (axes[i].tgf_1.flag || axes[i].tgf_2.flag)
      return true;
  }
  return false;
}

void AxisMng::abort() {
  LOG_I("Axes mng aborted\r\n");
  steps_seq.reset();
  steps_flag.reset();
  mq->reset();
  LOOP_SHAPER_AXES(i) {
    axes[i].reset();
  }
  LOG_I("Adding a empty move after abort\r\n");
  moveQueue.addEmptyMove(EMPTY_MOVE_TIME);
  axis_mng.prepare(moveQueue.move_tail);
}

void AxisMng::updateOldestPluesTick() {
  uint32_t sw_tick = axes[0].shaper_window.tick;
  uint32_t left_plues_tick = sw_tick + (-axes[0].left_delta * ms2tick);
  uint32_t opt = left_plues_tick;
  LOOP_SHAPER_AXES(i) {
    sw_tick = axes[i].shaper_window.tick;
    left_plues_tick = sw_tick - (axes[i].left_delta * ms2tick);
    if (PENDING(left_plues_tick, opt)) {
      opt = left_plues_tick;
    }
  }
  oldest_plues_tick = opt;
}

AxisInputShaper *AxisMng::findMinPrintTickAxis() {
  AxisInputShaper *nearest_axis = nullptr;
  LOOP_SHAPER_AXES(i) {
    axes[i].genNextStepTime();
    if (axes[i].have_gen_step_tick) {
      if (nearest_axis) {
        if (PENDING(axes[i].print_tick, nearest_axis->print_tick)) {
          nearest_axis = &axes[i];
        }
      }
      else {
        nearest_axis = &axes[i];
      }
    }
  }

  if (nearest_axis && ELAPSED(nearest_axis->print_tick, mq->can_print_head_tick)) {
    // LOG_I("ActiveDM(%d)'s tick must wait for move's gen step tick\r\n", nearest_axis->axis);
    nearest_axis = nullptr;
  }

  return nearest_axis;
}