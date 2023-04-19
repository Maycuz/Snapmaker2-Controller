#include "AxisInputShaper.h"
#include "MoveQueue.h"
#include "TimeGenFunc.h"
#include "DiagnoseLog.h"
#include "../../../../snapmaker/src/common/debug.h"


#define CALC_OF_BINOMIAL(tgf)     ((tgf.coef_a * tgf.time_wind + tgf.coef_b) * tgf.time_wind)


AxisMng axis_mng;
struct pos_trace pt;
const char* input_shaper_type_name[] = {"none", "ei", "ei2", "ei3", "mzv", "zv", "zvd", "zvdd", "zvddd"};


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

  // return genNextStep();
  return getStep();
}

bool AxisInputShaper::genNextStep(struct genStep &gs) {

  if (tgf_1.flag) {
    if (getTimeFromTgf(tgf_1)){
      gs.dir = dir;
      gs.tick = print_tick;
      gs.file_pos = file_pos;
      gs.sync_pos = sync_pos;
      gs.valid = true;
      gs.out_step = true;
      gs.pos = print_pos;
      file_pos = INVALID_FILE_POS;
      sync_pos = INVALID_SYNC_POS;
      return true;
    }
    else {
      tgf_1.flag = 0;
      return genNextStep(gs);
    }
  }
  else if(tgf_2.flag) {
    if (getTimeFromTgf(tgf_2)){
      gs.dir = dir;
      gs.tick = print_tick;
      gs.file_pos = file_pos;
      gs.sync_pos = sync_pos;
      gs.valid = true;
      gs.out_step = true;
      gs.pos = print_pos;
      file_pos = INVALID_FILE_POS;
      sync_pos = INVALID_SYNC_POS;
      return true;
    }
    else {
      tgf_2.flag = 0;
      return genNextStep(gs);
    }
  }
  else {
    for (;;) {
      if (moveShaperWindowToNext()) {
        calcShaperWindowEndPosAndTime();
        if (generateShapedFuncParams()) {
          return genNextStep(gs);
        }
        else {
          // current print tick and positoin must update to shapper window
          print_tick = shaper_window.tick;
          print_pos = shaper_window.pos;
        }
      }
      else {
        gs.valid = false;
        return false;
      }
    }
  }

}

bool AxisInputShaper::getStep() {
  if (g1.valid) {
    return true;
  }

  if (g2.valid) {
    g1 = g2;
  }

  if (!g1.valid) {
    genNextStep(g1);
    if (!g1.valid) {
      return false;
    }
  }

  // To here g1 has got step
  const_dist_hold = false;
  genNextStep(g2);

  if (g1.valid && g2.valid && (g1.dir != g2.dir)) {
    if (!const_dist_hold) {
      g1.out_step = g2.out_step = false;
      #ifdef SHAPER_LOG_ENABLE
      LOG_I("Abolish steps: axis %d, pos %f == %f\n", axis, g1.pos, g2.pos);
      #endif
    }
    else {
      LOG_I("A const hold move segment, do NOT abolish steps when change dir\n");
    }
  }

  return true;
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
  print_tick = shaper_window.ltick;

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

  sync_pos = INVALID_SYNC_POS;

  if (InputShaperType::none != type) {
    // Shaper axis do NOT support sync in moving as position is not in the correction. Skip the sync move
    do {
      if (!mq->isBetween(cls_p_m_idx)) {
        #ifdef SHAPER_LOG_ENABLE
        // LOG_I("Axis %d move queue is empty, cls_p_m_idx %d\r\n", axis, cls_p_m_idx);
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
      // LOG_I("Axis %d move queue is empty, cls_p_m_idx %d\r\n", axis, cls_p_m_idx);
      #endif
      return false;
    }
    // Push the sync's target position
    if (mq->moves[cls_p_m_idx].flag & BLOCK_FLAG_SYNC_POSITION) {
      sync_pos = mq->moves[cls_p_m_idx].sync_target_pos[axis];
    }
  }

  cls_p.m_idx = cls_p_m_idx;
  // First pluse move to next move, update file pos
  if (0 == shaper_window.t_cls_pls) {
    file_pos = mq->moves[cls_p.m_idx].file_pos;
  }
  else {
    file_pos = INVALID_FILE_POS;
  }
  return true;
}

bool AxisInputShaper::generateShapedFuncParams() {
  tgf_1.flag = tgf_2.flag = 0;
  tgf_1.coef_a = tgf_coef_a_sum;

  // A sync
  // if (0 == shaper_window.wind_tick && InputShaperType::none == type) {
  if (INVALID_SYNC_POS != sync_pos) {
    // LOG_I("Axis %d sync in gen shape fun param\n", axis);
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
    if (dt > 5.0) // millisecond
      const_dist_hold = true;
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
      #ifdef LOG_MIDDLE_POS
      if(axis == X_AXIS)
        axis_mng.tgf_middle_pos_rb.push(fabs(middle_p));
      #endif
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
    tgf.flag = 0;
    return true;
  }

  float safe_strip = EPSILON;
  if (tgf.monotone < 0) {
    np = print_pos - mm_per_step;
    ns = print_pos - mm_per_half_step;
    // if (ns < tgf.end_pos - EPSILON) {
    // if (np < tgf.end_pos - EPSILON) {
    if (ns - safe_strip < tgf.end_pos) {
      #ifdef SHAPER_LOG_ENABLE
      LOG_I("TGF end: axis %d, PP %f,  SP %f tgf.end_pos %f(%d)\r\n", axis, np, ns, tgf.end_pos, tgf.monotone);
      #endif
      return false;
    }
  }
  else if (tgf.monotone > 0) {
    np = print_pos + mm_per_step;
    ns = print_pos + mm_per_half_step;
    // if (ns > tgf.end_pos + EPSILON) {
    // if (np > tgf.end_pos + EPSILON) {
    if (ns + safe_strip > tgf.end_pos) {
      #ifdef SHAPER_LOG_ENABLE
      LOG_I("TGF end: axis %d, PP %f,  SP %f tgf.end_pos %f(%d)\r\n", axis, np, ns, tgf.end_pos, tgf.monotone);
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

  #ifdef SHAPER_LOG_ENABLE
  float itv = ((float)print_tick - lt) / ms2tick;
  LOG_I("Axis %d, print_pos %f, sample_pos %f, start_tick %d, strip %f, ", axis, np, ns, tgf.start_tick, safe_strip);
  LOG_I("tgf return t %.3fms, last tick %d, cur tick %d, itv %.3fms, dir %d\n", t_ms, lt, print_tick, itv, dir);
  #endif

  return true;
}

void AxisMng::init(MoveQueue *mq, uint32_t ms2t) {
  reqAbort = false;
  ms2tick = ms2t;
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

  // uint32_t sw;
  // max_shaper_window_tick = 0;
  // max_shaper_window_right_delta_tick = 0;
  // LOOP_SHAPER_AXES(i) {
  //   sw = axes[i].getShaperWindown();
  //   if (max_shaper_window_tick < sw)
  //     max_shaper_window_tick = sw;
  //   sw = axes[i].right_delta * ms2tick;
  //   if (max_shaper_window_right_delta_tick < sw)
  //     max_shaper_window_right_delta_tick = sw;
  // }

  is_init = true;
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

void AxisMng::update_shaper(void) {
  LOG_I("update_shaper, adding a empty move after update\r\n");
  moveQueue.addEmptyMove(2 * max_shaper_window_tick);
  axis_mng.prepare(moveQueue.move_tail);
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

void AxisMng::enable_shaper(void) {
  planner.synchronize();
  abort();

  LOG_I("All axis enable\n");
  LOOP_SHAPER_AXES(i) {
    axes[i].enable();
  }
  update_shaper();
}

void AxisMng::disable_shaper(void) {
  planner.synchronize();
  abort();

  LOG_I("All axis disable\n");
  LOOP_SHAPER_AXES(i) {
    axes[i].disable();
  }
  update_shaper();
}

void AxisMng::reset_shaper(void) {
  abort();
  init(mq, ms2tick);

  LOG_I("reset shaper, Add empty move\n");

  moveQueue.addEmptyMove(2 * max_shaper_window_tick);
  axis_mng.prepare(moveQueue.move_tail);
}

void AxisMng::log_xy_shpaer(void) {
  LOG_I("X type: %s, frequency: %lf, zeta: %lf\n", input_shaper_type_name[int(x_sp->type)], x_sp->frequency, x_sp->zeta);
  LOG_I("Y type: %s, frequency: %lf, zeta: %lf\n", input_shaper_type_name[int(y_sp->type)], y_sp->frequency, y_sp->zeta);
}

bool AxisMng::prepare(uint8_t m_idx) {

  uint32_t sw;
  max_shaper_window_tick = 0;
  max_shaper_window_right_delta_tick = 0;
  LOOP_SHAPER_AXES(i) {
    if (axes[i].prepare(m_idx)) {
      pt.axis[i] = axes[i].print_pos;
    }

    sw = axes[i].getShaperWindown();
    if (max_shaper_window_tick < sw)
      max_shaper_window_tick = sw;
    sw = axes[i].right_delta * ms2tick;
    if (max_shaper_window_right_delta_tick < sw)
      max_shaper_window_right_delta_tick = sw;
  }

  LOG_I("max_shaper_window_tick %d, max_shaper_window_right_delta_tick %d\r\n", max_shaper_window_tick, max_shaper_window_right_delta_tick);
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

bool AxisMng::getNextStep(StepInfo &step_info) {

  AxisInputShaper *dm = findNearestPrintTickAxis();
  if (dm) {
    // if (PENDING(dm->print_tick, cur_print_tick)) {
    //   LOG_E("### ERROR ####: cur print tick < last print tick %d\r\n", int(cur_print_tick - dm->print_tick));
    //   dm->print_tick = cur_print_tick;
    // }
    // if (!dm->have_gen_step_tick) {
    //   LOG_E("### ERROR ####: got a in-have gen step tick\r\n");
    // }
    if (PENDING(dm->g1.tick, cur_print_tick)) {
      LOG_E("### ERROR ####: cur print tick < last print tick %d\r\n", int(cur_print_tick - dm->g1.tick));
      dm->g1.tick = cur_print_tick;
    }
    if (!dm->g1.valid) {
      LOG_E("### ERROR ####: got a in-have gen step tick\r\n");
    }

    // step time
    step_info.time_dir.itv = (uint16_t)(dm->g1.tick - cur_print_tick);
    step_info.time_dir.dir = dm->g1.dir > 0 ? 1 : 0;
    step_info.time_dir.out_step = dm->g1.out_step;
    step_info.time_dir.move_bits = 1<<dm->axis;
    step_info.time_dir.axis = dm->axis;

    // step flag data, sync and block position
    step_info.time_dir.sync = 0;
    if (INVALID_SYNC_POS != dm->g1.sync_pos) {
      step_info.time_dir.sync = 1;
      step_info.flag_data.sync_pos = dm->g1.sync_pos;
    }
    step_info.time_dir.update_file_pos = 0;
    if (E_AXIS == dm->axis && INVALID_FILE_POS != dm->g1.file_pos) {
      step_info.time_dir.update_file_pos = 1;
      step_info.flag_data.file_pos = dm->g1.file_pos;
    }

    cur_print_tick = dm->g1.tick;
    dm->g1.valid = false;

    return true;
  }
  else {
    return false;
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
  LOG_I("Axes mng aborted\r\n");
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

AxisInputShaper *AxisMng::findNearestPrintTickAxis() {
  AxisInputShaper *nearest_axis = nullptr;
  LOOP_SHAPER_AXES(i) {
    // axes[i].genNextStep();
    // if (axes[i].have_gen_step_tick) {
    if (axes[i].getStep()) {
      if (nearest_axis) {
        // if (PENDING(axes[i].print_tick, nearest_axis->print_tick)) {
        if (PENDING(axes[i].g1.tick, nearest_axis->g1.tick)) {
          nearest_axis = &axes[i];
        }
      }
      else {
        nearest_axis = &axes[i];
      }
    }
  }

  // if (nearest_axis && ELAPSED(nearest_axis->print_tick, mq->can_print_head_tick)) {
  if (nearest_axis && ELAPSED(nearest_axis->g1.tick, mq->can_print_head_tick)) {
    // LOG_I("ActiveDM(%d)'s tick must wait for move's gen step tick\r\n", nearest_axis->axis);
    nearest_axis = nullptr;
  }

  return nearest_axis;
}