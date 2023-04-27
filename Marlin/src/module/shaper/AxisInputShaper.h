#pragma once


#include "TimeGenFunc.h"
#include "MoveQueue.h"
#include "StepsSeq.h"
#include "CircularBuffer.h"


#define CALC_OF_BINOMIAL(tgf)     ((tgf.coef_a * tgf.time_wind + tgf.coef_b) * tgf.time_wind)


// #define LOG_MIDDLE_POS
// #define SHAPER_LOG_ENABLE
// #define LOG_MOTION_INFO
// #define LOG_PRINT_POS
#define START_TICK                    (0xFAA2B57F)
#define SHAPER_VIBRATION_REDUCTION    (20)
#define LOOP_SHAPER_AXES(VAR)         LOOP_S_L_N(VAR, 0, NUM_AXIS)
#define INVALID_FILE_POS              (0xFFFFFFFF)
#define INVALID_SYNC_POS              (0x7FFFFFFF)
#define EMPTY_MOVE_TIME_TICK          (200 * STEPPER_TIMER_TICKS_PER_MS)

#define SP_DEFT_TYPE                  (InputShaperType::ei)
#define SP_DEFT_FREQ                  (50)
#define SP_DEFT_ZETA                  (0.1)


enum class InputShaperType : int
{
  none = 0,
  ei = 1,
  ei2 = 2,
  ei3 = 3,
  mzv = 4,
  zv = 5,
  zvd = 6,
  zvdd = 7,
  zvddd = 8
};

class ShaperPluse
{
public:
  int n;
  float A[5];
  float T[5];
  void log() {
    LOG_I("\r\n==== Shaper pluse ====\r\n");
    for (int i = 0; i < n; i++) {
      LOG_I("Pluse: %d, A: %lf, T: %lf\n", i, A[i], T[i]);
    }
  }
};

class CalcPluseInfo
{
public:
  uint8_t m_idx;            // move index
  float   A;                // Pluse's strenghth
  int     T;                // Pluse's time, tick
};

class ShaperWindow
{
public:
  int pls_cnt;                // Plues count
  int t_cls_pls;              // Time closest plues index
  float pos, lpos;            // Current position and last position
  uint32_t tick, ltick;       // Current tick time and last tick
  uint32_t wind_tick;         // windown continue time spand
  CalcPluseInfo pluse[5];     // Calculate plues's information

public:
  void reset() { pos = lpos = 0; tick = ltick = 0; }
  void log(int Axis, uint32_t ms2tick) {
    LOG_I("\r\n===== AXIS %D shaper window =====\r\n", Axis);
    LOG_I("pls_cnt: %d\r\n", pls_cnt);
    LOG_I("closest_pls: %d\r\n", t_cls_pls);
    LOG_I("dist: %f\r\n", pos - lpos);
    LOG_I("lpos %f ~ pos %f\r\n", lpos, pos);
    LOG_I("ltick %d ~ tick %d\r\n", tick - wind_tick, tick);
    LOG_I("spand %f ms\r\n", (float)wind_tick / ms2tick);
    for(int i = 0; i < pls_cnt; i++) {
      LOG_I("pluse[%d] = mi:%d\n", i, pluse[i].m_idx);
    }
    LOG_I("\r\n");
  }
};

struct genStep {
  uint8_t   valid;
  uint8_t   out_step;
  int8_t    dir;
  uint8_t   reserve;
  uint32_t  tick;
  uint32_t  sync_pos;
  uint32_t  file_pos;
};

struct input_shaper_setting {
  int type;
  float freq;
  float dampe;
};

struct step_seq_statistics_info {
  uint32_t sys_time_ms;
  float use_rate;
  float prepare_time_ms;
};

struct motion_info {
  char        tag[4];
  uint32_t    sys_time_ms;
  uint32_t    block_count;
  uint32_t    block_planned_count;
  uint32_t    move_count;
  uint32_t    step_count;
  float       block_use_rate;
  float       move_use_rate;
  float       step_use_rate;
  float       step_prepare_time;
};

class AxisInputShaper
{
public:
  int axis;
  int dir;

  float print_pos;
  uint32_t print_tick;
  uint32_t sync_tick;
  uint32_t block_sync_tick;
  bool have_gen_step_tick;

  bool const_dist_hold;
  struct genStep g1, g2;

  int sync_pos;
  uint32_t file_pos;

  float right_delta;              // millisecond
  float left_delta;               // millisecond
  ShaperWindow shaper_window;

  TimeGenFunc tgf_1, tgf_2;
  float delta_e = 0;
  #ifdef LOG_MOTION_INFO
  circular_buffer<TimeGenFunc> tgf_rb;
  #endif

  MoveQueue *mq;
  InputShaperType type, backup_type;
  float frequency;
  float zeta;
  ShaperPluse origin_pluse;
  ShaperPluse shift_pluse;
  uint32_t ms2tick;
  float tgf_coef_a_sum;
  float mm_per_step;
  float mm_per_half_step;

  bool no_more_move;

public:
  AxisInputShaper(){};
  void init(int axis, MoveQueue *mq, InputShaperType type, float freq, float zeta, uint32_t s2t);
  void shaper_init(void);
  void shaper_init(InputShaperType type, float frequency, float zeta);
  void reset();
  void setConfig(int type, float frequency, float zeta);
  bool prepare(int m_idx);
  bool genNextStep() {
    if (have_gen_step_tick) {
      return true;
    }
    if (tgf_1.flag) {
      if (getTimeFromTgf(tgf_1)){
        return true;
      }
      else {
        tgf_1.flag = 0;
        return genNextStep();
      }
    }
    else if(tgf_2.flag) {
      if (getTimeFromTgf(tgf_2)){
        return true;
      }
      else {
        tgf_2.flag = 0;
        return genNextStep();
      }
    }
    else {
      for (;;) {
        if (moveShaperWindowToNext()) {
          calcShaperWindowEndPosAndTime();
          if (generateShapedFuncParams()) {
            return genNextStep();
          }
          else {
            // current print tick and positoin must update to shapper window
            print_tick = shaper_window.tick;
            print_pos = shaper_window.pos;
          }
        }
        else {
          print_tick = shaper_window.tick;
          return false;
        }
      }
    }
  }
  #if 0
  bool genNextStep(struct genStep &gs) {
    if (tgf_1.flag) {
      if (getTimeFromTgf(tgf_1)){
        gs.dir = dir;
        gs.tick = print_tick;
        gs.file_pos = file_pos;
        gs.sync_pos = sync_pos;
        gs.valid = true;
        gs.out_step = true;
        // gs.pos = print_pos;
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
        // gs.pos = print_pos;
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
  #endif

  #if 0
  bool getStep() {
    if (g1.valid) {
      return true;
    }
    else {
      genNextStep(g1);
      if (g1.valid) {
        return true;
      }
      else {
        return false;
      }
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
      // if (!const_dist_hold) {
      {
        g1.out_step = g2.out_step = false;
        #ifdef SHAPER_LOG_ENABLE
        LOG_I("Abolish steps: axis %d, pos %f == %f\n", axis, g1.pos, g2.pos);
        #endif
      }
      // else {
      //   LOG_I("A const hold move segment, do NOT abolish steps when change dir\n");
      // }
    }

    return true;
  }
  #endif

  void logShaperWindow();
  uint32_t getShaperWindown();
  void enable();
  void disable();

private:
  void shiftPulses();
  float calcPosition(void) {
    float res = 0;
    for (int i = 0; i < shaper_window.pls_cnt; i++) {
      CalcPluseInfo &pluse = shaper_window.pluse[i];
      float pd = pluse.A * mq->getAxisPosition(pluse.m_idx, axis, pluse.T + shaper_window.tick);
      res += pd;
    }
    return res;
  }
  bool alignToStartMove(int m_idx);
  void calcShaperWindowEndPosAndTime() {
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
    LOG_I("Axis %d cloesest move index update to %d in plues\r\n", axis, shaper_window.pluse[shaper_window.t_cls_pls].m_idx, shaper_window.t_cls_pls);
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
        LOG_I("delta_v %f delta_e %d, Eda %d\r\n", move->delta_v, delta_e, eda);
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
  bool moveShaperWindowToNext() {
    shaper_window.lpos = shaper_window.pos;
    shaper_window.ltick = shaper_window.tick;
    // The closest pluse move to next move
    CalcPluseInfo &cls_p = shaper_window.pluse[shaper_window.t_cls_pls];
    uint8_t cls_p_m_idx = mq->nextMoveIndex(cls_p.m_idx);

    // sync_pos = INVALID_SYNC_POS;
    if (InputShaperType::none != type) {
      // Shaper axis do NOT support sync in moving as position is not in the correction. Skip the sync move
      do {
        if (!mq->isBetween(cls_p_m_idx)) {
          #ifdef SHAPER_LOG_ENABLE
          // LOG_I("Axis %d move queue is empty, cls_p_m_idx %d\r\n", axis, cls_p_m_idx);
          #endif
          if (!no_more_move) {
            // LOG_I("Axis %u have no move, pluse move index %u, move head %u, move tail %u\n", axis, cls_p.m_idx, mq->move_head, mq->move_tail);
            no_more_move = true;
          }
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
        if (!no_more_move) {
          // LOG_I("Axis %u have no move, pluse move index %u, move head %u, move tail %u\n", axis, cls_p.m_idx, mq->move_head, mq->move_tail);
          no_more_move = true;
        }
        return false;
      }
      // Push the sync's target position
      if (axis == E_AXIS && mq->moves[cls_p_m_idx].flag & BLOCK_FLAG_SYNC_POSITION) {
        sync_pos = mq->moves[cls_p_m_idx].sync_target_pos[axis];
        sync_tick = mq->moves[cls_p_m_idx].end_tick;
      }
    }

    no_more_move = false;
    cls_p.m_idx = cls_p_m_idx;
    // First pluse move to next move, update file pos
    // If this is a G92 position sync block, give up this block sync
    if (axis == E_AXIS && INVALID_SYNC_POS == sync_pos) {
      file_pos = mq->moves[cls_p.m_idx].file_pos;
      block_sync_tick = mq->moves[cls_p_m_idx].start_tick;
    }
    else {
      file_pos = INVALID_FILE_POS;
    }
    return true;
  }

  bool generateShapedFuncParams() {

    tgf_1.flag = tgf_2.flag = 0;
    tgf_1.coef_a = tgf_coef_a_sum;

    // A sync tgf, no motion, just return
    if (INVALID_SYNC_POS != sync_pos) {
      tgf_1.flag |= TimeGenFunc::TGF_SYNC_FLAG;
      return true;
    }

    // A E block sync for pause.
    bool ret = false;
    if (INVALID_FILE_POS != file_pos) {
      tgf_1.flag |= TimeGenFunc::TGF_BLOCK_SYNC_FLAG;
      ret = true;
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
      return ret;
    }

    tgf_1.coef_b = ds / dt - tgf_1.coef_a * dt;
    if (IS_ZERO(tgf_1.coef_a)) {
      if (IS_ZERO(tgf_1.coef_b)) {
        LOG_E("#e# Remove a const tgf\r\n");
        return ret;
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

  FORCE_INLINE bool getTimeFromTgf(TimeGenFunc &tgf) {
    float np;
    float ns;

    if (tgf.flag & TimeGenFunc::TGF_SYNC_FLAG) {
      tgf.flag = 0;
      print_tick = sync_tick;
      have_gen_step_tick = true;
      return true;
    }

    if (tgf.flag & TimeGenFunc::TGF_BLOCK_SYNC_FLAG) {
      tgf.flag &= ~(TimeGenFunc::TGF_BLOCK_SYNC_FLAG);   // just clear this flag
      print_tick = block_sync_tick;
      have_gen_step_tick = true;
      return true;
    }
    // float safe_strip = EPSILON;
    if (tgf.monotone < 0) {
      np = print_pos - mm_per_step;
      ns = print_pos - mm_per_half_step;
      // if (ns < tgf.end_pos - EPSILON) {
      if (ns < tgf.end_pos) {
        #ifdef SHAPER_LOG_ENABLE
        LOG_I("TGF end: axis %d, PP %f,  SP %f tgf.end_pos %f(%d)\r\n", axis, np, ns, tgf.end_pos, tgf.monotone);
        #endif
        return false;
      }
    }
    // else (tgf.monotone > 0) {
    else {
      np = print_pos + mm_per_step;
      ns = print_pos + mm_per_half_step;
      // if (ns > tgf.end_pos + EPSILON) {
      if (ns > tgf.end_pos) {
        #ifdef SHAPER_LOG_ENABLE
        LOG_I("TGF end: axis %d, PP %f,  SP %f tgf.end_pos %f(%d)\r\n", axis, np, ns, tgf.end_pos, tgf.monotone);
        #endif
        return false;
      }
    }
    // else {
    //   return false;
    // }

    // float delta_dist = (float)(tgf.start_pos - ns);
    // #ifdef SHAPER_LOG_ENABLE
    // LOG_I("Axis %d delta_dist %f\r\n", axis, delta_dist);
    // #endif
    // float t_ms = tgf.dist2time(delta_dist);
    // if (t_ms < 0.0) {
    //   // When we can make sure this will NOT hanppen?
    //   LOG_E("### ERROR ###: t_ms(%f) < 0.0)\r\n", t_ms);
    //   t_ms = 0;
    // }

    #ifdef SHAPER_LOG_ENABLE
    // uint32_t lt = print_tick;
    #endif
    print_tick = tgf.start_tick + LROUND(tgf.dist2time(tgf.start_pos - ns) * ms2tick);
    // if (!IS_ZERO(tgf.avg_itv)) {
    //   print_tick += LROUND(tgf.avg_itv * ms2tick);
    // }
    // else {
    //   print_tick = tgf.start_tick + LROUND(tgf.dist2time(tgf.start_pos - ns) * ms2tick);
    // }

    dir = tgf.monotone;
    print_pos = np;

    #ifdef SHAPER_LOG_ENABLE
    // float itv = ((float)print_tick - lt) / ms2tick;
    // LOG_I("Axis %d, print_pos %.1f, sample_pos %.1f, start_tick %u, ", axis, np, ns, tgf.start_tick);
    // LOG_I("last tick %u, cur tick %u, itv %.3fms, dir %d\n", lt, print_tick, itv, dir);
    #endif

    have_gen_step_tick = true;
    return true;
  };
};

class AxisMng
{

public:
  void init(MoveQueue *mq, uint32_t m2t);
  void loop(void);
  bool planner_sync(void);
  void load_shaper_setting(void);
  bool update_shaper(void);
  bool input_shaper_set(int axis, int type, float freq, float dampe);
  bool input_shaper_get(int axis, int &type, float &freq, float &dampe);
  bool endisable_shaper(bool endisable);
  bool reset_shaper(void);
  void log_xy_shpaer(void);
  bool prepare(uint8_t m_idx);
  void logShaperWindows();

  bool getNextStep(StepInfo &step_info) {
    AxisInputShaper *dm = findNearestPrintTickAxis();
    if (dm) {
      if (PENDING(dm->print_tick, cur_print_tick)) {
        LOG_E("### ERROR ####: cur print tick(%d) < last print tick %d\r\n", dm->axis, (cur_print_tick - dm->print_tick));
        dm->print_tick = cur_print_tick;
      }

      // Set from dm
      step_info.time_dir.axis = dm->axis;
      step_info.time_dir.dir = dm->dir > 0 ? 1 : 0;
      step_info.time_dir.move_bits = 1<<dm->axis;
      step_info.time_dir.itv = (uint16_t)(dm->print_tick - cur_print_tick);
      cur_print_tick = dm->print_tick;

      // A normal step output, update cur_print_tick;
      if (INVALID_SYNC_POS == dm->sync_pos) {
        step_info.time_dir.out_step = 1;
        step_info.time_dir.sync = 0;
      }
      // G92 sync block, do NOT update the cur_print_tick
      else {
        step_info.time_dir.out_step = 0;
        step_info.time_dir.sync = 1;
        step_info.flag_data.sync_pos = dm->sync_pos;
      }

      // FILE POSITION
      step_info.time_dir.update_file_pos = 0;
      if (E_AXIS == dm->axis && INVALID_FILE_POS != dm->file_pos) {
        step_info.time_dir.update_file_pos = 1;
        step_info.flag_data.file_pos = dm->file_pos;
      }

      // Clear dm's data
      dm->have_gen_step_tick = false;
      dm->sync_pos = INVALID_SYNC_POS;
      dm->file_pos = INVALID_FILE_POS;
      return true;
    }
    else {
      return false;
    }
  }

  #if 0
  bool getNextStep(StepInfo &step_info) {
    AxisInputShaper *dm = findNearestPrintTickAxis();
    if (dm) {
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
  #endif
  bool tgfValid();
  void abort();
  void updateOldestPluesTick();

  AxisInputShaper *findNearestPrintTickAxis() {
    AxisInputShaper *nearest_axis = nullptr;
    LOOP_SHAPER_AXES(i) {
      axes[i].genNextStep();
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
      // LOG_I("Axis %d tick must wait for move's gen step tick\r\n", nearest_axis->axis, nearest_axis->axis);
      nearest_axis = nullptr;
    }

    return nearest_axis;
  }

  #if 0
  AxisInputShaper *findNearestPrintTickAxis() {
    AxisInputShaper *nearest_axis = nullptr;
    LOOP_SHAPER_AXES(i) {
      if (axes[i].getStep()) {
        if (nearest_axis) {
          if (PENDING(axes[i].g1.tick, nearest_axis->g1.tick)) {
            nearest_axis = &axes[i];
          }
        }
        else {
          nearest_axis = &axes[i];
        }
      }
    }
    if (nearest_axis && ELAPSED(nearest_axis->g1.tick, mq->can_print_head_tick)) {
      // LOG_I("ActiveDM(%d)'s tick must wait for move's gen step tick\r\n", nearest_axis->axis);
      nearest_axis = nullptr;
    }
    return nearest_axis;
  }
  #endif

  bool req_endisable_shaper(bool endisble);
  bool req_update_shaper(void);
  bool req_reset_shaper(void);

public:
  bool is_init = false;
  AxisInputShaper *x_sp;
  AxisInputShaper *y_sp;
  AxisInputShaper *z_sp;
  AxisInputShaper *b_sp;
  AxisInputShaper *e_sp;
  AxisInputShaper axes[NUM_AXIS];
  uint32_t cur_print_tick;
  uint32_t oldest_plues_tick;
  MoveQueue *mq;
  bool reqAbort;
  uint32_t ms2tick;
  uint32_t max_shaper_window_tick;
  uint32_t max_shaper_window_right_delta_tick;
  struct input_shaper_setting is_setting[2];     // just for XY
  #ifdef LOG_MIDDLE_POS
  circular_buffer<float> tgf_middle_pos_rb;
  #endif
  circular_buffer<struct step_seq_statistics_info> step_seq_statistics_rb;
  static circular_buffer<struct motion_info> motion_info_rb;

  bool endisable;                     // Shaper enable or disabel
  bool req_endisable_shaper_flag;     // Request flag
  bool req_shaper_status;             // When request from other task, record the request's status

  bool req_update_shaper_flag;        // Request update flag
  bool req_reset_shaper_flag;         // Request reset flag
};

extern AxisMng axis_mng;