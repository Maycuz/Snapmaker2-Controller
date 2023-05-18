#pragma once


#include "TimeGenFunc.h"
#include "MoveQueue.h"
#include "StepsSeq.h"
#include "CircularBuffer.h"


// #define SHAPER_LOG_ENABLE
#define ENABLE_DIR_ELIMINATE
// #define LOG_MIDDLE_POS
// #define LOG_MOTION_INFO
// #define LOG_PRINT_POS
#define START_TICK                    (0xFAA2B57F)
#define SHAPER_VIBRATION_REDUCTION    (20)
#define LOOP_SHAPER_AXES(VAR)         LOOP_S_L_N(VAR, 0, NUM_AXIS)
#define INVALID_FILE_POS              (0xFFFFFFFF)
#define INVALID_SYNC_POS              (0x7FFFFFFF)
#define INVALID_EXTRUDER              (-1)
#define EMPTY_MOVE_TIME_TICK          (200 * STEPPER_TIMER_TICKS_PER_MS)
#define SP_DEFT_TYPE                  (InputShaperType::ei)
#define SP_DEFT_FREQ                  (50)
#define SP_DEFT_ZETA                  (0.1)
#define CALC_OF_BINOMIAL(tgf)         ((tgf.coef_a * tgf.time_wind + tgf.coef_b) * tgf.time_wind)
#define E_RESET_VALUE                 (1000.0)
#define B_RESET_VALUE                 (1000.0)


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
  uint8_t m_idx;              // move index
  float   A;                  // Pluse's strenghth
  int     T;                  // Pluse's time, tick
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
  char      extruder;
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
  uint32_t    move_tail_tick;
  uint32_t    move_head_tick;
  uint32_t    move_can_print_tick;
  uint32_t    current_print_tick;
};


class AxisInputShaper
{
public:
  int dir;
  int axis;
  float print_pos;
  uint32_t print_tick;
  uint32_t sync_tick;
  uint32_t block_sync_tick;
  uint32_t extruder_chg_tick;
  int sync_pos;
  uint32_t file_pos;
  char target_extruder;
  TimeGenFunc tgf_1, tgf_2;
  struct genStep g1, g2;
  float delta_e;
  bool no_more_move;
  float right_delta;              // millisecond
  float left_delta;               // millisecond
  ShaperWindow shaper_window;
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

public:
  void init(int axis, MoveQueue *mq, InputShaperType type, float freq, float zeta, uint32_t s2t);
  void shaper_init(void);
  void shaper_init(InputShaperType type, float frequency, float zeta);
  void reset();
  void setConfig(int type, float frequency, float zeta);
  void enable();
  void disable();
  bool prepare(int m_idx);
  bool alignToStartMove(int m_idx);
  void shiftPulses();
  void logShaperWindow();
  uint32_t getShaperWindown();

  float calcPosition(void) {
    float res = 0;
    for (int i = 0; i < shaper_window.pls_cnt; i++) {
      CalcPluseInfo &pluse = shaper_window.pluse[i];
      float pd = pluse.A * mq->getAxisPosition(pluse.m_idx, axis, pluse.T + shaper_window.tick);
      res += pd;
    }
    return res;
  }

bool moveShaperWindowToNext() {
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
          // LOG_I("Axis %d move queue is empty, cls_p_m_idx %d\r\n", axis, cls_p_m_idx);
          #endif
          if (!no_more_move) {
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
          no_more_move = true;
        }
        return false;
      }
    }

    no_more_move = false;
    cls_p.m_idx = cls_p_m_idx;

    if (E_AXIS == axis) {
      // G92 sync
      if (mq->moves[cls_p_m_idx].flag & BLOCK_FLAG_SYNC_POSITION) {
        sync_pos = mq->moves[cls_p_m_idx].sync_target_pos[axis];
        sync_tick = mq->moves[cls_p_m_idx].end_tick;
      }
      else {
        // If this is a G92 position sync block, give up this block sync
        file_pos = mq->moves[cls_p.m_idx].file_pos;
        block_sync_tick = mq->moves[cls_p_m_idx].start_tick;
      }

      // E reset
      if (mq->moves[cls_p_m_idx].flag & BLOCK_FLAG_RESET_E_SHAPER_POSITION) {
        delta_e = print_pos = shaper_window.lpos = shaper_window.pos = 0.0;
        LOG_I("Shaper E position reset\n");
      }

      // Change extruder
      if (mq->moves[cls_p_m_idx].flag & BLOCK_FLAG_CHG_EXTRUDER) {
        target_extruder = mq->moves[cls_p_m_idx].extruder;
        extruder_chg_tick = mq->moves[cls_p_m_idx].start_tick;
        LOG_I("Change extruder to %d\n", target_extruder);
      }
    }

    if (B_AXIS == axis) {
      if (mq->moves[cls_p_m_idx].flag & BLOCK_FLAG_RESET_B_SHAPER_POSITION) {
        shaper_window.lpos = 0.0;
        print_pos = 0.0;
        LOG_I("Shaper B position reset\n");
      }
    }

    return true;
  }

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
  }

  bool generateShapedFuncParams() {
    bool ret = false;
    tgf_1.flag = tgf_2.flag = 0;
    tgf_1.coef_a = tgf_coef_a_sum;

    if (E_AXIS == axis) {
      // A sync tgf, no motion, just return
      if (INVALID_SYNC_POS != sync_pos) {
        tgf_1.flag |= TimeGenFunc::TGF_SYNC_FLAG;
        return true;
      }

      // A block file positon sync.
      if (INVALID_FILE_POS != file_pos) {
        tgf_1.flag |= TimeGenFunc::TGF_FILE_POS_SYNC_FLAG;
        ret = true;
      }

      // Change extruder
      if (INVALID_EXTRUDER != target_extruder) {
        tgf_1.flag |= TimeGenFunc::TGF_CHG_EXTRUDER;
        ret = true;
      }
    }

    float s1 = shaper_window.lpos;
    float s2 = shaper_window.pos;
    float ds = s2 - s1;
    float dt = (float)shaper_window.wind_tick / ms2tick;

    if (dt < EPSILON || fabs(ds) < EPSILON) {
      #ifdef SHAPER_LOG_ENABLE
      LOG_I("ds %f, dt %f\r\n", ds, dt);
      #endif
      return ret;
    }

    tgf_1.coef_b = ds / dt - tgf_1.coef_a * dt;
    if (IS_ZERO(tgf_1.coef_a)) {
      if (IS_ZERO(tgf_1.coef_b)) {
        LOG_E("### error ###: Remove a const tgf\n");
        return ret;
      }
      tgf_1.monotone = tgf_1.coef_b > 0.0 ? 1 : -1;
      tgf_1.start_tick = shaper_window.ltick;
      tgf_1.end_tick = shaper_window.tick;
      tgf_1.time_wind = dt;
      tgf_1.start_pos = s1;
      tgf_1.end_pos = s2;
      tgf_1.flag |= TimeGenFunc::TGF_STEP_FLAG;
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
        tgf_1.flag |= TimeGenFunc::TGF_STEP_FLAG;
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
        tgf_2.flag |= TimeGenFunc::TGF_STEP_FLAG;
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
        tgf_1.flag |= TimeGenFunc::TGF_STEP_FLAG;
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

  FORCE_INLINE bool getTimeFromTgf(TimeGenFunc &tgf, struct genStep &gs) {
    gs.valid = false;
    gs.out_step = false;
    gs.file_pos = INVALID_FILE_POS;
    gs.sync_pos = INVALID_SYNC_POS;
    gs.extruder = INVALID_EXTRUDER;

    // A sync, no step output
    if (E_AXIS == axis) {
      gs.file_pos = file_pos;
      gs.sync_pos = sync_pos;
      gs.extruder = target_extruder;

      if (tgf.flag & TimeGenFunc::TGF_SYNC_FLAG) {
        gs.tick = sync_tick;
        gs.valid = true;
        sync_pos = INVALID_SYNC_POS;
        tgf.flag &= ~(TimeGenFunc::TGF_SYNC_FLAG);
        return true;
      }

      // A file position sync
      if (tgf.flag & TimeGenFunc::TGF_FILE_POS_SYNC_FLAG) {
        gs.tick = block_sync_tick;
        gs.valid = true;
        file_pos = INVALID_FILE_POS;
        tgf.flag &= ~(TimeGenFunc::TGF_FILE_POS_SYNC_FLAG);
      }

      // Change extruder
      if (tgf.flag & TimeGenFunc::TGF_CHG_EXTRUDER) {
        gs.tick = extruder_chg_tick;
        gs.valid = true;
        target_extruder = INVALID_EXTRUDER;
        tgf.flag &= ~(TimeGenFunc::TGF_CHG_EXTRUDER);
      }
    }

    // A steps output
    if (tgf.flag & TimeGenFunc::TGF_STEP_FLAG) {
      float np;
      float ns;
      if (tgf.monotone < 0) {
        np = print_pos - mm_per_step;
        ns = print_pos - mm_per_half_step;
        if (ns < tgf.end_pos) {
          #ifdef SHAPER_LOG_ENABLE
          LOG_I("TGF end: axis %d, PP %f,  SP %f tgf.end_pos %f(%d)\r\n", axis, np, ns, tgf.end_pos, tgf.monotone);
          #endif
          tgf.flag &= ~(TimeGenFunc::TGF_STEP_FLAG);
          // Update print tick for E when there is just a lot of file sync for E
          // and the print tick will not update to the right time tick.
          if (gs.valid) {
            print_tick = shaper_window.tick;
          }
          return gs.valid;
        }
      }
      else {
        np = print_pos + mm_per_step;
        ns = print_pos + mm_per_half_step;
        if (ns > tgf.end_pos) {
          #ifdef SHAPER_LOG_ENABLE
          LOG_I("TGF end: axis %d, PP %f,  SP %f tgf.end_pos %f(%d)\r\n", axis, np, ns, tgf.end_pos, tgf.monotone);
          #endif
          tgf.flag &= ~(TimeGenFunc::TGF_STEP_FLAG);
          // Update print tick for E when there is just a lot of file sync for E
          // and the print tick will not update to the right time tick.
          if (gs.valid) {
            print_tick = shaper_window.tick;
          }
          return gs.valid;
        }
      }

      print_tick = tgf.start_tick + LROUND(tgf.dist2time(tgf.start_pos - ns) * ms2tick);
      dir = tgf.monotone;
      print_pos = np;

      #ifdef SHAPER_LOG_ENABLE
      // float itv = ((float)print_tick - lt) / ms2tick;
      // LOG_I("Axis %d, print_pos %.1f, sample_pos %.1f, start_tick %u, ", axis, np, ns, tgf.start_tick);
      // LOG_I("last tick %u, cur tick %u, itv %.3fms, dir %d\n", lt, print_tick, itv, dir);
      #endif

      gs.dir = dir;
      gs.tick = print_tick;
      gs.out_step = true;
      gs.valid = true;
    }

    // Update print tick for E when there is just a lot of file sync for E
    // and the print tick will not update to the right time tick.
    if (gs.valid && !gs.out_step) {
      print_tick = gs.tick;
    }

    return gs.valid;
  }

  bool genNextStep(struct genStep &gs) {
    if (tgf_1.flag){
      if (getTimeFromTgf(tgf_1, gs))
        return true;
      else
        return genNextStep(gs);
    }
    else if(tgf_2.flag) {
      if (getTimeFromTgf(tgf_2, gs))
        return true;
      else
        return genNextStep(gs);
    }
    else {
      for (;;) {
        if (moveShaperWindowToNext()) {
          calcShaperWindowEndPosAndTime();
          if (generateShapedFuncParams()) {
            return genNextStep(gs);
          }
          else {
            // If generateShapedFuncParams return false, means no more motion
            // print_tick and position update to shaper_window
            print_tick = shaper_window.tick;
            print_pos = shaper_window.pos;
          }
        }
        else {
          // A adding empty move may generating any steps as position keep
          // the same. If a adding emtpy move doest not generate steps, than
          // the print tick remain on the last move. This will cause
          // planner::synchronization halt. So when moveShaperWindowToNext()
          // return false, no more move, we need to update the print_tick.
          // But not update the position
          print_tick = shaper_window.tick;
          gs.valid = false;
          return false;
        }
      }
    }
  }

  bool getStep(void) {
    #ifndef ENABLE_DIR_ELIMINATE
    if (!g1.valid) {
      return genNextStep(g1);
    }
    else {
      return true;
    }
    #else
    if (g1.valid) {
      if (!g2.valid) {
        genNextStep(g2);
      }
    }
    else {
      if (g2.valid) {
        g1 = g2;
        g2.valid = false;
        genNextStep(g2);
      }
      else {
        genNextStep(g1);
        if (g1.valid) {
          genNextStep(g2);
        }
      }
    }

    if (g1.valid && g2.valid && (g1.dir != g2.dir) && (g1.out_step && g2.out_step)) {
      g1.out_step = g2.out_step = false;
      #ifdef SHAPER_LOG_ENABLE
      LOG_I("Abolish steps: axis %d\n", axis);
      #endif
    }

    return g1.valid;
    #endif
  }
};

class AxisMng {
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
  bool tgfValid();
  void abort();
  void updateOldestPluesTick();
  bool req_endisable_shaper(bool endisble);
  bool req_update_shaper(void);
  bool req_reset_shaper(void);

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
      nearest_axis = nullptr;
    }

    return nearest_axis;
  }

  bool getNextStep(StepInfo &step_info) {
    AxisInputShaper *dm = findNearestPrintTickAxis();
    if (dm) {
      if (PENDING(dm->g1.tick, cur_print_tick)) {
        LOG_E("### ERROR ####: cur print tick < last print tick %d\r\n", int(cur_print_tick - dm->g1.tick));
        dm->g1.tick = cur_print_tick;
      }

      // Pluse
      step_info.time_dir.axis = dm->axis;
      step_info.time_dir.itv = (uint16_t)(dm->g1.tick - cur_print_tick);
      step_info.time_dir.dir = dm->g1.dir > 0 ? 1 : 0;
      step_info.time_dir.out_step = dm->g1.out_step;
      step_info.time_dir.move_bits = 1<<dm->axis;

      // Position sync
      step_info.time_dir.sync = 0;
      if (INVALID_SYNC_POS != dm->g1.sync_pos) {
        step_info.time_dir.sync = 1;
        step_info.flag_data.sync_pos = dm->g1.sync_pos;
      }

      // Block sync
      step_info.time_dir.update_file_pos = 0;
      if (INVALID_FILE_POS != dm->g1.file_pos) {
        step_info.time_dir.update_file_pos = 1;
        step_info.flag_data.file_pos = dm->g1.file_pos;
      }

      // Change extruder
      step_info.time_dir.chg_extruder = 0;
      if (INVALID_EXTRUDER != dm->g1.extruder) {
        step_info.time_dir.chg_extruder = 1;
        step_info.flag_data.extruder = dm->g1.extruder;
      }

      // Update and clear
      cur_print_tick = dm->g1.tick;
      dm->g1.valid = false;

      return true;
    }
    else {
      return false;
    }
  }

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