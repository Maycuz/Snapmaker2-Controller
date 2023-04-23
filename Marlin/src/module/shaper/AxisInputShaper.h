#pragma once


#include "TimeGenFunc.h"
#include "MoveQueue.h"
#include "StepsSeq.h"
#include "CircularBuffer.h"


// #define LOG_MIDDLE_POS
// #define SHAPER_LOG_ENABLE
// #define LOG_MOTION_INFO
// #define LOG_PRINT_POS
#define START_TICK                    (0xFAA2B57F)
#define SHAPER_VIBRATION_REDUCTION    (20)
#define LOOP_SHAPER_AXES(VAR)         LOOP_S_L_N(VAR, 0, NUM_AXIS)
#define INVALID_FILE_POS              (0xFFFFFFFF)
#define INVALID_SYNC_POS              (0x7FFFFFFF)
#define EMPTY_MOVE_TIME_TICK          (500 * STEPPER_TIMER_TICKS_PER_MS)

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
  void reset() {pos = lpos = 0; tick = ltick = 0; }
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
  float     pos;
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

class AxisInputShaper
{
public:
  AxisInputShaper(){};
  void init(int axis, MoveQueue *mq, InputShaperType type, float freq, float zeta, uint32_t s2t);
  void shaper_init(void);
  void shaper_init(InputShaperType type, float frequency, float zeta);
  void reset();
  void setConfig(int type, float frequency, float zeta);
  bool prepare(int m_idx);
  bool genNextStep(struct genStep &gs);
  bool getStep();
  void logShaperWindow();
  uint32_t getShaperWindown();
  void enable();
  void disable();

private:
  void shiftPulses();
  float calcPosition();
  bool alignToStartMove(int m_idx);
  void calcShaperWindowEndPosAndTime();
  bool moveShaperWindowToNext();
  bool generateShapedFuncParams();
  bool getTimeFromTgf(TimeGenFunc &tgf);

public:
  int axis;
  int dir;

  float print_pos;
  uint32_t print_tick;

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
  bool getNextStep(StepInfo &step_info);
  bool tgfValid();
  void abort();
  void updateOldestPluesTick();
  AxisInputShaper *findNearestPrintTickAxis();

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

  bool endisable;                     // Shaper enable or disabel
  bool req_endisable_shaper_flag;     // Request flag
  bool req_shaper_status;             // When request from other task, record the request's status

  bool req_update_shaper_flag;        // Request update flag
  bool req_reset_shaper_flag;         // Request reset flag
};

extern AxisMng axis_mng;