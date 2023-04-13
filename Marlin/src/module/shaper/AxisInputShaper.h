#pragma once


#include "TimeGenFunc.h"
#include "MoveQueue.h"
#include "FuncManager.h"
#include "StepsSeq.h"
#include "CircularBuffer.h"


// #define SHAPER_LOG_ENABLE
// #define LOG_MOTION_INFO
#define AXES_NUM                      (4)
#define EMPTY_MOVE_TIME               (100 * STEPPER_TIMER_TICKS_PER_MS)
#define SHAPER_VIBRATION_REDUCTION    (20)
#define LOOP_SHAPER_AXES(VAR)         LOOP_S_L_N(VAR, 0, AXES_NUM)



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

class AxisInputShaper
{
public:
  AxisInputShaper(){};
  AT_END_OF_TEXT void init(int axis, MoveQueue *mq, InputShaperType type, uint32_t s2t);
  void reset();
  void setConfig(int type, float frequency, float zeta);
  bool prepare(int m_idx);
  bool genNextStepTime();
  void logShaperWindow();
  uint32_t getShaperWindown();
  bool alignToMoveHead();

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
  uint8_t axis_bit_mask;
  int dir;
  float print_pos;
  bool have_gen_step_tick;
  uint32_t print_tick;
  uint32_t last_print_tick;
  bool sync_trigger_flag;
  circular_buffer<int> sync_pos_rb;
  float right_delta;              // millisecond
  float left_delta;               // millisecond
  ShaperWindow shaper_window;
  TimeGenFunc tgf_1, tgf_2;
  float delta_e = 0;
  #ifdef LOG_MOTION_INFO
  circular_buffer<TimeGenFunc> tgf_rb;
  #endif

private:
  MoveQueue *mq;
  InputShaperType type;
  ShaperPluse origin_pluse;
  ShaperPluse shift_pluse;
  float frequency = 50;
  float zeta = 0.1;
  uint32_t ms2tick;
  float tgf_coef_a_sum;
  float mm_per_step;
  float mm_per_half_step;
};

class AxisMng
{

public:
  void init(MoveQueue *mq, uint32_t m2t);
  bool prepare(uint8_t m_idx);
  void logShaperWindows();
  void insertDM(AxisInputShaper *axis);
  bool getNextStep(StepInfo &step_info);
  bool tgfValid();
  void abort();
  void updateOldestPluesTick();
  AxisInputShaper *findMinPrintTickAxis();

public:
  AxisInputShaper *x_sp;
  AxisInputShaper *y_sp;
  AxisInputShaper *z_sp;
  AxisInputShaper *e_sp;
  AxisInputShaper axes[AXES_NUM];
  uint32_t cur_print_tick;
  uint32_t oldest_plues_tick;
  MoveQueue *mq;
  bool reqAbort;
  uint32_t ms2tick;
  uint32_t max_shaper_window_tick;
  uint32_t max_shaper_window_right_delta_tick;
  circular_buffer<float> tgf_middle_pos_rb;
};

extern AxisMng axis_mng;