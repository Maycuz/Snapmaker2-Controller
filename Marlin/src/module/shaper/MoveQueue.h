#pragma once


#include "../planner.h"
#include "shaper_type_define.h"


#define MOVE_SIZE                   (128)
#define MOVE_MOD(n)                 ((n + MOVE_SIZE)%MOVE_SIZE)
#define RB_IS_FULL(h, t, s)         (((h+1)%s) == t)
#define RB_IS_EMPTY(h, t)           ((h) == (t))
#define EMPTY_TIME                  100
#define SAFE_ISOLATION_TIME_STRIP   (STEPPER_TIMER_TICKS_PER_MS * 2)

/*
Distance: unint step
Time: unit millisecond
*/
class Move {
public:
  uint32_t flag;
  int sync_target_pos[NUM_AXIS];
  float start_v;
  float end_v;
  float accelerate;
  float distance;
  float start_pos[NUM_AXIS];
  float end_pos[NUM_AXIS];
  // int start_pos[NUM_AXIS]; // unit: 0.1 step
  // int end_pos[NUM_AXIS];   // unit: 0.1 step
  float axis_r[NUM_AXIS];
  uint32_t start_tick = 0;
  uint32_t end_tick = 0;
  uint32_t t = 0;
  float delta_v;
  bool use_advance;

public:
  void log(uint8_t idx);
  void reset() {flag = 0;}
};

class MoveQueue {

public:
  MoveQueue();
  void init(uint32_t s2t, uint32_t mswt, uint32_t mswrdt);
  FORCE_INLINE constexpr uint8_t nextMoveIndex(const uint8_t m_index) { return MOVE_MOD(m_index + 1);};
  FORCE_INLINE constexpr uint8_t prevMoveIndex(const uint8_t m_index) { return MOVE_MOD(m_index - 1);};
  FORCE_INLINE bool isBetween(const uint8_t m_index) { return (m_index != move_head) && (MOVE_MOD(move_head - m_index) + MOVE_MOD(m_index - move_tail)) == MOVE_MOD(move_head - move_tail);};
  FORCE_INLINE int getMoveSize() { return MOVE_MOD(move_head - move_tail); };
  FORCE_INLINE int getFreeMoveSize() { return MOVE_SIZE - 1 - getMoveSize(); }

  void reset();
  void addEmptyMove(uint32_t time);
  void addSyncMove(int sync_pos[]);
  Move *addMove(float sv, float ev, float acc, float dist, float axis_r[], uint32_t t);
  bool genMoves(block_t* block);
  float getAxisPosition(int m_idx, int axis, uint32_t time);
  // int getAxisPosition(int m_idx, int axis, uint32_t time);
  void moveTailForward(uint32_t print_tick);
  bool haveMotion();
  float spandTimeWindow();
  void log();

public:
  static uint32_t ms2tick;

public:
  uint8_t move_tail;
  uint8_t move_head;
  Move moves[MOVE_SIZE];
  uint32_t moves_head_tick;
  uint32_t moves_tail_tick;
  uint32_t can_print_head_tick;
  // int last_mq_pos[NUM_AXIS];
  float last_mq_pos[NUM_AXIS];
  uint32_t max_shape_window_tick;
  uint32_t max_shape_window_right_delta_tick;
};

extern MoveQueue moveQueue;