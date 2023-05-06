#pragma once

#include <cstdint>
#include "../../Marlin.h"
#include "../../../../snapmaker/src/common/debug.h"


struct StepFlagData {
  int         sync_pos;
  uint32_t    file_pos;
  uint16_t    laser_pwr;
  uint16_t    reserve;
};

struct StepTimeDir{
  uint16_t itv;
  uint16_t axis: 4;
  uint16_t dir: 1;
  uint16_t out_step: 1;
  uint16_t sync: 1;
  uint16_t update_file_pos: 1;
  uint16_t move_bits: 8;
};

class StepFlag {
public:
  static const uint32_t SIZE = 128;
  struct StepFlagData buf[SIZE];
  volatile uint32_t head;
  volatile uint32_t tail;

public:
  void reset();
  FORCE_INLINE bool isFull() { return ((head + 1) % SIZE) == tail; };
  FORCE_INLINE bool isEmpty() { return head == tail; };
  FORCE_INLINE bool pushQueue(struct StepFlagData &fd) {
    if (!isFull()) {
      buf[head] = fd;
      head = (head + 1) % SIZE;
      return true;
    }
    else {
      return false;
    }
  };
  FORCE_INLINE bool popQueue(struct StepFlagData *fd) {
    if (!isEmpty()) {
      *fd = buf[tail];
      tail = (tail + 1) % SIZE;
      return true;
    }
    else {
      return false;
    }
  };
};

class StepsSeq {
public:
  static const uint32_t SIZE = 1024;
  struct StepTimeDir buf[SIZE];
  volatile uint32_t head;
  volatile uint32_t tail;
  volatile uint32_t buf_tick_head;
  volatile uint32_t buf_tick_tail;

public:
  void reset();
  FORCE_INLINE bool isFull() { return ((head + 1) & (SIZE-1)) == tail; };
  FORCE_INLINE bool isEmpty() { return head == tail; };
  FORCE_INLINE bool pushQueue(struct StepTimeDir &sif) {
    if (!isFull()) {
      buf[head] = sif;
      buf_tick_head += buf[head].itv;
      head = (head + 1) & (SIZE-1);
      return true;
    }
    else {
      return false;
    }
  };
  FORCE_INLINE bool popQueue(struct StepTimeDir *sif) {
    if (!isEmpty()) {
      *sif = buf[tail];
      buf_tick_tail += buf[tail].itv;
      tail = (tail + 1) & (SIZE-1);
      // buf_tick_tail += sif->itv;
      return true;
    }
    else {
      return false;
    }
  };

  FORCE_INLINE uint32_t count() { return ((int)(head - tail) + SIZE) & (SIZE - 1); };
  FORCE_INLINE uint32_t getBufTick() { return (buf_tick_head - buf_tick_tail); };
  FORCE_INLINE float getBufMilliseconds() { return (float)getBufTick() / STEPPER_TIMER_TICKS_PER_MS; };
  FORCE_INLINE float useRate(void) { return 100.0 * count() / SIZE; };

  void logSize();
  void push_pop_test(uint32_t round);
  void buf_tick_test(uint32_t round);
};

struct StepInfo {
  struct StepTimeDir time_dir;
  struct StepFlagData flag_data;
};

extern StepsSeq steps_seq;
extern StepFlag steps_flag;