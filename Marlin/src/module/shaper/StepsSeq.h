#pragma once

#include <cstdint>
#include "../../Marlin.h"
#include "../../../../snapmaker/src/common/debug.h"


struct StepFlagData {
  int         sync_pos;
  uint32_t    file_pos;
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
  void reset();
  bool pushQueue(struct StepFlagData &fd);
  bool popQueue(struct StepFlagData *fd);
  bool isEmpty();
  bool isFull();

public:
  static const uint32_t SIZE = 64;

public:
  struct StepFlagData buf[SIZE];
  volatile uint32_t head;
  volatile uint32_t tail;
};

class StepsSeq {
public:
  void reset();
  bool pushQueue(struct StepTimeDir &sif);
  bool popQueue(struct StepTimeDir *sif);
  bool isEmpty();
  bool isFull();
  uint32_t count();
  uint32_t getBufTick();
  float getBufMilliseconds();
  void logSize();
  void push_pop_test(uint32_t round);
  void buf_tick_test(uint32_t round);

public:
  static const uint32_t SIZE = 1024;

private:
  struct StepTimeDir buf[SIZE];
  volatile uint32_t head;
  volatile uint32_t tail;
  volatile uint32_t buf_tick_head;
  volatile uint32_t buf_tick_tail;
  uint32_t s2t = STEPPER_TIMER_RATE;
};

struct StepInfo {
  struct StepTimeDir time_dir;
  struct StepFlagData flag_data;
};

extern StepsSeq steps_seq;
extern StepFlag steps_flag;