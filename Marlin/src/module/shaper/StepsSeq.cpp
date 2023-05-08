#include "StepsSeq.h"
#include <stdlib.h>
#include "../../../../snapmaker/src/common/debug.h"


StepsSeq steps_seq;
StepFlag steps_flag;


void StepFlag::reset() {
  head = tail = 0;
}

void StepsSeq::reset() {
  head = tail = 0;
  buf_tick_head = buf_tick_tail = 0;
}

void StepsSeq::logSize() {
  LOG_I("empty: %s\r\n", isEmpty() ? "yes" : "no");
  LOG_I("full: %s\r\n", isFull() ? "yes" : "no");
  LOG_I("count(): %d\r\n", count());
}

void StepsSeq::push_pop_test(uint32_t round) {
  struct StepTimeDir sif;

  LOG_I("\r\nInit\r\n");
  logSize();

  while(round--) {
    srand(millis());
    int c = (rand() % SIZE);
    if (c&0x1) {
      LOG_I("\r\n+++ PUSH %d\r\n", c);
      while(c--) {
        pushQueue(sif);
      }
    }
    else {
      LOG_I("\r\n--- POP %d\r\n", c);
      while(c--) {
        popQueue(&sif);
      }
    }
    logSize();
  }
}

void StepsSeq::buf_tick_test(uint32_t round) {
  uint32_t buf_tick = 0;
  struct StepTimeDir sif;

  uint32_t i = round;
  reset();
  while(i--) {
    srand(millis());
    sif.itv = (rand() & 0xFFFF);
    buf_tick += sif.itv;
    pushQueue(sif);
    LOG_I("\r\nround %d buf tick %d, getBufTick() %d\r\n", i, buf_tick, getBufTick());
  }

  i = round;
  while(i--) {
    popQueue(&sif);
    buf_tick -= sif.itv;
    LOG_I("\r\nround %d buf tick %d, getBufTick() %d\r\n", i, buf_tick, getBufTick());
  }
}